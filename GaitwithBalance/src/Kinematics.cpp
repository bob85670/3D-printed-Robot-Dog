#include <Arduino.h>
#include "Kinematics.hpp"
#include "Config.hpp"

Kinematics::Kinematics(Hardware& h) :
    base_offset{0, -1, 0, -2},
    stored_0x(0.f),
    hardware(h) {
}

void Kinematics::handle_kinematics(int state, datatypes::Vector2D direction, float turn, float height, float period) {
    for (int leg = 0; leg < 4; leg++) {
        // Calculate base position with height adjustment
        float base = c_base(90) + base_offset[leg] + height;

        // Calculate movement direction with turn
        datatypes::Vector2D dir = {
            PRECISION + direction.x + turn * l_inv[leg][1],
            PRECISION + direction.y + turn * l_inv[leg][0]
        };

        // Update leg timing
        count_c(leg, dir, period);

        // Calculate direction ratios
        datatypes::Vector2D ratio_dir = c_direction_ratio(dir);
        datatypes::Vector position = {0, 0, 0};

        // Calculate leg positions based on state
        switch (state) {
            case 1: // Trot gait
                if (abs(dir.x) > PRECISION || abs(dir.y) > PRECISION) {
                    position = trot_gait_func(
                        {ratio_dir.x * c[leg], l_inv[leg][1] * ratio_dir.y * c[leg]},
                        dir,
                        boolean(leg % 2) ^ boolean(c_inv[leg] % 2)
                    );
                }
                break;

            case 2: // Yaw control
                position = yaw_axis(leg, turn);
                break;

            case 3: // Pitch-roll control
                position = pitch_roll_axis(leg, base, {0, direction.x, direction.y});
                break;

            case 4: // Object detection mode
                position = yaw_axis(leg, stored_0x);
                if (abs(stored_0x + turn / 4.0f) < 32.0f) {
                    stored_0x = inter(stored_0x, stored_0x + turn / 4.0f, 0.5f);
                }
                break;
        }

        // Convert position to joint angles and apply
        datatypes::Rotator joint_angles = k_model(
            vrt_offset, hrz_offset, base,
            0, 0, position
        );
        hardware.set_leg(leg, joint_angles);
    }
}

void Kinematics::count_c(int leg_index, datatypes::Vector2D dir, float period) {
    // Calculate step width based on direction
    float width = step_extent.x * mm / (2 / max(abs(dir.x), abs(dir.y)));
    
    // Calculate current position in cycle
    float position = (width * 2) * (c_iter[leg_index] / round(FREQUENCY / period)) - width;

    c[leg_index] = position;
    c_iter[leg_index] += 1.0f;

    // Reset cycle if complete
    if (c_iter[leg_index] > round(FREQUENCY / period)) {
        c[leg_index] = -width;
        c_iter[leg_index] = 1.0f;
        c_inv[leg_index] = !c_inv[leg_index];
    }
}

datatypes::Vector Kinematics::trot_gait_func(datatypes::Vector2D c0, datatypes::Vector2D dir, bool invert) {
    // Calculate step dimensions
    float width = step_extent.x * mm / 2 * dir.x;
    float length = step_extent.y * mm * 4 * dir.y;
    float height = step_extent.z * mm;

    // Invert position if needed
    if (!invert) {
        c0 = {-c0.x, -c0.y};
    }

    // Calculate step height using elliptical path
    float step_height = sqrt(abs((1 - sq(c0.x / width) - sq(c0.y / length)) * sq(height)));
    
    return {
        c0.x / mm,
        c0.y / mm,
        step_height / mm * int(invert)
    };
}

float Kinematics::c_base(float angle) {
    return sin(radians(angle / 2)) * BONE_LENGTH * 2;
}

datatypes::Vector Kinematics::pitch_roll_axis(int leg, float base, datatypes::Rotator rotation) {
    // Calculate initial positions
    float width = body_transform.scl.x / 2 * l_inv[leg][0] + p_joint_origin[leg].x - vrt_offset;
    float length = body_transform.scl.z / 2 + hrz_offset;

    // Convert angles to radians
    float pitch_rad = radians(rotation.pitch);
    float roll_rad = radians(rotation.roll) * l_inv[leg][1];

    // Calculate pitch components
    float pitch_sin = sin(pitch_rad) * width;
    float pitch_offset = (1 - cos(pitch_rad)) * -width;
    float pitch_magnitude = sqrt(sq(base + pitch_sin) + sq(pitch_offset));
    pitch_rad += asin(pitch_offset / pitch_magnitude);

    // Calculate intermediate values
    float pitch_cos = cos(pitch_rad) * pitch_magnitude;
    float pitch_sin_final = sin(pitch_rad) * pitch_magnitude;

    // Calculate roll components
    float roll_sin = sin(roll_rad) * length;
    float roll_offset = (1 - cos(roll_rad)) * length;
    float roll_magnitude = sqrt(sq(pitch_cos - roll_sin) + sq(roll_offset));
    roll_rad += asin(roll_offset / roll_magnitude);

    // Calculate final positions
    float roll_cos = cos(roll_rad) * roll_magnitude;
    float roll_sin_final = sin(roll_rad) * roll_magnitude;

    return {pitch_sin_final, roll_sin_final, base - roll_cos};
}

datatypes::Vector Kinematics::yaw_axis(int leg, float yaw) {
    // Calculate initial positions
    float x = body_transform.scl.x / 2 - abs(p_joint_origin[leg].x) - vrt_offset * l_inv[leg][0];
    float y = body_transform.scl.z / 2 + hrz_offset;
    
    // Calculate rotation
    float radius = sqrt(sq(x) + sq(y));
    float angle = asin(y / radius) - radians(yaw) * l_inv[leg][0] * l_inv[leg][1];

    // Calculate new positions
    return {
        (x - cos(angle) * radius) * l_inv[leg][0],
        sin(angle) * radius - y,
        0
    };
}

datatypes::Vector2D Kinematics::c_direction_ratio(datatypes::Vector2D dir) {
    float max_component = max(abs(dir.x), abs(dir.y));
    return {
        dir.x / max_component,
        dir.y / max_component
    };
}

datatypes::Rotator Kinematics::k_model(float x0, float y0, float z0,
                                     float pitch_offset, float roll_offset,
                                     datatypes::Vector vec) {
    // Calculate final positions
    float x = x0 + vec.x;
    float y = y0 + vec.y;
    float z = z0 - vec.z;

    // Calculate intermediate values
    float base_xy = sqrt(sq(x) + sq(y));
    float height = sqrt(sq(base_xy) + sq(z));

    // Calculate angles
    float pitch = degrees(atan(x / z));
    float roll = degrees(atan(y / z));

    return c_triangle(pitch + pitch_offset, roll + roll_offset, height);
}

datatypes::Rotator Kinematics::c_triangle(float angle0, float angle1, float base) {
    float angle3 = degrees(asin((base / 2.0) / BONE_LENGTH)) * 2;
    float angle2 = angle3 / 2 + angle0;

    return {angle1, angle2, angle3};
}

float Kinematics::inter(float current, float target, float step) {
    if (current < target - step) {
        return ((current * 1000.0f) + (step * 1000.0f)) / 1000.0f;
    } else if (current > target + step) {
        return ((current * 1000.0f) - (step * 1000.0f)) / 1000.0f;
    }
    return target;
}

int Kinematics::sign(float num) {
    return int(num >= 0) - int(num < 0);
}
