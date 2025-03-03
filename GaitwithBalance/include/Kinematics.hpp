#ifndef __KINEMATICS__
#define __KINEMATICS__

#include "datatypes.h"
#include "Hardware.hpp"

class Kinematics {
public:
    // Constructor
    Kinematics(Hardware& h);

    // Main control functions
    void handle_kinematics(int state, datatypes::Vector2D direction, float turn, float height, float period);
    
    // Utility functions
    static float inter(float current, float target, float step);
    static int sign(float num);

    // Public variables for offset control
    float base_offset[4];
    float vrt_offset = -16.50; // Vertical offset in mm
    float hrz_offset = -6.00;  // Horizontal offset in mm

private:
    // Constants
    static constexpr float PRECISION = 0.001;   // Precision in mm
    static constexpr float FREQUENCY = 700.0;   // Loop frequency in Hz
    static constexpr float BONE_LENGTH = 105.0; // Leg segment length in mm

    // Body transform parameters
    const datatypes::Transform body_transform = {
        {0, 0, 0},      // Position (mm)
        {0, 0, 0},      // Rotation (degrees)
        {300, 40, 180}  // Scale (mm)
    };

    // Joint origin positions relative to body
    const datatypes::Vector p_joint_origin[4] = {
        {-50, 0, 0},  // Right back leg
        {+50, 0, 0},  // Right front leg
        {+50, 0, 0},  // Left front leg
        {-50, 0, 0}   // Left back leg
    };

    // Step parameters
    const datatypes::Vector step_extent = {40, 40, 26}; // Step size in mm

    // Leg direction multipliers
    const float l_inv[4][2] = {
        {+1.f, -1.f},  // Right back leg
        {-1.f, -1.f},  // Right front leg
        {-1.f, +1.f},  // Left front leg
        {+1.f, +1.f}   // Left back leg
    };

    // Internal state variables
    float c[4];        // Current leg positions
    float c_iter[4];   // Iteration counters
    bool c_inv[4];     // Inversion flags
    float stored_0x;   // Stored X position
    Hardware& hardware;

    // Internal calculation functions
    void count_c(int leg_index, datatypes::Vector2D direction, float period);
    datatypes::Vector trot_gait_func(datatypes::Vector2D c0, datatypes::Vector2D direction, bool invert);
    float c_base(float angle);
    datatypes::Vector pitch_roll_axis(int leg, float base, datatypes::Rotator rotation);
    datatypes::Vector yaw_axis(int leg, float yaw);
    datatypes::Vector2D c_direction_ratio(datatypes::Vector2D direction);
    datatypes::Rotator k_model(float x0, float y0, float z0, float pitch_offset, float roll_offset, datatypes::Vector vec);
    datatypes::Rotator c_triangle(float angle0, float angle1, float base);
};

#endif // __KINEMATICS__