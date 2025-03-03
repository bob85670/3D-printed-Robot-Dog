#ifndef __HARDWARE__
#define __HARDWARE__

#include "Config.hpp"
#include <Adafruit_PWMServoDriver.h>
#include "datatypes.h"

/**
 * @brief Hardware control class for quadruped robot
 * Manages servo motors and sensor interfaces
 */
class Hardware : public Adafruit_PWMServoDriver {
public:
    // Constructors
    explicit Hardware(int addr);
    Hardware();

    // Core functions
    void init_hardware();
    void handle_hardware();
    
    // Servo control
    void attach();
    void detach();
    void set_leg(int leg, datatypes::Rotator rot);
    void set_servo(int leg, int joint, float pulse);

private:
    // Servo configuration
    static constexpr int SERVO_CHANNELS[4][3] = {
        {4, 5, 6},     // Right back leg  (shoulder, upper, lower)
        {0, 1, 2},     // Right front leg (shoulder, upper, lower)
        {15, 14, 13},  // Left front leg  (shoulder, upper, lower)
        {11, 10, 9}    // Left back leg   (shoulder, upper, lower)
    };

    // Servo pulse limits
    static constexpr int PULSE_MIN = 105;
    static constexpr int PULSE_MAX = 500;

    // Servo center offsets (adjust for 90° position)
    int servo_offset[4][3] = {
        {0, 0, 0},  // Right back leg  (shoulder, upper, lower)
        {0, 0, 0},  // Right front leg (shoulder, upper, lower)
        {0, 0, 0},  // Left front leg  (shoulder, upper, lower)
        {0, 0, 0}   // Left back leg   (shoulder, upper, lower)
    };

    // Servo direction multipliers (1 = normal, 0 = inverted)
    const int servo_direction[4][3] = {
        {0, 0, 0},  // Right back leg
        {1, 0, 0},  // Right front leg
        {0, 1, 1},  // Left front leg
        {1, 1, 1}   // Left back leg
    };

    // Joint angle constraints (degrees)
    const int ANGLE_MIN[3] = {-70, 20, 40};   // Minimum angles for shoulder, upper, lower
    const int ANGLE_MAX[3] = {70, 110, 150};  // Maximum angles for shoulder, upper, lower

    // State tracking
    bool servos_attached = false;

    // Internal functions
    void set_joint(int leg, int joint, float degrees);
    void init_mpu();
    void update_mpu_data();
};

#endif // __HARDWARE__
