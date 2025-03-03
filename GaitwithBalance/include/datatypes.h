#ifndef __DATA_TYPES__
#define __DATA_TYPES__

/**
 * Unit conversion constant
 * Used to convert millimeters to internal units
 */
#define mm 0.1

namespace datatypes {

/**
 * @brief Represents a single step movement
 */
struct Step {
    float base;   ///< Base position
    float angle;  ///< Step angle
};

/**
 * @brief 3D vector with double precision
 * Used for precise position calculations
 */
struct Vector {
    double x;  ///< X coordinate
    double y;  ///< Y coordinate
    double z;  ///< Z coordinate
};

/**
 * @brief 2D vector with float precision
 * Used for planar movements and directions
 */
struct Vector2D {
    float x;  ///< X coordinate
    float y;  ///< Y coordinate
};

/**
 * @brief Represents orientation in 3D space
 * Uses Euler angles in degrees
 */
struct Rotator {
    float yaw;    ///< Rotation around Z axis
    float pitch;  ///< Rotation around Y axis
    float roll;   ///< Rotation around X axis
};

/**
 * @brief Complete transform in 3D space
 * Combines position, rotation, and scale
 */
struct Transform {
    Vector pos;    ///< Position in 3D space
    Rotator rot;   ///< Orientation
    Vector scl;    ///< Scale factors
};

} // namespace datatypes

#endif // __DATA_TYPES__
