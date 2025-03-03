# 3D-printed Robot Dog with Self-Balancing Capability

A quadruped robot project that implements walking gait and self-balancing algorithms to enable stable movement on uneven terrain. This project combines 3D-printed hardware with sophisticated control software running on an ESP32 microcontroller.

## Hardware Components

| Component | Specification | Purpose |
|-----------|--------------|----------|
| Microcontroller | ESP32 Development Board E1 | Main control unit |
| IMU Sensor | MPU-9250 | Motion sensing and balance detection |
| Actuators | Servo Motors | Leg joint movement |
| Structure | 3D Printed Parts | Robot body and leg segments |
| Optional Controllers | PS4/PS2/Bluetooth | Remote control |

## Movement States and Controls

| State | Mode | Description | Primary Controls |
|-------|------|-------------|------------------|
| 0 | Idle | Static standing position | Height adjustment |
| 1 | Trot Gait | Dynamic walking motion | Forward/backward, turning |
| 2 | Yaw Control | Rotation control | Directional rotation |
| 3 | Pitch-Roll Balance | Self-balancing mode | Tilt compensation |
| 4 | Object Detection | Sensor-based movement | Autonomous control |

## Controller Mapping

| Control Input | PS4/PS2 | Bluetooth | Function |
|--------------|----------|-----------|-----------|
| Left Stick X | ✓ | ✓ | Lateral movement |
| Left Stick Y | ✓ | ✓ | Forward/backward |
| Right Stick X | ✓ | ✓ | Rotation |
| Right Stick Y | ✓ | ✓ | Height adjustment |
| D-Pad | ✓ | ✓ | State selection |
| Triangle/Circle | ✓ | - | Servo attach/detach |

## Technical Parameters

### Kinematics Configuration

| Parameter | Value | Description |
|-----------|-------|-------------|
| Leg Length | 105mm | Length of each leg segment |
| Vertical Offset | -16.50mm | Default vertical position |
| Horizontal Offset | -6.00mm | Default horizontal position |
| Step Extent | 40x40x26mm | Maximum step size (X,Y,Z) |
| Loop Frequency | 700Hz | Main control loop rate |
| Movement Precision | 0.001mm | Minimum movement increment |

### Balance Control Parameters

| Parameter | Pitch | Roll | Description |
|-----------|-------|------|-------------|
| Kp | 0 | 0 | Proportional gain |
| Ki | 0.3 | 0.3 | Integral gain |
| Kd | 0 | 0 | Derivative gain |
| Filter Coefficient | 0.9 | 0.9 | Complementary filter |
| Integral Limit | ±20° | ±20° | Maximum integral term |

## Setup Instructions

1. Install required libraries:
   - Adafruit MPU6050
   - PS4Controller (if using PS4)
   - PCA9685 Servo Driver

2. Configure controller:
   - Set controller type in Config.hpp
   - Configure controller MAC address/pins
   - Upload code to ESP32

3. Calibration:
   - Use SERVO_CAL_PIN for servo calibration mode
   - Adjust base offsets for each leg
   - Fine-tune PID parameters if needed

## Code Structure

| File | Purpose |
|------|---------|
| Main.cpp | Core program logic and control loop |
| Kinematics.hpp/cpp | Inverse kinematics and gait calculations |
| Hardware.hpp | Hardware abstraction layer |
| Config.hpp | System configuration |
| Controller Libraries | PS4/PS2/GoBLE support |
