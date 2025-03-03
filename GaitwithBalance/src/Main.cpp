/*
  Quadruped robot arduino sketch with self-balancing capability.
  Based on https://github.com/maestrakos/quad by Alexandros Petkos
*/

#include <Arduino.h>
#include "Config.hpp"
#include "Buzzer.hpp"
#include "Hardware.hpp"
#include "Kinematics.hpp"
#include <Adafruit_MPU6050.h>

// Controller includes based on configuration
#ifdef __PS4_GAMEPAD__
#include <PS4Controller.h>
#endif

#ifdef __PS2_GAMEPAD__
#include "PS2X_lib.h"
PS2X ps2x;
const bool RUMBLE = true;
const bool PRESSURES = false;
int gamepad_error = 0;
byte gamepad_type = 0;
byte gamepad_vibrate = 0;
#endif

#ifdef __GOBLE__
#include "GoBLE.hpp"
#endif

// Core components
Buzzer buzzer(BUZZER_PIN);
Hardware hardware(PCA9685_I2C_ADDR);
Kinematics kinematics(hardware);

// Movement control variables
datatypes::Vector2D movement_direction = {0, 0};
float turn = 0;   // Rotation direction
float height = 0; // Leg extension
int state = 3;    // Gait type: 0=idle, 1=trot, 2=yaw, 3=pitch-roll, 4=object-detection
float step_period = 10.0; // Steps per second

// Stored offsets
float vertical_offset = kinematics.vrt_offset;
float horizontal_offset = kinematics.hrz_offset;

// Joystick inputs
int8_t joystickLX = 0;
int8_t joystickLY = 0;
int8_t joystickRX = 0;
int8_t joystickRY = 0;

// Input processing parameters
const float STICK_DEADZONE = 6.0f;
float lx = 0, ly = 0, rx = 0, ry = 0;

// Self-balancing variables
Adafruit_MPU6050 mpu;
const float COMPLEMENTARY_FILTER_ALPHA = 0.9;
float complementary_pitch = 0;
float complementary_roll = 0;
unsigned long prev_time = 0;

// PID control variables
float pitch_error_sum = 0, roll_error_sum = 0;
float pitch_prev_error = 0, roll_prev_error = 0;
const float Kp_pitch = 0;
const float Ki_pitch = 0.3;
const float Kd_pitch = 0;
const float Kp_roll = 0;
const float Ki_roll = 0.3;
const float Kd_roll = 0;
const float INTEGRAL_LIMIT = 20;

unsigned long last_update = 0;
unsigned long duration = 0;

#ifdef __PS2_GAMEPAD__
void init_ps2_controller() {
  for (int attempt = 0; attempt < 3; attempt++) {
    gamepad_error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_ATT, PS2_DAT, true, true);
    if (gamepad_error == 0) {
      Console.println("PS2 Controller configured successfully");
      break;
    }
    delay(1000);
  }

  if (gamepad_error == 1) {
    Console.println("No PS2 controller found");
    aborted();
  } else if (gamepad_error == 2) {
    Console.println("PS2 Controller found but not accepting commands");
    aborted();
  }

  gamepad_type = ps2x.readType();
  const char* controller_types[] = {
    "Unknown Controller",
    "DualShock Controller",
    "GuitarHero Controller",
    "Wireless Sony DualShock Controller"
  };
  Console.println(controller_types[min(gamepad_type, 3)]);
  
  gamepad_vibrate = 0;
}
#endif

void init_controller() {
#ifdef __PS2_GAMEPAD__
  init_ps2_controller();
  Console.println("Using PS2 Controller");
#elif defined(__GOBLE__)
  Goble.begin(GOBLE_BAUD_RATE);
  Console.println("Using BlueTooth");
#elif defined(__PS4_GAMEPAD__)
  PS4.begin((char*)PS4_MAC_ADDR);
  Console.println("Using PS4 Controller");
#else
  Console.println("No Controller configured");
#endif
}

void aborted() {
  Console.println("Program aborted!");
  buzzer.beepError();
  while (1) ; // Infinite loop
}

void process_input() {
  // Interpolate joystick values for smooth movement
  lx = Kinematics::inter(lx, joystickLX / 4.0f, 0.5f);
  ly = Kinematics::inter(ly, joystickLY / 4.0f, 0.5f);
  rx = Kinematics::inter(rx, joystickRX / 4.0f, 0.5f);
  ry = Kinematics::inter(ry, joystickRY / 4.0f, 0.5f);

  // Process X-axis movement
  if (abs(lx) > STICK_DEADZONE) {
    float x_movement = lx - STICK_DEADZONE * Kinematics::sign(lx);
    movement_direction.y = (state == 1) ? 0 : x_movement / 2;
  } else {
    movement_direction.y = 0;
  }

  // Process Y-axis movement
  if (abs(ly) > STICK_DEADZONE) {
    float y_movement = ly - STICK_DEADZONE * Kinematics::sign(ly);
    if (state == 1) {
      movement_direction.x = y_movement / 10.0f;
      kinematics.vrt_offset = Kinematics::inter(
        kinematics.vrt_offset,
        vertical_offset + (y_movement > 0 ? -6.0f : 3.0f),
        2.0f
      );
    } else if (state != 4) {
      movement_direction.x = y_movement / 2;
      kinematics.vrt_offset = vertical_offset;
    }
  } else {
    movement_direction.x = 0;
    kinematics.vrt_offset = vertical_offset;
  }

  // Process rotation
  if (abs(rx) > STICK_DEADZONE) {
    float rotation = rx - STICK_DEADZONE * Kinematics::sign(rx);
    turn = (state == 1) ? rotation / 16.0f : (state != 4 ? rotation : 0);
  } else {
    turn = 0;
  }

  // Process height adjustment
  if (abs(ry) > STICK_DEADZONE) {
    height = ry - STICK_DEADZONE * Kinematics::sign(ry);
  } else {
    height = 0;
  }
}

#ifdef __DEBUG__
void process_serial_command(float val1, float val2, float val3) {
  const int CALIBRATION_MODE = 0;
  const int BALANCE_MODE = 1;
  
  if (CALIBRATION_MODE == 0) {
    int leg = val1;
    int joint = val2;
    int servo = val3;
    Console.printf("Setting leg %d joint %d to %d\n", leg, joint, servo);
    hardware.set_servo(leg, joint, servo);
  } else if (BALANCE_MODE == 1) {
    int leg = val1;
    int empty = val2;
    int amount = val3;
    Console.printf("Adjusting leg %d balance by %d\n", leg, amount);
    kinematics.base_offset[leg] = amount;
  }
}

void handle_serial_input() {
  const int CALIBRATION_MODE = 0;
  float values[3] = {0, 0, 0};
  int value_index = 0;
  String buffer;

  while (Console.available()) {
    char c = Console.read();
    if (c == 13 || c == 32 || c == '\n') {
      values[value_index++] = buffer.toFloat();
      buffer = "";
    } else {
      buffer += c;
    }
  }

  if (CALIBRATION_MODE == 0) {
    process_serial_command(values[0], values[1], values[2]);
  } else if (state == 4) {
    movement_direction = {values[0], values[1]};
    turn = values[2];
  }
}
#endif

void setup() {
#ifdef __DEBUG__
  Console.begin(115200);
  Console.println("Debug mode enabled");
#endif

  hardware.init_hardware();
  init_controller();

  // Servo calibration mode
  pinMode(SERVO_CAL_PIN, INPUT_PULLDOWN);
  while (digitalRead(SERVO_CAL_PIN)) {
    delay(1000);
  }

  buzzer.beepShort();
  Console.println("Initialization complete");
}

void update_balance() {
  sensors_event_t accel, gyro, temp;
  mpu.getEvent(&accel, &gyro, &temp);

  // Calculate angles from accelerometer data
  float acc_pitch = atan2(accel.acceleration.x, 
    sqrt(sq(accel.acceleration.y) + sq(accel.acceleration.z))) * RAD_TO_DEG;
  float acc_roll = atan2(accel.acceleration.y,
    sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.z))) * RAD_TO_DEG;

  // Update complementary filter
  unsigned long current_time = millis();
  float dt = (current_time - prev_time) / 1000.0f;
  prev_time = current_time;

  complementary_pitch = COMPLEMENTARY_FILTER_ALPHA * (complementary_pitch + gyro.gyro.x * dt) +
                       (1 - COMPLEMENTARY_FILTER_ALPHA) * acc_pitch;
  complementary_roll = COMPLEMENTARY_FILTER_ALPHA * (complementary_roll + gyro.gyro.y * dt) +
                      (1 - COMPLEMENTARY_FILTER_ALPHA) * acc_roll;

  // Calculate PID terms
  float pitch_error = -complementary_pitch;
  float roll_error = -complementary_roll;

  pitch_error_sum = constrain(pitch_error_sum + pitch_error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  roll_error_sum = constrain(roll_error_sum + roll_error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);

  float pitch_derivative = (pitch_error - pitch_prev_error) / dt;
  float roll_derivative = (roll_error - roll_prev_error) / dt;

  pitch_prev_error = pitch_error;
  roll_prev_error = roll_error;

  // Calculate final outputs
  float pitch_output = Kp_pitch * pitch_error + Ki_pitch * pitch_error_sum + Kd_pitch * pitch_derivative;
  float roll_output = Kp_roll * roll_error + Ki_roll * roll_error_sum + Kd_roll * roll_derivative;

  // Convert PID outputs to joystick values
  joystickLY = constrain(int(-pitch_output/19*128), -128, 127);
  joystickLX = constrain(int(-roll_output/22*128), -128, 127);

  // Debug output
  Serial.printf("Roll: %.2f, Pitch: %.2f, Outputs: %.2f, %.2f\n",
    complementary_roll, complementary_pitch, roll_output, pitch_output);
}

void handle_test_mode() {
  static bool test_mode_active = false;
  static long last_check = 0;

  if ((duration - last_check) > 1000) {
    last_check = duration;
    if (digitalRead(SERVO_CAL_PIN)) {
      hardware.attach();
      joystickLY = 127;
      joystickRX = 127;
      state = 1;
      test_mode_active = true;
      return;
    } else if (test_mode_active) {
      joystickLX = joystickLY = joystickRX = joystickRY = 0;
      test_mode_active = false;
    }
  }
}

void handle_bluetooth() {
#ifdef __GOBLE__
  static long last_activity = 0;
  const long TIMEOUT = 60000; // 1 minute timeout

  if ((duration - last_activity) > TIMEOUT) {
    last_activity = duration;
    hardware.detach();
    joystickLX = joystickLY = joystickRX = joystickRY = 0;
    buzzer.beepShort();
    Console.println("Power saving mode activated");
    return;
  }

  if (Goble.available()) {
    last_activity = duration;
    hardware.attach();
    
    switch (state) {
      case 1:
        joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
        break;
      case 0:
      case 2:
        joystickRY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickRX = map(Goble.readJoystickX(), 255, 0, 127, -128);
        break;
      case 3:
        joystickLY = map(Goble.readJoystickY(), 255, 0, 127, -128);
        joystickLX = map(Goble.readJoystickX(), 0, 255, 127, -128);
        break;
      default:
        joystickLX = joystickLY = joystickRX = joystickRY = 0;
    }

    // Handle button presses
    if (Goble.readSwitchUp() == PRESSED) {
      state = 0;
      buzzer.beepShort();
    } else if (Goble.readSwitchDown() == PRESSED) {
      state = 2;
      buzzer.beepShort();
    } else if (Goble.readSwitchLeft() == PRESSED) {
      state = 3;
      buzzer.beepShort();
    } else if (Goble.readSwitchRight() == PRESSED) {
      state = 1;
      buzzer.beepShort();
    } else if (Goble.readSwitchStart() == PRESSED) {
      hardware.detach();
      joystickLX = joystickLY = joystickRX = joystickRY = 0;
      last_activity = duration;
      buzzer.beepShort();
    }
  }
#endif
}

void handle_gamepad() {
#if defined(__PS4_GAMEPAD__)
  static long last_check = 0;
  if ((duration - last_check) > 30) {
    last_check = duration;
    if (PS4.isConnected()) {
      joystickLX = PS4.data.analog.stick.lx;
      joystickLY = PS4.data.analog.stick.ly;
      joystickRX = PS4.data.analog.stick.rx;
      joystickRY = PS4.data.analog.stick.ry;

      if (PS4.data.button.up) {
        state = 0;
        buzzer.beepShort();
      } else if (PS4.data.button.down) {
        state = 2;
        buzzer.beepShort();
      } else if (PS4.data.button.left) {
        state = 3;
        buzzer.beepShort();
      } else if (PS4.data.button.right) {
        state = 1;
        buzzer.beepShort();
      } else if (PS4.data.button.triangle) {
        hardware.detach();
        buzzer.beepShort();
      } else if (PS4.data.button.cross) {
        hardware.attach();
        buzzer.beepShort();
      }
    }
  }
#elif defined(__PS2_GAMEPAD__)
  static long last_check = 0;
  if ((duration - last_check) > 30) {
    last_check = duration;
    ps2x.read_gamepad(false, gamepad_vibrate);
    
    joystickLX = map(ps2x.Analog(PSS_LX), 0, 255, 127, -128);
    joystickLY = map(ps2x.Analog(PSS_LY), 0, 255, 127, -128);
    joystickRX = map(ps2x.Analog(PSS_RX), 255, 0, 127, -128);
    joystickRY = map(ps2x.Analog(PSS_RY), 255, 0, 127, -128);

    if (ps2x.ButtonPressed(PSB_PAD_DOWN)) {
      state = 2;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_PAD_LEFT)) {
      state = 3;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_PAD_UP)) {
      state = 0;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_PAD_RIGHT)) {
      state = 1;
      buzzer.beepShort();
    } else if (ps2x.ButtonPressed(PSB_CIRCLE)) {
      state = 4;
      buzzer.beepShort();
    }
  }
#endif
}

void loop() {
  duration = millis();

  // Update balance control
  update_balance();

  // Update hardware and kinematics
  hardware.handle_hardware();
  kinematics.handle_kinematics(state, movement_direction, turn, height, step_period);

  // Handle test mode
  handle_test_mode();

  // Handle controller input
  handle_bluetooth();
  handle_gamepad();

  // Process inputs
  process_input();

#ifdef __DEBUG__
  if (Console.available()) {
    handle_serial_input();
  }
#endif
}
