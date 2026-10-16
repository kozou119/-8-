#ifndef ROBOT_HARDWARE_CONFIG_H
#define ROBOT_HARDWARE_CONFIG_H
// Confirmed board: Arduino UNO R4 Minima, arduino:renesas_uno:minima.
// Select the actual driver only after verifying its wiring and power supply.
namespace hardware {
enum MotorProfile { UNCONFIGURED, ZUMO_SHIELD_PWM_DIR };
constexpr MotorProfile MOTOR_PROFILE = ZUMO_SHIELD_PWM_DIR;
// Zumo Shield documented connections: left PWM D10 / DIR D8, right PWM D9 / DIR D7.
constexpr int LEFT_PWM = MOTOR_PROFILE == ZUMO_SHIELD_PWM_DIR ? 10 : -1;
constexpr int LEFT_DIR = MOTOR_PROFILE == ZUMO_SHIELD_PWM_DIR ? 8 : -1;
constexpr int RIGHT_PWM = MOTOR_PROFILE == ZUMO_SHIELD_PWM_DIR ? 9 : -1;
constexpr int RIGHT_DIR = MOTOR_PROFILE == ZUMO_SHIELD_PWM_DIR ? 7 : -1;
constexpr int BUTTON_PIN = MOTOR_PROFILE == ZUMO_SHIELD_PWM_DIR ? 12 : -1;
constexpr bool FLIP_LEFT = false;
constexpr bool FLIP_RIGHT = false;
// Optional trigger/echo pulse sensor profile. Do not enable without its datasheet.
constexpr bool TRIGGER_ECHO_CONFIRMED = true;
constexpr int ULTRASONIC_TRIGGER = 2;
constexpr int ULTRASONIC_ECHO = 4;
// Contact switch wired to GND with INPUT_PULLUP, ONLY when actually installed.
constexpr int CUP_SWITCH_PIN = -1;
// Experiment 1/2 describes a passive arm, not an actuated gripper or contact switch.
// Measure the echo distance to a cup seated IN THE ARM; these are provisional bounds.
constexpr int CAPTURE_MIN_MM = 35;
constexpr int CAPTURE_MAX_MM = 75;
// Experiment 4: LSM303, X forward. Verify right-turn sign on the actual robot.
constexpr float COMPASS_CLOCKWISE_SIGN = 1.0f;
constexpr float COMPASS_OFFSET_DEG = 0.0f;
}
#endif
