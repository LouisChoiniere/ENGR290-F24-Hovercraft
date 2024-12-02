#pragma once

// ----- Settings -----
#define ENABLE_BATTERY_CHECK 1
#define ENABLE_FANS 1
#define ENABLE_SERVO 1
#define ENABLE_DEBUG 0

#define LIFT_FAN_SPEED 90
#define LIFT_FAN_SPEED_SLOW 60
#define THRUST_FAN_SPEED_HIGH 85
#define THRUST_FAN_SPEED 65

// ----- Control system -----

// Start turn
#define TURNING_DISTANCE_THRESHOLD_CM 40

// Drop
#define ANGLE_THRESHOLD_END_TURN 15
#define MIN_TURNING_TIME_MS 750

// Resume
#define DROP_TIME_MS 1500

// Servo
#define SERVO_P_GAIN 0.85
#define SERVO_EXP_AMP 20
#define SERVO_EXP_WDT 500
#define SERVO_EXP_CNT 85

// ----- Pinout selections -----
#define IR_LEFT 0           // P5
#define IR_RIGHT 1          // P8
#define LIFT_FAN_PORT 'A'   // P4
#define THRUST_FAN_PORT 'B' // P3