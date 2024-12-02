#pragma once

// ----- Settings -----
#define ENABLE_BATTERY_CHECK 1
#define ENABLE_FANS 1
#define ENABLE_SERVO 1
#define ENABLE_DEBUG 0

#define LIFT_FAN_SPEED 97
#define LIFT_FAN_SPEED_LOW 80

#define THRUST_FAN_SPEED_HIGH 100
#define THRUST_FAN_SPEED 85


// ----- Control system -----

// Start turn
#define TURNING_DISTANCE_THRESHOLD_CM 45

// Drop
#define ANGLE_THRESHOLD_END_TURN 40
#define MIN_TURNING_TIME_MS 20

// Resume
#define DROP_TIME_MS 500

// Forward
#define FORWARD_DISTANCE_THRESHOLD_1_CM 25
#define MIN_FORWARD_TIME_2_MS 1000

// Servo
#define SERVO_P_GAIN 0.9
#define SERVO_EXP_AMP 15
#define SERVO_EXP_WDT 500
#define SERVO_EXP_CNT 85

// ----- Pinout selections -----
#define IR_LEFT 0           // P5
#define IR_RIGHT 1          // P8
#define LIFT_FAN_PORT 'A'   // P4
#define THRUST_FAN_PORT 'B' // P3