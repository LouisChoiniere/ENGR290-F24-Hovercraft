#pragma once

// ----- Settings -----
#define ENABLE_BATTERY_CHECK 1
#define ENABLE_FANS 1
#define ENABLE_SERVO 1

#define LIFT_FAN_SPEED 92
#define LIFT_FAN_SPEED_SLOW 80
#define THRUST_FAN_SPEED_HIGH 85
#define THRUST_FAN_SPEED 65

// ----- Control system -----

#define TURNING_DISTANCE_THRESHOLD_CM 40
#define ANGLE_THRESHOLD_END_TURN 10
#define TURNING_TIME_MS 1700

#define SERVO_P_GAIN 0.9
#define SERVO_EXP_AMP 15
#define SERVO_EXP_WDT 550
#define SERVO_EXP_CNT 85

// ----- Pinout selections -----
#define IR_LEFT 0           // P5
#define IR_RIGHT 1          // P8
#define LIFT_FAN_PORT 'A'   // P4
#define THRUST_FAN_PORT 'B' // P3