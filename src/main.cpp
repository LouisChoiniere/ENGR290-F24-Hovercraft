#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "actuators/fan.h"
#include "actuators/servo.h"
#include "config.h"
#include "sensors/GP2Y0A21YK.h"
#include "sensors/HCSR04.h"
#include "sensors/MPU6050.h"
#include "util/ADC.h"
#include "util/BatteryVoltage.h"
#include "util/timer.h"

// ----- Global variables -----
volatile uint8_t INT0_EDGE = 1;

volatile struct {
  uint8_t sample : 1;
  uint8_t stop : 1;
  uint8_t turning : 1;
  uint8_t dropped : 1;
  uint8_t creep : 1;
} flags;

volatile uint32_t time_ms = 0;
uint32_t time_ms_turnning_start = 0;
uint32_t time_ms_drop_start = 0;
uint32_t time_ms_forward_start = 0;

uint8_t number_of_turns = 0;

MPU6050_Struct IMU;

float yawRef = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial)
    _delay_ms(10);

  Serial.println("Setup");
  Serial.flush();

  // Setup timers
  timer0_setup();
  timer1_setup();

  // ADC and set channels for GP2Y0A21YK
  ADC_setup();
  ADC_setup_channel(BATTERY_VOLTAGE_ADC_CHANNEL);
  ADC_setup_channel(IR_LEFT);

  HCSR04_Setup();

  // I2C setup and setup MPU6050
  Wire.begin();
  MPU6050_WakeUpMPU();
  MPU6050_SetAccelerationSensitivity(&IMU, MPU6050_ACCELEROMETER_RANGE_4G);
  MPU6050_SetGyroSensitivity(&IMU, MPU6050_GYROSCOPE_RANGE_1000);
  MPU6050_SetFilter(MPU6050_FILTER_BW_20);

  Serial.println("Calibration of MPU6050");
  Serial.flush();

  MPU6050_Calibrate(&IMU, 1000);

  Serial.println("Start prechecks");
  Serial.flush();

  // Batteries are almost empty do not start execution
  if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() <= 1) {
    Serial.println("Batteries are too low to start run");
    Serial.flush();
    flags.stop = 1;
    while (true)
      ;
  }

  // Check if valid config
  if (!ENABLE_BATTERY_CHECK && ENABLE_FANS) {
    Serial.println("Invalid configuration");
    Serial.flush();
    flags.stop = 1;
  }

  if (flags.stop) {
    Serial.println("Failled prechecks");
    Serial.flush();
    while (true)
      ;
  }

  Serial.println("Startup procedure");
  Serial.flush();

  // Start lift fan
  FAN_setSpeed(LIFT_FAN_PORT, LIFT_FAN_SPEED);
  _delay_ms(100);

  Serial.println("Start of control loop");
  Serial.flush();

  // Setup Interrupts
  EICRA |= (1 << ISC00); // Rising edge
  EIMSK |= (1 << INT0);  // Enable INT0
  sei();                 // Enable global interrupts
}

ISR(TIMER1_CAPT_vect) { // 50Hz, 20ms
  time_ms += 20;
  flags.sample = 1;
}

ISR(INT0_vect) {
  HCSR04_HandleInterupt(INT0_EDGE);

  // Toggle edge
  INT0_EDGE = !INT0_EDGE;
  if (INT0_EDGE) {
    EICRA |= (1 << ISC00); // Rising edge
  } else {
    EICRA &= ~(1 << ISC00); // Falling edge
  }
}

void loop() {
  // ----- Stop execution -----
  if (flags.stop) {
    FAN_setSpeed(LIFT_FAN_PORT, 0);
    FAN_setSpeed(THRUST_FAN_PORT, 0);
    Serial.println("Stop execution");
    while (true)
      ;
  }

  // ----- Logic -----
  if (flags.sample) {
    flags.sample = 0;

    // ----- Aquire data -----
    MPU6050_ALL(&IMU, TIMER1_INTERVAL_MS);
    float dist_left = GP2Y0A21YK_GetDistance(IR_LEFT);
    float dist_right = GP2Y0A21YK_GetDistance(IR_RIGHT);
    float dist_front = HCSR04_GetDistance();

    float delta_yaw = IMU.yaw - yawRef; // Positive is right, negative is left

    // ----- Turning logic -----
    float dist_side = dist_left - dist_right; // Positive is right from center, negative is left from center

    // Start turn
    if (!flags.turning && !flags.dropped && !flags.creep && abs(dist_side) > TURNING_DISTANCE_THRESHOLD_CM && dist_front < TRUNING_FRONT_DISTANCE_THRESHOLD_CM) {
      if (dist_side > 0) {
        yawRef += 90;
      } else {
        yawRef += -90;
      }

      flags.turning = 1;
      time_ms_turnning_start = time_ms;
    }

    // End turn
    if (flags.turning && time_ms - time_ms_turnning_start > MIN_TURNING_TIME_MS && abs(delta_yaw) < ANGLE_THRESHOLD_END_TURN) {
      flags.turning = 0;
      number_of_turns++;

      flags.dropped = 1;
      time_ms_drop_start = time_ms;
    }

    // Stop drop condition
    if (flags.dropped && time_ms - time_ms_drop_start > DROP_TIME_MS) {

      flags.dropped = 0;
      if (dist_front > FORWARD_DISTANCE_THRESHOLD_1_CM) {
        flags.creep = 1;
        time_ms_forward_start = time_ms;
      }
    }

    // Stop creep condition
    if (flags.creep) {

      // After second part of turn
      if (number_of_turns % 2 == 0) {
        if (time_ms - time_ms_forward_start > MIN_FORWARD_TIME_2_MS) {
          flags.creep = 0;
        }
      }

      // After first part of turn
      else if (number_of_turns % 2 == 1) {
        if (dist_front < FORWARD_DISTANCE_THRESHOLD_1_CM) {
          flags.creep = 0;
        }
      }
    }

    // ----- Servo angle control -----
    // If the craft is not turning adjust angle to the side with more distance
    if (!flags.turning && !flags.dropped && !flags.creep) {
      delta_yaw -= dist_side * 0.5f;
    }

    // Controller for the servo
    float servo_p_gain = SERVO_P_GAIN;
    if (delta_yaw > 0) {
      servo_p_gain -= 0.5;
    } else {
      servo_p_gain += 1.2;
    }

    float servo_angle = delta_yaw * servo_p_gain + SERVO_EXP_AMP * exp(-pow(delta_yaw - SERVO_EXP_CNT, 2) / SERVO_EXP_WDT);
    int8_t servo_angle_int = round(servo_angle);
    SERVO_setPosition(servo_angle_int);

    // ----- Fan speed control -----
    FAN_setSpeed(THRUST_FAN_PORT, THRUST_FAN_SPEED);

    if (flags.dropped) {
      FAN_setSpeed(LIFT_FAN_PORT, 0);
    } else if (flags.turning) {
      FAN_setSpeed(LIFT_FAN_PORT, LIFT_FAN_SPEED_LOW);
    } else if (flags.creep) {
      FAN_setSpeed(LIFT_FAN_PORT, LIFT_FAN_SPEED_LOW);
      FAN_setSpeed(THRUST_FAN_PORT, THRUST_FAN_SPEED_HIGH);
    } else {
      FAN_setSpeed(LIFT_FAN_PORT, LIFT_FAN_SPEED);
    }

    // every 40ms trigger the HCSR04
    if (time_ms % 40 == 0) {
      HCSR04_Trigger();
    }

    // Every 5s check battery voltage
    if (time_ms % 5000 == 0) {
      // Batteries are empty stop execution immediately and turn of fans
      if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() == 0) {
        Serial.println("Batteries are empty");
        flags.stop = 1;
      }
    }

    // Every 1s print debug information
    if (time_ms % 1000 == 0 && ENABLE_DEBUG) {
      Serial.print("Distance: ");
      Serial.print(dist_front);
      Serial.print(",\tLeft distance: ");
      Serial.print(dist_left);
      Serial.print(",\tRight distance: ");
      Serial.print(dist_right);

      Serial.print(",\tYaw: ");
      Serial.print(IMU.yaw);

      Serial.print(",\tYaw ref: ");
      Serial.print(yawRef);

      Serial.print(",\tServo angle: ");
      Serial.print(servo_angle_int);

      Serial.print(",\tTurning: ");
      Serial.print(flags.turning);

      Serial.println();
    }
  }
}
