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
} flags;

volatile uint32_t time_ms = 0;
uint32_t time_ms_turnning_start = 0;
uint32_t time_ms_drop_start = 0;

uint8_t consecutive_turns = 0;
uint8_t time_ms_last_turn = 0;

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

    if (time_ms - time_ms_last_turn > 1000) {
      consecutive_turns = 0;
    }

    // ----- End of circuit -----
    // ~ BETA ~
    // Detect when the course is finished and stop execution
    // if (dist_left > 30 && dist_right > 30) {
    //   flags.stop = 1;
    // }

    // ----- Turning logic -----
    // When the side distance is more than 20cm turn 90 degrees to the side with more distance
    // start turning
    // change the reference angle and set the turning flag
    float dist_side = dist_left - dist_right; // Positive is right from center, negative is left from center

    // Start turn
    if (!flags.turning && !flags.dropped && abs(dist_side) > TURNING_DISTANCE_THRESHOLD_CM && consecutive_turns < 2) {
      if (dist_side > 0) {
        yawRef += 90;
      } else {
        yawRef += -90;
      }
      flags.turning = 1;
      time_ms_turnning_start = time_ms;

      if (time_ms - time_ms_last_turn < 1000) {
        consecutive_turns++;
      }
    }

    // Drop
    if (flags.turning && !flags.dropped && time_ms - time_ms_turnning_start > MIN_TURNING_TIME_MS && abs(delta_yaw) < ANGLE_THRESHOLD_END_TURN) {
      flags.turning = 0;
      flags.dropped = 1;
      time_ms_drop_start = time_ms;
    }

    // Resume
    if (flags.dropped && time_ms - time_ms_drop_start > DROP_TIME_MS) {
      flags.dropped = 0;
      time_ms_last_turn = time_ms;
    }

    // ----- Servo angle control -----
    // If the craft is not turning adjust angle to the side with more distance
    if (!flags.turning) {
      delta_yaw -= dist_side * 1;
    }

    // P controller for the servo
    float servo_angle = delta_yaw * SERVO_P_GAIN + SERVO_EXP_AMP * exp(-pow(delta_yaw - SERVO_EXP_CNT, 2) / SERVO_EXP_WDT);
    int8_t servo_angle_int = round(servo_angle);
    SERVO_setPosition(servo_angle_int);

    // ----- Fan speed control -----
    // If the front distance is more than 100cm set the thrust fan to high speed
    // Goal start from standstill
    if (dist_front > 100 || servo_angle > 25) {
      FAN_setSpeed(THRUST_FAN_PORT, THRUST_FAN_SPEED_HIGH);
    } else {
      FAN_setSpeed(THRUST_FAN_PORT, THRUST_FAN_SPEED);
    }

    if (flags.dropped) {
      FAN_setSpeed(LIFT_FAN_PORT, LIFT_FAN_SPEED_SLOW);
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
