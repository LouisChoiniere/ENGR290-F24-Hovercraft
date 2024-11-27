#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "actuators/fan.h"
#include "actuators/servo.h"
#include "sensors/GP2Y0A21YK.h"
#include "sensors/HCSR04.h"
#include "sensors/MPU6050.h"
#include "util/ADC.h"
#include "util/BatteryVoltage.h"
#include "util/timer.h"

// Settings
#define ENABLE_BATTERY_CHECK 1

#define LIFT_FAN_SPEED 87
#define THRUST_FAN_SPEED 80

// Pinout selections
#define IR_LEFT 0           // P5
#define IR_RIGHT 1          // P8
#define LIFT_FAN_PORT 'A'   // P4
#define THRUST_FAN_PORT 'B' // P3


volatile uint8_t INT0_EDGE = 1;

volatile struct {
  uint8_t sample : 1;
  uint8_t stop : 1;
  uint8_t turning : 1;
} flags;

volatile uint32_t time_ms;
volatile uint32_t delay1_ms;

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
  MPU6050_Calibrate(&IMU, 1000);

  Serial.println("Start prechecks");
  Serial.flush();

  // Batteries are almost empty do not start execution
  if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() <= 1) {
    Serial.println("Batteries are empty");
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

  // Start thrust fan
  FAN_setSpeed(THRUST_FAN_PORT, THRUST_FAN_SPEED);
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

void changeYawRef(float value) {
  yawRef += value;

  if (yawRef > 360) {
    yawRef -= 360;
  } else if (yawRef < 0) {
    yawRef += 360;
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

    // Every 20ms read the IMU and IR sensors
    MPU6050_ALL(&IMU, TIMER1_INTERVAL_MS);
    float dist_left = GP2Y0A21YK_GetDistance(IR_LEFT);
    float dist_right = GP2Y0A21YK_GetDistance(IR_RIGHT);
    float dist_front = HCSR04_GetDistance();

    float dist_side = dist_left - dist_right; // Positive is right from center, negative is left from center

    // When turning, check if the front distance is less than 10cm and turn 90 degrees to the side with more distance
    if (flags.turning && dist_front < 10) {
     if (dist_side > 0) {
       changeYawRef(90);
     } else {
       changeYawRef(-90);
     }
     flags.turning = 1;
    }

    // When the side distance is more than 20cm turn 90 degrees to the side with more distance
    if (!flags.turning && abs(dist_side) > 20) {
      if (dist_side > 0) {
        changeYawRef(-90);
      } else {
        changeYawRef(90);
      }
      flags.turning = 1;
    }

    // Offset the yaw angle to the reference angle
    // Positive is right, negative is left
    float delta_yaw = IMU.yaw - yawRef;

    // P controller for the servo
    float servo_angle = delta_yaw * 0.8;
    // SERVO_setPosition((uint8_t)(-servo_angle));

    SERVO_setPosition((uint8_t)IMU.yaw);

    // every 40ms trigger the HCSR04
    if (time_ms % 40 == 0) {
      HCSR04_Trigger();
    }

    // Every 5s check battery voltage
    if (time_ms % 5000 == 0) {
      // Batteries are empty stop execution immediately and turn of fans
      if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() == 0) {
        FAN_setSpeed(LIFT_FAN_PORT, 0);
        FAN_setSpeed(THRUST_FAN_PORT, 0);
        Serial.println("Batteries are empty");
        flags.stop = 1;
      }
    }

    // Every 1s print debug information
    if (time_ms % 1000 == 0) {
      Serial.print("Distance: ");
      Serial.print(dist_front);
      Serial.print(",\tLeft distance: ");
      Serial.print(dist_left);
      Serial.print(",\tRight distance: ");
      Serial.print(dist_right);

      Serial.print(",\tYaw: ");
      Serial.print(IMU.yaw);

      Serial.println();
    }
  }
}
