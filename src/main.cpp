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

#define LIFT_FAN_SPEED 90
#define THRUST_FAN_SPEED 50

// Pinout selections
#define IR_LEFT 0           // P5
#define IR_RIGHT 1          // P8
#define LIFT_FAN_PORT 'A'   // P4
#define THRUST_FAN_PORT 'B' // P3

MPU6050_Struct IMU;

volatile uint8_t INT0_EDGE = 1;

volatile struct {
  uint64_t time_ms;
  uint8_t sample : 1;
  uint8_t stop : 1;
} flags;

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
}

ISR(TIMER1_CAPT_vect) { // 50Hz, 20ms
  flags.time_ms += 20;
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

  // ----- Data collection -----
  if (flags.sample) {
    flags.sample = 0;

    // Every 20ms read the IMU and IR sensors
    MPU6050_ALL(&IMU, TIMER1_INTERVAL_MS);
    float dist_left = GP2Y0A21YK_GetDistance(IR_LEFT);
    float dist_right = GP2Y0A21YK_GetDistance(IR_RIGHT);
    float dist_front = HCSR04_GetDistance();

    // every 40ms trigger the HCSR04
    if (flags.time_ms % 1000 == 0) {
      HCSR04_Trigger();
    }

    // Every 5s check battery voltage
    if (flags.time_ms % 5000 == 0) {
      // Batteries are empty stop execution immediately and turn of fans
      if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() == 0) {
        FAN_setSpeed(LIFT_FAN_PORT, 0);
        FAN_setSpeed(THRUST_FAN_PORT, 0);
        Serial.println("Batteries are empty");
        flags.stop = 1;
      }
    }

    // Every 1s print debug information
    if (flags.time_ms % 1000 == 0) {
      Serial.print("Distance: ");
      Serial.print(dist_front);
      Serial.print(",\tEcho lenght: ");
      Serial.print(HCSR04_echo_length_us);

      Serial.println();
    }
  }

  // ----- Control System -----
}
