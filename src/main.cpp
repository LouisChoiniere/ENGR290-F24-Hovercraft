#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "actuators/fan.h"
#include "actuators/servo.h"
#include "sensors/GP2Y0A21YK.h"
#include "sensors/MPU6050.h"
#include "util/ADC.h"
#include "util/BatteryVoltage.h"
#include "util/timer.h"

// Settings
#define ENABLE_BATTERY_CHECK 0

#define LIFT_FAN_SPEED 20


// Pinout selections
#define IR_LEFT 0           // P5
#define IR_RIGHT 1          // P8
#define LIFT_FAN_PORT 'A'   // P4
#define THRUST_FAN_PORT 'B' // P3

MPU6050_Struct IMU;

uint64_t counter = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial)
    _delay_ms(10);

  Serial.println("Setup");
  Serial.flush();

  // ADC and set channels for GP2Y0A21YK
  ADC_setup();
  ADC_setup_channel(BATTERY_VOLTAGE_ADC_CHANNEL);
  ADC_setup_channel(IR_LEFT);

  // Setup timers
  timer0_setup();
  timer1_setup();

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
}

#define LOOP_INTERVAL_MS 5
void loop() {
  counter++;

  // Batteries are empty stop execution immediately and turn of fans
  if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() == 0) {
    FAN_setSpeed(LIFT_FAN_PORT, 0);
    FAN_setSpeed(THRUST_FAN_PORT, 0);
    while (true)
      ;
  }


  // ----- Data collection -----
  MPU6050_ALL(&IMU, LOOP_INTERVAL_MS);

  double dist_left = GP2Y0A21YK_GetDistance(IR_LEFT);
  double dist_right = GP2Y0A21YK_GetDistance(IR_RIGHT);
  double dist_front = 0;

  // ----- Control System -----


  _delay_ms(LOOP_INTERVAL_MS);
}
