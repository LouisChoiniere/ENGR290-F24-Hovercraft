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
#define GP2Y0A21YK_ADC_CHANNEL 0 // P5
#define LIFT_FAN_PORT 'A' // P4
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
  ADC_setup_channel(GP2Y0A21YK_ADC_CHANNEL);

  // Setup timers
  timer0_setup();
  timer1_setup();

  // I2C setup and setup MPU6050
  Wire.begin();
  MPU6050_WakeUpMPU();
  MPU6050_SetAccelerationSensitivity(&IMU, MPU6050_ACCELEROMETER_RANGE_4G);
  MPU6050_SetGyroSensitivity(&IMU, MPU6050_GYROSCOPE_RANGE_1000);
  MPU6050_SetFilter(MPU6050_FILTER_BW_20);
  MPU6050_Calibrate(&IMU, 5000);

  Serial.println("Start prechecks");
  Serial.flush();

  // Batteries are almost empty do not start execution
  if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() <= 1) {
    Serial.println("Batteries are empty");
    Serial.flush();
    while (true)
      ;
  }

  // Start lift fan
  FAN_setSpeed(LIFT_FAN_PORT, LIFT_FAN_SPEED);

  _delay_ms(100);
}

void loop() {
  counter++;

  // Batteries are empty stop execution immediately and turn of fans
  if (ENABLE_BATTERY_CHECK && BatteryVoltage_GetState() == 0) {
    FAN_setSpeed(LIFT_FAN_PORT, 0);
    FAN_setSpeed(THRUST_FAN_PORT, 0);
    while (true)
      ;
  }

  double dist = GP2Y0A21YK_GetDistance(GP2Y0A21YK_ADC_CHANNEL);

  MPU6050_ALL(&IMU, 5);

  if (counter % 100 == 0) {
    Serial.print("Accel (X, Y, Z): (");
    Serial.print(IMU.acceleration_X);
    Serial.print(", ");
    Serial.print(IMU.acceleration_Y);
    Serial.print(",");
    Serial.print(IMU.acceleration_Z);
    Serial.print(")");

    Serial.print("\tGyro (X, Y, Z): (");
    Serial.print(IMU.gyroscope_X);
    Serial.print(", ");
    Serial.print(IMU.gyroscope_Y);
    Serial.print(", ");
    Serial.print(IMU.gyroscope_Z);
    Serial.print(")");

    // Serial.print("\tVelocity (X, Y, Z): (");
    // Serial.print(IMU.velocity_X);
    // Serial.print(", ");
    // Serial.print(IMU.velocity_Y);
    // Serial.print(", ");
    // Serial.print(IMU.velocity_Z);
    // Serial.print(")");

    // Serial.print("\tPosition (X, Y, Z): (");
    // Serial.print(IMU.position_X);
    // Serial.print(", ");
    // Serial.print(IMU.position_Y);
    // Serial.print(", ");
    // Serial.print(IMU.position_Z);
    // Serial.print(")");

    Serial.print("\tRoll: ");
    Serial.print(IMU.roll);
    Serial.print("\tPitch: ");
    Serial.print(IMU.pitch);
    Serial.print("\tYaw: ");
    Serial.print(IMU.yaw);

    Serial.print("\tDistance: ");
    Serial.println(dist, 2);
    Serial.println();
    Serial.flush();
  }

  _delay_ms(5);
}

// int main(void) {
//   Serial.begin(115200); // Initialize serial communication
//   while (!Serial)
//     _delay_ms(10);
//   Serial.println("Start of program");
//   Serial.flush();

//   mySetup();

//   Serial.println("Start of Loop");
//   Serial.flush();

//   while (1) {
//     myLoop();
//   }
// }