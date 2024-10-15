#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "actuators/servo.h"
#include "sensors/GP2Y0A21YK.h"
#include "sensors/MPU6050.h"
#include "util/ADC.h"
#include "util/timer.h"

#define GP2Y0A21YK_ADC_CHANNEL 0

MPU6050_Struct IMU;

uint64_t counter = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial)
    _delay_ms(10);
  Serial.println("Start of program");
  Serial.flush();

  ADC_setup();
  timer1_setup();

  Wire.begin(); // Start the I2C communication

  MPU6050_WakeUpMPU(); // Send wake bit to IMU
  MPU6050_SetAccelerationSensitivity(&IMU, MPU6050_ACCELEROMETER_RANGE_4G);
  MPU6050_SetGyroSensitivity(&IMU, MPU6050_GYROSCOPE_RANGE_500);


  Serial.println("Start of Loop");
  Serial.flush();
}

void loop() {

  counter++;

  double dist = GP2Y0A21YK_GetDistance(GP2Y0A21YK_ADC_CHANNEL);

  MPU6050_ALL(&IMU);

  if (counter % 1000 == 0) {
    Serial.print(" Accel_X: ");
    Serial.print(IMU.acceleration_X);
    Serial.print(" Accel_Y: ");
    Serial.print(IMU.acceleration_Y);
    Serial.print(" Accel_Z: ");
    Serial.print(IMU.acceleration_Z);

    Serial.print(" Gyro_X: ");
    Serial.print(IMU.gyroscope_X);
    Serial.print(" Gyro_Y: ");
    Serial.print(IMU.gyroscope_Y);
    Serial.print(" Gyro_Z: ");
    Serial.print(IMU.gyroscope_Z);
    
    Serial.print(" Velocity_X: ");
    Serial.print(IMU.velocity_X);
    Serial.print(" Velocity_Y: ");
    Serial.print(IMU.velocity_Y);
    Serial.print(" Velocity_Z: ");
    Serial.print(IMU.velocity_Z);

    Serial.print(" Position_X: ");
    Serial.print(IMU.position_X);
    Serial.print(" Position_Y: ");
    Serial.print(IMU.position_Y);
    Serial.print(" Position_Z: ");
    Serial.print(IMU.position_Z);

    Serial.print(" Pitch: ");
    Serial.print(IMU.pitch);
    Serial.print(" Roll: ");
    Serial.print(IMU.roll);
    Serial.print(" Yaw: ");
    Serial.print(IMU.yaw);



    Serial.print(" Distance: ");
    Serial.println(dist, 2);
    Serial.flush();
  }

  _delay_ms(10);
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