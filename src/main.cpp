#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include <Arduino.h>
#include <HardwareSerial.h>
#include <Wire.h>

#include "sensors/GP2Y0A21YK.h"
#include "sensors/MPU6050.h"
#include "util/ADC.h"

#define GP2Y0A21YK_ADC_CHANNEL 0

MPU6050_Struct IMU;

int pos = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication
  while (!Serial)
    _delay_ms(10);
  Serial.println("Start of program");
  Serial.flush();

  ADC_setup();

  Wire.begin(); // Start the I2C communication

  MPU6050_WakeUpMPU(); // Send wake bit to IMU
  MPU6050_SetAccelerationSensitivity();
  MPU6050_SetGyroSensitivity();

#pragma region timer0

  // // Set as output
  // DDRD = ((1 << PD6) | (1 << PD5));

  // // Fast PMW, Non inverting output, 64 Prescaler
  // TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  // TCCR0B |= (1 << CS02) | (1 << CS00);

  // OCR0A = 30;
  // OCR0B = 0;

#pragma endregion timer0

// Setup timer1 region
#pragma region timer1
  // // Set output
  // DDRB = ((1 << PB2) | (1 << PB1));

  // // Fast PMW, Non inverting output, 64 Prescaler
  // TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  // TCCR1B |= (1 << WGM13 | (1 << CS11) | (1 << CS10));

  // ICR1 = 2500; // Max value of the counter before reset

  // OCR1A = 150; // Servo angle
  // OCR1B = 0;   // Output unsued

#pragma endregion timer1

  Serial.println("Start of Loop");
  Serial.flush();
}

void loop() {

  double dist = GP2Y0A21YK_GetDistance(GP2Y0A21YK_ADC_CHANNEL);

  MPU6050_ReadAcceleromterData(&IMU);
  MPU6050_ReadGyroscopeData(&IMU);

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

  Serial.print(" Distance: ");
  Serial.println(dist, 2);
  Serial.flush();

  _delay_ms(500);
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