#ifndef MPU6050_H
#define MPU6050_H

#include <Wire.h>
#include <avr/io.h>

#define MPU6050_ADDR 0x68

typedef struct {

  uint8_t acceleration_range;
  uint8_t gyroscope_range;

  int16_t acceleration_X;
  int16_t acceleration_Y;
  int16_t acceleration_Z;

  int16_t gyroscope_X;
  int16_t gyroscope_Y;
  int16_t gyroscope_Z;
} MPU6050_Struct;

void MPU6050_WakeUpMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
};

void MPU6050_SetAccelerationSensitivity() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1C); // Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x10); // Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);
};

void MPU6050_SetGyroSensitivity() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x1B); // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x10); // Set the register bits as 00010000 (1000deg/s full scale)
  Wire.endTransmission(true);
};

void MPU6050_setFilterBandwidth();


// Returns m/s^2 ??
void MPU6050_ReadAcceleromterData(MPU6050_Struct *MPU6050) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // 0x3B -> ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  // For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  MPU6050->acceleration_X = (Wire.read() << 8 | Wire.read()) / 16384.0;
  MPU6050->acceleration_Y = (Wire.read() << 8 | Wire.read()) / 16384.0;
  MPU6050->acceleration_Z = (Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.endTransmission(true);
};

// Returns rad/s ??
void MPU6050_ReadGyroscopeData(MPU6050_Struct *MPU6050) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43); // 0x43 -> Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  MPU6050->gyroscope_X = (Wire.read() << 8 | Wire.read()) / 131.0;
  MPU6050->gyroscope_Y = (Wire.read() << 8 | Wire.read()) / 131.0;
  MPU6050->gyroscope_Z = (Wire.read() << 8 | Wire.read()) / 131.0;
  Wire.endTransmission(true);
};

// void MPU6050_Read

#endif