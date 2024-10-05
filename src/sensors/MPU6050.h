#ifndef MPU6050_H
#define MPU6050_H

#include <avr/io.h>
#include <Wire.h>

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
} MPU6050_Values_Struct;

void MPU6050_WakeUpMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
};

void MPU6050_SetAccelerationRange();
void MPU6050_SetGyroRange();
void MPU6050_setFilterBandwidth();

void MPU6050_ReadAcceleromterData(MPU6050_Values_Struct* MPU6050) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // 0x3B -> ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  MPU6050->acceleration_X = (Wire.read() << 8 | Wire.read()) / 16384.0;
  MPU6050->acceleration_Y = (Wire.read() << 8 | Wire.read()) / 16384.0;
  MPU6050->acceleration_Z = (Wire.read() << 8 | Wire.read()) / 16384.0;
  Wire.endTransmission(true);
};

void MPU6050_ReadGyroscopeData(MPU6050_Values_Struct* MPU6050){
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