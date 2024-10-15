#pragma once

#include <Wire.h>
#include <avr/io.h>

#define MPU6050_ADDR 0x68

#define MPU6050_ACCELEROMETER_CONFIG_REGISTERS 0x1C
#define MPU6050_GYROSCOPE_CONFIG_REGISTERS 0x1B
#define MPU6050_FILTER_CONFIG_REGISTERS 0x1A

#define MPU6050_ACCELEROMETER_RANGE_2G 0x00
#define MPU6050_ACCELEROMETER_RANGE_4G 0x08
#define MPU6050_ACCELEROMETER_RANGE_8G 0x10
#define MPU6050_ACCELEROMETER_RANGE_16G 0x18

#define MPU6050_GYROSCOPE_RANGE_250 0x00
#define MPU6050_GYROSCOPE_RANGE_500 0x08
#define MPU6050_GYROSCOPE_RANGE_1000 0x10
#define MPU6050_GYROSCOPE_RANGE_2000 0x18

#define ACCEL_DIV_2G 16384
#define ACCEL_DIV_4G 8192
#define ACCEL_DIV_8G 4096
#define ACCEL_DIV_16G 2048

#define GYRO_DIV_250 131
#define GYRO_DIV_500 66
#define GYRO_DIV_1000 33
#define GYRO_DIV_2000 16

typedef struct {
  uint16_t acceleration_division_factor;
  uint16_t gyroscope_division_factor;

  int16_t acceleration_X;
  int16_t acceleration_Y;
  int16_t acceleration_Z;

  int16_t gyroscope_X;
  int16_t gyroscope_Y;
  int16_t gyroscope_Z;

  uint16_t velocity_X;
  uint16_t velocity_Y;
  uint16_t velocity_Z;

  uint16_t position_X;
  uint16_t position_Y;
  uint16_t position_Z;

  double roll;
  double pitch;
  double yaw;

  uint64_t last_time;
  uint64_t dt;
} MPU6050_Struct;

void MPU6050_WakeUpMPU() {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission(true);
};

void MPU6050_SetAccelerationSensitivity(MPU6050_Struct *MPU6050, int sensitivity) {
  switch (sensitivity) {
  case MPU6050_ACCELEROMETER_RANGE_2G:
    MPU6050->acceleration_division_factor = ACCEL_DIV_2G;
    break;
  case MPU6050_ACCELEROMETER_RANGE_4G:
    MPU6050->acceleration_division_factor = ACCEL_DIV_4G;
    break;
  case MPU6050_ACCELEROMETER_RANGE_8G:
    MPU6050->acceleration_division_factor = ACCEL_DIV_8G;
    break;
  case MPU6050_ACCELEROMETER_RANGE_16G:
    MPU6050->acceleration_division_factor = ACCEL_DIV_16G;
    break;
  default:
    MPU6050->acceleration_division_factor = ACCEL_DIV_2G; // Default to 2G if unknown sensitivity
    break;
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCELEROMETER_CONFIG_REGISTERS);
  Wire.write(sensitivity);
  Wire.endTransmission(true);
};

void MPU6050_SetGyroSensitivity(MPU6050_Struct *MPU6050, int sensitivity) {
  switch (sensitivity) {
  case MPU6050_GYROSCOPE_RANGE_250:
    MPU6050->gyroscope_division_factor = GYRO_DIV_250;
    break;
  case MPU6050_GYROSCOPE_RANGE_500:
    MPU6050->gyroscope_division_factor = GYRO_DIV_500;
    break;
  case MPU6050_GYROSCOPE_RANGE_1000:
    MPU6050->gyroscope_division_factor = GYRO_DIV_1000;
    break;
  case MPU6050_GYROSCOPE_RANGE_2000:
    MPU6050->gyroscope_division_factor = GYRO_DIV_2000;
    break;
  default:
    MPU6050->gyroscope_division_factor = GYRO_DIV_250; // Default to 250 if unknown sensitivity
    break;
  }

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYROSCOPE_CONFIG_REGISTERS);
  Wire.write(sensitivity);
  Wire.endTransmission(true);
};

// Returns m/s^2 ??
void MPU6050_ReadAcceleromterData(MPU6050_Struct *MPU6050) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B); // 0x3B -> Acel data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  MPU6050->acceleration_X = (Wire.read() << 8 | Wire.read()) / MPU6050->acceleration_division_factor;
  MPU6050->acceleration_Y = (Wire.read() << 8 | Wire.read()) / MPU6050->acceleration_division_factor;
  MPU6050->acceleration_Z = (Wire.read() << 8 | Wire.read()) / MPU6050->acceleration_division_factor;
  Wire.endTransmission(true);
};

// Returns rad/s ??
void MPU6050_ReadGyroscopeData(MPU6050_Struct *MPU6050) {

  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43); // 0x43 -> Gyro data
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  MPU6050->gyroscope_X = (Wire.read() << 8 | Wire.read()) / MPU6050->gyroscope_division_factor;
  MPU6050->gyroscope_Y = (Wire.read() << 8 | Wire.read()) / MPU6050->gyroscope_division_factor;
  MPU6050->gyroscope_Z = (Wire.read() << 8 | Wire.read()) / MPU6050->gyroscope_division_factor;
  Wire.endTransmission(true);
};

void MPU6050_ALL(MPU6050_Struct *MPU6050) {
  MPU6050_ReadAcceleromterData(MPU6050);
  MPU6050_ReadGyroscopeData(MPU6050);

  uint64_t current_time = millis();
  MPU6050->dt = current_time - MPU6050->last_time;

  MPU6050->velocity_X = MPU6050->acceleration_X * MPU6050->dt;
  MPU6050->velocity_Y = MPU6050->acceleration_Y * MPU6050->dt;
  MPU6050->velocity_Z = MPU6050->acceleration_Z * MPU6050->dt;

  MPU6050->position_X += MPU6050->velocity_X * MPU6050->dt;
  MPU6050->position_Y += MPU6050->velocity_Y * MPU6050->dt;
  MPU6050->position_Z += MPU6050->velocity_Z * MPU6050->dt;

  MPU6050->pitch = 0;
  MPU6050->roll = 0;
  MPU6050->yaw = 0;
}

