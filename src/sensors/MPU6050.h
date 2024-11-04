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

#define MPU6050_FILTER_BW_260 0x00
#define MPU6050_FILTER_BW_184 0x01
#define MPU6050_FILTER_BW_94 0x02
#define MPU6050_FILTER_BW_42 0x03
#define MPU6050_FILTER_BW_20 0x04
#define MPU6050_FILTER_BW_10 0x05
#define MPU6050_FILTER_BW_5 0x06

#define ACCEL_DIV_2G 16384
#define ACCEL_DIV_4G 8192
#define ACCEL_DIV_8G 4096
#define ACCEL_DIV_16G 2048

#define GYRO_DIV_250 131
#define GYRO_DIV_500 66
#define GYRO_DIV_1000 33
#define GYRO_DIV_2000 16

#define GRAVITY 9.81

typedef struct {
  uint16_t acceleration_division_factor;
  uint16_t gyroscope_division_factor;

  float calibration_acceleraton_X;
  float calibration_acceleraton_Y;
  float calibration_acceleraton_Z;

  float calibration_gyroscope_X;
  float calibration_gyroscope_Y;
  float calibration_gyroscope_Z;

  // Acceleration in m/s^2
  float acceleration_X;
  float acceleration_Y;
  float acceleration_Z;

  // Angular velocity in rad/s
  float gyroscope_X;
  float gyroscope_Y;
  float gyroscope_Z;

  float velocity_X = 0;
  float velocity_Y = 0;
  float velocity_Z = 0;

  float position_X = 0;
  float position_Y = 0;
  float position_Z = 0;

  double roll = 0;
  double pitch = 0;
  double yaw = 0;

  uint64_t last_time;
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

void MPU6050_SetFilter(int filter) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_FILTER_CONFIG_REGISTERS);
  Wire.write(filter);
  Wire.endTransmission(true);
};

void MPU6050_ReadAcceleromterData(MPU6050_Struct *MPU6050) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  int16_t raw_accel_X = (Wire.read() << 8 | Wire.read());
  int16_t raw_accel_Y = (Wire.read() << 8 | Wire.read());
  int16_t raw_accel_Z = (Wire.read() << 8 | Wire.read());

  uint16_t division_factor = MPU6050->acceleration_division_factor;
  MPU6050->acceleration_X = ((float)raw_accel_X / division_factor) * 9.8 - MPU6050->calibration_acceleraton_X;
  MPU6050->acceleration_Y = ((float)raw_accel_Y / division_factor) * 9.8 - MPU6050->calibration_acceleraton_Y;
  MPU6050->acceleration_Z = ((float)raw_accel_Z / division_factor) * 9.8 - MPU6050->calibration_acceleraton_Z;

  Wire.endTransmission(true);
};

void MPU6050_ReadGyroscopeData(MPU6050_Struct *MPU6050) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);

  uint16_t raw_gyro_X = (Wire.read() << 8 | Wire.read());
  uint16_t raw_gyro_Y = (Wire.read() << 8 | Wire.read());
  uint16_t raw_gyro_Z = (Wire.read() << 8 | Wire.read());

  uint16_t division_factor = MPU6050->gyroscope_division_factor;
  MPU6050->gyroscope_X = ((float)raw_gyro_X / division_factor) - MPU6050->calibration_gyroscope_X;
  MPU6050->gyroscope_Y = ((float)raw_gyro_Y / division_factor) - MPU6050->calibration_gyroscope_Y;
  MPU6050->gyroscope_Z = ((float)raw_gyro_Z / division_factor) - MPU6050->calibration_gyroscope_Z;

  Wire.endTransmission(true);
};

void MPU6050_Calibrate(MPU6050_Struct *MPU6050, uint32_t time) {
  float sum_accel_X = 0, sum_accel_Y = 0, sum_accel_Z = 0;
  float sum_gyro_X = 0, sum_gyro_Y = 0, sum_gyro_Z = 0;
  uint32_t samples = 0;
  uint32_t start_time = millis();

  while (millis() - start_time < time) {
    MPU6050_ReadAcceleromterData(MPU6050);
    MPU6050_ReadGyroscopeData(MPU6050);

    sum_accel_X += MPU6050->acceleration_X;
    sum_accel_Y += MPU6050->acceleration_Y;
    sum_accel_Z += MPU6050->acceleration_Z;

    sum_gyro_X += MPU6050->gyroscope_X;
    sum_gyro_Y += MPU6050->gyroscope_Y;
    sum_gyro_Z += MPU6050->gyroscope_Z;

    samples++;
    delay(10); // Small delay to allow sensor to stabilize
  }

  MPU6050->calibration_acceleraton_X = sum_accel_X / samples;
  MPU6050->calibration_acceleraton_Y = sum_accel_Y / samples;
  MPU6050->calibration_acceleraton_Z = (sum_accel_Z / samples) - GRAVITY;

  MPU6050->calibration_gyroscope_X = sum_gyro_X / samples;
  MPU6050->calibration_gyroscope_Y = sum_gyro_Y / samples;
  MPU6050->calibration_gyroscope_Z = sum_gyro_Z / samples;
}

void MPU6050_ALL(MPU6050_Struct *MPU6050) {
  MPU6050_ReadAcceleromterData(MPU6050);
  MPU6050_ReadGyroscopeData(MPU6050);

  uint64_t current_time = millis();
  uint16_t dt = current_time - MPU6050->last_time;
  float dt_seconds = dt / 1000.0;
  MPU6050->last_time = current_time;

  MPU6050->velocity_X += MPU6050->acceleration_X * dt_seconds;
  MPU6050->velocity_Y += MPU6050->acceleration_Y * dt_seconds;
  MPU6050->velocity_Z += (MPU6050->acceleration_Z - GRAVITY) * dt_seconds;

  MPU6050->position_X += MPU6050->velocity_X * dt_seconds;
  MPU6050->position_Y += MPU6050->velocity_Y * dt_seconds;
  MPU6050->position_Z += MPU6050->velocity_Z * dt_seconds;

  // MPU6050->pitch = 0;
  // MPU6050->roll = 0;
  // MPU6050->yaw = 0;

}
