#pragma once

#include <stdint.h>

#define BATTERY_VOLTAGE_ADC_CHANNEL 7

#ifdef __cplusplus
extern "C" {
#endif

uint8_t BatteryVoltage_GetState();

#ifdef __cplusplus
}
#endif