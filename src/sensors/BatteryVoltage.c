#include "BatteryVoltage.h"

#include <stdint.h>
#include <math.h>

#include "util/ADC.h"

float BatteryVoltage_GetVoltage(uint8_t channel) {

  // TODO: Check if there is hardware division to the voltage or any processing nedded
  float volts = ADC_read_voltage(channel);

  return volts;
};