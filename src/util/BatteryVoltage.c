#include "BatteryVoltage.h"

#include <math.h>
#include <stdint.h>

#include "util/ADC.h"

uint8_t BatteryVoltage_GetState() {

  uint16_t value = ADC_read(BATTERY_VOLTAGE_ADC_CHANNEL);

  if (value < 150)
    return 0; // empty (13v)
  else if (value < 700)
    return 1; // low below 25% (15v)
  else
    return 2; // above 25%
};