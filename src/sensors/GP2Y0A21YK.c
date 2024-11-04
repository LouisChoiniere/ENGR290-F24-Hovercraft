#include "GP2Y0A21YK.h"

#include <stdint.h>
#include <math.h>

#include "util/ADC.h"

float GP2Y0A21YK_GetDistance(uint8_t channel) {

  float volts = ADC_read_voltage(channel);
  float distance = (float)29.988 * (float)pow(volts, (float)-1.173);

  return distance;
};