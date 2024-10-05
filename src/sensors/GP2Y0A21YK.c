#include "GP2Y0A21YK.h"

#include <stdint.h>
#include <math.h>

#include "util/ADC.h"

double GP2Y0A21YK_GetDistance(uint8_t channel) {

  uint16_t ADC_value = ADC_read(channel);

  double volts = (double)ADC_value / (double)1024 * VREF;
  double distance = (double)29.988 * (double)pow(volts, (double)-1.173);

  return distance;
};