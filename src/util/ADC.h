#pragma once

#include <stdint.h>

#define VREF 3.4

#ifdef __cplusplus
extern "C" {
#endif

void ADC_setup();
uint16_t ADC_read(uint8_t channel);

#ifdef __cplusplus
}
#endif