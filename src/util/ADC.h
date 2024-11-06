#pragma once

#include <stdint.h>

#define VREF 3.8

#ifdef __cplusplus
extern "C" {
#endif

void ADC_setup();
void ADC_setup_channel(uint8_t channel);

uint16_t ADC_read(uint8_t channel);
float ADC_read_voltage(uint8_t channel);

#ifdef __cplusplus
}
#endif