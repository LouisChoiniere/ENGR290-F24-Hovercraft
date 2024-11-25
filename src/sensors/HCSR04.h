#pragma once

#include <stdint.h>

volatile uint16_t HCSR04_rising_edge;
volatile uint16_t HCSR04_falling_edge;
volatile uint16_t HCSR04_echo_length_us;
volatile float HCSR04_distance_cm;

#ifdef __cplusplus
extern "C" {
#endif

void HCSR04_Setup();
void HCSR04_Trigger();
void HCSR04_HandleInterupt(uint8_t edge);
float HCSR04_GetDistance();

#ifdef __cplusplus
}
#endif