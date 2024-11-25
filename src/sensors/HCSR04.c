#include "HCSR04.h"

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

void HCSR04_Setup() {
  DDRB |= (1 << PB3);  // Set PB3 as output
  DDRD &= ~(1 << PD2); // Set PD2 (INT0) as input
}

void HCSR04_Trigger() {
  PORTB |= (1 << PB3);  // Set PB3 high
  _delay_us(10);        // Wait 10us
  PORTB &= ~(1 << PB3); // Set PB3 low
}

void HCSR04_HandleInterupt(uint8_t edge) {
  if (edge) {
    // Rising edge
    HCSR04_rising_edge = TCNT1;
  } else {
    // Falling edge
    HCSR04_falling_edge = TCNT1;

    uint16_t echo_length = 0;
    if (HCSR04_falling_edge >= HCSR04_rising_edge) {
      echo_length = HCSR04_falling_edge - HCSR04_rising_edge;
    } else {
      echo_length = HCSR04_falling_edge + 20000 - HCSR04_rising_edge;
    }

    HCSR04_echo_length_us = echo_length / 2;
    HCSR04_distance_cm = (float)echo_length / 116.0;
  }
}

float HCSR04_GetDistance() {
  return HCSR04_distance_cm;
}