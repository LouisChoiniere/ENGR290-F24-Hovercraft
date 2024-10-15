#include "timer.h"

#include <avr/io.h>

void timer0_setup() {
  // // Set as output
  // DDRD = ((1 << PD6) | (1 << PD5));

  // // Fast PMW, Non inverting output, 64 Prescaler
  // TCCR0A |= (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  // TCCR0B |= (1 << CS02) | (1 << CS00);

  // OCR0A = 30;
  // OCR0B = 0;
}

void timer1_setup() {
  // Set output
  DDRB = ((1 << PB2) | (1 << PB1));

  // Fast PMW, Non inverting output, 64 Prescaler
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1);
  TCCR1B |= (1 << WGM13 | (1 << CS11) | (1 << CS10));

  ICR1 = 2500; // Max value of the counter before reset

  OCR1A = 150; // Servo angle
  OCR1B = 0;   // Output unsued
}