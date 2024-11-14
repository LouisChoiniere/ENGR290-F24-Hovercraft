#include "timer.h"

#include <avr/io.h>

void timer0_setup() {
  
  // Set as output
  DDRD |= (1 << PD6) | (1 << PD5);

  // Fast PMW, Clear on compare match, set at bottom, (non inverting mode), 1024 Prescaler
  // COM0A1 -> Clear OC0A on compare match, set OC0A at BOTTOM, (non-inverting mode)
  // COM0B1 -> Clear OC0B on compare match, set OC0B at BOTTOM, (non-inverting mode)
  // WGM01 & WGM00 -> Fast PWM, TOP at 0xFF
  // CS02 & CS00 -> 1024 Prescaler
  TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM01) | (1 << WGM00);
  TCCR0B = (1 << CS02) | (1 << CS00);

  // Set frequency to 0 before starting
  OCR0A = 0;
  OCR0B = 0;
}

void timer1_setup() {
  
  // Set output
  DDRB |= (1 << PB2) | (1 << PB1);

  TCCR1A = (1 << COM1A1) | (1 << COM1B1);
  TCCR1B = (1 << WGM13) | (1 << CS11) | (1 << CS10);

  ICR1 = 2500; // Max value of the counter before reset

  OCR1A = 150; // Servo angle
  OCR1B = 0;   // Output unsued
}