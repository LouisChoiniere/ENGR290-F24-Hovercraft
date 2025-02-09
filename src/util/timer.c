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

/*
* Timer 1 setup
* One tick is 0.1us
* Frequency = 50Hz
* Period = 20ms
*/
void timer1_setup() {
  
  // Set output
  DDRB |= (1 << PB2) | (1 << PB1);

  TCCR1A = (1 << COM1A1) | (1 << COM1B1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);

  ICR1 = 40000; // Max value of the counter before reset

  OCR1A = 3000; // Servo angle (Default to 0 degrees)
  OCR1B = 0;   // Output unsued

  TIMSK1 |= (1 << ICIE1); // Enable Timer/Counter1 Input Capture interrupt
}