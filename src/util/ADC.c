#include "ADC.h"

#include <avr/io.h>

void ADC_setup() {
  ADMUX = 0; // Analog ref external and left adjust result
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // Enable the ADC and set the clock prescaler to 128
}

void ADC_setup_channel(uint8_t channel) {
  DDRC &= ~(1 << channel); // Set the channel (0 to 7) as input
}

uint16_t ADC_read(uint8_t channel) {

  ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Set the channel (0 to 7)

  ADCSRA |= (1 << ADSC); // Start the conversion
  while (ADCSRA & (1 << ADSC))
    ; // Wait for the conversion to complete

  (void)ADC;

  ADCSRA |= (1 << ADSC); // Start the conversion
  while (ADCSRA & (1 << ADSC))
    ; // Wait for the conversion to complete

  return ADC;
}

float ADC_read_voltage(uint8_t channel) {
  return (float)ADC_read(channel) / 1024.0 * VREF;
}
