#include "ADC.h"

#include <avr/io.h>

void ADC_setup() {

  // ADMUX &= ~((1 << REFS1) | (1 << REFS0)); // Clear REFS1 and REFS0 bits
  ADMUX |= (0 << REFS1) | (0 << REFS0);    // Analog ref external

  ADCSRA |= (1 << ADEN);                                // Enable the ADC
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // 128 prescaler
}

uint16_t ADC_read(uint8_t channel) {

  ADMUX = (ADMUX & 0xF8) | (channel & 0x07); // Set the channel (0 to 7)
  ADCSRA |= (1 << ADSC);                     // Start the conversion

  while (ADCSRA & (1 << ADSC)) // Wait for the conversion to complete
    ;

  return ADC;
}

float ADC_read_voltage(uint8_t channel) {
  return (float)ADC_read(channel) / 1024.0 * VREF;
}
