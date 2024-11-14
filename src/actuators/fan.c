#include "fan.h"

void FAN_setSpeed(uint8_t port, uint8_t speed) {

  if (port == 'A') {
    OCR0A = speed * 2.55;
  } else if (port == 'B') {
    OCR0B = speed * 2.55;
  }
}