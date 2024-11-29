#include "fan.h"

#include "config.h"

void FAN_setSpeed(uint8_t port, uint8_t speed) {

  if(!ENABLE_FANS) {
    return;
  }

  speed = speed > 100 ? 100 : speed;

  if (port == 'A') {
    OCR0A = speed * 2.55;
  } else if (port == 'B') {
    OCR0B = speed * 2.55;
  }
}