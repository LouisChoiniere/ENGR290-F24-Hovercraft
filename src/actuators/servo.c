#include "servo.h"

void SERVO_setPosition(int8_t angle) {
    if (angle > 90) {
        angle = 90;
    } else if (angle < -90) {
        angle = -90;
    }

    // One tick of the timer is 0.5us
    // 3000 ticks is the middle position
    uint16_t ticks = 3000 + (angle * 20);

    OCR1A = ticks;
}