#include "servo.h"

void SERVO_setPosition(int8_t angle) {
    if (angle > 90) {
        angle = 90;
    } else if (angle < -90) {
        angle = -90;
    }

    // One tick of the timer is 0.1us
    // 1500 ticks is the middle position
    uint16_t ticks = 1500 + (angle * 10);

    OCR1A = ticks;
}