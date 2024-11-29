#include "servo.h"

#include "config.h"

#define OFFSET_DEGREES -6
#define OFFSET_TICKS OFFSET_DEGREES*20

void SERVO_setPosition(int8_t angle) {

    if(!ENABLE_SERVO) {
        return;
    }

    if (angle > 90) {
        angle = 90;
    } else if (angle < -90) {
        angle = -90;
    }

    // One tick of the timer is 0.5us
    // 3000 ticks is the middle position
    uint16_t ticks = OFFSET_TICKS + 3000 + (angle * 20);

    OCR1A = ticks;
}