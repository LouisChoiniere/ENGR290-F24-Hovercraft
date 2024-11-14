#include "servo.h"

void SERVO_setPosition(int8_t angle) {
    if (angle > 90) {
        angle = 90;
    } else if (angle < -90) {
        angle = -90;
    }

    double ticks = (double)187.5 + ((double)angle * (double)1.35);

    OCR1A = (int)ticks;
}