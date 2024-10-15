#include "servo.h"

void SERVO_setPosition(int8_t angle) {
    if (angle > 45) {
        angle = 45;
    } else if (angle < -45) {
        angle = -45;
    }

    double ticks = (double)187.5 + angle * (double)1.11;

    OCR1A = (int)ticks;
}