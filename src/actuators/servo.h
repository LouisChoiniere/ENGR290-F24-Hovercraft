#pragma once

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sets position of the servo.
 *
 * @param angle In degress from -90 to +90.
 */
void SERVO_setPosition(int8_t angle);

#ifdef __cplusplus
}
#endif