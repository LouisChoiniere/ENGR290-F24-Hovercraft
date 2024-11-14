#pragma once

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sets position of the servo in degress.
 *
 * @param angle Desired angle from -90 degress to +90 degress.
 */
void SERVO_setPosition(int8_t angle);

#ifdef __cplusplus
}
#endif