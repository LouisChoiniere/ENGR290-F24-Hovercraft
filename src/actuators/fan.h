#pragma once

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Sets the speed of the fan.
 *
 * @param port The desired port of the fan, either 'A' (P4) or 'B' (P3).
 * @param speed The desired speed of the fan, in percentage. From (0-100).
 */
void FAN_setSpeed(uint8_t port, uint8_t speed);

#ifdef __cplusplus
}
#endif