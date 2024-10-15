#pragma once

#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

void SERVO_setPosition(int8_t angle);

#ifdef __cplusplus
}
#endif