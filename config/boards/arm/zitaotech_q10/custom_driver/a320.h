/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef A320_H
#define A320_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief check if the touchpad is touched
 *
 * @return true if touched
 */
bool tp_is_touched(void);

/**
 * @brief Total trackpad motion (|dx| + |dy| in sensor counts) since boot, wrapping on overflow.
 *
 * Compare two readings with unsigned subtraction to get the motion in between.
 */
uint32_t a320_motion_counts(void);

#ifdef __cplusplus
}
#endif

#endif // A320_H
