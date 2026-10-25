/*
 * SPDX-License-Identifier: MIT
 */
#pragma once

#include <stdint.h>
#include <zephyr/sys/util.h>

/*
 * 亮度（0–100）按平方换算成 PWM 占空比，低档更暗、更接近人眼的感觉：10 → 1%，20 → 4%，50 → 25%，100 → 100%。
 * 非零亮度至少 1%，不会被换算成熄灭。
 */
static inline uint8_t led_duty(uint8_t level) {
    if (level == 0) {
        return 0;
    }
    return MAX(1, (level * level + 50) / 100);
}
