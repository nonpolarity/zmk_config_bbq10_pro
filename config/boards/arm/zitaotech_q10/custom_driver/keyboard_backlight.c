/*
 * Copyright (c) 2023 ZitaoTech
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/led.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>

#include <zmk/rgb_underglow.h>
#include <zmk/event_manager.h>
#include <zmk/activity.h>
#include <zmk/keymap.h>
#include <zmk/events/activity_state_changed.h>

#include "led_duty.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

BUILD_ASSERT(DT_HAS_CHOSEN(zmk_keyboard_backlight),
             "keyboard_backlight: No zmk_keyboard_backlight chosen node found");

static const struct device *const indiled_dev = DEVICE_DT_GET(DT_CHOSEN(zmk_keyboard_backlight));

#define CHILD_COUNT(...) +1
#define DT_NUM_CHILD(node_id) (DT_FOREACH_CHILD(node_id, CHILD_COUNT))
#define INDICATOR_LED_NUM_LEDS (DT_NUM_CHILD(DT_CHOSEN(zmk_keyboard_backlight)))

#define BRT_MAX 90
#define BRT_BLINK_HIGH 100
#define BRT_BLINK_LOW 10
#define BLINK_INTERVAL_MS 500

#define CYCLE_BRT_MIN 10
#define CYCLE_BRT_MAX 100
#define CYCLE_BRT_STEP 5
#define CYCLE_INTERVAL_MS 20

/* 层3双闪：亮、暗、亮、长暗（单位 ms） */
static const uint16_t double_flash_steps_ms[] = {100, 100, 100, 700};

/*
 * 层1–3 的闪烁、呼吸、双闪以 RGB 亮度为最亮一档，暗的一档按比例变暗，再按 led_duty() 曲线输出。
 * RGB 亮度低于 INDICATOR_BRT_MIN 时按它计算（最亮约 4%）：占空比只能按 1% 调，再暗就分不出闪烁和呼吸了。
 */
#define INDICATOR_BRT_MIN 20

static uint8_t indicator_peak = INDICATOR_BRT_MIN;

static bool prev_active = false;
static int prev_layer = -1;
static bool blink_on = false;
static bool blink_start_high = true;
static uint8_t cycle_brightness = CYCLE_BRT_MIN;
static bool cycle_direction_up = true;
static uint8_t flash_step = 0;

static struct k_work_delayable polling_work;
static struct k_work_delayable blink_work;
static struct k_work_delayable cycle_work;
static struct k_work_delayable flash_work;

/*
 * 保存的 RGB 亮度和开关要等 main() 里的 settings_load() 才读回来，比本驱动开始轮询晚。
 * 之前读到的是 ZMK 的默认值（亮度 100），第 0 层会一直按它点灯，开机后像是亮度被重设了。
 * settings_load() 读完后会调用各处理器的 commit，之后才开始点灯。
 */
static bool settings_loaded = !IS_ENABLED(CONFIG_SETTINGS);

#if IS_ENABLED(CONFIG_SETTINGS)
static int keyboard_backlight_settings_set(const char *name, size_t len, settings_read_cb read_cb,
                                           void *cb_arg) {
    return -ENOENT; /* 本驱动不保存设置，只用 commit 判断设置已读完 */
}

static int keyboard_backlight_settings_commit(void) {
    settings_loaded = true;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(keyboard_backlight, "keyboard_backlight", NULL,
                               keyboard_backlight_settings_set, keyboard_backlight_settings_commit,
                               NULL);
#endif

static void set_led_brightness(uint8_t level) {
    if (!device_is_ready(indiled_dev)) {
        LOG_ERR("Indicator LED device not ready");
        return;
    }
    for (int i = 0; i < INDICATOR_LED_NUM_LEDS; i++) {
        int err = led_set_brightness(indiled_dev, i, level);
        if (err < 0) {
            LOG_ERR("Failed to set LED[%d] brightness: %d", i, err);
        }
    }
}

/* level：指示效果里的相对亮度（0–100），按当前 RGB 亮度缩放 */
static void set_indicator_brightness(uint8_t level) {
    set_led_brightness(led_duty(level * indicator_peak / 100));
}

/* 层1闪烁 */
static void blink_work_handler(struct k_work *work) {
    if (prev_layer != 1) {
        set_led_brightness(0);
        return;
    }

    blink_on = !blink_on;
    set_indicator_brightness(blink_on ? BRT_BLINK_HIGH : BRT_BLINK_LOW);

    k_work_reschedule(&blink_work, K_MSEC(BLINK_INTERVAL_MS));
}

/* 层3（F 键，Fn）双闪 */
static bool is_fn_layer(int layer) { return layer == 3; }

static void flash_work_handler(struct k_work *work) {
    if (!is_fn_layer(prev_layer)) {
        set_led_brightness(0);
        return;
    }

    flash_step = (flash_step + 1) % ARRAY_SIZE(double_flash_steps_ms);
    set_indicator_brightness((flash_step % 2 == 0) ? BRT_BLINK_HIGH : BRT_BLINK_LOW);

    k_work_reschedule(&flash_work, K_MSEC(double_flash_steps_ms[flash_step]));
}

/* 层2呼吸 */
static void cycle_work_handler(struct k_work *work) {
    if (prev_layer != 2) {
        set_led_brightness(0);
        return;
    }

    set_indicator_brightness(cycle_brightness);

    if (cycle_direction_up) {
        cycle_brightness += CYCLE_BRT_STEP;
        if (cycle_brightness >= CYCLE_BRT_MAX) {
            cycle_brightness = CYCLE_BRT_MAX;
            cycle_direction_up = false;
        }
    } else {
        if (cycle_brightness < CYCLE_BRT_STEP) {
            cycle_brightness = CYCLE_BRT_MIN;
            cycle_direction_up = true;
        } else {
            cycle_brightness -= CYCLE_BRT_STEP;
        }
    }
    k_work_reschedule(&cycle_work, K_MSEC(CYCLE_INTERVAL_MS));
}

static void polling_work_handler(struct k_work *work) {
    if (!settings_loaded) {
        k_work_reschedule(&polling_work, K_MSEC(100));
        return;
    }

    bool active = (zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    int current_layer = zmk_keymap_highest_layer_active();
    struct zmk_led_hsb ug = zmk_rgb_underglow_calc_brt(0);
    uint8_t ug_brt = ug.b;
    bool rgb_on = true;
#if IS_ENABLED(CONFIG_ZMK_RGB_UNDERGLOW)
    if (zmk_rgb_underglow_get_state(&rgb_on) < 0) {
        rgb_on = true;
    }
#endif

    /* 每次轮询都更新，在导航层调 RGB 亮度时呼吸立即跟着变 */
    indicator_peak = MAX(ug_brt, INDICATOR_BRT_MIN);

    if (current_layer != prev_layer || active != prev_active) {
        prev_layer = current_layer;
        prev_active = active;

        k_work_cancel_delayable(&blink_work);
        k_work_cancel_delayable(&cycle_work);
        k_work_cancel_delayable(&flash_work);
        blink_on = false;
        cycle_brightness = CYCLE_BRT_MIN;
        cycle_direction_up = true;

        switch (current_layer) {
        case 0:
            /* 层0：跟随 RGB 亮度和活动状态；按 led_duty() 曲线输出，最低一档（10）约 1%，只留微光 */
            uint8_t brt = (rgb_on && active) ? led_duty(ug_brt) : 0;
            set_led_brightness(brt);
            break;

        case 1:
            /*
             * 层1：闪烁
             * RGB 关闭 → 先高亮再低亮
             * RGB 开启 → 先低亮再高亮
             */
            blink_start_high = !rgb_on ? true : false;
            blink_on = blink_start_high;
            set_indicator_brightness(blink_on ? BRT_BLINK_HIGH : BRT_BLINK_LOW);
            /* 使用半周期，避免首次到第二次闪烁间隔过长 */
            k_work_reschedule(&blink_work, K_MSEC(BLINK_INTERVAL_MS / 2));
            break;

        case 2:
            /* 层2：始终呼吸 */
            k_work_reschedule(&cycle_work, K_MSEC(100));
            break;

        case 3:
            /* 层3（F 键）：双闪 */
            flash_step = 0;
            set_indicator_brightness(BRT_BLINK_HIGH);
            k_work_reschedule(&flash_work, K_MSEC(double_flash_steps_ms[0]));
            break;

        default:
            set_led_brightness(0);
            break;
        }
    }

    k_work_reschedule(&polling_work, K_MSEC(100));
}

static int keyboardbacklight_init(void) {
    if (!device_is_ready(indiled_dev)) {
        LOG_ERR("LED indicator device not ready");
        return -ENODEV;
    }

    prev_active = (zmk_activity_get_state() == ZMK_ACTIVITY_ACTIVE);
    prev_layer = -1;

    k_work_init_delayable(&polling_work, polling_work_handler);
    k_work_init_delayable(&blink_work, blink_work_handler);
    k_work_init_delayable(&cycle_work, cycle_work_handler);
    k_work_init_delayable(&flash_work, flash_work_handler);

    k_work_reschedule(&polling_work, K_MSEC(100));
    return 0;
}

SYS_INIT(keyboardbacklight_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
