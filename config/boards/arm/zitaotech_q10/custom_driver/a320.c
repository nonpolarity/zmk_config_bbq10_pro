/*
 * A320 optical sensor driver (polling via motion GPIO, Zephyr input subsystem)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT avago_a320

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/input/input.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>

#include <stdlib.h>
#include <zmk/event_manager.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/hid_indicators_changed.h>

#include "trackpad_led.h"

LOG_MODULE_REGISTER(a320, CONFIG_A320_LOG_LEVEL);

/* =========================
 * Configurable parameters
 * ========================= */

#define SHIFT_ARROW_DEBOUNCE_MS 50 // 
#define SHIFT_ARROW_RELEASE_MS 20  // 

/*
 * 鼠标模式下按住左 aA（Shift）减速、按住 0 键（Ctrl）加速，与 Windows“鼠标键”的习惯一致。
 * 按物理位置判断（符号层上 0 键为数字 0）；滚动模式不受影响。
 */
#define SLOW_KEY_POSITION 37
#define FAST_KEY_POSITION 38

#ifndef CONFIG_A320_POLL_INTERVAL_MS
#define CONFIG_A320_POLL_INTERVAL_MS 2
#endif

#ifndef CONFIG_A320_TRACKPAD_SCROLL_INTERVAL
#define CONFIG_A320_TRACKPAD_SCROLL_INTERVAL 50
#endif

/* 手指停住或抬起超过这个时间，丢弃不足一格的剩余滚动量 */
#define SCROLL_IDLE_RESET_MS 200

/* =========================
 * HID indicators
 * ========================= */

static zmk_hid_indicators_t current_indicators;
#define HID_INDICATORS_CAPS_LOCK (1 << 1)
#define HID_INDICATORS_SCROLL_LOCK (1 << 2)
/* 触控板滚动模式：Scroll Lock（Call 键）或 CapsLock 打开时 */
#define HID_INDICATORS_SCROLL_MODE (HID_INDICATORS_CAPS_LOCK | HID_INDICATORS_SCROLL_LOCK)

/* =========================
 * Motion GPIO
 * ========================= */

#define MOTION_GPIO_NODE DT_NODELABEL(gpio1)
#define MOTION_GPIO_PIN 1
static const struct device *motion_gpio_dev;

/* =========================
 * State flags
 * ========================= */

static bool touched = false;
static bool slow_key_pressed = false;
static bool fast_key_pressed = false;
static bool shift_pressed = false;

static bool arrow_pressed = false;
static int64_t last_arrow_time = 0;
static const struct device *trackpoint_dev_ref = NULL;

static int32_t scroll_remainder = 0;
static int64_t scroll_last_motion_ms = 0;

static int16_t slow_remainder_x = 0;
static int16_t slow_remainder_y = 0;

static struct k_work_delayable arrow_release_work;

/* =========================
 * Data & Config structs
 * ========================= */

struct a320_dev_config {
    struct i2c_dt_spec i2c;
};

typedef int (*a320_read_fn_t)(const struct device *dev, int16_t *dx, int16_t *dy);

struct a320_data {
    const struct device *dev;
    struct k_work_delayable poll_work;
    a320_read_fn_t read_motion;
};

/* =========================
 * Arrow release work
 * ========================= */

/*static void arrow_release_work_handler(struct k_work *work) {
    if (arrow_pressed && trackpoint_dev_ref) {

        input_report_key(trackpoint_dev_ref, INPUT_BTN_1, 0, true, K_FOREVER);
        input_report_key(trackpoint_dev_ref, INPUT_BTN_1, 0, true, K_FOREVER);
        input_report_key(trackpoint_dev_ref, INPUT_BTN_1, 0, true, K_FOREVER);
        input_report_key(trackpoint_dev_ref, INPUT_BTN_1, 0, true, K_FOREVER);

        arrow_pressed = false;
    }
}
*/
/* =========================
 * Key listener (Ctrl + Shift)
 * ========================= */

static int key_listener_cb(const zmk_event_t *eh) {

    const struct zmk_position_state_changed *ev = as_zmk_position_state_changed(eh);

    if (!ev)
        return 0;

    if (ev->position == SLOW_KEY_POSITION) {
        slow_key_pressed = ev->state;
    } else if (ev->position == FAST_KEY_POSITION) {
        fast_key_pressed = ev->state;
    }

    /*  if (ev->position == 27) {
          shift_pressed = ev->state;
      }
  */
    return 0;
}

ZMK_LISTENER(a320_key_listener, key_listener_cb);
ZMK_SUBSCRIPTION(a320_key_listener, zmk_position_state_changed);

/* =========================
 * HID indicator listener
 * ========================= */

static int hid_indicators_listener(const zmk_event_t *eh) {
    const struct zmk_hid_indicators_changed *ev = as_zmk_hid_indicators_changed(eh);

    if (ev)
        current_indicators = ev->indicators;

    return ZMK_EV_EVENT_BUBBLE;
}

/* =========================
 * I2C read variants
 * ========================= */

static int a320_read_motion_3b(const struct device *dev, int16_t *dx, int16_t *dy) {
    const struct a320_dev_config *cfg = dev->config;
    uint8_t buf[3];
    uint8_t reg = 0x82;

    if (i2c_write_dt(&cfg->i2c, &reg, 1) < 0)
        return -EIO;

    if (i2c_burst_read_dt(&cfg->i2c, 0x82, buf, sizeof(buf)) < 0)
        return -EIO;

    *dx = -(int8_t)buf[2];
    *dy = -(int8_t)buf[1];

    return 0;
}

static int a320_read_motion_37(const struct device *dev, int16_t *dx, int16_t *dy) {
    const struct a320_dev_config *cfg = dev->config;
    uint8_t buf[7];
    uint8_t reg = 0x0A;

    if (i2c_write_dt(&cfg->i2c, &reg, 1) < 0)
        return -EIO;

    if (i2c_burst_read_dt(&cfg->i2c, 0x0A, buf, sizeof(buf)) < 0)
        return -EIO;

    *dy = (int8_t)buf[3];
    *dx = -(int8_t)buf[1];

    return 0;
}

/* =========================
 * Scroll accumulation
 * ========================= */

/*
 * 滚动模式下累积垂直位移，每满 CONFIG_A320_TRACKPAD_SCROLL_INTERVAL 个计数滚一格，
 * 不足一格的余量留到下次，慢速滑动也能滚动。返回本次要发送的滚轮格数。
 */
static int32_t a320_scroll_steps(int16_t dy, int64_t now) {
    int32_t per_step = CONFIG_A320_TRACKPAD_SCROLL_INTERVAL;

    if (now - scroll_last_motion_ms > SCROLL_IDLE_RESET_MS) {
        scroll_remainder = 0;
    }
    scroll_last_motion_ms = now;

    /* 手指上移 = 滚轮向上；想反向就把 -= 改成 += */
    scroll_remainder -= dy;

    int32_t steps = scroll_remainder / per_step;
    scroll_remainder -= steps * per_step;
    return steps;
}

/* =========================
 * Pointer speed keys
 * ========================= */

/* 位移减半，奇数余量留到下一次，慢慢移动时不会被截断成 0 */
static int16_t a320_halve_with_carry(int16_t v, int16_t *remainder) {
    int32_t total = v + *remainder;
    int32_t out = total / 2;

    *remainder = total - out * 2;
    return out;
}

/* 按住 Shift 减速一半、按住 Ctrl 加速一倍，两个都按住时不变 */
static void a320_apply_speed_keys(int16_t *dx, int16_t *dy) {
    if (slow_key_pressed && !fast_key_pressed) {
        *dx = a320_halve_with_carry(*dx, &slow_remainder_x);
        *dy = a320_halve_with_carry(*dy, &slow_remainder_y);
        return;
    }

    slow_remainder_x = 0;
    slow_remainder_y = 0;

    if (fast_key_pressed && !slow_key_pressed) {
        *dx *= 2;
        *dy *= 2;
    }
}

/* =========================
 * Poll work
 * ========================= */

static void a320_poll_work_handler(struct k_work *work) {
    struct k_work_delayable *dwork = CONTAINER_OF(work, struct k_work_delayable, work);
    struct a320_data *data = CONTAINER_OF(dwork, struct a320_data, poll_work);

    int pin_state = gpio_pin_get(motion_gpio_dev, MOTION_GPIO_PIN);

    if (pin_state == 0) {

        int16_t dx = 0, dy = 0;

        if (data->read_motion(data->dev, &dx, &dy) == 0 && (dx || dy)) {

            bool scroll_mode = current_indicators & HID_INDICATORS_SCROLL_MODE;

            /* ===== SHIFT → ARROW MODE ===== */
            /*
            if (shift_pressed) {

                int64_t now = k_uptime_get();

                if (!arrow_pressed && (now - last_arrow_time >= SHIFT_ARROW_DEBOUNCE_MS)) {

                    uint16_t keycode;

                    if (abs(dx) > abs(dy)) {
                        keycode = (dx > 0) ? INPUT_BTN_1 : INPUT_BTN_1;
                    } else {
                        keycode = (dy > 0) ? INPUT_BTN_1 : INPUT_BTN_1;
                    }

                    input_report_key(data->dev, keycode, 1, true, K_FOREVER);

                    arrow_pressed = true;
                    trackpoint_dev_ref = data->dev;
                    last_arrow_time = now;

                    k_work_schedule(&arrow_release_work, K_MSEC(SHIFT_ARROW_RELEASE_MS));
                }

                goto reschedule;
            }
*/
            /* ===== Normal mouse mode ===== */

            if (!scroll_mode) {
                uint8_t brt = indicator_tp_get_last_valid_brightness();
                float factor = 0.4f + 0.01f * brt;

                dx = dx * 3 / 2 * factor;
                dy = dy * 3 / 2 * factor;
            }

            if (scroll_mode) {
                /* Shift、Ctrl 保留 Windows 的滚轮含义（横向滚动、缩放），不调整速度 */
                int32_t steps = a320_scroll_steps(dy, k_uptime_get());

                /* 不足一格不发送，避免每 2 ms 一个空报告挤占蓝牙队列 */
                if (steps != 0) {
                    input_report_rel(data->dev, INPUT_REL_WHEEL, steps, true, K_FOREVER);
                }
            } else {
                a320_apply_speed_keys(&dx, &dy);
                touched = true;

                if (dx || dy) {
                    input_report_rel(data->dev, INPUT_REL_X, dx, false, K_FOREVER);
                    input_report_rel(data->dev, INPUT_REL_Y, dy, true, K_FOREVER);
                }
            }
        }
    } else {
        touched = false;
    }

reschedule:
    k_work_reschedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));
}

bool tp_is_touched(void) { return touched; }

/* =========================
 * Init
 * ========================= */

static int a320_init(const struct device *dev) {
    const struct a320_dev_config *cfg = dev->config;
    struct a320_data *data = dev->data;

    if (!device_is_ready(cfg->i2c.bus))
        return -ENODEV;

    motion_gpio_dev = DEVICE_DT_GET(MOTION_GPIO_NODE);
    if (!device_is_ready(motion_gpio_dev))
        return -ENODEV;

    gpio_pin_configure(motion_gpio_dev, MOTION_GPIO_PIN, GPIO_INPUT | GPIO_PULL_UP);

    switch (cfg->i2c.addr) {
    case 0x3B:
        data->read_motion = a320_read_motion_3b;
        break;
    case 0x37:
        data->read_motion = a320_read_motion_37;
        break;
    default:
        LOG_ERR("Unsupported A320 addr: 0x%02X", cfg->i2c.addr);
        return -ENODEV;
    }

    data->dev = dev;

    k_work_init_delayable(&data->poll_work, a320_poll_work_handler);

    // k_work_init_delayable(&arrow_release_work, arrow_release_work_handler);

    k_work_schedule(&data->poll_work, K_MSEC(CONFIG_A320_POLL_INTERVAL_MS));

    LOG_INF("A320 init OK addr=0x%02X", cfg->i2c.addr);

    return 0;
}

/* =========================
 * Device define
 * ========================= */

#define A320_INIT_PRIORITY CONFIG_INPUT_A320_INIT_PRIORITY

#define A320_DEFINE(inst)                                                                          \
    static struct a320_data a320_data_##inst;                                                      \
    static const struct a320_dev_config a320_cfg_##inst = {                                        \
        .i2c = I2C_DT_SPEC_INST_GET(inst),                                                         \
    };                                                                                             \
    DEVICE_DT_INST_DEFINE(inst, a320_init, NULL, &a320_data_##inst, &a320_cfg_##inst, POST_KERNEL, \
                          A320_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(A320_DEFINE)

ZMK_LISTENER(a320_hid_listener, hid_indicators_listener);
ZMK_SUBSCRIPTION(a320_hid_listener, zmk_hid_indicators_changed);
