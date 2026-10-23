/*
 * Trackpad click behavior: a short press clicks the first binding, moving the trackpad while it is
 * pressed holds the first binding (drag), and holding the trackpad still for long-press-ms clicks
 * the second binding.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_trackpad_click

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>

#include "a320.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define DECIDE_INTERVAL_MS 5

struct behavior_trackpad_click_config {
    const char *click_behavior_dev;
    const char *long_press_behavior_dev;
    int32_t long_press_ms;
    uint32_t move_threshold;
};

struct behavior_trackpad_click_data {
    struct behavior_parameter_metadata_set set;
};

enum trackpad_click_state {
    TRACKPAD_CLICK_IDLE,
    TRACKPAD_CLICK_UNDECIDED,
    TRACKPAD_CLICK_DRAGGING,
    TRACKPAD_CLICK_LONG_PRESSED,
};

/* There is only one trackpad, so a single press is tracked at a time */
static struct {
    enum trackpad_click_state state;
    const struct behavior_trackpad_click_config *config;
    struct zmk_behavior_binding_event event;
    uint32_t click_param;
    uint32_t long_press_param;
    uint32_t motion_at_press;
} active;

static struct k_work_delayable decide_work;

static void invoke(const char *behavior_dev, uint32_t param, bool pressed) {
    struct zmk_behavior_binding binding = {.behavior_dev = behavior_dev, .param1 = param};
    struct zmk_behavior_binding_event event = active.event;

    event.timestamp = k_uptime_get();
    zmk_behavior_invoke_binding(&binding, event, pressed);
}

static void decide_work_handler(struct k_work *work) {
    const struct behavior_trackpad_click_config *cfg = active.config;

    if (active.state != TRACKPAD_CLICK_UNDECIDED) {
        return;
    }

    if (a320_motion_counts() - active.motion_at_press > cfg->move_threshold) {
        LOG_DBG("trackpad click: drag");
        active.state = TRACKPAD_CLICK_DRAGGING;
        invoke(cfg->click_behavior_dev, active.click_param, true);
    } else if (k_uptime_get() - active.event.timestamp >= cfg->long_press_ms) {
        LOG_DBG("trackpad click: long press");
        active.state = TRACKPAD_CLICK_LONG_PRESSED;
        invoke(cfg->long_press_behavior_dev, active.long_press_param, true);
        invoke(cfg->long_press_behavior_dev, active.long_press_param, false);
    } else {
        k_work_reschedule(&decide_work, K_MSEC(DECIDE_INTERVAL_MS));
    }
}

static int on_trackpad_click_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);

    active.state = TRACKPAD_CLICK_UNDECIDED;
    active.config = dev->config;
    active.event = event;
    active.click_param = binding->param1;
    active.long_press_param = binding->param2;
    active.motion_at_press = a320_motion_counts();
    k_work_reschedule(&decide_work, K_MSEC(DECIDE_INTERVAL_MS));
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_trackpad_click_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    const struct behavior_trackpad_click_config *cfg = active.config;

    k_work_cancel_delayable(&decide_work);

    if (active.state == TRACKPAD_CLICK_UNDECIDED) {
        invoke(cfg->click_behavior_dev, active.click_param, true);
        invoke(cfg->click_behavior_dev, active.click_param, false);
    } else if (active.state == TRACKPAD_CLICK_DRAGGING) {
        invoke(cfg->click_behavior_dev, active.click_param, false);
    }

    active.state = TRACKPAD_CLICK_IDLE;
    return ZMK_BEHAVIOR_OPAQUE;
}

static int behavior_trackpad_click_init(const struct device *dev) {
    k_work_init_delayable(&decide_work, decide_work_handler);
    return 0;
}

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
/* Like hold-tap: param1 takes the click binding's values, param2 the long press binding's */
static int trackpad_click_parameter_metadata(const struct device *dev,
                                             struct behavior_parameter_metadata *param_metadata) {
    const struct behavior_trackpad_click_config *cfg = dev->config;
    struct behavior_trackpad_click_data *data = dev->data;
    struct behavior_parameter_metadata child_meta;
    int err;

    err = behavior_get_parameter_metadata(zmk_behavior_get_binding(cfg->click_behavior_dev),
                                          &child_meta);
    if (err < 0) {
        return err;
    }
    if (child_meta.sets_len > 0) {
        data->set.param1_values = child_meta.sets[0].param1_values;
        data->set.param1_values_len = child_meta.sets[0].param1_values_len;
    }

    err = behavior_get_parameter_metadata(zmk_behavior_get_binding(cfg->long_press_behavior_dev),
                                          &child_meta);
    if (err < 0) {
        return err;
    }
    if (child_meta.sets_len > 0) {
        data->set.param2_values = child_meta.sets[0].param1_values;
        data->set.param2_values_len = child_meta.sets[0].param1_values_len;
    }

    param_metadata->sets = &data->set;
    param_metadata->sets_len = 1;
    return 0;
}
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)

static const struct behavior_driver_api behavior_trackpad_click_driver_api = {
    .binding_pressed = on_trackpad_click_pressed,
    .binding_released = on_trackpad_click_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = trackpad_click_parameter_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define TRACKPAD_CLICK_INST(n)                                                                     \
    static const struct behavior_trackpad_click_config behavior_trackpad_click_config_##n = {      \
        .click_behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 0)),              \
        .long_press_behavior_dev = DEVICE_DT_NAME(DT_INST_PHANDLE_BY_IDX(n, bindings, 1)),         \
        .long_press_ms = DT_INST_PROP(n, long_press_ms),                                           \
        .move_threshold = DT_INST_PROP(n, move_threshold),                                         \
    };                                                                                             \
    static struct behavior_trackpad_click_data behavior_trackpad_click_data_##n = {};              \
    BEHAVIOR_DT_INST_DEFINE(n, behavior_trackpad_click_init, NULL,                                 \
                            &behavior_trackpad_click_data_##n,                                     \
                            &behavior_trackpad_click_config_##n, POST_KERNEL,                      \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_trackpad_click_driver_api);

DT_INST_FOREACH_STATUS_OKAY(TRACKPAD_CLICK_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
