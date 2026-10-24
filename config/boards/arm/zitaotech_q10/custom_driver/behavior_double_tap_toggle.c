/*
 * Double-tap toggle behavior: every press runs the first binding at once, so a single tap has no
 * delay. A second press within tapping-term-ms of the first runs the first binding again, which
 * undoes a toggle such as Scroll Lock, and then taps the second binding.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_double_tap_toggle

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <drivers/behavior.h>
#include <zmk/behavior.h>
#include <zmk/keymap.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

struct behavior_double_tap_toggle_config {
    struct zmk_behavior_binding tap_binding;
    struct zmk_behavior_binding double_tap_binding;
    int32_t tapping_term_ms;
};

struct behavior_double_tap_toggle_data {
    int64_t last_tap_ms;
    bool armed;    /* the last press was a single tap that a quick second press turns into a double */
    bool tap_held; /* the first binding stays pressed until the key is released */
};

static void tap_binding(const struct zmk_behavior_binding *binding,
                        struct zmk_behavior_binding_event event) {
    zmk_behavior_invoke_binding(binding, event, true);
    zmk_behavior_invoke_binding(binding, event, false);
}

static int on_double_tap_toggle_pressed(struct zmk_behavior_binding *binding,
                                        struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_double_tap_toggle_config *cfg = dev->config;
    struct behavior_double_tap_toggle_data *data = dev->data;

    if (data->armed && event.timestamp - data->last_tap_ms <= cfg->tapping_term_ms) {
        LOG_DBG("double tap");
        data->armed = false;
        data->tap_held = false;
        tap_binding(&cfg->tap_binding, event);
        tap_binding(&cfg->double_tap_binding, event);
        return ZMK_BEHAVIOR_OPAQUE;
    }

    data->armed = true;
    data->last_tap_ms = event.timestamp;
    data->tap_held = true;
    zmk_behavior_invoke_binding(&cfg->tap_binding, event, true);
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_double_tap_toggle_released(struct zmk_behavior_binding *binding,
                                         struct zmk_behavior_binding_event event) {
    const struct device *dev = zmk_behavior_get_binding(binding->behavior_dev);
    const struct behavior_double_tap_toggle_config *cfg = dev->config;
    struct behavior_double_tap_toggle_data *data = dev->data;

    if (data->tap_held) {
        data->tap_held = false;
        zmk_behavior_invoke_binding(&cfg->tap_binding, event, false);
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_double_tap_toggle_driver_api = {
    .binding_pressed = on_double_tap_toggle_pressed,
    .binding_released = on_double_tap_toggle_released,
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
    .get_parameter_metadata = zmk_behavior_get_empty_param_metadata,
#endif // IS_ENABLED(CONFIG_ZMK_BEHAVIOR_METADATA)
};

#define DOUBLE_TAP_TOGGLE_INST(n)                                                                  \
    static const struct behavior_double_tap_toggle_config                                          \
        behavior_double_tap_toggle_config_##n = {                                                  \
            .tap_binding = ZMK_KEYMAP_EXTRACT_BINDING(0, DT_DRV_INST(n)),                          \
            .double_tap_binding = ZMK_KEYMAP_EXTRACT_BINDING(1, DT_DRV_INST(n)),                   \
            .tapping_term_ms = DT_INST_PROP(n, tapping_term_ms),                                   \
    };                                                                                             \
    static struct behavior_double_tap_toggle_data behavior_double_tap_toggle_data_##n = {};        \
    BEHAVIOR_DT_INST_DEFINE(n, NULL, NULL, &behavior_double_tap_toggle_data_##n,                   \
                            &behavior_double_tap_toggle_config_##n, POST_KERNEL,                   \
                            CONFIG_KERNEL_INIT_PRIORITY_DEFAULT,                                   \
                            &behavior_double_tap_toggle_driver_api);

DT_INST_FOREACH_STATUS_OKAY(DOUBLE_TAP_TOGGLE_INST)

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */
