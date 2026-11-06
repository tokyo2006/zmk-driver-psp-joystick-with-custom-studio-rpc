/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
#include <cormoran/zmk/custom_settings.h>
#endif

#include <tokyo2006/psp-joystick/psp_joystick_settings.h>
#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PSP_NS "tokyo2006__psp_joystick"

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)

#define PSP_INT_SETTING(_name, _key, _default)                                                     \
    ZMK_CUSTOM_SETTING_DEFINE(                                                                     \
        psp_setting_##_name, PSP_NS, _key, ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,                    \
        ZMK_CUSTOM_SETTING_VALUE_INT32(_default), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,   \
        ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,            \
        ZMK_CUSTOM_SETTING_NO_CONSTRAINT)

#define PSP_BOOL_SETTING(_name, _key, _default)                                                    \
    ZMK_CUSTOM_SETTING_DEFINE(                                                                     \
        psp_setting_##_name, PSP_NS, _key, ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL,                     \
        ZMK_CUSTOM_SETTING_VALUE_BOOL(_default), ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,    \
        ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,            \
        ZMK_CUSTOM_SETTING_NO_CONSTRAINT)

PSP_INT_SETTING(deadzone_percent, "deadzone_percent", PSP_JOYSTICK_DEADZONE_PERCENT_DEFAULT);
PSP_INT_SETTING(max_speed, "max_speed", PSP_JOYSTICK_MAX_SPEED_DEFAULT);
PSP_INT_SETTING(curve_exponent_x100, "curve_exponent_x100",
                PSP_JOYSTICK_CURVE_EXPONENT_X100_DEFAULT);
PSP_INT_SETTING(smoothing_percent, "smoothing_percent", PSP_JOYSTICK_SMOOTHING_PERCENT_DEFAULT);
PSP_INT_SETTING(sample_interval_ms, "sample_interval_ms", PSP_JOYSTICK_SAMPLE_INTERVAL_MS_DEFAULT);
PSP_BOOL_SETTING(invert_x, "invert_x", false);
PSP_BOOL_SETTING(invert_y, "invert_y", false);
PSP_BOOL_SETTING(swap_xy, "swap_xy", false);
PSP_INT_SETTING(center_x, "center_x", PSP_JOYSTICK_CENTER_DEFAULT);
PSP_INT_SETTING(center_y, "center_y", PSP_JOYSTICK_CENTER_DEFAULT);
PSP_INT_SETTING(min_x, "min_x", PSP_JOYSTICK_MIN_DEFAULT);
PSP_INT_SETTING(max_x, "max_x", PSP_JOYSTICK_MAX_DEFAULT);
PSP_INT_SETTING(min_y, "min_y", PSP_JOYSTICK_MIN_DEFAULT);
PSP_INT_SETTING(max_y, "max_y", PSP_JOYSTICK_MAX_DEFAULT);
PSP_BOOL_SETTING(arrow_mode, "arrow_mode", PSP_JOYSTICK_ARROW_MODE_DEFAULT);
PSP_INT_SETTING(arrow_threshold_percent, "arrow_threshold_percent",
                PSP_JOYSTICK_ARROW_THRESHOLD_PERCENT_DEFAULT);
PSP_BOOL_SETTING(center_valid, "center_valid", false);

int psp_joystick_settings_install_dt_defaults(const struct device *dev) {
    if (dev == NULL) {
        return 0;
    }

#define INSTALL_INT(_name, _getter)                                                                \
    do {                                                                                           \
        static struct zmk_custom_setting_value _name##_dt;                                         \
        _name##_dt =                                                                               \
            (struct zmk_custom_setting_value)ZMK_CUSTOM_SETTING_VALUE_INT32(_getter(dev));         \
        (void)zmk_custom_setting_set_default(&psp_setting_##_name, &_name##_dt);                   \
    } while (0)

#define INSTALL_BOOL(_name, _getter)                                                               \
    do {                                                                                           \
        static struct zmk_custom_setting_value _name##_dt;                                         \
        _name##_dt = (struct zmk_custom_setting_value)ZMK_CUSTOM_SETTING_VALUE_BOOL(_getter(dev)); \
        (void)zmk_custom_setting_set_default(&psp_setting_##_name, &_name##_dt);                   \
    } while (0)

    INSTALL_INT(deadzone_percent, psp_joystick_state_get_deadzone_percent);
    INSTALL_INT(max_speed, psp_joystick_state_get_max_speed);
    INSTALL_INT(curve_exponent_x100, psp_joystick_state_get_curve_exponent_x100);
    INSTALL_INT(smoothing_percent, psp_joystick_state_get_smoothing_percent);
    INSTALL_INT(sample_interval_ms, psp_joystick_state_get_sample_interval_ms);
    INSTALL_BOOL(invert_x, psp_joystick_state_get_invert_x);
    INSTALL_BOOL(invert_y, psp_joystick_state_get_invert_y);
    INSTALL_BOOL(swap_xy, psp_joystick_state_get_swap_xy);
    INSTALL_INT(center_x, psp_joystick_state_get_center_x);
    INSTALL_INT(center_y, psp_joystick_state_get_center_y);
    INSTALL_INT(min_x, psp_joystick_state_get_min_x);
    INSTALL_INT(max_x, psp_joystick_state_get_max_x);
    INSTALL_INT(min_y, psp_joystick_state_get_min_y);
    INSTALL_INT(max_y, psp_joystick_state_get_max_y);
    INSTALL_BOOL(arrow_mode, psp_joystick_state_get_arrow_mode);
    INSTALL_INT(arrow_threshold_percent, psp_joystick_state_get_arrow_threshold_percent);

#undef INSTALL_INT
#undef INSTALL_BOOL
    return 0;
}

int psp_joystick_settings_save_all(const struct device *dev, bool include_calibration) {
    int first_rc = 0;
    int rc;

#define SAVE_INT(name, getter)                                                                     \
    do {                                                                                           \
        rc = zmk_custom_setting_set_int32(&psp_setting_##name, (int32_t)getter(dev),               \
                                          ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);                  \
        if (rc != 0 && first_rc == 0) {                                                            \
            first_rc = rc;                                                                         \
            LOG_WRN("save %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

#define SAVE_BOOL(name, getter)                                                                    \
    do {                                                                                           \
        rc = zmk_custom_setting_set_bool(&psp_setting_##name, getter(dev),                         \
                                         ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);                   \
        if (rc != 0 && first_rc == 0) {                                                            \
            first_rc = rc;                                                                         \
            LOG_WRN("save %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

    SAVE_INT(deadzone_percent, psp_joystick_state_get_deadzone_percent);
    SAVE_INT(max_speed, psp_joystick_state_get_max_speed);
    SAVE_INT(curve_exponent_x100, psp_joystick_state_get_curve_exponent_x100);
    SAVE_INT(smoothing_percent, psp_joystick_state_get_smoothing_percent);
    SAVE_INT(sample_interval_ms, psp_joystick_state_get_sample_interval_ms);
    SAVE_BOOL(invert_x, psp_joystick_state_get_invert_x);
    SAVE_BOOL(invert_y, psp_joystick_state_get_invert_y);
    SAVE_BOOL(swap_xy, psp_joystick_state_get_swap_xy);
    SAVE_BOOL(arrow_mode, psp_joystick_state_get_arrow_mode);
    SAVE_INT(arrow_threshold_percent, psp_joystick_state_get_arrow_threshold_percent);

    /* Calibration is only meaningful on the half that owns the ADC. */
    if (include_calibration && dev != NULL) {
        SAVE_INT(center_x, psp_joystick_state_get_center_x);
        SAVE_INT(center_y, psp_joystick_state_get_center_y);
        SAVE_INT(min_x, psp_joystick_state_get_min_x);
        SAVE_INT(max_x, psp_joystick_state_get_max_x);
        SAVE_INT(min_y, psp_joystick_state_get_min_y);
        SAVE_INT(max_y, psp_joystick_state_get_max_y);

        if (first_rc == 0) {
            rc = zmk_custom_setting_set_bool(&psp_setting_center_valid, true,
                                             ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);
            if (rc != 0) {
                first_rc = rc;
                LOG_WRN("save center_valid failed: %d", rc);
            }
        }
    }

#undef SAVE_INT
#undef SAVE_BOOL

    return first_rc;
}

int psp_joystick_settings_reset_all(const struct device *dev) {
    ARG_UNUSED(dev);
    uint32_t affected = 0;
    int rc = zmk_custom_settings_reset_scope(PSP_NS, NULL, NULL, &affected);
    if (rc != 0) {
        LOG_WRN("reset scope failed: %d (affected=%u)", rc, affected);
    }
    return rc;
}

int psp_joystick_settings_load_cb(const struct device *dev) {
    int rc;
    int32_t value;

#define LOAD_INT(name, setter)                                                                     \
    do {                                                                                           \
        rc = zmk_custom_setting_get_int32(&psp_setting_##name, &value);                            \
        if (rc == 0) {                                                                             \
            (void)setter(dev, value);                                                              \
        } else if (rc != -ENOENT) {                                                                \
            LOG_WRN("load %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

#define LOAD_BOOL(name, setter)                                                                    \
    do {                                                                                           \
        bool b;                                                                                    \
        rc = zmk_custom_setting_get_bool(&psp_setting_##name, &b);                                 \
        if (rc == 0) {                                                                             \
            (void)setter(dev, b);                                                                  \
        } else if (rc != -ENOENT) {                                                                \
            LOG_WRN("load %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

    LOAD_INT(deadzone_percent, psp_joystick_state_set_deadzone_percent);
    LOAD_INT(max_speed, psp_joystick_state_set_max_speed);
    LOAD_INT(curve_exponent_x100, psp_joystick_state_set_curve_exponent_x100);
    LOAD_INT(smoothing_percent, psp_joystick_state_set_smoothing_percent);
    LOAD_INT(sample_interval_ms, psp_joystick_state_set_sample_interval_ms);
    LOAD_BOOL(invert_x, psp_joystick_state_set_invert_x);
    LOAD_BOOL(invert_y, psp_joystick_state_set_invert_y);
    LOAD_BOOL(swap_xy, psp_joystick_state_set_swap_xy);
    LOAD_INT(center_x, psp_joystick_state_set_center_x);
    LOAD_INT(center_y, psp_joystick_state_set_center_y);
    LOAD_INT(min_x, psp_joystick_state_set_min_x);
    LOAD_INT(max_x, psp_joystick_state_set_max_x);
    LOAD_INT(min_y, psp_joystick_state_set_min_y);
    LOAD_INT(max_y, psp_joystick_state_set_max_y);
    LOAD_BOOL(arrow_mode, psp_joystick_state_set_arrow_mode);
    LOAD_INT(arrow_threshold_percent, psp_joystick_state_set_arrow_threshold_percent);

#undef LOAD_INT
#undef LOAD_BOOL

    bool center_valid = false;
    rc = zmk_custom_setting_get_bool(&psp_setting_center_valid, &center_valid);
    if (rc != 0) {
        center_valid = false;
    }
    if (dev != NULL && psp_joystick_driver_on_settings_loaded != NULL) {
        psp_joystick_driver_on_settings_loaded(dev, center_valid);
    }
    return 0;
}

static int psp_joystick_settings_on_initialized(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    const struct device *dev = DEVICE_DT_GET_ANY(tokyo2006_psp_joystick);
    (void)psp_joystick_settings_load_cb(dev);
    (void)psp_joystick_state_apply_all(dev);
    return ZMK_EV_EVENT_HANDLED;
}

ZMK_LISTENER(psp_joystick_settings_listener, psp_joystick_settings_on_initialized);
ZMK_SUBSCRIPTION(psp_joystick_settings_listener, zmk_custom_settings_initialized);

#else /* !CONFIG_ZMK_CUSTOM_SETTINGS */

int psp_joystick_settings_save_all(const struct device *dev, bool include_calibration) {
    ARG_UNUSED(dev);
    ARG_UNUSED(include_calibration);
    return 0;
}

int psp_joystick_settings_install_dt_defaults(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

int psp_joystick_settings_reset_all(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

int psp_joystick_settings_load_cb(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

#endif /* CONFIG_ZMK_CUSTOM_SETTINGS */
