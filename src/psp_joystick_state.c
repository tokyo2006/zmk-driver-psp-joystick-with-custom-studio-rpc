/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(zmk_psp_joystick_status_changed);

static struct psp_joystick_runtime_state fallback_state;
static bool fallback_initialized;

static void set_hard_defaults(struct psp_joystick_runtime_state *s) {
    s->deadzone_percent = PSP_JOYSTICK_DEADZONE_PERCENT_DEFAULT;
    s->max_speed = PSP_JOYSTICK_MAX_SPEED_DEFAULT;
    s->curve_exponent_x100 = PSP_JOYSTICK_CURVE_EXPONENT_X100_DEFAULT;
    s->smoothing_percent = PSP_JOYSTICK_SMOOTHING_PERCENT_DEFAULT;
    s->sample_interval_ms = PSP_JOYSTICK_SAMPLE_INTERVAL_MS_DEFAULT;
    s->invert_x = false;
    s->invert_y = false;
    s->swap_xy = false;
    s->center_x = PSP_JOYSTICK_CENTER_DEFAULT;
    s->center_y = PSP_JOYSTICK_CENTER_DEFAULT;
    s->min_x = PSP_JOYSTICK_MIN_DEFAULT;
    s->max_x = PSP_JOYSTICK_MAX_DEFAULT;
    s->min_y = PSP_JOYSTICK_MIN_DEFAULT;
    s->max_y = PSP_JOYSTICK_MAX_DEFAULT;
}

struct psp_joystick_runtime_state *psp_joystick_state_get_struct(const struct device *dev) {
    if (dev == NULL || psp_joystick_driver_get_state == NULL) {
        if (!fallback_initialized) {
            set_hard_defaults(&fallback_state);
            fallback_initialized = true;
        }
        return &fallback_state;
    }
    return psp_joystick_driver_get_state(dev);
}

static void notify_changed(void) {
    struct zmk_psp_joystick_status_changed ev = {0};
    raise_zmk_psp_joystick_status_changed(ev);
}

#define DEFINE_GETTER(type, name, field)                                                           \
    type psp_joystick_state_get_##name(const struct device *dev) {                                 \
        return psp_joystick_state_get_struct(dev)->field;                                          \
    }

#define DEFINE_SETTER(type, name, field, validate_expr)                                            \
    int psp_joystick_state_set_##name(const struct device *dev, type v) {                          \
        if (!(validate_expr)) {                                                                    \
            return -EINVAL;                                                                        \
        }                                                                                          \
        psp_joystick_state_get_struct(dev)->field = v;                                             \
        notify_changed();                                                                          \
        return 0;                                                                                  \
    }

#define VALID_U8(v, max) ((uint32_t)(v) <= (uint32_t)(max))
#define VALID_ADC_RAW(v) ((int32_t)(v) >= 0 && (int32_t)(v) <= 4095)
#define VALID_CURVE(v) ((v) == 100 || (v) == 150 || (v) == 200 || (v) == 250 || (v) == 300)

DEFINE_GETTER(uint8_t, deadzone_percent, deadzone_percent)
DEFINE_GETTER(uint8_t, max_speed, max_speed)
DEFINE_GETTER(uint16_t, curve_exponent_x100, curve_exponent_x100)
DEFINE_GETTER(uint8_t, smoothing_percent, smoothing_percent)
DEFINE_GETTER(uint8_t, sample_interval_ms, sample_interval_ms)
DEFINE_GETTER(bool, invert_x, invert_x)
DEFINE_GETTER(bool, invert_y, invert_y)
DEFINE_GETTER(bool, swap_xy, swap_xy)
DEFINE_GETTER(int16_t, center_x, center_x)
DEFINE_GETTER(int16_t, center_y, center_y)
DEFINE_GETTER(int16_t, min_x, min_x)
DEFINE_GETTER(int16_t, max_x, max_x)
DEFINE_GETTER(int16_t, min_y, min_y)
DEFINE_GETTER(int16_t, max_y, max_y)

DEFINE_SETTER(int32_t, deadzone_percent, deadzone_percent, VALID_U8(v, 50))
DEFINE_SETTER(int32_t, max_speed, max_speed, ((v) >= 1 && (v) <= 64))
DEFINE_SETTER(int32_t, curve_exponent_x100, curve_exponent_x100, VALID_CURVE(v))
DEFINE_SETTER(int32_t, smoothing_percent, smoothing_percent, VALID_U8(v, 90))
DEFINE_SETTER(int32_t, sample_interval_ms, sample_interval_ms, ((v) >= 4 && (v) <= 50))
DEFINE_SETTER(bool, invert_x, invert_x, true)
DEFINE_SETTER(bool, invert_y, invert_y, true)
DEFINE_SETTER(bool, swap_xy, swap_xy, true)
DEFINE_SETTER(int32_t, center_x, center_x, VALID_ADC_RAW(v))
DEFINE_SETTER(int32_t, center_y, center_y, VALID_ADC_RAW(v))
DEFINE_SETTER(int32_t, min_x, min_x, VALID_ADC_RAW(v))
DEFINE_SETTER(int32_t, max_x, max_x, VALID_ADC_RAW(v))
DEFINE_SETTER(int32_t, min_y, min_y, VALID_ADC_RAW(v))
DEFINE_SETTER(int32_t, max_y, max_y, VALID_ADC_RAW(v))

int psp_joystick_state_load_defaults(const struct device *dev) {
    if (dev != NULL && psp_joystick_driver_load_defaults != NULL) {
        return psp_joystick_driver_load_defaults(dev);
    }
    set_hard_defaults(psp_joystick_state_get_struct(dev));
    return 0;
}

int psp_joystick_state_apply_all(const struct device *dev) {
    if (dev != NULL && psp_joystick_driver_apply_state != NULL) {
        return psp_joystick_driver_apply_state(dev);
    }
    return 0;
}
