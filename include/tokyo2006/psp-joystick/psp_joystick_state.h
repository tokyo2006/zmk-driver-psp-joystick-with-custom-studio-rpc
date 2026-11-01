/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zmk/event_manager.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSP_JOYSTICK_DEADZONE_PERCENT_DEFAULT 8
#define PSP_JOYSTICK_MAX_SPEED_DEFAULT 8
#define PSP_JOYSTICK_CURVE_EXPONENT_X100_DEFAULT 100
#define PSP_JOYSTICK_SMOOTHING_PERCENT_DEFAULT 0
#define PSP_JOYSTICK_SAMPLE_INTERVAL_MS_DEFAULT 8
#define PSP_JOYSTICK_CENTER_DEFAULT 2048
#define PSP_JOYSTICK_MIN_DEFAULT 0
#define PSP_JOYSTICK_MAX_DEFAULT 4095

struct psp_joystick_runtime_state {
    uint8_t deadzone_percent;
    uint8_t max_speed;
    uint16_t curve_exponent_x100;
    uint8_t smoothing_percent;
    uint8_t sample_interval_ms;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    int16_t center_x;
    int16_t center_y;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
};

struct zmk_psp_joystick_status_changed {
    uint8_t reserved;
};

ZMK_EVENT_DECLARE(zmk_psp_joystick_status_changed);

struct psp_joystick_runtime_state *psp_joystick_state_get_struct(const struct device *dev);

uint8_t psp_joystick_state_get_deadzone_percent(const struct device *dev);
uint8_t psp_joystick_state_get_max_speed(const struct device *dev);
uint16_t psp_joystick_state_get_curve_exponent_x100(const struct device *dev);
uint8_t psp_joystick_state_get_smoothing_percent(const struct device *dev);
uint8_t psp_joystick_state_get_sample_interval_ms(const struct device *dev);
bool psp_joystick_state_get_invert_x(const struct device *dev);
bool psp_joystick_state_get_invert_y(const struct device *dev);
bool psp_joystick_state_get_swap_xy(const struct device *dev);
int16_t psp_joystick_state_get_center_x(const struct device *dev);
int16_t psp_joystick_state_get_center_y(const struct device *dev);
int16_t psp_joystick_state_get_min_x(const struct device *dev);
int16_t psp_joystick_state_get_max_x(const struct device *dev);
int16_t psp_joystick_state_get_min_y(const struct device *dev);
int16_t psp_joystick_state_get_max_y(const struct device *dev);

int psp_joystick_state_set_deadzone_percent(const struct device *dev, int32_t v);
int psp_joystick_state_set_max_speed(const struct device *dev, int32_t v);
int psp_joystick_state_set_curve_exponent_x100(const struct device *dev, int32_t v);
int psp_joystick_state_set_smoothing_percent(const struct device *dev, int32_t v);
int psp_joystick_state_set_sample_interval_ms(const struct device *dev, int32_t v);
int psp_joystick_state_set_invert_x(const struct device *dev, bool v);
int psp_joystick_state_set_invert_y(const struct device *dev, bool v);
int psp_joystick_state_set_swap_xy(const struct device *dev, bool v);
int psp_joystick_state_set_center_x(const struct device *dev, int32_t v);
int psp_joystick_state_set_center_y(const struct device *dev, int32_t v);
int psp_joystick_state_set_min_x(const struct device *dev, int32_t v);
int psp_joystick_state_set_max_x(const struct device *dev, int32_t v);
int psp_joystick_state_set_min_y(const struct device *dev, int32_t v);
int psp_joystick_state_set_max_y(const struct device *dev, int32_t v);

int psp_joystick_state_load_defaults(const struct device *dev);
int psp_joystick_state_apply_all(const struct device *dev);

/*
 * Driver hooks. The state layer links without a driver (Studio-only builds,
 * split halves without the stick) because these are weak declarations.
 */
struct psp_joystick_runtime_state *psp_joystick_driver_get_state(const struct device *dev)
    __attribute__((weak));
int psp_joystick_driver_load_defaults(const struct device *dev) __attribute__((weak));
int psp_joystick_driver_apply_state(const struct device *dev) __attribute__((weak));
int psp_joystick_driver_calibrate(const struct device *dev, int32_t *center_x, int32_t *center_y)
    __attribute__((weak));
void psp_joystick_driver_on_settings_loaded(const struct device *dev, bool center_persisted)
    __attribute__((weak));

#ifdef __cplusplus
}
#endif
