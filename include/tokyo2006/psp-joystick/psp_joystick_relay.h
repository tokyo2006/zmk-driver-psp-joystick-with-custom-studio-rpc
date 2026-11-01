/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zmk/event_manager.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSP_JOYSTICK_RELAY_STATE_VERSION 1

struct psp_joystick_relay_state {
    uint8_t source;
    uint8_t version;
    uint8_t deadzone_percent;
    uint8_t max_speed;
    uint16_t curve_exponent_x100;
    uint8_t smoothing_percent;
    uint8_t sample_interval_ms;
    /* bit0 invert_x, bit1 invert_y, bit2 swap_xy, bit3 calibration present */
    uint8_t flags;
    int16_t center_x;
    int16_t center_y;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
} __packed;

ZMK_EVENT_DECLARE(psp_joystick_relay_state);

void psp_joystick_relay_send_state(const struct device *dev);

#ifdef __cplusplus
}
#endif
