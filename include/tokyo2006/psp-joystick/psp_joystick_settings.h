/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

int psp_joystick_settings_save_all(const struct device *dev);
int psp_joystick_settings_reset_all(const struct device *dev);
int psp_joystick_settings_load_cb(const struct device *dev);

#ifdef __cplusplus
}
#endif
