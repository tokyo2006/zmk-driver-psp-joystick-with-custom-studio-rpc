/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Map a raw ADC reading to [-32767, 32767] using the calibrated center and
 * the positive/negative half-ranges. Returns 0 if a half-range is degenerate.
 */
int16_t psp_joystick_normalize(int32_t raw, int32_t center, int32_t min, int32_t max);

/*
 * Zero the magnitude inside deadzone_percent of full scale, then rescale the
 * remaining range so full deflection still maps to +/-32767.
 */
int16_t psp_joystick_apply_deadzone(int16_t norm, uint8_t deadzone_percent);

/*
 * Shape the magnitude by exponent_x100 / 100. Supported exponents are exactly
 * 100 (linear), 150, 200, 250, and 300. Unknown values fall back to linear.
 * Output is monotonic and maps 0 -> 0 and +/-32767 -> +/-32767.
 */
int16_t psp_joystick_apply_curve(int16_t norm, uint16_t exponent_x100);

/*
 * Scale the shaped value to REL units per sample and keep the fractional part
 * in *remainder (Q16.16). The remainder must be initialized to 0 and reset
 * when the axis returns to rest.
 */
int16_t psp_joystick_apply_speed(int16_t norm, uint8_t max_speed, int32_t *remainder);

/* Apply swap first, then invert X, then invert Y. */
void psp_joystick_apply_transforms(int16_t *x, int16_t *y, bool swap_xy, bool invert_x,
                                   bool invert_y);

/* Integer floor(sqrt(v)); exposed for tests. */
uint64_t psp_joystick_isqrt64(uint64_t v);

#ifdef __cplusplus
}
#endif
