/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <tokyo2006/psp-joystick/psp_joystick_math.h>

#define PSP_HALF_SCALE 32767

static int32_t abs32(int32_t v) { return v < 0 ? -v : v; }

int16_t psp_joystick_normalize(int32_t raw, int32_t center, int32_t min, int32_t max) {
    int64_t delta = (int64_t)raw - center;
    int64_t denom = delta >= 0 ? (int64_t)max - center : (int64_t)center - min;

    if (denom <= 0) {
        return 0;
    }

    int64_t value = delta * PSP_HALF_SCALE / denom;
    if (value > PSP_HALF_SCALE) {
        value = PSP_HALF_SCALE;
    } else if (value < -PSP_HALF_SCALE) {
        value = -PSP_HALF_SCALE;
    }
    return (int16_t)value;
}

int16_t psp_joystick_apply_deadzone(int16_t norm, uint8_t deadzone_percent) {
    int32_t dz = (int32_t)deadzone_percent * PSP_HALF_SCALE / 100;
    int32_t magnitude = abs32(norm);

    if (dz == 0) {
        return norm;
    }
    if (dz >= PSP_HALF_SCALE || magnitude <= dz) {
        return 0;
    }

    int32_t scaled = (magnitude - dz) * PSP_HALF_SCALE / (PSP_HALF_SCALE - dz);
    return (int16_t)(norm < 0 ? -scaled : scaled);
}

uint64_t psp_joystick_isqrt64(uint64_t v) {
    uint64_t result = 0;
    uint64_t bit = 1ULL << 62;

    while (bit > v) {
        bit >>= 2;
    }
    while (bit != 0) {
        if (v >= result + bit) {
            v -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }
        bit >>= 2;
    }
    return result;
}

int16_t psp_joystick_apply_curve(int16_t norm, uint16_t exponent_x100) {
    uint64_t x = (uint64_t)abs32(norm);
    uint64_t y;

    switch (exponent_x100) {
    case 150:
        y = x * psp_joystick_isqrt64(x * PSP_HALF_SCALE) / PSP_HALF_SCALE;
        break;
    case 200:
        y = x * x / PSP_HALF_SCALE;
        break;
    case 250:
        y = x * x * psp_joystick_isqrt64(x * PSP_HALF_SCALE) /
            ((uint64_t)PSP_HALF_SCALE * PSP_HALF_SCALE);
        break;
    case 300:
        y = x * x * x / ((uint64_t)PSP_HALF_SCALE * PSP_HALF_SCALE);
        break;
    case 100:
    default:
        y = x;
        break;
    }

    if (y > PSP_HALF_SCALE) {
        y = PSP_HALF_SCALE;
    }
    return (int16_t)(norm < 0 ? -(int32_t)y : (int32_t)y);
}

int16_t psp_joystick_apply_speed(int16_t norm, uint8_t max_speed, int32_t *remainder) {
    if (max_speed == 0) {
        *remainder = 0;
        return 0;
    }

    int64_t target_q16 = (int64_t)norm * max_speed * 65536 / PSP_HALF_SCALE;
    int64_t scaled = target_q16 + *remainder;
    int16_t out = (int16_t)(scaled / 65536);
    *remainder = (int32_t)(scaled - (int64_t)out * 65536);
    return out;
}

void psp_joystick_apply_transforms(int16_t *x, int16_t *y, bool swap_xy, bool invert_x,
                                   bool invert_y) {
    if (swap_xy) {
        int16_t tmp = *x;
        *x = *y;
        *y = tmp;
    }
    if (invert_x) {
        *x = -*x;
    }
    if (invert_y) {
        *y = -*y;
    }
}
