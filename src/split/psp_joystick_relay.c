/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>

#include <tokyo2006/psp-joystick/psp_joystick_relay.h>
#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(psp_joystick_relay_state);

ZMK_RELAY_EVENT_CENTRAL_TO_PERIPHERAL(psp_joystick_relay_state, Pjs, source)
ZMK_RELAY_EVENT_HANDLE(psp_joystick_relay_state, Pjs, source)

#if !IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)

static int psp_joystick_relay_on_state(const zmk_event_t *eh) {
    const struct psp_joystick_relay_state *ev = as_psp_joystick_relay_state(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (ev->version != PSP_JOYSTICK_RELAY_STATE_VERSION) {
        LOG_WRN("Ignoring relayed state v%u (expected v%u)", ev->version,
                PSP_JOYSTICK_RELAY_STATE_VERSION);
        return ZMK_EV_EVENT_HANDLED;
    }

    const struct device *dev = DEVICE_DT_GET_ANY(tokyo2006_psp_joystick);
    if (dev == NULL) {
        LOG_WRN("No psp-joystick device on peripheral; ignoring relayed state");
        return ZMK_EV_EVENT_HANDLED;
    }

    (void)psp_joystick_state_set_deadzone_percent(dev, ev->deadzone_percent);
    (void)psp_joystick_state_set_max_speed(dev, ev->max_speed);
    (void)psp_joystick_state_set_curve_exponent_x100(dev, ev->curve_exponent_x100);
    (void)psp_joystick_state_set_smoothing_percent(dev, ev->smoothing_percent);
    (void)psp_joystick_state_set_sample_interval_ms(dev, ev->sample_interval_ms);
    (void)psp_joystick_state_set_invert_x(dev, (ev->flags & BIT(0)) != 0);
    (void)psp_joystick_state_set_invert_y(dev, (ev->flags & BIT(1)) != 0);
    (void)psp_joystick_state_set_swap_xy(dev, (ev->flags & BIT(2)) != 0);
    if ((ev->flags & BIT(3)) != 0) {
        (void)psp_joystick_state_set_center_x(dev, ev->center_x);
        (void)psp_joystick_state_set_center_y(dev, ev->center_y);
        (void)psp_joystick_state_set_min_x(dev, ev->min_x);
        (void)psp_joystick_state_set_max_x(dev, ev->max_x);
        (void)psp_joystick_state_set_min_y(dev, ev->min_y);
        (void)psp_joystick_state_set_max_y(dev, ev->max_y);
    }
    (void)psp_joystick_state_apply_all(dev);
    return ZMK_EV_EVENT_HANDLED;
}

ZMK_LISTENER(psp_joystick_relay_listener, psp_joystick_relay_on_state);
ZMK_SUBSCRIPTION(psp_joystick_relay_listener, psp_joystick_relay_state);

#endif /* !CONFIG_ZMK_SPLIT_ROLE_CENTRAL */

void psp_joystick_relay_send_state(const struct device *dev) {
    struct psp_joystick_relay_state ev = {
        .source = ZMK_RELAY_EVENT_SOURCE_SELF,
        .version = PSP_JOYSTICK_RELAY_STATE_VERSION,
        .deadzone_percent = psp_joystick_state_get_deadzone_percent(dev),
        .max_speed = psp_joystick_state_get_max_speed(dev),
        .curve_exponent_x100 = psp_joystick_state_get_curve_exponent_x100(dev),
        .smoothing_percent = psp_joystick_state_get_smoothing_percent(dev),
        .sample_interval_ms = psp_joystick_state_get_sample_interval_ms(dev),
        .flags = (psp_joystick_state_get_invert_x(dev) ? BIT(0) : 0) |
                 (psp_joystick_state_get_invert_y(dev) ? BIT(1) : 0) |
                 (psp_joystick_state_get_swap_xy(dev) ? BIT(2) : 0),
    };

    /*
     * Only ship calibration when the central actually has a local stick.
     * A central without one would otherwise overwrite the peripheral's
     * boot-time auto-calibration with its fallback center.
     */
    if (dev != NULL) {
        ev.flags |= BIT(3);
        ev.center_x = psp_joystick_state_get_center_x(dev);
        ev.center_y = psp_joystick_state_get_center_y(dev);
        ev.min_x = psp_joystick_state_get_min_x(dev);
        ev.max_x = psp_joystick_state_get_max_x(dev);
        ev.min_y = psp_joystick_state_get_min_y(dev);
        ev.max_y = psp_joystick_state_get_max_y(dev);
    }

    raise_psp_joystick_relay_state(ev);
}
