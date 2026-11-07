/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT tokyo2006_psp_joystick

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/dt-bindings/input/input-event-codes.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>
#include <zmk/events/layer_state_changed.h>

#include <tokyo2006/psp-joystick/psp_joystick_math.h>
#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_REGISTER(psp_joystick, CONFIG_ZMK_PSP_JOYSTICK_LOG_LEVEL);

#define PSP_RING_SIZE 64
#define PSP_CAL_WINDOW 16
#define PSP_ADC_MAX 4095
#define PSP_HALF_SCALE 32767
#define PSP_FIRST_TICK_MS 1000
#define PSP_CAL_SPREAD_LIMIT (PSP_ADC_MAX / 5)
#define PSP_CAL_CENTER_LIMIT (PSP_ADC_MAX / 4)
#define PSP_MANUAL_SPREAD_LIMIT (PSP_ADC_MAX / 10)
#define PSP_RECENTER_WINDOW (PSP_ADC_MAX / 16)

struct psp_joystick_config {
    struct adc_dt_spec adc_x;
    struct adc_dt_spec adc_y;
    uint16_t sample_interval_ms;
    uint16_t auto_center_ms;
    bool auto_center;
    bool auto_recenter;
    uint16_t auto_recenter_ms;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    bool arrow_mode;
    uint8_t arrow_threshold_percent;
    int8_t arrow_layer;
    bool has_dt_center;
    int16_t dt_center_x;
    int16_t dt_center_y;
    int16_t dt_min_x;
    int16_t dt_max_x;
    int16_t dt_min_y;
    int16_t dt_max_y;
    uint8_t dt_deadzone_percent;
    uint8_t dt_max_speed;
    uint16_t dt_curve_exponent_x100;
    uint8_t dt_smoothing_percent;
};

struct psp_joystick_data {
    const struct device *dev;
    struct k_work_delayable work;
    struct psp_joystick_runtime_state state;
    uint16_t raw_x[PSP_RING_SIZE];
    uint16_t raw_y[PSP_RING_SIZE];
    uint8_t ring_head;
    uint8_t ring_count;
    uint8_t cal_target;
    bool calibrating;
    bool calibration_done;
    bool skip_auto_center;
    uint16_t recenter_count;
    uint8_t arrow_dir;
    bool layer_arrow_active;
    bool reported_nonzero;
    bool adc_error_logged;
    bool ema_valid;
    int16_t ema_x;
    int16_t ema_y;
    int32_t remainder_x;
    int32_t remainder_y;
};

#if IS_ENABLED(CONFIG_ZMK_PSP_JOYSTICK_DEDICATED_WORKQUEUE)
K_THREAD_STACK_DEFINE(psp_joystick_work_q_stack, CONFIG_ZMK_PSP_JOYSTICK_WORKQUEUE_STACK_SIZE);
static struct k_work_q psp_joystick_work_q;
static bool psp_joystick_work_q_started;

#define PSP_WORKQ_SCHEDULE(dwork, delay)                                                           \
    k_work_schedule_for_queue(&psp_joystick_work_q, dwork, delay)
#define PSP_WORKQ_RESCHEDULE(dwork, delay)                                                         \
    k_work_reschedule_for_queue(&psp_joystick_work_q, dwork, delay)
#else
#define PSP_WORKQ_SCHEDULE(dwork, delay) k_work_schedule(dwork, delay)
#define PSP_WORKQ_RESCHEDULE(dwork, delay) k_work_reschedule(dwork, delay)
#endif

static int psp_read_channel(const struct adc_dt_spec *spec, uint16_t *out) {
    uint16_t buffer = 0;
    struct adc_sequence sequence = {
        .buffer = &buffer,
        .buffer_size = sizeof(buffer),
    };

    int rc = adc_sequence_init_dt(spec, &sequence);
    if (rc != 0) {
        return rc;
    }
    rc = adc_read_dt(spec, &sequence);
    if (rc != 0) {
        return rc;
    }
    *out = buffer;
    return 0;
}

static void ring_push(struct psp_joystick_data *data, uint16_t x, uint16_t y) {
    data->raw_x[data->ring_head] = x;
    data->raw_y[data->ring_head] = y;
    data->ring_head = (data->ring_head + 1U) % PSP_RING_SIZE;
    if (data->ring_count < PSP_RING_SIZE) {
        data->ring_count++;
    }
}

static uint8_t ring_tail(const struct psp_joystick_data *data, uint8_t count) {
    if (count >= PSP_RING_SIZE) {
        return data->ring_head;
    }
    return (uint8_t)((data->ring_head + PSP_RING_SIZE - count) % PSP_RING_SIZE);
}

static void copy_last(const uint16_t *ring, const struct psp_joystick_data *data, uint16_t *out,
                      uint8_t wanted) {
    uint8_t count = MIN(wanted, data->ring_count);
    uint8_t start = ring_tail(data, count);

    for (uint8_t i = 0; i < count; i++) {
        out[i] = ring[(start + i) % PSP_RING_SIZE];
    }
}

static void sort_u16(uint16_t *values, uint8_t count) {
    for (uint8_t i = 1; i < count; i++) {
        uint16_t key = values[i];
        uint8_t j = i;
        while (j > 0 && values[j - 1] > key) {
            values[j] = values[j - 1];
            j--;
        }
        values[j] = key;
    }
}

static uint16_t median_of(uint16_t *values, uint8_t count) {
    sort_u16(values, count);
    return values[count / 2];
}

static uint16_t spread_of(const uint16_t *values, uint8_t count) {
    uint16_t min = values[0];
    uint16_t max = values[0];

    for (uint8_t i = 1; i < count; i++) {
        min = MIN(min, values[i]);
        max = MAX(max, values[i]);
    }
    return max - min;
}

static void measurement_from_ring(const struct psp_joystick_data *data, uint8_t wanted,
                                  uint16_t *center_x, uint16_t *center_y, uint16_t *spread_x,
                                  uint16_t *spread_y) {
    uint16_t xs[PSP_RING_SIZE];
    uint16_t ys[PSP_RING_SIZE];
    uint8_t count = MIN(wanted, data->ring_count);

    copy_last(data->raw_x, data, xs, count);
    copy_last(data->raw_y, data, ys, count);

    *spread_x = spread_of(xs, count);
    *spread_y = spread_of(ys, count);
    *center_x = median_of(xs, count);
    *center_y = median_of(ys, count);
}

static void finalize_calibration(const struct device *dev) {
    const struct psp_joystick_config *config = dev->config;
    struct psp_joystick_data *data = dev->data;
    uint16_t cx, cy, sx, sy;

    measurement_from_ring(data, data->ring_count, &cx, &cy, &sx, &sy);

    bool ok = sx < PSP_CAL_SPREAD_LIMIT && sy < PSP_CAL_SPREAD_LIMIT;
    if (ok && config->has_dt_center) {
        ok = abs((int)cx - config->dt_center_x) < PSP_CAL_CENTER_LIMIT &&
             abs((int)cy - config->dt_center_y) < PSP_CAL_CENTER_LIMIT;
    }

    if (ok) {
        data->state.center_x = (int16_t)cx;
        data->state.center_y = (int16_t)cy;
        LOG_INF("auto-centered at x=%u y=%u", cx, cy);
    } else {
        LOG_WRN("auto-center rejected (spread x=%u y=%u); keeping x=%d y=%d", sx, sy,
                data->state.center_x, data->state.center_y);
    }

    data->calibrating = false;
    data->calibration_done = true;
}

int psp_joystick_driver_calibrate(const struct device *dev, int32_t *center_x, int32_t *center_y) {
    struct psp_joystick_data *data = dev->data;
    uint16_t cx, cy, sx, sy;

    if (data->ring_count < PSP_CAL_WINDOW) {
        return -EAGAIN;
    }

    measurement_from_ring(data, PSP_CAL_WINDOW, &cx, &cy, &sx, &sy);
    if (sx >= PSP_MANUAL_SPREAD_LIMIT || sy >= PSP_MANUAL_SPREAD_LIMIT) {
        return -EBUSY;
    }

    data->state.center_x = (int16_t)cx;
    data->state.center_y = (int16_t)cy;
    data->skip_auto_center = true;
    data->calibrating = false;
    data->calibration_done = true;
    *center_x = cx;
    *center_y = cy;
    return 0;
}

void psp_joystick_driver_on_settings_loaded(const struct device *dev, bool center_persisted) {
    struct psp_joystick_data *data = dev->data;

    if (center_persisted) {
        data->skip_auto_center = true;
        data->calibrating = false;
        data->calibration_done = true;
    }
}

struct psp_joystick_runtime_state *psp_joystick_driver_get_state(const struct device *dev) {
    return &((struct psp_joystick_data *)dev->data)->state;
}

int psp_joystick_driver_load_defaults(const struct device *dev) {
    const struct psp_joystick_config *config = dev->config;
    struct psp_joystick_runtime_state *state = &((struct psp_joystick_data *)dev->data)->state;

    state->deadzone_percent = CLAMP(config->dt_deadzone_percent, 0, 50);
    state->max_speed = CLAMP(config->dt_max_speed, 1, 64);
    state->curve_exponent_x100 = config->dt_curve_exponent_x100;
    state->smoothing_percent = CLAMP(config->dt_smoothing_percent, 0, 90);
    state->sample_interval_ms = CLAMP(config->sample_interval_ms, 4, 50);
    state->invert_x = config->invert_x;
    state->invert_y = config->invert_y;
    state->swap_xy = config->swap_xy;
    state->arrow_mode = config->arrow_mode;
    state->arrow_threshold_percent = CLAMP(config->arrow_threshold_percent, 1, 95);
    state->center_x =
        CLAMP(config->has_dt_center ? config->dt_center_x : PSP_JOYSTICK_CENTER_DEFAULT, 0, 4095);
    state->center_y =
        CLAMP(config->has_dt_center ? config->dt_center_y : PSP_JOYSTICK_CENTER_DEFAULT, 0, 4095);
    state->min_x = CLAMP(config->dt_min_x, 0, 4095);
    state->max_x = CLAMP(config->dt_max_x, 0, 4095);
    state->min_y = CLAMP(config->dt_min_y, 0, 4095);
    state->max_y = CLAMP(config->dt_max_y, 0, 4095);
    return 0;
}

int psp_joystick_driver_apply_state(const struct device *dev) {
    struct psp_joystick_data *data = dev->data;

    data->state.sample_interval_ms = CLAMP(data->state.sample_interval_ms, 4, 50);
    PSP_WORKQ_RESCHEDULE(&data->work, K_MSEC(data->state.sample_interval_ms));
    return 0;
}

enum psp_arrow_dir {
    PSP_ARROW_NONE = 0,
    PSP_ARROW_UP,
    PSP_ARROW_DOWN,
    PSP_ARROW_LEFT,
    PSP_ARROW_RIGHT,
};

static uint16_t psp_arrow_code(uint8_t dir) {
    switch (dir) {
    case PSP_ARROW_UP:
        return INPUT_KEY_UP;
    case PSP_ARROW_DOWN:
        return INPUT_KEY_DOWN;
    case PSP_ARROW_LEFT:
        return INPUT_KEY_LEFT;
    case PSP_ARROW_RIGHT:
        return INPUT_KEY_RIGHT;
    default:
        return 0;
    }
}

static int16_t psp_arrow_magnitude(uint8_t dir, int16_t x, int16_t y) {
    return (dir == PSP_ARROW_LEFT || dir == PSP_ARROW_RIGHT) ? abs((int32_t)x) : abs((int32_t)y);
}

static void psp_arrow_set(struct psp_joystick_data *data, uint8_t dir) {
    if (dir == data->arrow_dir) {
        return;
    }

    if (data->arrow_dir != PSP_ARROW_NONE) {
        input_report_key(data->dev, psp_arrow_code(data->arrow_dir), 0, dir == PSP_ARROW_NONE,
                         K_NO_WAIT);
    }
    if (dir != PSP_ARROW_NONE) {
        input_report_key(data->dev, psp_arrow_code(dir), 1, true, K_NO_WAIT);
    }
    data->arrow_dir = dir;
}

static void psp_joystick_work_cb(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct psp_joystick_data *data = CONTAINER_OF(dwork, struct psp_joystick_data, work);
    const struct device *dev = data->dev;
    const struct psp_joystick_config *config = dev->config;
    uint16_t raw_x;
    uint16_t raw_y;

    int rc = psp_read_channel(&config->adc_x, &raw_x);
    if (rc == 0) {
        rc = psp_read_channel(&config->adc_y, &raw_y);
    }
    if (rc != 0) {
        if (!data->adc_error_logged) {
            LOG_WRN("ADC read failed: %d", rc);
            data->adc_error_logged = true;
        }
        goto reschedule;
    }
    data->adc_error_logged = false;

    raw_x = MIN(raw_x, PSP_ADC_MAX);
    raw_y = MIN(raw_y, PSP_ADC_MAX);

    if (!data->calibration_done && !data->skip_auto_center && config->auto_center) {
        if (!data->calibrating) {
            data->calibrating = true;
            data->ring_head = 0;
            data->ring_count = 0;
            uint16_t interval = CLAMP(config->sample_interval_ms, 4, 50);
            data->cal_target = (uint8_t)CLAMP(config->auto_center_ms / interval, 8, PSP_RING_SIZE);
        }
    }

    ring_push(data, raw_x, raw_y);

    if (data->calibrating) {
        if (data->ring_count >= data->cal_target) {
            finalize_calibration(dev);
        }
        goto reschedule;
    }

    struct psp_joystick_runtime_state *state = &data->state;

    /*
     * Track slow rest drift / mechanical hysteresis: after the stick has
     * stayed still near the current center for auto-recenter-ms, move the
     * center to the median of the recent samples. This keeps the pointer
     * from drifting left after a left push (or right after a right push)
     * without needing a large deadzone.
     */
    if (data->calibration_done && config->auto_recenter && data->ring_count >= PSP_CAL_WINDOW) {
        int32_t dx = abs((int32_t)raw_x - state->center_x);
        int32_t dy = abs((int32_t)raw_y - state->center_y);
        uint16_t target = (uint16_t)CLAMP(
            config->auto_recenter_ms / MAX(state->sample_interval_ms, 1), 4, 1000);

        if (dx <= PSP_RECENTER_WINDOW && dy <= PSP_RECENTER_WINDOW) {
            if (++data->recenter_count >= target) {
                uint16_t cx, cy, sx, sy;
                measurement_from_ring(data, PSP_CAL_WINDOW, &cx, &cy, &sx, &sy);
                if (sx < PSP_MANUAL_SPREAD_LIMIT && sy < PSP_MANUAL_SPREAD_LIMIT) {
                    state->center_x = (int16_t)cx;
                    state->center_y = (int16_t)cy;
                }
                data->recenter_count = 0;
            }
        } else {
            data->recenter_count = 0;
        }
    }

    int16_t x = psp_joystick_normalize(raw_x, state->center_x, state->min_x, state->max_x);
    int16_t y = psp_joystick_normalize(raw_y, state->center_y, state->min_y, state->max_y);

    if (state->smoothing_percent > 0) {
        if (!data->ema_valid) {
            data->ema_x = x;
            data->ema_y = y;
            data->ema_valid = true;
        } else {
            int32_t weight = state->smoothing_percent;
            data->ema_x =
                (int16_t)(((int32_t)data->ema_x * (100 - weight) + (int32_t)x * weight) / 100);
            data->ema_y =
                (int16_t)(((int32_t)data->ema_y * (100 - weight) + (int32_t)y * weight) / 100);
        }
        x = data->ema_x;
        y = data->ema_y;
    } else {
        data->ema_valid = false;
    }

    psp_joystick_apply_transforms(&x, &y, state->swap_xy, state->invert_x, state->invert_y);

    if (state->arrow_mode || data->layer_arrow_active) {
        int32_t threshold = (int32_t)state->arrow_threshold_percent * PSP_HALF_SCALE / 100;
        int32_t ax = abs((int32_t)x);
        int32_t ay = abs((int32_t)y);
        uint8_t candidate = PSP_ARROW_NONE;

        if (ax >= threshold && ax >= ay) {
            candidate = x > 0 ? PSP_ARROW_RIGHT : PSP_ARROW_LEFT;
        } else if (ay >= threshold) {
            candidate = y > 0 ? PSP_ARROW_DOWN : PSP_ARROW_UP;
        }

        if (data->arrow_dir != PSP_ARROW_NONE) {
            int32_t magnitude = psp_arrow_magnitude(data->arrow_dir, x, y);
            int32_t release = threshold * 6 / 10;

            if (magnitude >= release) {
                if (candidate == PSP_ARROW_NONE) {
                    candidate = data->arrow_dir;
                } else if (candidate != data->arrow_dir &&
                           psp_arrow_magnitude(candidate, x, y) < magnitude * 12 / 10) {
                    candidate = data->arrow_dir;
                }
            }
        }

        psp_arrow_set(data, candidate);
        goto reschedule;
    }

    if (data->arrow_dir != PSP_ARROW_NONE) {
        psp_arrow_set(data, PSP_ARROW_NONE);
    }

    x = psp_joystick_apply_deadzone(x, state->deadzone_percent);
    y = psp_joystick_apply_deadzone(y, state->deadzone_percent);
    x = psp_joystick_apply_curve(x, state->curve_exponent_x100);
    y = psp_joystick_apply_curve(y, state->curve_exponent_x100);

    if (x == 0) {
        data->remainder_x = 0;
    }
    if (y == 0) {
        data->remainder_y = 0;
    }

    int16_t dx = psp_joystick_apply_speed(x, state->max_speed, &data->remainder_x);
    int16_t dy = psp_joystick_apply_speed(y, state->max_speed, &data->remainder_y);

    if (dx == 0 && dy == 0) {
        if (data->reported_nonzero) {
            input_report_rel(dev, INPUT_REL_X, 0, false, K_NO_WAIT);
            input_report_rel(dev, INPUT_REL_Y, 0, true, K_NO_WAIT);
            data->reported_nonzero = false;
        }
    } else {
        input_report_rel(dev, INPUT_REL_X, dx, false, K_NO_WAIT);
        input_report_rel(dev, INPUT_REL_Y, dy, true, K_NO_WAIT);
        data->reported_nonzero = true;
    }

reschedule:
    PSP_WORKQ_RESCHEDULE(&data->work, K_MSEC(CLAMP(data->state.sample_interval_ms, 4, 50)));
}

static int psp_joystick_init(const struct device *dev) {
    const struct psp_joystick_config *config = dev->config;
    struct psp_joystick_data *data = dev->data;

    data->dev = dev;

    if (!adc_is_ready_dt(&config->adc_x) || !adc_is_ready_dt(&config->adc_y)) {
        LOG_ERR("ADC device is not ready");
        return -ENODEV;
    }
    if (adc_channel_setup_dt(&config->adc_x) != 0 || adc_channel_setup_dt(&config->adc_y) != 0) {
        LOG_ERR("ADC channel setup failed");
        return -EIO;
    }

    k_work_init_delayable(&data->work, psp_joystick_work_cb);

#if IS_ENABLED(CONFIG_ZMK_PSP_JOYSTICK_DEDICATED_WORKQUEUE)
    if (!psp_joystick_work_q_started) {
        k_work_queue_init(&psp_joystick_work_q);
        k_work_queue_start(&psp_joystick_work_q, psp_joystick_work_q_stack,
                           K_THREAD_STACK_SIZEOF(psp_joystick_work_q_stack),
                           CONFIG_ZMK_PSP_JOYSTICK_WORKQUEUE_PRIORITY, NULL);
        psp_joystick_work_q_started = true;
    }
#endif

    (void)psp_joystick_state_load_defaults(dev);
    if (psp_joystick_settings_install_dt_defaults != NULL) {
        (void)psp_joystick_settings_install_dt_defaults(dev);
    }
    PSP_WORKQ_SCHEDULE(&data->work, K_MSEC(PSP_FIRST_TICK_MS));
    return 0;
}

#if !IS_ENABLED(CONFIG_ZMK_SPLIT) || IS_ENABLED(CONFIG_ZMK_SPLIT_ROLE_CENTRAL)
static int psp_joystick_layer_listener(const zmk_event_t *eh) {
    const struct zmk_layer_state_changed *ev = as_zmk_layer_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct device *dev = DEVICE_DT_GET_ANY(tokyo2006_psp_joystick);
    if (dev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    const struct psp_joystick_config *config = dev->config;
    struct psp_joystick_data *data = dev->data;
    if (config->arrow_layer >= 0 && ev->layer == (uint8_t)config->arrow_layer) {
        data->layer_arrow_active = ev->state;
    }

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(psp_joystick_layer, psp_joystick_layer_listener);
ZMK_SUBSCRIPTION(psp_joystick_layer, zmk_layer_state_changed);
#endif /* central or unibody: split peripherals have no local layer state */

#define PSP_JOYSTICK_INST(n)                                                                       \
    static const struct psp_joystick_config psp_joystick_config_##n = {                            \
        .adc_x = ADC_DT_SPEC_INST_GET_BY_IDX(n, 0),                                                \
        .adc_y = ADC_DT_SPEC_INST_GET_BY_IDX(n, 1),                                                \
        .sample_interval_ms = DT_INST_PROP_OR(n, sample_interval_ms, 8),                           \
        .auto_center_ms = DT_INST_PROP_OR(n, auto_center_ms, 500),                                 \
        .auto_center = DT_INST_PROP_OR(n, auto_center, 1),                                         \
        .auto_recenter = DT_INST_PROP_OR(n, auto_recenter, 1),                                     \
        .auto_recenter_ms = DT_INST_PROP_OR(n, auto_recenter_ms, 1000),                            \
        .invert_x = DT_INST_PROP_OR(n, invert_x, 0),                                               \
        .invert_y = DT_INST_PROP_OR(n, invert_y, 0),                                               \
        .swap_xy = DT_INST_PROP_OR(n, swap_xy, 0),                                                 \
        .arrow_mode = DT_INST_PROP_OR(n, arrow_mode, 0),                                           \
        .arrow_threshold_percent = DT_INST_PROP_OR(n, arrow_threshold_percent, 50),                \
        .arrow_layer = DT_INST_PROP_OR(n, arrow_layer, -1),                                        \
        .has_dt_center = DT_INST_NODE_HAS_PROP(n, center_x) && DT_INST_NODE_HAS_PROP(n, center_y), \
        .dt_center_x = DT_INST_PROP_OR(n, center_x, PSP_JOYSTICK_CENTER_DEFAULT),                  \
        .dt_center_y = DT_INST_PROP_OR(n, center_y, PSP_JOYSTICK_CENTER_DEFAULT),                  \
        .dt_min_x = DT_INST_PROP_OR(n, min_x, PSP_JOYSTICK_MIN_DEFAULT),                           \
        .dt_max_x = DT_INST_PROP_OR(n, max_x, PSP_JOYSTICK_MAX_DEFAULT),                           \
        .dt_min_y = DT_INST_PROP_OR(n, min_y, PSP_JOYSTICK_MIN_DEFAULT),                           \
        .dt_max_y = DT_INST_PROP_OR(n, max_y, PSP_JOYSTICK_MAX_DEFAULT),                           \
        .dt_deadzone_percent = DT_INST_PROP_OR(n, deadzone_percent, 8),                            \
        .dt_max_speed = DT_INST_PROP_OR(n, max_speed, 8),                                          \
        .dt_curve_exponent_x100 = DT_INST_PROP_OR(n, curve_exponent_x100, 100),                    \
        .dt_smoothing_percent = DT_INST_PROP_OR(n, smoothing_percent, 0),                          \
    };                                                                                             \
    static struct psp_joystick_data psp_joystick_data_##n;                                         \
    DEVICE_DT_INST_DEFINE(n, psp_joystick_init, NULL, &psp_joystick_data_##n,                      \
                          &psp_joystick_config_##n, POST_KERNEL, CONFIG_INPUT_INIT_PRIORITY,       \
                          NULL);

DT_INST_FOREACH_STATUS_OKAY(PSP_JOYSTICK_INST)
