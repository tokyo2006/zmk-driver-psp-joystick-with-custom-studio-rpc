/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdio.h>

#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>
#include <zmk/studio/custom.h>

#include <tokyo2006/psp-joystick/psp_joystick.pb.h>

#include <tokyo2006/psp-joystick/psp_joystick_settings.h>
#include <tokyo2006/psp-joystick/psp_joystick_state.h>

#if IS_ENABLED(CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY)
#include <tokyo2006/psp-joystick/psp_joystick_relay.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct zmk_rpc_custom_subsystem_meta psp_joystick_subsystem_meta = {
    .ui_urls = NULL,
    .ui_urls_count = 0,
    .security = ZMK_STUDIO_RPC_HANDLER_UNSECURED,
};

ZMK_RPC_CUSTOM_SUBSYSTEM(tokyo2006__psp_joystick, &psp_joystick_subsystem_meta,
                         psp_joystick_rpc_handle_request);
ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER(tokyo2006__psp_joystick, tokyo2006_psp_joystick_Response);

#define PSP_DEVICE() DEVICE_DT_GET_ANY(tokyo2006_psp_joystick)

static void set_error(tokyo2006_psp_joystick_Response *resp, const char *message) {
    tokyo2006_psp_joystick_ErrorResponse err = tokyo2006_psp_joystick_ErrorResponse_init_zero;
    snprintf(err.message, sizeof(err.message), "%s", message);
    resp->which_response_type = tokyo2006_psp_joystick_Response_error_tag;
    resp->response_type.error = err;
}

static void fill_state(const struct device *dev, tokyo2006_psp_joystick_PspJoystickState *s) {
    s->deadzone_percent = psp_joystick_state_get_deadzone_percent(dev);
    s->max_speed = psp_joystick_state_get_max_speed(dev);
    s->curve_exponent_x100 = psp_joystick_state_get_curve_exponent_x100(dev);
    s->smoothing_percent = psp_joystick_state_get_smoothing_percent(dev);
    s->sample_interval_ms = psp_joystick_state_get_sample_interval_ms(dev);
    s->invert_x = psp_joystick_state_get_invert_x(dev);
    s->invert_y = psp_joystick_state_get_invert_y(dev);
    s->swap_xy = psp_joystick_state_get_swap_xy(dev);
    s->center_x = psp_joystick_state_get_center_x(dev);
    s->center_y = psp_joystick_state_get_center_y(dev);
    s->min_x = psp_joystick_state_get_min_x(dev);
    s->max_x = psp_joystick_state_get_max_x(dev);
    s->min_y = psp_joystick_state_get_min_y(dev);
    s->max_y = psp_joystick_state_get_max_y(dev);

    s->has_deadzone_percent = true;
    s->has_max_speed = true;
    s->has_curve_exponent_x100 = true;
    s->has_smoothing_percent = true;
    s->has_sample_interval_ms = true;
    s->has_invert_x = true;
    s->has_invert_y = true;
    s->has_swap_xy = true;
    s->has_center_x = true;
    s->has_center_y = true;
    s->has_min_x = true;
    s->has_max_x = true;
    s->has_min_y = true;
    s->has_max_y = true;
}

static int handle_get_state(const tokyo2006_psp_joystick_GetStateRequest *req,
                            tokyo2006_psp_joystick_Response *resp) {
    ARG_UNUSED(req);

    tokyo2006_psp_joystick_GetStateResponse out = tokyo2006_psp_joystick_GetStateResponse_init_zero;
    fill_state(PSP_DEVICE(), &out.state);
    out.has_state = true;
    resp->which_response_type = tokyo2006_psp_joystick_Response_get_state_tag;
    resp->response_type.get_state = out;
    return 0;
}

static int handle_set_state(const tokyo2006_psp_joystick_SetStateRequest *req,
                            tokyo2006_psp_joystick_Response *resp) {
    const struct device *dev = PSP_DEVICE();
    int first_rc = 0;
    char first_err[128] = {0};

#define APPLY(field, setter)                                                                       \
    do {                                                                                           \
        if (req->state.has_##field) {                                                              \
            int rc = setter(dev, req->state.field);                                                \
            if (rc != 0 && first_rc == 0) {                                                        \
                first_rc = rc;                                                                     \
                snprintf(first_err, sizeof(first_err), "%s failed (%d)", #field, rc);              \
            }                                                                                      \
        }                                                                                          \
    } while (0)

    APPLY(deadzone_percent, psp_joystick_state_set_deadzone_percent);
    APPLY(max_speed, psp_joystick_state_set_max_speed);
    APPLY(curve_exponent_x100, psp_joystick_state_set_curve_exponent_x100);
    APPLY(smoothing_percent, psp_joystick_state_set_smoothing_percent);
    APPLY(sample_interval_ms, psp_joystick_state_set_sample_interval_ms);
    APPLY(invert_x, psp_joystick_state_set_invert_x);
    APPLY(invert_y, psp_joystick_state_set_invert_y);
    APPLY(swap_xy, psp_joystick_state_set_swap_xy);
    APPLY(center_x, psp_joystick_state_set_center_x);
    APPLY(center_y, psp_joystick_state_set_center_y);
    APPLY(min_x, psp_joystick_state_set_min_x);
    APPLY(max_x, psp_joystick_state_set_max_x);
    APPLY(min_y, psp_joystick_state_set_min_y);
    APPLY(max_y, psp_joystick_state_set_max_y);

#undef APPLY

    if (first_rc != 0) {
        set_error(resp, first_err);
        return first_rc;
    }

    (void)psp_joystick_state_apply_all(dev);

    bool include_calibration = req->state.has_center_x || req->state.has_center_y ||
                               req->state.has_min_x || req->state.has_max_x ||
                               req->state.has_min_y || req->state.has_max_y;

    int save_rc = 0;
    if (req->persist) {
        save_rc = psp_joystick_settings_save_all(dev, include_calibration);
    }

#if IS_ENABLED(CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY)
    psp_joystick_relay_send_state(dev);
#endif

    tokyo2006_psp_joystick_SetStateResponse out = tokyo2006_psp_joystick_SetStateResponse_init_zero;
    fill_state(dev, &out.state);
    out.has_state = true;
    out.persisted = req->persist && save_rc == 0;
    resp->which_response_type = tokyo2006_psp_joystick_Response_set_state_tag;
    resp->response_type.set_state = out;
    return 0;
}

static int handle_calibrate(const tokyo2006_psp_joystick_CalibrateRequest *req,
                            tokyo2006_psp_joystick_Response *resp) {
    const struct device *dev = PSP_DEVICE();

    if (dev == NULL || psp_joystick_driver_calibrate == NULL) {
        set_error(resp,
                  "calibration requires a local device; the split peripheral auto-centers at boot");
        return -ENODEV;
    }

    int32_t center_x = 0;
    int32_t center_y = 0;
    int rc = psp_joystick_driver_calibrate(dev, &center_x, &center_y);
    if (rc == -EBUSY) {
        set_error(resp, "joystick is moving; release it and retry");
        return rc;
    }
    if (rc == -EAGAIN) {
        set_error(resp, "not enough samples yet; try again");
        return rc;
    }
    if (rc != 0) {
        set_error(resp, "calibration failed");
        return rc;
    }

    (void)psp_joystick_state_apply_all(dev);
    if (req->persist) {
        (void)psp_joystick_settings_save_all(dev, true);
    }

    tokyo2006_psp_joystick_CalibrateResponse out =
        tokyo2006_psp_joystick_CalibrateResponse_init_zero;
    fill_state(dev, &out.state);
    out.has_state = true;
    out.measured_center_x = center_x;
    out.measured_center_y = center_y;
    resp->which_response_type = tokyo2006_psp_joystick_Response_calibrate_tag;
    resp->response_type.calibrate = out;
    return 0;
}

static int handle_reset(const tokyo2006_psp_joystick_ResetRequest *req,
                        tokyo2006_psp_joystick_Response *resp) {
    const struct device *dev = PSP_DEVICE();

    (void)psp_joystick_state_load_defaults(dev);
    (void)psp_joystick_state_apply_all(dev);

    if (req->factory_defaults) {
        (void)psp_joystick_settings_reset_all(dev);
    }

#if IS_ENABLED(CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY)
    psp_joystick_relay_send_state(dev);
#endif

    tokyo2006_psp_joystick_ResetResponse out = tokyo2006_psp_joystick_ResetResponse_init_zero;
    fill_state(dev, &out.state);
    out.has_state = true;
    resp->which_response_type = tokyo2006_psp_joystick_Response_reset_tag;
    resp->response_type.reset = out;
    return 0;
}

static bool psp_joystick_rpc_handle_request(const zmk_custom_CallRequest *raw_request,
                                            pb_callback_t *encode_response) {
    tokyo2006_psp_joystick_Response *resp =
        ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER_ALLOCATE(tokyo2006__psp_joystick, encode_response);

    tokyo2006_psp_joystick_Request req = tokyo2006_psp_joystick_Request_init_zero;
    pb_istream_t stream =
        pb_istream_from_buffer(raw_request->payload.bytes, raw_request->payload.size);
    if (!pb_decode(&stream, tokyo2006_psp_joystick_Request_fields, &req)) {
        LOG_WRN("Failed to decode psp_joystick request: %s", PB_GET_ERROR(&stream));
        set_error(resp, "Failed to decode request");
        return true;
    }

    int rc = 0;
    switch (req.which_request_type) {
    case tokyo2006_psp_joystick_Request_get_state_tag:
        rc = handle_get_state(&req.request_type.get_state, resp);
        break;
    case tokyo2006_psp_joystick_Request_set_state_tag:
        rc = handle_set_state(&req.request_type.set_state, resp);
        break;
    case tokyo2006_psp_joystick_Request_calibrate_tag:
        rc = handle_calibrate(&req.request_type.calibrate, resp);
        break;
    case tokyo2006_psp_joystick_Request_reset_tag:
        rc = handle_reset(&req.request_type.reset, resp);
        break;
    default:
        LOG_WRN("Unsupported psp_joystick request type: %d", req.which_request_type);
        rc = -1;
    }

    if (rc != 0 && resp->which_response_type != tokyo2006_psp_joystick_Response_error_tag) {
        set_error(resp, "Failed to process request");
    }
    return true;
}
