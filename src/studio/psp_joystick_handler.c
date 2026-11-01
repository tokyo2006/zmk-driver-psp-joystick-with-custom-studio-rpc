#include <pb_decode.h>
#include <pb_encode.h>
#include <zephyr/sys/util.h>
#include <zmk/studio/custom.h>
#include <tokyo2006/psp-joystick/psp_joystick.pb.h>

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
#include <cormoran/zmk/custom_settings.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
#include <tokyo2006/psp-joystick/template_relay.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

static struct zmk_rpc_custom_subsystem_meta psp_joystick_subsystem_meta = {
    ZMK_RPC_CUSTOM_SUBSYSTEM_UI_URLS("https://tokyo2006.github.io/zmk-driver-psp-joystick-with-custom-studio-rpc/"),
    // Unsecured is suggested by default to avoid unlocking in un-reliable
    // environments.
    // The web template already implements the unlock prompt/retry flow (see
    // web/src/App.tsx), so switching this to ZMK_STUDIO_RPC_HANDLER_SECURED
    // requires no web changes.
    .security = ZMK_STUDIO_RPC_HANDLER_UNSECURED,
};

ZMK_RPC_CUSTOM_SUBSYSTEM(tokyo2006__psp_joystick, &psp_joystick_subsystem_meta, psp_joystick_rpc_handle_request);

ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER(tokyo2006__psp_joystick, tokyo2006_psp_joystick_Response);

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
ZMK_CUSTOM_SETTING_DEFINE(psp_joystick_sample_bool, "tokyo2006__psp_joystick", "sample_bool",
                          ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL, ZMK_CUSTOM_SETTING_VALUE_BOOL(true),
                          ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,
                          ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,
                          ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE, ZMK_CUSTOM_SETTING_NO_CONSTRAINT);
#endif

static int handle_sample_request(const tokyo2006_psp_joystick_SampleRequest *req,
                                 tokyo2006_psp_joystick_Response *resp);

static bool psp_joystick_rpc_handle_request(const zmk_custom_CallRequest *raw_request,
                                        pb_callback_t *encode_response) {
    tokyo2006_psp_joystick_Response *resp =
        ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER_ALLOCATE(tokyo2006__psp_joystick, encode_response);

    tokyo2006_psp_joystick_Request req = tokyo2006_psp_joystick_Request_init_zero;

    pb_istream_t req_stream =
        pb_istream_from_buffer(raw_request->payload.bytes, raw_request->payload.size);
    if (!pb_decode(&req_stream, tokyo2006_psp_joystick_Request_fields, &req)) {
        LOG_WRN("Failed to decode psp_joystick request: %s", PB_GET_ERROR(&req_stream));
        tokyo2006_psp_joystick_ErrorResponse err = tokyo2006_psp_joystick_ErrorResponse_init_zero;
        snprintf(err.message, sizeof(err.message), "Failed to decode request");
        resp->which_response_type = tokyo2006_psp_joystick_Response_error_tag;
        resp->response_type.error = err;
        return true;
    }

    int rc = 0;
    switch (req.which_request_type) {
    case tokyo2006_psp_joystick_Request_sample_tag:
        rc = handle_sample_request(&req.request_type.sample, resp);
        break;
    default:
        LOG_WRN("Unsupported psp_joystick request type: %d", req.which_request_type);
        rc = -1;
    }

    if (rc != 0) {
        tokyo2006_psp_joystick_ErrorResponse err = tokyo2006_psp_joystick_ErrorResponse_init_zero;
        snprintf(err.message, sizeof(err.message), "Failed to process request");
        resp->which_response_type = tokyo2006_psp_joystick_Response_error_tag;
        resp->response_type.error = err;
    }
    return true;
}

static int handle_sample_request(const tokyo2006_psp_joystick_SampleRequest *req,
                                 tokyo2006_psp_joystick_Response *resp) {
    LOG_DBG("Received sample request with value: %d", req->value);

#if IS_ENABLED(CONFIG_ZMK_SPLIT_RELAY_EVENT)
    // Split-relay sample: forward the received value to the split
    // peripheral(s) over ZMK's split event-relay as a plain packed C struct
    // (see src/split/template_relay.c). A no-op unless this build is a split
    // central with a connected peripheral.
    template_relay_send_sample(req->value);
#endif

    tokyo2006_psp_joystick_SampleResponse result = tokyo2006_psp_joystick_SampleResponse_init_zero;

    snprintf(result.value, sizeof(result.value), "Hello from firmware! Received: %d", req->value);

    resp->which_response_type = tokyo2006_psp_joystick_Response_sample_tag;
    resp->response_type.sample = result;
    return 0;
}
