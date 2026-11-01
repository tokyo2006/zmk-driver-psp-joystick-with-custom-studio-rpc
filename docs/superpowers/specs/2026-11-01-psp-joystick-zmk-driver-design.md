# PSP Joystick ZMK Driver with Custom Studio RPC — Design

Date: 2026-11-01
Status: Approved for planning
Repository: `tokyo2006/zmk-driver-psp-joystick-with-custom-studio-rpc`

## 1. Problem and Context

A PSP-1000/2000/3000 analog thumbstick is two ratiometric 10 kΩ
potentiometers sharing VCC and GND. It has no digital interface: the host
must sample two ADC channels, remove the mechanical center offset, apply a
deadzone and a response curve, and emit relative motion.

We want to use it on a ZMK keyboard as a **pointing device** (mouse pointer),
with a **layer-triggered scroll mode**. Configuration must be readable and
writable at runtime through DYA Studio over a custom RPC subsystem, persisted
across reboots, and synchronized between split halves.

The target SoC is the nRF52840 (SAADC). The reference ZMK tree is
`cormoran/zmk` on a branch that provides the custom Studio RPC protocol and,
optionally, custom settings (`main+custom-studio-protocol`, `main+dya`).

## 2. Goals

- Sample two ADC channels and emit `INPUT_REL_X` / `INPUT_REL_Y` events that
  ZMK's pointing pipeline consumes as mouse movement.
- Auto-calibrate the stick center at boot, with a Studio-triggered manual
  recalibration that persists.
- Apply configurable deadzone, response curve, speed limit, smoothing, axis
  inversion, and axis swap in the driver.
- Provide runtime get/set/reset/calibrate through a custom Studio RPC
  subsystem named `tokyo2006__psp_joystick`.
- Persist configuration through the cormoran `ZMK_CUSTOM_SETTINGS` API when
  available; degrade to in-memory state otherwise.
- Work whether the stick is on the central half or a split peripheral.
- Provide a layer-triggered scroll mode using upstream ZMK input processors.
- Be testable: motion math is pure C with unit tests; CI compiles a
  representative nRF52840 build.

## 3. Non-Goals

- Web/React configuration UI in this repository (planned separately).
- Standard HID gamepad output. ZMK does not expose a gamepad HID descriptor;
  adding one is out of scope.
- Joystick push-button input. The common PSP thumbstick module has no switch;
  if a variant does, it should be a separate GPIO key.
- Absolute-position pointing mode.
- Supporting RP2040 in v1, though the driver stays SoC-agnostic and the DT
  binding does not assume Nordic.

## 4. Approaches Considered

| Approach | Description | Decision |
|---|---|---|
| **A. Zephyr input driver + ZMK pointing** | Driver emits REL events; ZMK listener, HID, input-split, and input processors do the rest | **Chosen** |
| B. Direct HID calls in the driver | Driver calls `zmk_hid_mouse_movement()` directly | Rejected: bypasses input processors, scroll layer, split relay, and testability |
| C. Sensor driver | Expose axes as sensor channels | Rejected: does not match ZMK's pointing pipeline |

## 5. Architecture and Data Flow

```
PSP stick (X/Y potentiometers)
  -> nRF SAADC (2 channels, 12-bit)
  -> input_psp_joystick driver
       sample -> auto-center -> normalize -> deadzone -> curve -> speed -> remainder
  -> INPUT_REL_X / INPUT_REL_Y
  -> zmk,input-listener (central)
       base: pointer movement
       layer override: code-mapper X->HWHEEL, Y->WHEEL + scaler
  -> ZMK HID mouse report

Split peripheral:
  input_psp_joystick -> zmk,input-split -> radio -> central proxy device
  -> central zmk,input-listener (same processors)
```

Runtime state lives on both halves. The central owns Studio RPC and persistent
storage; it relays state changes to the peripheral. Input events themselves
use ZMK's existing `zmk,input-split` transport and are not re-implemented.

## 6. Module Layout

Implementation base: bootstrap from
`cormoran/zmk-module-template-with-custom-studio-rpc` and run its
`scripts/init_module.py --namespace tokyo2006 --module psp-joystick --repo
zmk-driver-psp-joystick-with-custom-studio-rpc`. That supplies the CMake,
Kconfig, `zephyr/module.yml`, CI workflows, Studio-RPC scaffolding, and build
test harness. The web UI, Renode, and BabbleSim test suites are removed for
v1 (no web UI, and the driver is verified by host math tests plus build
tests).

Files provided by the template and kept:

```
CMakeLists.txt
Kconfig
zephyr/module.yml
.github/workflows/zmk-module.yml
tests/zmk-config/...
tests/studio/...
tests/test.dtsi
```

Feature files added or replaced:

```
CMakeLists.txt
Kconfig
zephyr/module.yml
drivers/input/input_psp_joystick.c
dts/bindings/input/tokyo2006,psp-joystick.yaml
dts/bindings/vendor-prefixes.txt
dts/tokyo2006/psp_joystick.dtsi
include/tokyo2006/psp-joystick/psp_joystick_state.h
include/tokyo2006/psp-joystick/psp_joystick_settings.h
include/tokyo2006/psp-joystick/psp_joystick_relay.h
include/tokyo2006/psp-joystick/psp_joystick_math.h
src/psp_joystick_state.c
src/psp_joystick_math.c
src/studio/psp_joystick_handler.c
src/studio/psp_joystick_settings.c
src/split/psp_joystick_relay.c
proto/tokyo2006/psp-joystick/psp_joystick.proto
proto/tokyo2006/psp-joystick/psp_joystick.options
tests/psp_joystick_math/*.c
README.md
```

`zephyr/module.yml` uses `build.cmake: .`, `build.kconfig: Kconfig`, and
`build.settings.dts_root: .`, matching the existing cirque module.

## 7. Device Tree Binding (`tokyo2006,psp-joystick`)

Required:
- `io-channels`: two ADC channel specifiers, X then Y.
- `io-channel-names = "x", "y"`.

Optional:
- `sample-interval-ms` (int, default 8, range 4–50)
- `deadzone-percent` (int, default 8, range 0–50)
- `max-speed` (int, default 8, range 1–64) — REL units per sample at full
  deflection
- `curve-exponent-x100` (int, default 100 = linear; allowed values
  `100`, `150`, `200`, `250`, `300`)
- `smoothing-percent` (int, default 0, range 0–90)
- `invert-x` (bool, default false)
- `invert-y` (bool, default false)
- `swap-xy` (bool, default false)
- `auto-center` (int, default 1, enum 0/1; 0 disables boot auto-centering)
- `auto-center-ms` (int, default 500, range 100–2000)
- `center-x`, `center-y`, `min-x`, `max-x`, `min-y`, `max-y` (int, optional raw
  ADC values; used as fallback and as the expected midpoint check)

The binding includes `base.yaml` and declares `io-channels` /
`io-channel-names` directly. Channel configuration (gain, reference,
`input-positive`, resolution) remains in the board/user overlay, because pins
and SAADC input numbers are board-specific.

nRF52840 wiring note: power the stick from the same 3.3 V rail and use
`ADC_GAIN_1_6` with the internal 0.6 V reference so the full 3.3 V swing is in
range (6 × 0.6 V = 3.6 V). The potentiometers are ratiometric; do not power
them from a different rail than the ADC reference assumptions.

## 8. Driver Behavior

### 8.1 Sampling

- A `k_work_delayable` reschedules itself every `sample-interval-ms`.
- Each tick reads X and Y with `adc_read_dt()`, using one `adc_sequence` per
  channel. Two 12-bit conversions per tick are cheap at 125 Hz.
- Reads run on the system work queue. If profiling later shows contention, the
  driver can move to a dedicated work queue without changing the API.
- The work item has two phases. While calibrating, ticks feed the calibration
  ring buffer and no input events are emitted. After the calibration window
  closes, ticks emit pointer events. Reporting therefore starts at most one
  `sample-interval-ms` after the calibration window ends.

### 8.2 Auto-center

On the first boot without a persisted center:

1. Collect `auto-center-ms / sample-interval-ms` raw samples per axis into a
   fixed 64-entry ring buffer (no heap).
2. Compute the median of the collected samples.
3. Reject the result if `max - min` over the samples exceeds 20% of the ADC
   range, or if the median is farther than 25% of the range from the DT
   expected midpoint (when `center-x`/`center-y` are present). A rejected
   calibration logs a warning and keeps the DT/persisted center.
4. Store the center in runtime state. Do not persist automatically; only the
   Studio `Calibrate` request persists.

If persisted settings provide a center, auto-center is skipped.

Ordering: the driver does not start the calibration phase until either
`psp_joystick_settings_load_cb()` has run (it sets `settings_loaded`) or a 1 s
fallback timeout expires (also used when `CONFIG_ZMK_CUSTOM_SETTINGS` is
disabled). This avoids overwriting a persisted center with a boot-time
measurement.

The ring buffer keeps running after calibration so a manual `Calibrate`
request can read the current median and spread without blocking the Studio
work queue. `Calibrate` requires `max - min` over the last 16 samples to be
below 10% of the ADC range; otherwise it fails with
`"joystick is moving; release it and retry"`.

### 8.3 Normalization

For each axis with raw value `r`, center `c`, minimum `min`, maximum `max`:

```
delta = r - c
if delta >= 0: norm =  delta * 32767 / max(1, max - c)
else:          norm = -(-delta * 32767) / max(1, c - min)
clamp(norm, -32767, 32767)
```

Defaults when DT min/max are absent: `min = 0`, `max = 4095` for a 12-bit ADC,
with `center = 2048` if no calibrated center is available. Studio may overwrite
min/max later.

### 8.4 Deadzone and rescale

```
dz = deadzone_percent * 32767 / 100
if |norm| <= dz: norm = 0
else: norm = sign(norm) * (|norm| - dz) * 32767 / (32767 - dz)
```

This preserves full-scale reachability after the deadzone.

### 8.5 Response curve, speed, remainder

Work in a 0..32767 domain. Let `x = |norm|`. `curve_exponent_x100` is
restricted at validation time to the set `{100, 150, 200, 250, 300}` so the
implementation stays exact and testable with integer math:

- `100`: `y = x`
- `150`: `y = (x * isqrt64(x * 32767)) / 32767`
- `200`: `y = (x * x) / 32767`
- `250`: `y = (x * x * isqrt64(x * 32767)) / (32767 * 32767)`
- `300`: `y = (x * x * x) / (32767 * 32767)`

`isqrt64()` is an integer square root returning `floor(sqrt(v))`. Intermediate
products use `int64_t`. Every formula is monotonic and maps `0 -> 0` and
`32767 -> 32767`.

Then:

```
target   = sign(norm) * (int64_t)y * max_speed / 32767   // REL units/sample
scaled   = target + remainder
out      = (int16_t)scaled                               // truncation toward zero
remainder = scaled - out
```

`remainder` is `int32_t` per axis and is reset when the axis returns to zero.
Remainder accumulation prevents slow stick pushes from being lost to integer
truncation.

Optional smoothing (`smoothing-percent > 0`) applies a one-pole EMA to the
normalized value before the curve. Default 0 keeps latency minimal.

### 8.6 Axis transforms

Apply in this order, after normalization:

1. `swap-xy`
2. `invert-x` / `invert-y`

The state values, not the DT booleans directly, are authoritative so Studio can
change them live.

### 8.7 Event emission

- Compute `dx`, `dy`.
- If `dx == 0 && dy == 0` and the previous report was also zero, emit nothing.
- If `dx == 0 && dy == 0` and the previous report was non-zero, emit one zero
  `REL_X`/`REL_Y` pair with `sync = true` to stop motion.
- Otherwise emit `input_report_rel(dev, INPUT_REL_X, dx, false, K_NO_WAIT)` and
  `input_report_rel(dev, INPUT_REL_Y, dy, true, K_NO_WAIT)`.
- Both axes are emitted together so the listener sees one coherent sample per
  sync.

## 9. Runtime State

`struct psp_joystick_runtime_state` fields and defaults:

| Field | Type | Default | Notes |
|---|---|---|---|
| `deadzone_percent` | u8 | 8 | 0–50 |
| `max_speed` | u8 | 8 | REL units/sample |
| `curve_exponent_x100` | u16 | 100 | {100,150,200,250,300} |
| `smoothing_percent` | u8 | 0 | 0–90 |
| `sample_interval_ms` | u8 | 8 | 4–50 |
| `invert_x` | bool | false | |
| `invert_y` | bool | false | |
| `swap_xy` | bool | false | |
| `center_x` | i16 | from DT/auto | raw ADC |
| `center_y` | i16 | from DT/auto | raw ADC |
| `min_x` | i16 | 0 | raw ADC |
| `max_x` | i16 | 4095 | raw ADC |
| `min_y` | i16 | 0 | raw ADC |
| `max_y` | i16 | 4095 | raw ADC |

The scroll step is not part of runtime state in v1; it is a DT scaler cell
(§11). Runtime scroll tuning would require a custom processor and is deferred.

The driver owns the state struct in its device data and exposes it through a
`psp_joystick_driver_get_state(dev)` function. `src/psp_joystick_state.c`
provides getters/setters and a static fallback state for builds or split halves
without a local device, mirroring the cirque design. Setters validate ranges,
write the field, and raise `zmk_psp_joystick_status_changed` so other
components can react.

## 10. Studio RPC

Subsystem identifier: `tokyo2006__psp_joystick`.
Proto package: `tokyo2006.psp_joystick`.

```proto
syntax = "proto3";
package tokyo2006.psp_joystick;

message PspJoystickState {
  optional uint32 deadzone_percent      = 1;
  optional uint32 max_speed             = 2;
  optional uint32 curve_exponent_x100   = 3;
  optional uint32 smoothing_percent     = 4;
  optional uint32 sample_interval_ms    = 5;
  optional bool   invert_x              = 6;
  optional bool   invert_y              = 7;
  optional bool   swap_xy               = 8;
  optional int32  center_x              = 9;
  optional int32  center_y              = 10;
  optional int32  min_x                 = 11;
  optional int32  max_x                 = 12;
  optional int32  min_y                 = 13;
  optional int32  max_y                 = 14;
}

message GetStateRequest {}
message GetStateResponse { PspJoystickState state = 1; }

message SetStateRequest {
  PspJoystickState state = 1;
  bool persist = 2;
}
message SetStateResponse { PspJoystickState state = 1; bool persisted = 2; }

message CalibrateRequest { bool persist = 1; }
message CalibrateResponse {
  PspJoystickState state = 1;
  int32 measured_center_x = 2;
  int32 measured_center_y = 3;
}

message ResetRequest { bool factory_defaults = 1; }
message ResetResponse { PspJoystickState state = 1; }

message Request {
  oneof request_type {
    GetStateRequest  get_state = 1;
    SetStateRequest  set_state = 2;
    CalibrateRequest calibrate = 3;
    ResetRequest     reset     = 4;
  }
}

message ErrorResponse { string message = 1; }

message Response {
  oneof response_type {
    ErrorResponse      error     = 1;
    GetStateResponse   get_state = 2;
    SetStateResponse   set_state = 3;
    CalibrateResponse  calibrate = 4;
    ResetResponse      reset     = 5;
  }
}
```

All `optional` fields must set the nanopb `has_*` flag when returning state;
otherwise proto3 presence omits them. `psp_joystick.options` sets
`ErrorResponse.message max_size:128`.

`SetState` applies every present field, does not abort on the first validation
error, reports the first failure in `ErrorResponse`, and on success calls
`psp_joystick_state_apply_all()` so the driver picks up the new values on the
next sample tick. `Calibrate` reads the current median from the driver's raw
ring buffer (no blocking), requires the stick to be near-still, updates
`center_x`/`center_y`, and persists when requested. If no local device exists it
returns the split-calibration error described in §13. `Reset` restores defaults
and persists their removal when `factory_defaults` is true. `SetState` and
`Reset` relay the resulting state to peripheral halves. `Calibrate` does not
relay; it operates only on a local device.

The subsystem is registered **unsecured** by default, matching cirque, so the
DYA Studio UI can read and write without an unlock prompt.

## 11. Scroll Mode

Scroll uses upstream ZMK input processors; no custom processor is required in
v1.

The module ships `dts/tokyo2006/psp_joystick.dtsi`, which defines only the
scroll processor helpers:

- `psp_scroll_map`: `zmk,input-processor-code-mapper` mapping
  `INPUT_REL_X -> INPUT_REL_HWHEEL`, `INPUT_REL_Y -> INPUT_REL_WHEEL`
- `psp_scroll_scale`: `zmk,input-processor-scaler` over `WHEEL`/`HWHEEL` with
  `track-remainders`

The listener node is defined by the keyboard, not by the module, so the same
label is not declared twice when the split helper is also included. Unibody or
central-installed example:

```dts
#include <tokyo2006/psp_joystick.dtsi>

/ {
    psp_joystick_listener: psp_joystick_listener {
        compatible = "zmk,input-listener";
        device = <&psp_joystick>;
    };
};

&psp_joystick_listener {
    scroll {
        layers = <SCROLL>;
        input-processors = <&psp_scroll_map>, <&psp_scroll_scale 1 24>;
    };
};
```

Because the listener applies a layer override and `process-next` is unset, the
override **replaces** the base processor list on the scroll layer, so no pointer
movement is produced while scrolling.

The scroll step is the second scaler cell (`24` above), which is a divisor:
lower it to scroll faster and raise it to scroll slower. Axis inversion for
scrolling is done with the scaler's sign convention or a second transform
processor in DT. Runtime scroll tuning is not part of v1 and would require a
custom processor that reads runtime state.

## 12. Persistence

`src/studio/psp_joystick_settings.c` defines one
`ZMK_CUSTOM_SETTING_DEFINE` per numeric/bool field under namespace
`tokyo2006__psp_joystick`, guarded by `CONFIG_ZMK_CUSTOM_SETTINGS`. Signed
`INT32` storage is used for all numbers and re-cast on read/write, matching the
cirque implementation.

- `psp_joystick_settings_save_all(dev)` called on `SetState`/`Calibrate` with
  `persist = true`.
- `psp_joystick_settings_reset_all(dev)` called on `Reset` with
  `factory_defaults = true`.
- `psp_joystick_settings_load_cb(dev)` subscribed to
  `zmk_custom_settings_initialized`; applies persisted values after boot.
- Without `CONFIG_ZMK_CUSTOM_SETTINGS`, all three are no-ops returning 0.

## 13. Split Support

Input events use the standard ZMK input-split pattern:

```dts
/* shared dtsi */
/ {
    split_inputs {
        #address-cells = <1>;
        #size-cells = <0>;

        psp_joystick_split: psp_joystick_split@0 {
            compatible = "zmk,input-split";
            reg = <0>;
        };
    };

    psp_joystick_listener: psp_joystick_listener {
        compatible = "zmk,input-listener";
        status = "disabled";
        device = <&psp_joystick_split>;
    };
};
```

- Peripheral overlay: `&psp_joystick_split { device = <&psp_joystick>; };`
- Central overlay: `&psp_joystick_listener { status = "okay"; };`
- Split central with the stick installed locally: skip the split node and bind
  the listener directly with `device = <&psp_joystick>`.
- The shared dtsi also includes `<tokyo2006/psp_joystick.dtsi>` so the keymap
  can attach the scroll override to this listener label.

The central listener applies pointer and scroll processors; the peripheral
`zmk,input-split` node carries no processors unless the axes need a
peripheral-side fix-up.

Configuration state: `src/split/psp_joystick_relay.c` carries a packed struct
of the runtime state via `ZMK_RELAY_EVENT_CENTRAL_TO_PERIPHERAL` /
`ZMK_RELAY_EVENT_HANDLE`, gated by `CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY` and
`CONFIG_ZMK_SPLIT`. The central sends after `SetState`/`Reset`; the peripheral
applies without persisting.

The state layer's fallback state keeps the RPC functional on a half without a
local device (e.g., Studio over USB on the central while the stick is on the
peripheral).

**Calibration limitation in v1:** `Calibrate` requires a local ADC device. If
the stick lives on a peripheral, the central cannot measure it synchronously
within an RPC call. The peripheral still auto-centers at boot, and all other
settings relay normally. Manual calibration from
Studio is therefore available for unibody and central-installed sticks; a
peripheral round-trip calibration (request relay plus measured-center relay
back) is deferred. The RPC returns `"calibration requires a local device; the
split peripheral auto-centers at boot"` in that case.

## 14. Error Handling

- ADC setup failures log an error and leave the device non-functional rather
  than crashing. `adc_is_ready_dt()` is checked at init.
- Invalid raw samples (negative after conversion, outside 0–4095 for 12-bit)
  are clamped before normalization.
- Auto-center rejection falls back to DT/persisted center.
- Studio validation rejects out-of-range fields and reports the first failure;
  other valid fields in the same request still apply.
- Missing `CONFIG_ZMK_CUSTOM_SETTINGS` is not an error; persistence is skipped.
- No local device on a split half is not an error; the fallback state is used.

## 15. Testing

- **Math unit tests** (`tests/psp_joystick_math/`): plain C harness built with
  the host C compiler and wrapped by `test_math.py`, so `python3 -m unittest`
  runs it on any developer machine without a Zephyr native-posix toolchain.
  Cover center mapping, deadzone boundary and rescale, full-scale clamp,
  curve monotonicity, invert/swap order, remainder accumulation, and zero
  output at rest.
- **Build test** (template `tests/zmk-config/`, extended): CI builds
  `xiao_ble//zmk` (nRF52840) with `CONFIG_ZMK_POINTING`, `CONFIG_ADC`,
  `CONFIG_ADC_NRFX_SAADC`, `CONFIG_INPUT`, Studio RPC enabled, and custom
  settings enabled. The `tester_xiao.overlay` instantiates the driver on two
  ADC channels and an input listener. A fourth artifact uses the wired-split
  shield to compile the split-relay path.
- **Manual smoke test**: README provides a minimal keymap, a scroll layer, and
  example Studio RPC calls. Acceptance: pointer moves in all directions,
  returns to rest with no drift, scroll layer converts motion to wheel, reboot
  keeps persisted settings, and recalibration changes the resting point.

## 16. Kconfig

```
menuconfig ZMK_PSP_JOYSTICK
    bool "Enable psp-joystick feature"
    help
      Enable PSP thumbstick state, Studio RPC, and the ADC driver.

if ZMK_PSP_JOYSTICK

config ZMK_PSP_JOYSTICK_DRIVER
    bool "PSP joystick ADC input driver"
    default y
    depends on INPUT && ADC
    depends on DT_HAS_TOKYO2006_PSP_JOYSTICK_ENABLED

config ZMK_PSP_JOYSTICK_STUDIO_RPC
    bool "Enable PSP joystick Studio RPC subsystem"
    depends on ZMK_STUDIO
    help
      Register the tokyo2006__psp_joystick RPC subsystem.

config ZMK_PSP_JOYSTICK_SPLIT_RELAY
    bool "Relay PSP joystick configuration to split peripherals"
    depends on ZMK_SPLIT && ZMK_SPLIT_RELAY_EVENT && ZMK_SPLIT_BLE

module = ZMK_PSP_JOYSTICK
module-str = ZMK_PSP_JOYSTICK
source "${ZEPHYR_BASE}/subsys/logging/Kconfig.template.log_config"

endif
```

The user application must still enable `CONFIG_ZMK_POINTING=y`.

## 17. Version Compatibility

- ZMK: `cormoran/zmk` branch with custom Studio RPC
  (`main+custom-studio-protocol` or `main+dya`).
- Persistence: optional `ZMK_CUSTOM_SETTINGS` from
  `cormoran/zmk-feature-custom-settings`.
- Zephyr: the version pinned by that ZMK tree; uses `input_report_rel`,
  `adc_dt_spec`, `k_work_delayable`, and the input relay macros already present
  in the tree.
- SoC: nRF52840 verified in CI. The binding is SoC-agnostic; RP2040 support is
  a follow-up once a board overlay is added.

## 18. Open Decisions

None blocking. The following are explicitly deferred:

- Runtime scroll divisor/invert (v1 uses DT scaler cells).
- Web UI.
- Gamepad HID.
- RP2040 CI matrix.
