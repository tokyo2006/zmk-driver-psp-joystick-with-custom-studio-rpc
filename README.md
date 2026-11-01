# PSP Joystick ZMK Driver with Custom Studio RPC

A ZMK module that turns a two-potentiometer PSP-1000/2000/3000 analog
thumbstick into a pointing device. The stick moves the mouse pointer, a layer
can turn it into a scroll wheel, and DYA Studio can tune and recalibrate it at
runtime through a custom RPC subsystem.

## Features

- Two-channel ADC sampling at 125 Hz (configurable 4-50 ms).
- Boot-time auto-center calibration with a still-stick sanity check.
- Studio-triggered manual calibration for central/unibody builds.
- Runtime deadzone, response curve, speed, smoothing, invert, and swap.
- Persistent settings through `cormoran/zmk-feature-custom-settings`.
- Layer-triggered scrolling using upstream ZMK input processors.
- Split support: the stick can live on the central or a peripheral half.
- No web UI dependency; the firmware exposes a custom Studio RPC subsystem.

## Required ZMK

This module targets `cormoran/zmk` with the custom Studio RPC protocol:

```yaml
manifest:
  remotes:
    - name: cormoran
      url-base: https://github.com/cormoran
    - name: tokyo2006
      url-base: https://github.com/tokyo2006
  projects:
    - name: zmk
      remote: cormoran
      revision: main+custom-studio-protocol
      import:
        file: app/west.yml
    - name: zmk-driver-psp-joystick-with-custom-studio-rpc
      remote: tokyo2006
      revision: main
      import: true
```

Enable the module in your `config/<shield>.conf`:

```conf
CONFIG_INPUT=y
CONFIG_ADC=y
CONFIG_ADC_NRFX_SAADC=y
CONFIG_ZMK_POINTING=y
CONFIG_ZMK_PSP_JOYSTICK=y

# Optional: custom Studio RPC
CONFIG_ZMK_STUDIO=y
CONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y
CONFIG_ZMK_CUSTOM_SETTINGS=y
CONFIG_ZMK_CUSTOM_SETTINGS_STUDIO_RPC=y
CONFIG_ZMK_STUDIO_RPC_RX_BUF_SIZE=128
CONFIG_ZMK_LOW_PRIORITY_THREAD_STACK_SIZE=2048

# Optional: split relay (both halves)
CONFIG_ZMK_SPLIT_RELAY_EVENT=y
CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY=y
```

## Wiring

The common PSP thumbstick has four pads: two potentiometer wipers (X and Y)
and a shared VCC/GND pair.

| Stick pad | Connect to |
|---|---|
| X | an nRF52840 AIN pin configured as an ADC channel |
| Y | a second AIN pin configured as an ADC channel |
| VCC | the same 3.3 V rail used by the MCU |
| GND | GND |

Power the potentiometers from the same 3.3 V rail. The nRF52840 SAADC must use
`ADC_GAIN_1_6` with the internal 0.6 V reference so the full 3.3 V swing is in
range (6 × 0.6 V = 3.6 V). The measurements are ratiometric; do not power the
stick from a different rail than the one the ADC range assumes.

## Device Tree

### Unibody or central-installed stick

```dts
#include <zephyr/dt-bindings/adc/adc.h>
#include <zephyr/dt-bindings/adc/nrf-adc.h>

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN0>;
        zephyr,resolution = <12>;
    };

    channel@1 {
        reg = <1>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <NRF_SAADC_AIN1>;
        zephyr,resolution = <12>;
    };
};

/ {
    psp_joystick: psp_joystick {
        compatible = "tokyo2006,psp-joystick";
        io-channels = <&adc 0>, <&adc 1>;
        io-channel-names = "x", "y";
        deadzone-percent = <8>;
        max-speed = <8>;
        curve-exponent-x100 = <100>;
        /* invert-x; */
        /* invert-y; */
        /* swap-xy; */
    };

    psp_joystick_listener: psp_joystick_listener {
        compatible = "zmk,input-listener";
        device = <&psp_joystick>;
    };
};
```

### Stick on a split peripheral

Shared `.dtsi` included by both halves:

```dts
#include <tokyo2006/psp_joystick.dtsi>

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

Peripheral overlay:

```dts
#include "<keyboard>.dtsi"

&psp_joystick_split {
    device = <&psp_joystick>;
};
```

Central overlay:

```dts
#include "<keyboard>.dtsi"

&psp_joystick_listener {
    status = "okay";
};
```

The `psp_joystick` node itself (ADC channels plus the `tokyo2006,psp-joystick`
node) goes in the peripheral overlay. The central only needs the listener and
the split proxy node.

## Scroll Layer

Add a layer override to the listener. On the scroll layer, the code mapper
turns X/Y into horizontal/vertical wheel and the scaler sets the step size:

```dts
#include <tokyo2006/psp_joystick.dtsi>

&psp_joystick_listener {
    scroll {
        layers = <SCROLL>;
        input-processors = <&psp_scroll_map>, <&psp_scroll_scale 1 24>;
    };
};
```

Because the override does not set `process-next`, it replaces the base
processors, so the pointer does not move while scrolling. Lower the second
scaler cell (divisor `24`) to scroll faster; raise it to scroll slower. (The
first cell is the multiplier.)

## Studio RPC

Subsystem identifier: `tokyo2006__psp_joystick`.

| Field | Range | Meaning |
|---|---|---|
| `deadzone_percent` | 0-50 | Deadzone as a percentage of full deflection |
| `max_speed` | 1-64 | REL units per sample at full deflection |
| `curve_exponent_x100` | 100/150/200/250/300 | Linear, 1.5, quadratic, 2.5, cubic |
| `smoothing_percent` | 0-90 | EMA weight of the new sample |
| `sample_interval_ms` | 4-50 | Sampling period |
| `invert_x`, `invert_y`, `swap_xy` | bool | Axis transforms |
| `center_x`, `center_y` | 0-4095 | Raw ADC center |
| `min_x`, `max_x`, `min_y`, `max_y` | 0-4095 | Raw ADC range |

Requests: `GetState`, `SetState` (with `persist`), `Calibrate` (with `persist`),
and `Reset` (with `factory_defaults`).

`Calibrate` reads the last 16 samples without blocking. Release the stick
before calibrating. The request fails with
`"joystick is moving; release it and retry"` if the stick is moving, and with
`"not enough samples yet; try again"` when fewer than 16 samples have been
collected.

## Calibration and Persistence

- Without a persisted center, the driver auto-centers on boot by collecting a
  500 ms window; without custom settings, this happens on every boot. The
  measurement is rejected if the stick is moving or, when `center-x`/`center-y`
  are set, far from that DT expected center; otherwise the median becomes the
  center.
- When custom settings are enabled and a center was persisted, boot-time
  auto-centering is skipped.
- `SetState(persist=true)` and `Calibrate(persist=true)` write every field.
- `Reset` always restores the in-memory defaults; `Reset(factory_defaults=true)`
  additionally erases the persisted records.

## Split Limitation

Manual `Calibrate` requires a local ADC device. If the stick is on a split
peripheral, the central cannot measure it synchronously inside an RPC call, so
`Calibrate` returns:

```text
calibration requires a local device; the split peripheral auto-centers at boot
```

The peripheral still auto-centers at boot, and every other setting relays from
the central normally.

## Testing

```bash
# Host math tests (any machine with a C compiler)
python3 -m unittest test_math

# Firmware build tests (needs an initialized west workspace)
bash scripts/setup_workspace.sh
python3 -m unittest test
```

## License

MIT
