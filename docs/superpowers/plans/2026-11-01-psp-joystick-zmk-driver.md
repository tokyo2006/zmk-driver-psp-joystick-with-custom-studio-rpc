# PSP Joystick ZMK Driver with Custom Studio RPC — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ZMK module that turns a two-potentiometer PSP thumbstick into a pointing device with layer-triggered scrolling, runtime tuning through a DYA Studio custom RPC, persistence, and split-half support.

**Architecture:** A Zephyr input driver samples two nRF SAADC channels, auto-calibrates the center, normalizes and shapes the axes, and emits `INPUT_REL_X` / `INPUT_REL_Y`. ZMK's existing `zmk,input-listener` and `zmk,input-split` carry those events to HID. A runtime state layer feeds a custom Studio RPC subsystem (`tokyo2006__psp_joystick`) and optional cormoran custom-settings persistence. A packed relay carrier keeps the peripheral's runtime state in sync with the central.

**Tech Stack:** C11, ZMK (`cormoran/zmk` with custom Studio RPC), Zephyr input/ADC/kernel APIs, nanopb/protobuf, plain C host tests, GitHub Actions (`zmkfirmware/zmk-build-arm`).

**Spec:** `docs/superpowers/specs/2026-11-01-psp-joystick-zmk-driver-design.md`

## Global Constraints

- Module namespace `tokyo2006`; subsystem identifier `tokyo2006__psp_joystick`; proto package `tokyo2006.psp_joystick`.
- Kconfig prefix `ZMK_PSP_JOYSTICK`; child symbols `ZMK_PSP_JOYSTICK_DRIVER`, `ZMK_PSP_JOYSTICK_STUDIO_RPC`, `ZMK_PSP_JOYSTICK_SPLIT_RELAY`.
- Target SoC for CI: nRF52840 (`xiao_ble//zmk`). The DT binding stays SoC-agnostic.
- No web UI, no gamepad HID, no RP2040 CI matrix in this plan.
- Runtime scroll step is a DT scaler cell; `scroll_divisor`/`scroll_invert` are not part of runtime state.
- Axis transforms order: swap X/Y, then invert X, then invert Y.
- `curve_exponent_x100` is restricted to `{100, 150, 200, 250, 300}`.
- All new files are MIT licensed.
- Do not commit secrets.

---

## File Structure

Bootstrap files from `cormoran/zmk-module-template-with-custom-studio-rpc` (kept): `CMakeLists.txt`, `Kconfig`, `zephyr/module.yml`, `.github/workflows/zmk-module.yml`, `scripts/`, `west/`, `tests/zmk-config/`, `tests/studio/`, `tests/test.dtsi`.

Feature files created or replaced:

```
CMakeLists.txt
Kconfig
drivers/input/input_psp_joystick.c
dts/bindings/input/tokyo2006,psp-joystick.yaml
dts/bindings/vendor-prefixes.txt
dts/tokyo2006/psp_joystick.dtsi
include/tokyo2006/psp-joystick/psp_joystick_math.h
include/tokyo2006/psp-joystick/psp_joystick_state.h
include/tokyo2006/psp-joystick/psp_joystick_settings.h
include/tokyo2006/psp-joystick/psp_joystick_relay.h
src/psp_joystick_math.c
src/psp_joystick_state.c
src/studio/psp_joystick_handler.c
src/studio/psp_joystick_settings.c
src/split/psp_joystick_relay.c
proto/tokyo2006/psp-joystick/psp_joystick.proto
proto/tokyo2006/psp-joystick/psp_joystick.options
tests/psp_joystick_math/test_math.c
test_math.py
tests/zmk-config/config/tester_xiao.overlay
tests/zmk-config/config/renode_usb_wired_split.overlay
README.md
```

---

## Task 1: Bootstrap from the module template and trim scope

**Files:**
- Create (copy): entire template tree except `.git`
- Replace: `test.py`, `tests/zmk-config/build.yaml`, `.github/workflows/zmk-module.yml`
- Delete: `web/`, `.github/workflows/web-ui.yml`, `.github/workflows/web-e2e.yml`, `tests/renode/`, `tests/ble/`

**Interfaces:**
- Consumes: empty repository at `/Users/chenzeng/project/zmk-driver-psp-joystick-with-custom-studio-rpc`.
- Produces: initialized module repository with `CONFIG_ZMK_PSP_JOYSTICK` naming, template CI/west scaffolding, and no web/Renode/BLE suites.

- [ ] **Step 1: Copy the template into the empty repository**

Run from the repository root:

```bash
TEMPLATE_DIR="$(mktemp -d)"
git clone --depth 1 https://github.com/cormoran/zmk-module-template-with-custom-studio-rpc.git "$TEMPLATE_DIR"
rsync -a --exclude .git "$TEMPLATE_DIR"/ .
rm -rf "$TEMPLATE_DIR"
git add -A
git commit -m "chore: import module template"
```

Expected: the template files are committed. `scripts/init_module.py` uses `git ls-files`, so this commit is required before the next step.

- [ ] **Step 2: Initialize the template placeholders**

```bash
python3 scripts/init_module.py \
  --namespace tokyo2006 \
  --module psp-joystick \
  --repo zmk-driver-psp-joystick-with-custom-studio-rpc
python3 scripts/init_module.py --verify-only
```

Expected: `OK: no template placeholders left.` If anything remains, fix exactly those lines and re-run `--verify-only`.

- [ ] **Step 3: Remove the out-of-scope suites**

```bash
git rm -r web .github/workflows/web-ui.yml .github/workflows/web-e2e.yml tests/renode tests/ble
```

- [ ] **Step 4: Replace `test.py` with the lean build-test runner**

```python
from __future__ import annotations

import shutil
import subprocess
import unittest
from pathlib import Path

THIS_DIR = Path(__file__).parent.resolve()

ARTIFACTS = {
    "psp_joystick_basic": [
        "CONFIG_ZMK_PSP_JOYSTICK=y",
        "CONFIG_ZMK_PSP_JOYSTICK_DRIVER=y",
        "CONFIG_ZMK_POINTING=y",
        "CONFIG_INPUT=y",
        "CONFIG_ADC=y",
        "CONFIG_ADC_NRFX_SAADC=y",
    ],
    "psp_joystick_studio": [
        "CONFIG_ZMK_PSP_JOYSTICK=y",
        "CONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y",
        "CONFIG_ZMK_STUDIO=y",
    ],
    "psp_joystick_custom_settings": [
        "CONFIG_ZMK_PSP_JOYSTICK=y",
        "CONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y",
        "CONFIG_ZMK_CUSTOM_SETTINGS=y",
    ],
    "psp_joystick_split_central": [
        "CONFIG_ZMK_PSP_JOYSTICK=y",
        "CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY=y",
        "CONFIG_ZMK_SPLIT=y",
        "CONFIG_ZMK_SPLIT_ROLE_CENTRAL=y",
        "CONFIG_ZMK_SPLIT_RELAY_EVENT=y",
    ],
}


class BuildTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        result = subprocess.run(
            ["west", "topdir"], capture_output=True, text=True, cwd=THIS_DIR
        )
        if result.returncode != 0:
            raise unittest.SkipTest(
                "west workspace not initialized; run scripts/setup_workspace.sh"
            )
        cls.build_dir = Path(result.stdout.strip()) / "build"

    def test_zmk_build(self):
        for artifact in ARTIFACTS:
            shutil.rmtree(self.build_dir / artifact, ignore_errors=True)

        result = subprocess.run(
            ["west", "zmk-build", "tests/zmk-config", "-q"],
            capture_output=True,
            text=True,
            cwd=THIS_DIR,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)

        for artifact, expected in ARTIFACTS.items():
            config = self.build_dir / artifact / "zephyr" / ".config"
            self.assertTrue(config.exists(), f"{artifact}: .config missing")
            text = config.read_text()
            for line in expected:
                self.assertIn(line, text, f"{artifact}: {line} missing")
            uf2 = self.build_dir / artifact / "zephyr" / "zmk.uf2"
            self.assertTrue(uf2.exists(), f"{artifact}: zmk.uf2 missing")


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 5: Replace `tests/zmk-config/build.yaml`**

```yaml
include:
  - artifact: psp_joystick_basic
    board: xiao_ble//zmk
    shield: tester_xiao
    cmake-args: -DCONFIG_ZMK_PSP_JOYSTICK=y -DCONFIG_ZMK_POINTING=y

  - artifact: psp_joystick_studio
    board: xiao_ble//zmk
    shield: tester_xiao
    cmake-args: -DCONFIG_ZMK_PSP_JOYSTICK=y -DCONFIG_ZMK_POINTING=y -DCONFIG_ZMK_STUDIO=y -DCONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y
    snippet: studio-rpc-usb-uart

  - artifact: psp_joystick_custom_settings
    board: xiao_ble//zmk
    shield: tester_xiao
    cmake-args: -DCONFIG_ZMK_PSP_JOYSTICK=y -DCONFIG_ZMK_POINTING=y -DCONFIG_ZMK_STUDIO=y -DCONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y -DCONFIG_ZMK_CUSTOM_SETTINGS=y -DCONFIG_ZMK_CUSTOM_SETTINGS_STUDIO_RPC=y
    snippet: studio-rpc-usb-uart

  - artifact: psp_joystick_split_central
    board: xiao_ble//zmk
    shield: renode_usb_wired_split
    cmake-args: -DCONFIG_INPUT=y -DCONFIG_ADC=y -DCONFIG_ADC_NRFX_SAADC=y -DCONFIG_ZMK_POINTING=y -DCONFIG_ZMK_PSP_JOYSTICK=y -DCONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY=y -DCONFIG_ZMK_SPLIT_RELAY_EVENT=y -DCONFIG_ZMK_STUDIO=y -DCONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y
    snippet: studio-rpc-usb-uart
```

- [ ] **Step 6: Replace `.github/workflows/zmk-module.yml`**

```yaml
name: Test ZMK Module

on:
  push:
  pull_request:
  workflow_dispatch:

jobs:
  build:
    runs-on: ubuntu-latest
    timeout-minutes: 45
    container:
      image: zmkfirmware/zmk-build-arm:stable
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Init west workspace
        run: |
          git config --global --add safe.directory "${{ github.workspace }}"
          bash scripts/setup_workspace.sh

      - name: Host math tests
        run: python3 -m unittest test_math

      - name: ZMK build tests
        run: python3 -m unittest test

      - name: Upload firmware
        uses: actions/upload-artifact@v4
        with:
          name: zmk-firmware
          path: build/*/zephyr/zmk.uf2
```

- [ ] **Step 6b: Update `AGENTS.md` for the trimmed scope**

Delete the entire `## Initialization (first time only)` section (from that
heading up to, but not including, `## Dev Rules`). Replace the body of
`## Dev Rules` with:

```markdown
## Dev Rules

- When designing a new module or a major feature, read
  `skills/zmk-module-design/SKILL.md` first.
- Before writing or modifying proto, firmware, or test code, read
  `skills/zmk-module-dev/SKILL.md`. It has the implementation recipe and
  pitfalls that otherwise cause silent runtime failures.
- Commit changes at each milestone. Ensure pre-commit works and never bypass
  pre-commit.
- Write host math tests for pure logic and build tests in `tests/zmk-config/*`.
- For module-owned settings, prefer
  https://github.com/cormoran/zmk-feature-custom-settings.
- Update README.md for unfamiliar ZMK users. Keep it simple but sufficient.
- Create a pull request to origin after finishing.

## Commands

```bash
python3 -m unittest test_math
python3 -m unittest test
pre-commit run
```
```

`CLAUDE.md` is a symlink to `AGENTS.md`; do not edit it separately.

- [ ] **Step 7: Commit the bootstrap**

```bash
git add -A
git commit -m "chore: initialize psp-joystick module and trim v1 scope"
```

Expected: `git status` is clean and `python3 scripts/init_module.py --verify-only` prints OK.

---

## Task 2: Add the motion math and host tests (TDD)

**Files:**
- Create: `tests/psp_joystick_math/test_math.c`
- Create: `test_math.py`
- Create: `include/tokyo2006/psp-joystick/psp_joystick_math.h`
- Create: `src/psp_joystick_math.c`

**Interfaces:**
- Produces:
  - `int16_t psp_joystick_normalize(int32_t raw, int32_t center, int32_t min, int32_t max)`
  - `int16_t psp_joystick_apply_deadzone(int16_t norm, uint8_t deadzone_percent)`
  - `int16_t psp_joystick_apply_curve(int16_t norm, uint16_t exponent_x100)`
  - `int16_t psp_joystick_apply_speed(int16_t norm, uint8_t max_speed, int32_t *remainder)`
  - `void psp_joystick_apply_transforms(int16_t *x, int16_t *y, bool swap_xy, bool invert_x, bool invert_y)`
  - `uint64_t psp_joystick_isqrt64(uint64_t v)`

- [ ] **Step 1: Write the failing host test**

Create `tests/psp_joystick_math/test_math.c`:

```c
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "tokyo2006/psp-joystick/psp_joystick_math.h"

static void test_normalize(void) {
    assert(psp_joystick_normalize(2048, 2048, 0, 4095) == 0);
    assert(psp_joystick_normalize(4095, 2048, 0, 4095) == 32767);
    assert(psp_joystick_normalize(0, 2048, 0, 4095) == -32767);
    assert(psp_joystick_normalize(3071, 2048, 0, 4095) == 16383);
    assert(psp_joystick_normalize(5000, 2048, 0, 4095) == 32767);
    assert(psp_joystick_normalize(-100, 2048, 0, 4095) == -32767);
    assert(psp_joystick_normalize(1000, 1000, 0, 4095) == 0);
    assert(psp_joystick_normalize(1000, 1000, 1000, 1000) == 0);
}

static void test_deadzone(void) {
    assert(psp_joystick_apply_deadzone(0, 8) == 0);
    assert(psp_joystick_apply_deadzone(2621, 8) == 0);
    assert(psp_joystick_apply_deadzone(-2621, 8) == 0);
    assert(psp_joystick_apply_deadzone(3000, 8) == 411);
    assert(psp_joystick_apply_deadzone(-3000, 8) == -411);
    assert(psp_joystick_apply_deadzone(32767, 8) == 32767);
    assert(psp_joystick_apply_deadzone(-32767, 8) == -32767);
    assert(psp_joystick_apply_deadzone(5000, 0) == 5000);
}

static void test_curve(void) {
    assert(psp_joystick_apply_curve(0, 100) == 0);
    assert(psp_joystick_apply_curve(0, 300) == 0);
    assert(psp_joystick_apply_curve(32767, 100) == 32767);
    assert(psp_joystick_apply_curve(-32767, 100) == -32767);
    assert(psp_joystick_apply_curve(32767, 150) == 32767);
    assert(psp_joystick_apply_curve(32767, 200) == 32767);
    assert(psp_joystick_apply_curve(32767, 250) == 32767);
    assert(psp_joystick_apply_curve(32767, 300) == 32767);
    assert(psp_joystick_apply_curve(16384, 200) == 8192);
    assert(psp_joystick_apply_curve(-16384, 200) == -8192);
    assert(psp_joystick_apply_curve(10000, 200) <=
           psp_joystick_apply_curve(20000, 200));
    assert(psp_joystick_apply_curve(10000, 300) <=
           psp_joystick_apply_curve(20000, 300));
}

static void test_speed_remainder(void) {
    int32_t rem = 0;
    assert(psp_joystick_apply_speed(32767, 8, &rem) == 8);
    assert(rem == 0);

    rem = 0;
    int32_t sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += psp_joystick_apply_speed(1000, 8, &rem);
    }
    assert(sum == 1);

    rem = 0;
    sum = 0;
    for (int i = 0; i < 5; i++) {
        sum += psp_joystick_apply_speed(-1000, 8, &rem);
    }
    assert(sum == -1);

    rem = 0;
    assert(psp_joystick_apply_speed(32767, 0, &rem) == 0);
    assert(rem == 0);
}

static void test_transforms(void) {
    int16_t x = 100;
    int16_t y = 200;

    psp_joystick_apply_transforms(&x, &y, false, true, false);
    assert(x == -100);
    assert(y == 200);

    x = 100;
    y = 200;
    psp_joystick_apply_transforms(&x, &y, false, false, true);
    assert(x == 100);
    assert(y == -200);

    x = 100;
    y = 200;
    psp_joystick_apply_transforms(&x, &y, true, true, true);
    assert(x == -200);
    assert(y == -100);
}

static void test_isqrt(void) {
    assert(psp_joystick_isqrt64(0) == 0);
    assert(psp_joystick_isqrt64(1) == 1);
    assert(psp_joystick_isqrt64(4) == 2);
    assert(psp_joystick_isqrt64(1073676289ULL) == 32767);
}

int main(void) {
    test_normalize();
    test_deadzone();
    test_curve();
    test_speed_remainder();
    test_transforms();
    test_isqrt();
    printf("all math tests passed\n");
    return 0;
}
```

- [ ] **Step 2: Write the failing host-test runner**

Create `test_math.py`:

```python
from __future__ import annotations

import subprocess
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parent


class MathTests(unittest.TestCase):
    def test_math(self):
        with tempfile.TemporaryDirectory() as td:
            exe = Path(td) / "test_math"
            cmd = [
                "cc",
                "-std=c11",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-I",
                str(ROOT / "include"),
                str(ROOT / "tests" / "psp_joystick_math" / "test_math.c"),
                str(ROOT / "src" / "psp_joystick_math.c"),
                "-o",
                str(exe),
            ]
            build = subprocess.run(cmd, capture_output=True, text=True)
            self.assertEqual(build.returncode, 0, build.stdout + build.stderr)

            run = subprocess.run([str(exe)], capture_output=True, text=True)
            self.assertEqual(run.returncode, 0, run.stdout + run.stderr)
            self.assertIn("all math tests passed", run.stdout)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 3: Run the test to verify it fails**

```bash
python3 -m unittest test_math
```

Expected: FAIL — the compiler cannot find `psp_joystick_math.h` / `psp_joystick_math.c`.

- [ ] **Step 4: Write the math header**

Create `include/tokyo2006/psp-joystick/psp_joystick_math.h`:

```c
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
```

- [ ] **Step 5: Write the math implementation**

Create `src/psp_joystick_math.c`:

```c
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
```

- [ ] **Step 6: Run the test to verify it passes**

```bash
python3 -m unittest test_math
```

Expected: PASS, with `all math tests passed` in the output.

- [ ] **Step 7: Commit**

```bash
git add test_math.py tests/psp_joystick_math include/tokyo2006/psp-joystick src/psp_joystick_math.c
git commit -m "feat: add psp joystick motion math and host tests"
```

---

## Task 3: Add the runtime state layer

**Files:**
- Create: `include/tokyo2006/psp-joystick/psp_joystick_state.h`
- Create: `src/psp_joystick_state.c`

**Interfaces:**
- Consumes: weak driver hooks defined in Task 4.
- Produces:
  - `struct psp_joystick_runtime_state` and defaults.
  - `psp_joystick_state_get_struct()`, getters, setters, `psp_joystick_state_load_defaults()`, `psp_joystick_state_apply_all()`.
  - `ZMK_EVENT_DECLARE(zmk_psp_joystick_status_changed)`.
  - Weak declarations: `psp_joystick_driver_get_state()`, `psp_joystick_driver_load_defaults()`, `psp_joystick_driver_apply_state()`, `psp_joystick_driver_calibrate()`, `psp_joystick_driver_on_settings_loaded()`.

- [ ] **Step 1: Write the state header**

Create `include/tokyo2006/psp-joystick/psp_joystick_state.h`:

```c
/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <zephyr/device.h>
#include <zmk/event_manager.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PSP_JOYSTICK_DEADZONE_PERCENT_DEFAULT 8
#define PSP_JOYSTICK_MAX_SPEED_DEFAULT 8
#define PSP_JOYSTICK_CURVE_EXPONENT_X100_DEFAULT 100
#define PSP_JOYSTICK_SMOOTHING_PERCENT_DEFAULT 0
#define PSP_JOYSTICK_SAMPLE_INTERVAL_MS_DEFAULT 8
#define PSP_JOYSTICK_CENTER_DEFAULT 2048
#define PSP_JOYSTICK_MIN_DEFAULT 0
#define PSP_JOYSTICK_MAX_DEFAULT 4095

struct psp_joystick_runtime_state {
    uint8_t deadzone_percent;
    uint8_t max_speed;
    uint16_t curve_exponent_x100;
    uint8_t smoothing_percent;
    uint8_t sample_interval_ms;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
    int16_t center_x;
    int16_t center_y;
    int16_t min_x;
    int16_t max_x;
    int16_t min_y;
    int16_t max_y;
};

struct zmk_psp_joystick_status_changed {
    uint8_t reserved;
};

ZMK_EVENT_DECLARE(zmk_psp_joystick_status_changed);

struct psp_joystick_runtime_state *psp_joystick_state_get_struct(const struct device *dev);

uint8_t psp_joystick_state_get_deadzone_percent(const struct device *dev);
uint8_t psp_joystick_state_get_max_speed(const struct device *dev);
uint16_t psp_joystick_state_get_curve_exponent_x100(const struct device *dev);
uint8_t psp_joystick_state_get_smoothing_percent(const struct device *dev);
uint8_t psp_joystick_state_get_sample_interval_ms(const struct device *dev);
bool psp_joystick_state_get_invert_x(const struct device *dev);
bool psp_joystick_state_get_invert_y(const struct device *dev);
bool psp_joystick_state_get_swap_xy(const struct device *dev);
int16_t psp_joystick_state_get_center_x(const struct device *dev);
int16_t psp_joystick_state_get_center_y(const struct device *dev);
int16_t psp_joystick_state_get_min_x(const struct device *dev);
int16_t psp_joystick_state_get_max_x(const struct device *dev);
int16_t psp_joystick_state_get_min_y(const struct device *dev);
int16_t psp_joystick_state_get_max_y(const struct device *dev);

int psp_joystick_state_set_deadzone_percent(const struct device *dev, int32_t v);
int psp_joystick_state_set_max_speed(const struct device *dev, int32_t v);
int psp_joystick_state_set_curve_exponent_x100(const struct device *dev, int32_t v);
int psp_joystick_state_set_smoothing_percent(const struct device *dev, int32_t v);
int psp_joystick_state_set_sample_interval_ms(const struct device *dev, int32_t v);
int psp_joystick_state_set_invert_x(const struct device *dev, bool v);
int psp_joystick_state_set_invert_y(const struct device *dev, bool v);
int psp_joystick_state_set_swap_xy(const struct device *dev, bool v);
int psp_joystick_state_set_center_x(const struct device *dev, int32_t v);
int psp_joystick_state_set_center_y(const struct device *dev, int32_t v);
int psp_joystick_state_set_min_x(const struct device *dev, int32_t v);
int psp_joystick_state_set_max_x(const struct device *dev, int32_t v);
int psp_joystick_state_set_min_y(const struct device *dev, int32_t v);
int psp_joystick_state_set_max_y(const struct device *dev, int32_t v);

int psp_joystick_state_load_defaults(const struct device *dev);
int psp_joystick_state_apply_all(const struct device *dev);

/*
 * Driver hooks. The state layer links without a driver (Studio-only builds,
 * split halves without the stick) because these are weak declarations.
 */
struct psp_joystick_runtime_state *psp_joystick_driver_get_state(const struct device *dev)
    __attribute__((weak));
int psp_joystick_driver_load_defaults(const struct device *dev) __attribute__((weak));
int psp_joystick_driver_apply_state(const struct device *dev) __attribute__((weak));
int psp_joystick_driver_calibrate(const struct device *dev, int32_t *center_x, int32_t *center_y)
    __attribute__((weak));
void psp_joystick_driver_on_settings_loaded(const struct device *dev, bool center_persisted)
    __attribute__((weak));

#ifdef __cplusplus
}
#endif
```

- [ ] **Step 2: Write the state implementation**

Create `src/psp_joystick_state.c`:

```c
/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

ZMK_EVENT_IMPL(zmk_psp_joystick_status_changed);

static struct psp_joystick_runtime_state fallback_state;
static bool fallback_initialized;

static void set_hard_defaults(struct psp_joystick_runtime_state *s) {
    s->deadzone_percent = PSP_JOYSTICK_DEADZONE_PERCENT_DEFAULT;
    s->max_speed = PSP_JOYSTICK_MAX_SPEED_DEFAULT;
    s->curve_exponent_x100 = PSP_JOYSTICK_CURVE_EXPONENT_X100_DEFAULT;
    s->smoothing_percent = PSP_JOYSTICK_SMOOTHING_PERCENT_DEFAULT;
    s->sample_interval_ms = PSP_JOYSTICK_SAMPLE_INTERVAL_MS_DEFAULT;
    s->invert_x = false;
    s->invert_y = false;
    s->swap_xy = false;
    s->center_x = PSP_JOYSTICK_CENTER_DEFAULT;
    s->center_y = PSP_JOYSTICK_CENTER_DEFAULT;
    s->min_x = PSP_JOYSTICK_MIN_DEFAULT;
    s->max_x = PSP_JOYSTICK_MAX_DEFAULT;
    s->min_y = PSP_JOYSTICK_MIN_DEFAULT;
    s->max_y = PSP_JOYSTICK_MAX_DEFAULT;
}

struct psp_joystick_runtime_state *psp_joystick_state_get_struct(const struct device *dev) {
    if (dev == NULL || psp_joystick_driver_get_state == NULL) {
        if (!fallback_initialized) {
            set_hard_defaults(&fallback_state);
            fallback_initialized = true;
        }
        return &fallback_state;
    }
    return psp_joystick_driver_get_state(dev);
}

static void notify_changed(void) {
    struct zmk_psp_joystick_status_changed ev = {0};
    raise_zmk_psp_joystick_status_changed(ev);
}

#define DEFINE_GETTER(type, name, field)                                                           \
    type psp_joystick_state_get_##name(const struct device *dev) {                                 \
        return psp_joystick_state_get_struct(dev)->field;                                          \
    }

#define DEFINE_SETTER(type, name, field, validate_expr)                                            \
    int psp_joystick_state_set_##name(const struct device *dev, type v) {                          \
        if (!(validate_expr)) {                                                                    \
            return -EINVAL;                                                                        \
        }                                                                                          \
        psp_joystick_state_get_struct(dev)->field = v;                                             \
        notify_changed();                                                                          \
        return 0;                                                                                  \
    }

#define VALID_U8(v, max) ((uint32_t)(v) <= (uint32_t)(max))
#define VALID_I16(v) ((int32_t)(v) >= 0 && (int32_t)(v) <= 4095)
#define VALID_CURVE(v) ((v) == 100 || (v) == 150 || (v) == 200 || (v) == 250 || (v) == 300)

DEFINE_GETTER(uint8_t, deadzone_percent, deadzone_percent)
DEFINE_GETTER(uint8_t, max_speed, max_speed)
DEFINE_GETTER(uint16_t, curve_exponent_x100, curve_exponent_x100)
DEFINE_GETTER(uint8_t, smoothing_percent, smoothing_percent)
DEFINE_GETTER(uint8_t, sample_interval_ms, sample_interval_ms)
DEFINE_GETTER(bool, invert_x, invert_x)
DEFINE_GETTER(bool, invert_y, invert_y)
DEFINE_GETTER(bool, swap_xy, swap_xy)
DEFINE_GETTER(int16_t, center_x, center_x)
DEFINE_GETTER(int16_t, center_y, center_y)
DEFINE_GETTER(int16_t, min_x, min_x)
DEFINE_GETTER(int16_t, max_x, max_x)
DEFINE_GETTER(int16_t, min_y, min_y)
DEFINE_GETTER(int16_t, max_y, max_y)

DEFINE_SETTER(int32_t, deadzone_percent, deadzone_percent, VALID_U8(v, 50))
DEFINE_SETTER(int32_t, max_speed, max_speed, ((v) >= 1 && (v) <= 64))
DEFINE_SETTER(int32_t, curve_exponent_x100, curve_exponent_x100, VALID_CURVE(v))
DEFINE_SETTER(int32_t, smoothing_percent, smoothing_percent, VALID_U8(v, 90))
DEFINE_SETTER(int32_t, sample_interval_ms, sample_interval_ms, ((v) >= 4 && (v) <= 50))
DEFINE_SETTER(bool, invert_x, invert_x, true)
DEFINE_SETTER(bool, invert_y, invert_y, true)
DEFINE_SETTER(bool, swap_xy, swap_xy, true)
DEFINE_SETTER(int32_t, center_x, center_x, VALID_I16(v))
DEFINE_SETTER(int32_t, center_y, center_y, VALID_I16(v))
DEFINE_SETTER(int32_t, min_x, min_x, VALID_I16(v))
DEFINE_SETTER(int32_t, max_x, max_x, VALID_I16(v))
DEFINE_SETTER(int32_t, min_y, min_y, VALID_I16(v))
DEFINE_SETTER(int32_t, max_y, max_y, VALID_I16(v))

int psp_joystick_state_load_defaults(const struct device *dev) {
    if (dev != NULL && psp_joystick_driver_load_defaults != NULL) {
        return psp_joystick_driver_load_defaults(dev);
    }
    set_hard_defaults(psp_joystick_state_get_struct(dev));
    return 0;
}

int psp_joystick_state_apply_all(const struct device *dev) {
    if (dev != NULL && psp_joystick_driver_apply_state != NULL) {
        return psp_joystick_driver_apply_state(dev);
    }
    return 0;
}
```

- [ ] **Step 3: Add the state source to the build**

In `CMakeLists.txt`, add the state and math sources under the feature guard. Replace the file with:

```cmake
zephyr_include_directories(include)

if(CONFIG_ZMK_PSP_JOYSTICK)
    target_sources(app PRIVATE
        src/psp_joystick_math.c
        src/psp_joystick_state.c
    )

    if(CONFIG_ZMK_PSP_JOYSTICK_DRIVER)
        target_sources(app PRIVATE drivers/input/input_psp_joystick.c)
    endif()

    if(CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY)
        target_sources(app PRIVATE src/split/psp_joystick_relay.c)
    endif()

    if(CONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC)
        target_sources(app PRIVATE
            src/studio/psp_joystick_handler.c
            src/studio/psp_joystick_settings.c
        )

        list(APPEND CMAKE_MODULE_PATH ${ZEPHYR_BASE}/modules/nanopb)
        include(nanopb)
        set(NANOPB_GENERATE_CPP_APPEND_PATH TRUE)
        set(NANOPB_GENERATE_CPP_STANDALONE OFF)

        zephyr_library()
        file(GLOB_RECURSE PROTO_FILES ${CMAKE_CURRENT_SOURCE_DIR}/proto/*.proto)
        nanopb_generate_cpp(proto_srcs proto_hdrs RELPATH ${CMAKE_CURRENT_SOURCE_DIR} ${PROTO_FILES})
        target_include_directories(${ZEPHYR_CURRENT_LIBRARY} PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
        target_sources(${ZEPHYR_CURRENT_LIBRARY} PRIVATE ${proto_srcs} ${proto_hdrs})

        target_include_directories(app PUBLIC ${CMAKE_CURRENT_BINARY_DIR}/proto)
        target_link_libraries(app PRIVATE ${ZEPHYR_CURRENT_LIBRARY})
    endif()
endif()
```

- [ ] **Step 4: Commit**

```bash
git add CMakeLists.txt include/tokyo2006/psp-joystick/psp_joystick_state.h src/psp_joystick_state.c
git commit -m "feat: add psp joystick runtime state layer"
```

---

## Task 4: Add the DT binding and ADC driver

**Files:**
- Create: `dts/bindings/input/tokyo2006,psp-joystick.yaml`
- Create: `dts/bindings/vendor-prefixes.txt`
- Create: `drivers/input/input_psp_joystick.c`
- Modify: `Kconfig`

**Interfaces:**
- Consumes: `psp_joystick_math.h`, `psp_joystick_state.h`.
- Produces: `DEVICE_DT_INST_DEFINE` for every `tokyo2006,psp-joystick` node and the weak driver hooks used by the state and RPC layers.

- [ ] **Step 1: Write the DT binding**

Create `dts/bindings/input/tokyo2006,psp-joystick.yaml`:

```yaml
description: |
  PSP analog thumbstick built from two ratiometric potentiometers. The
  driver samples X and Y ADC channels, calibrates the center, and emits
  relative pointer events.

compatible: "tokyo2006,psp-joystick"

include: [base.yaml]

properties:
  io-channels:
    type: phandle-array
    required: true
    description: X and Y ADC channels, in that order.
  io-channel-names:
    type: string-array
    required: true
    description: Must be "x" and "y".
  sample-interval-ms:
    type: int
    default: 8
    description: Sampling period in milliseconds (4-50).
  deadzone-percent:
    type: int
    default: 8
    description: Deadzone as a percentage of full deflection (0-50).
  max-speed:
    type: int
    default: 8
    description: REL units per sample at full deflection (1-64).
  curve-exponent-x100:
    type: int
    default: 100
    enum: [100, 150, 200, 250, 300]
    description: Response-curve exponent times 100.
  smoothing-percent:
    type: int
    default: 0
    description: One-pole EMA weight of the new sample (0-90).
  invert-x:
    type: boolean
    description: Invert the X axis.
  invert-y:
    type: boolean
    description: Invert the Y axis.
  swap-xy:
    type: boolean
    description: Swap the X and Y axes.
  auto-center:
    type: int
    default: 1
    enum: [0, 1]
    description: |
      1 enables boot-time auto-centering unless a persisted center exists;
      0 disables it and uses center-x/center-y (or the default center).
  auto-center-ms:
    type: int
    default: 500
    description: Calibration window in milliseconds (100-2000).
  center-x:
    type: int
    description: Expected raw X center and fallback when calibration fails.
  center-y:
    type: int
    description: Expected raw Y center and fallback when calibration fails.
  min-x:
    type: int
    default: 0
    description: Raw X value at full negative deflection.
  max-x:
    type: int
    default: 4095
    description: Raw X value at full positive deflection.
  min-y:
    type: int
    default: 0
    description: Raw Y value at full negative deflection.
  max-y:
    type: int
    default: 4095
    description: Raw Y value at full positive deflection.
```

- [ ] **Step 2: Add the vendor prefix**

Create `dts/bindings/vendor-prefixes.txt`:

```text
tokyo2006 Tokyo2006
```

- [ ] **Step 3: Write the driver**

Create `drivers/input/input_psp_joystick.c`:

```c
/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT tokyo2006_psp_joystick

#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <tokyo2006/psp-joystick/psp_joystick_math.h>
#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_REGISTER(psp_joystick, CONFIG_ZMK_PSP_JOYSTICK_LOG_LEVEL);

#define PSP_RING_SIZE 64
#define PSP_CAL_WINDOW 16
#define PSP_ADC_MAX 4095
#define PSP_FIRST_TICK_MS 1000
#define PSP_CAL_SPREAD_LIMIT (PSP_ADC_MAX / 5)
#define PSP_CAL_CENTER_LIMIT (PSP_ADC_MAX / 4)
#define PSP_MANUAL_SPREAD_LIMIT (PSP_ADC_MAX / 10)

struct psp_joystick_config {
    struct adc_dt_spec adc_x;
    struct adc_dt_spec adc_y;
    uint16_t sample_interval_ms;
    uint16_t auto_center_ms;
    bool auto_center;
    bool invert_x;
    bool invert_y;
    bool swap_xy;
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
    bool reported_nonzero;
    bool ema_valid;
    int16_t ema_x;
    int16_t ema_y;
    int32_t remainder_x;
    int32_t remainder_y;
};

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
    state->center_x = CLAMP(config->has_dt_center ? config->dt_center_x
                                                  : PSP_JOYSTICK_CENTER_DEFAULT,
                            0, 4095);
    state->center_y = CLAMP(config->has_dt_center ? config->dt_center_y
                                                  : PSP_JOYSTICK_CENTER_DEFAULT,
                            0, 4095);
    state->min_x = CLAMP(config->dt_min_x, 0, 4095);
    state->max_x = CLAMP(config->dt_max_x, 0, 4095);
    state->min_y = CLAMP(config->dt_min_y, 0, 4095);
    state->max_y = CLAMP(config->dt_max_y, 0, 4095);
    return 0;
}

int psp_joystick_driver_apply_state(const struct device *dev) {
    struct psp_joystick_data *data = dev->data;

    data->state.sample_interval_ms = CLAMP(data->state.sample_interval_ms, 4, 50);
    k_work_reschedule(&data->work, K_MSEC(data->state.sample_interval_ms));
    return 0;
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
        LOG_WRN("ADC read failed: %d", rc);
        goto reschedule;
    }

    raw_x = MIN(raw_x, PSP_ADC_MAX);
    raw_y = MIN(raw_y, PSP_ADC_MAX);

    if (!data->calibration_done && !data->skip_auto_center && config->auto_center) {
        if (!data->calibrating) {
            data->calibrating = true;
            data->ring_head = 0;
            data->ring_count = 0;
            uint16_t interval = CLAMP(config->sample_interval_ms, 4, 50);
            data->cal_target =
                (uint8_t)CLAMP(config->auto_center_ms / interval, 8, PSP_RING_SIZE);
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
    int16_t x = psp_joystick_normalize(raw_x, state->center_x, state->min_x, state->max_x);
    int16_t y = psp_joystick_normalize(raw_y, state->center_y, state->min_y, state->max_y);

    if (state->smoothing_percent > 0) {
        if (!data->ema_valid) {
            data->ema_x = x;
            data->ema_y = y;
            data->ema_valid = true;
        } else {
            int32_t weight = state->smoothing_percent;
            data->ema_x = (int16_t)(((int32_t)data->ema_x * (100 - weight) + (int32_t)x * weight) /
                                    100);
            data->ema_y = (int16_t)(((int32_t)data->ema_y * (100 - weight) + (int32_t)y * weight) /
                                    100);
        }
        x = data->ema_x;
        y = data->ema_y;
    } else {
        data->ema_valid = false;
    }

    psp_joystick_apply_transforms(&x, &y, state->swap_xy, state->invert_x, state->invert_y);

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
    k_work_reschedule(&data->work, K_MSEC(CLAMP(data->state.sample_interval_ms, 4, 50)));
}

static int psp_joystick_init(const struct device *dev) {
    const struct psp_joystick_config *config = dev->config;
    struct psp_joystick_data *data = dev->data;

    data->dev = dev;

    if (!adc_is_ready_dt(&config->adc_x) || !adc_is_ready_dt(&config->adc_y)) {
        LOG_ERR("ADC device is not ready");
        return -ENODEV;
    }
    if (adc_channel_setup_dt(&config->adc_x) != 0 ||
        adc_channel_setup_dt(&config->adc_y) != 0) {
        LOG_ERR("ADC channel setup failed");
        return -EIO;
    }

    k_work_init_delayable(&data->work, psp_joystick_work_cb);
    (void)psp_joystick_state_load_defaults(dev);
    k_work_schedule(&data->work, K_MSEC(PSP_FIRST_TICK_MS));
    return 0;
}

#define PSP_JOYSTICK_INST(n)                                                                       \
    static const struct psp_joystick_config psp_joystick_config_##n = {                            \
        .adc_x = ADC_DT_SPEC_INST_GET_BY_IDX(n, 0),                                                \
        .adc_y = ADC_DT_SPEC_INST_GET_BY_IDX(n, 1),                                                \
        .sample_interval_ms = DT_INST_PROP_OR(n, sample_interval_ms, 8),                           \
        .auto_center_ms = DT_INST_PROP_OR(n, auto_center_ms, 500),                                 \
        .auto_center = DT_INST_PROP_OR(n, auto_center, 1),                                         \
        .invert_x = DT_INST_PROP_OR(n, invert_x, 0),                                               \
        .invert_y = DT_INST_PROP_OR(n, invert_y, 0),                                               \
        .swap_xy = DT_INST_PROP_OR(n, swap_xy, 0),                                                 \
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
```

- [ ] **Step 4: Extend Kconfig with the driver and split-relay symbols**

Replace `Kconfig`:

```kconfig
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
    bool "Enable psp-joystick custom Studio RPC"
    depends on ZMK_STUDIO

config ZMK_PSP_JOYSTICK_SPLIT_RELAY
    bool "Relay psp-joystick state to split peripherals"
    depends on ZMK_SPLIT && ZMK_SPLIT_RELAY_EVENT

module = ZMK_PSP_JOYSTICK
module-str = ZMK_PSP_JOYSTICK
source "${ZEPHYR_BASE}/subsys/logging/Kconfig.template.log_config"

endif
```

- [ ] **Step 5: Commit**

```bash
git add Kconfig dts/bindings drivers/input/input_psp_joystick.c
git commit -m "feat: add psp joystick ADC input driver and DT binding"
```

---

## Task 5: Replace the proto and Studio RPC handler

**Files:**
- Replace: `proto/tokyo2006/psp-joystick/psp_joystick.proto`
- Replace: `proto/tokyo2006/psp-joystick/psp_joystick.options`
- Replace: `src/studio/psp_joystick_handler.c`

**Interfaces:**
- Consumes: `psp_joystick_state.h` getters/setters, `psp_joystick_settings.h`, `psp_joystick_relay.h`.
- Produces: `ZMK_RPC_CUSTOM_SUBSYSTEM(tokyo2006__psp_joystick, &psp_joystick_subsystem_meta, psp_joystick_rpc_handle_request)` with GetState/SetState/Calibrate/Reset.

- [ ] **Step 1: Replace the proto**

`proto/tokyo2006/psp-joystick/psp_joystick.proto`:

```proto
syntax = "proto3";
package tokyo2006.psp_joystick;

message PspJoystickState {
  optional uint32 deadzone_percent    = 1;
  optional uint32 max_speed           = 2;
  optional uint32 curve_exponent_x100 = 3;
  optional uint32 smoothing_percent   = 4;
  optional uint32 sample_interval_ms  = 5;
  optional bool   invert_x            = 6;
  optional bool   invert_y            = 7;
  optional bool   swap_xy             = 8;
  optional int32  center_x            = 9;
  optional int32  center_y            = 10;
  optional int32  min_x               = 11;
  optional int32  max_x               = 12;
  optional int32  min_y               = 13;
  optional int32  max_y               = 14;
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
    ErrorResponse     error     = 1;
    GetStateResponse  get_state = 2;
    SetStateResponse  set_state = 3;
    CalibrateResponse calibrate = 4;
    ResetResponse     reset     = 5;
  }
}
```

- [ ] **Step 2: Replace the nanopb options**

`proto/tokyo2006/psp-joystick/psp_joystick.options`:

```text
tokyo2006.psp_joystick.ErrorResponse.message max_size:128
```

- [ ] **Step 3: Replace the RPC handler**

`src/studio/psp_joystick_handler.c`:

```c
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
ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER(tokyo2006__psp_joystick,
                                         tokyo2006_psp_joystick_Response);

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

    tokyo2006_psp_joystick_GetStateResponse out =
        tokyo2006_psp_joystick_GetStateResponse_init_zero;
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

    if (req->persist) {
        (void)psp_joystick_settings_save_all(dev);
    }

#if IS_ENABLED(CONFIG_ZMK_PSP_JOYSTICK_SPLIT_RELAY)
    psp_joystick_relay_send_state(dev);
#endif

    tokyo2006_psp_joystick_SetStateResponse out =
        tokyo2006_psp_joystick_SetStateResponse_init_zero;
    fill_state(dev, &out.state);
    out.has_state = true;
    out.persisted = req->persist;
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
        (void)psp_joystick_settings_save_all(dev);
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
    tokyo2006_psp_joystick_Response *resp = ZMK_RPC_CUSTOM_SUBSYSTEM_RESPONSE_BUFFER_ALLOCATE(
        tokyo2006__psp_joystick, encode_response);

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
```

- [ ] **Step 4: Commit**

```bash
git add proto/tokyo2006 src/studio/psp_joystick_handler.c
git commit -m "feat: add psp joystick Studio RPC protocol and handler"
```

---

## Task 6: Add settings persistence and the split relay

**Files:**
- Create: `include/tokyo2006/psp-joystick/psp_joystick_settings.h`
- Create: `src/studio/psp_joystick_settings.c`
- Create: `include/tokyo2006/psp-joystick/psp_joystick_relay.h`
- Rename + replace: `src/split/template_relay.c` → `src/split/psp_joystick_relay.c`
- Rename + replace: `include/tokyo2006/psp-joystick/template_relay.h` → `include/tokyo2006/psp-joystick/psp_joystick_relay.h`

**Interfaces:**
- Consumes: state getters/setters, `zmk_custom_setting_*` API.
- Produces: `psp_joystick_settings_save_all/reset_all/load_cb`, `psp_joystick_relay_send_state`, `struct psp_joystick_relay_state`.

- [ ] **Step 1: Write the settings header**

Create `include/tokyo2006/psp-joystick/psp_joystick_settings.h`:

```c
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
```

- [ ] **Step 2: Write the settings implementation**

Create `src/studio/psp_joystick_settings.c`:

```c
/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 */

#include <errno.h>
#include <stdbool.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

#include <zmk/event_manager.h>

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)
#include <cormoran/zmk/custom_settings.h>
#endif

#include <tokyo2006/psp-joystick/psp_joystick_settings.h>
#include <tokyo2006/psp-joystick/psp_joystick_state.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define PSP_NS "tokyo2006__psp_joystick"

#if IS_ENABLED(CONFIG_ZMK_CUSTOM_SETTINGS)

#define PSP_INT_SETTING(_name, _key, _default)                                                     \
    ZMK_CUSTOM_SETTING_DEFINE(psp_setting_##_name, PSP_NS, _key,                                   \
                              ZMK_CUSTOM_SETTING_VALUE_TYPE_INT32,                                 \
                              ZMK_CUSTOM_SETTING_VALUE_INT32(_default),                            \
                              ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,                       \
                              ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,                              \
                              ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,                              \
                              ZMK_CUSTOM_SETTING_NO_CONSTRAINT)

#define PSP_BOOL_SETTING(_name, _key, _default)                                                    \
    ZMK_CUSTOM_SETTING_DEFINE(psp_setting_##_name, PSP_NS, _key,                                   \
                              ZMK_CUSTOM_SETTING_VALUE_TYPE_BOOL,                                  \
                              ZMK_CUSTOM_SETTING_VALUE_BOOL(_default),                             \
                              ZMK_CUSTOM_SETTING_CONFIDENTIALITY_RPC_PUBLIC,                       \
                              ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,                              \
                              ZMK_CUSTOM_SETTING_PERMISSION_UNSECURE,                              \
                              ZMK_CUSTOM_SETTING_NO_CONSTRAINT)

PSP_INT_SETTING(deadzone_percent, "deadzone_percent", PSP_JOYSTICK_DEADZONE_PERCENT_DEFAULT);
PSP_INT_SETTING(max_speed, "max_speed", PSP_JOYSTICK_MAX_SPEED_DEFAULT);
PSP_INT_SETTING(curve_exponent_x100, "curve_exponent_x100",
                PSP_JOYSTICK_CURVE_EXPONENT_X100_DEFAULT);
PSP_INT_SETTING(smoothing_percent, "smoothing_percent", PSP_JOYSTICK_SMOOTHING_PERCENT_DEFAULT);
PSP_INT_SETTING(sample_interval_ms, "sample_interval_ms", PSP_JOYSTICK_SAMPLE_INTERVAL_MS_DEFAULT);
PSP_BOOL_SETTING(invert_x, "invert_x", false);
PSP_BOOL_SETTING(invert_y, "invert_y", false);
PSP_BOOL_SETTING(swap_xy, "swap_xy", false);
PSP_INT_SETTING(center_x, "center_x", PSP_JOYSTICK_CENTER_DEFAULT);
PSP_INT_SETTING(center_y, "center_y", PSP_JOYSTICK_CENTER_DEFAULT);
PSP_INT_SETTING(min_x, "min_x", PSP_JOYSTICK_MIN_DEFAULT);
PSP_INT_SETTING(max_x, "max_x", PSP_JOYSTICK_MAX_DEFAULT);
PSP_INT_SETTING(min_y, "min_y", PSP_JOYSTICK_MIN_DEFAULT);
PSP_INT_SETTING(max_y, "max_y", PSP_JOYSTICK_MAX_DEFAULT);

int psp_joystick_settings_save_all(const struct device *dev) {
    int first_rc = 0;
    int rc;

#define SAVE_INT(name, getter)                                                                     \
    do {                                                                                           \
        rc = zmk_custom_setting_set_int32(&psp_setting_##name, (int32_t)getter(dev),               \
                                          ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);                  \
        if (rc != 0 && first_rc == 0) {                                                            \
            first_rc = rc;                                                                         \
            LOG_WRN("save %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

#define SAVE_BOOL(name, getter)                                                                    \
    do {                                                                                           \
        rc = zmk_custom_setting_set_bool(&psp_setting_##name, getter(dev),                         \
                                         ZMK_CUSTOM_SETTING_WRITE_MODE_PERSIST);                   \
        if (rc != 0 && first_rc == 0) {                                                            \
            first_rc = rc;                                                                         \
            LOG_WRN("save %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

    SAVE_INT(deadzone_percent, psp_joystick_state_get_deadzone_percent);
    SAVE_INT(max_speed, psp_joystick_state_get_max_speed);
    SAVE_INT(curve_exponent_x100, psp_joystick_state_get_curve_exponent_x100);
    SAVE_INT(smoothing_percent, psp_joystick_state_get_smoothing_percent);
    SAVE_INT(sample_interval_ms, psp_joystick_state_get_sample_interval_ms);
    SAVE_BOOL(invert_x, psp_joystick_state_get_invert_x);
    SAVE_BOOL(invert_y, psp_joystick_state_get_invert_y);
    SAVE_BOOL(swap_xy, psp_joystick_state_get_swap_xy);

    /* Calibration is only meaningful on the half that owns the ADC. */
    if (dev != NULL) {
        SAVE_INT(center_x, psp_joystick_state_get_center_x);
        SAVE_INT(center_y, psp_joystick_state_get_center_y);
        SAVE_INT(min_x, psp_joystick_state_get_min_x);
        SAVE_INT(max_x, psp_joystick_state_get_max_x);
        SAVE_INT(min_y, psp_joystick_state_get_min_y);
        SAVE_INT(max_y, psp_joystick_state_get_max_y);
    }

#undef SAVE_INT
#undef SAVE_BOOL

    return first_rc;
}

int psp_joystick_settings_reset_all(const struct device *dev) {
    ARG_UNUSED(dev);
    uint32_t affected = 0;
    int rc = zmk_custom_settings_reset_scope(PSP_NS, NULL, NULL, &affected);
    if (rc != 0) {
        LOG_WRN("reset scope failed: %d (affected=%u)", rc, affected);
    }
    return rc;
}

int psp_joystick_settings_load_cb(const struct device *dev) {
    int rc;
    int32_t value;
    bool center_x_persisted = false;
    bool center_y_persisted = false;

#define LOAD_INT(name, setter)                                                                     \
    do {                                                                                           \
        rc = zmk_custom_setting_get_int32(&psp_setting_##name, &value);                            \
        if (rc == 0) {                                                                             \
            (void)setter(dev, value);                                                                    \
        } else if (rc != -ENOENT) {                                                                \
            LOG_WRN("load %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

#define LOAD_BOOL(name, setter)                                                                    \
    do {                                                                                           \
        bool b;                                                                                    \
        rc = zmk_custom_setting_get_bool(&psp_setting_##name, &b);                                 \
        if (rc == 0) {                                                                             \
            (void)setter(dev, b);                                                                  \
        } else if (rc != -ENOENT) {                                                                \
            LOG_WRN("load %s failed: %d", #name, rc);                                              \
        }                                                                                          \
    } while (0)

    LOAD_INT(deadzone_percent, psp_joystick_state_set_deadzone_percent);
    LOAD_INT(max_speed, psp_joystick_state_set_max_speed);
    LOAD_INT(curve_exponent_x100, psp_joystick_state_set_curve_exponent_x100);
    LOAD_INT(smoothing_percent, psp_joystick_state_set_smoothing_percent);
    LOAD_INT(sample_interval_ms, psp_joystick_state_set_sample_interval_ms);
    LOAD_BOOL(invert_x, psp_joystick_state_set_invert_x);
    LOAD_BOOL(invert_y, psp_joystick_state_set_invert_y);
    LOAD_BOOL(swap_xy, psp_joystick_state_set_swap_xy);
    LOAD_INT(min_x, psp_joystick_state_set_min_x);
    LOAD_INT(max_x, psp_joystick_state_set_max_x);
    LOAD_INT(min_y, psp_joystick_state_set_min_y);
    LOAD_INT(max_y, psp_joystick_state_set_max_y);

#undef LOAD_INT
#undef LOAD_BOOL

    rc = zmk_custom_setting_get_int32(&psp_setting_center_x, &value);
    if (rc == 0) {
        (void)psp_joystick_state_set_center_x(dev, value);
        center_x_persisted = true;
    }
    rc = zmk_custom_setting_get_int32(&psp_setting_center_y, &value);
    if (rc == 0) {
        (void)psp_joystick_state_set_center_y(dev, value);
        center_y_persisted = true;
    }

    if (dev != NULL && psp_joystick_driver_on_settings_loaded != NULL) {
        psp_joystick_driver_on_settings_loaded(dev, center_x_persisted && center_y_persisted);
    }
    return 0;
}

static int psp_joystick_settings_on_initialized(const zmk_event_t *eh) {
    ARG_UNUSED(eh);
    const struct device *dev = DEVICE_DT_GET_ANY(tokyo2006_psp_joystick);
    (void)psp_joystick_settings_load_cb(dev);
    (void)psp_joystick_state_apply_all(dev);
    return ZMK_EV_EVENT_HANDLED;
}

ZMK_LISTENER(psp_joystick_settings_listener, psp_joystick_settings_on_initialized);
ZMK_SUBSCRIPTION(psp_joystick_settings_listener, zmk_custom_settings_initialized);

#else /* !CONFIG_ZMK_CUSTOM_SETTINGS */

int psp_joystick_settings_save_all(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

int psp_joystick_settings_reset_all(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

int psp_joystick_settings_load_cb(const struct device *dev) {
    ARG_UNUSED(dev);
    return 0;
}

#endif /* CONFIG_ZMK_CUSTOM_SETTINGS */
```

- [ ] **Step 3: Write the relay header**

Create `include/tokyo2006/psp-joystick/psp_joystick_relay.h`:

```c
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
```

- [ ] **Step 4: Rename and replace the split relay**

```bash
git mv src/split/template_relay.c src/split/psp_joystick_relay.c
git mv include/tokyo2006/psp-joystick/template_relay.h include/tokyo2006/psp-joystick/psp_joystick_relay.h
```

Replace `src/split/psp_joystick_relay.c`:

```c
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
```

- [ ] **Step 5: Commit**

```bash
git add include/tokyo2006/psp-joystick src/studio/psp_joystick_settings.c src/split/psp_joystick_relay.c
git commit -m "feat: add psp joystick settings persistence and split relay"
```

---

## Task 7: Add the scroll helper dtsi and build-test overlays

**Files:**
- Create: `dts/tokyo2006/psp_joystick.dtsi`
- Replace: `tests/zmk-config/config/tester_xiao.overlay`
- Create: `tests/zmk-config/config/renode_usb_wired_split.overlay`
- Verify: `tests/studio/native_sim.conf`, `tests/studio/events.snapshot`

**Interfaces:**
- Consumes: upstream `zmk,input-processor-code-mapper` and `zmk,input-processor-scaler`.
- Produces: `psp_scroll_map`, `psp_scroll_scale`, and build artifacts that compile the driver and relay.

- [ ] **Step 1: Write the scroll helper dtsi**

Create `dts/tokyo2006/psp_joystick.dtsi`:

```dts
/*
 * Copyright (c) 2026 The ZMK Contributors
 * SPDX-License-Identifier: MIT
 *
 * Scroll helpers for the PSP joystick listener. Include this file, then add
 * a layer override to your listener:
 *
 *   &psp_joystick_listener {
 *       scroll {
 *           layers = <SCROLL>;
 *           input-processors = <&psp_scroll_map>, <&psp_scroll_scale 1 24>;
 *       };
 *   };
 */

#include <zephyr/dt-bindings/input/input-event-codes.h>

/ {
    psp_scroll_map: psp_scroll_map {
        compatible = "zmk,input-processor-code-mapper";
        #input-processor-cells = <0>;
        type = <INPUT_EV_REL>;
        map = <INPUT_REL_X INPUT_REL_HWHEEL INPUT_REL_Y INPUT_REL_WHEEL>;
    };

    psp_scroll_scale: psp_scroll_scale {
        compatible = "zmk,input-processor-scaler";
        #input-processor-cells = <2>;
        type = <INPUT_EV_REL>;
        codes = <INPUT_REL_WHEEL INPUT_REL_HWHEEL>;
        track-remainders;
    };
};
```

- [ ] **Step 2: Replace the `tester_xiao` build overlay**

`tests/zmk-config/config/tester_xiao.overlay`:

```dts
#include <zephyr/dt-bindings/adc/adc.h>

&board_cdc_acm_uart {
    status = "disabled";
};

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@4 {
        reg = <4>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <4>;
        zephyr,resolution = <12>;
    };

    channel@5 {
        reg = <5>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <5>;
        zephyr,resolution = <12>;
    };
};

/ {
    psp_joystick: psp_joystick {
        compatible = "tokyo2006,psp-joystick";
        io-channels = <&adc 4>, <&adc 5>;
        io-channel-names = "x", "y";
    };

    psp_joystick_listener: psp_joystick_listener {
        compatible = "zmk,input-listener";
        device = <&psp_joystick>;
    };
};
```

- [ ] **Step 2b: Enable ADC/input in the `tester_xiao` build config**

Append to `tests/zmk-config/config/tester_xiao.conf`:

```conf
CONFIG_INPUT=y
CONFIG_ADC=y
CONFIG_ADC_NRFX_SAADC=y
```

- [ ] **Step 3: Add the split build-test overlay**

Create `tests/zmk-config/config/renode_usb_wired_split.overlay`:

```dts
#include <zephyr/dt-bindings/adc/adc.h>

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@4 {
        reg = <4>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <4>;
        zephyr,resolution = <12>;
    };

    channel@5 {
        reg = <5>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <5>;
        zephyr,resolution = <12>;
    };
};

/ {
    psp_joystick: psp_joystick {
        compatible = "tokyo2006,psp-joystick";
        io-channels = <&adc 4>, <&adc 5>;
        io-channel-names = "x", "y";
    };
};
```

- [ ] **Step 4: Verify the template's Studio test was renamed correctly**

Open `tests/studio/events.snapshot`. It must contain exactly:

```text
cormoran_custom_settings, Security: Unsecured
tokyo2006__psp_joystick, Security: Unsecured
```

Open `tests/studio/native_sim.conf`. It must contain `CONFIG_ZMK_PSP_JOYSTICK=y` and `CONFIG_ZMK_PSP_JOYSTICK_STUDIO_RPC=y`. If `init_module.py` left the old names, replace them.

- [ ] **Step 5: Run the host tests**

```bash
python3 -m unittest test_math
```

Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add dts/tokyo2006 tests/zmk-config tests/studio
git commit -m "test: add psp joystick scroll helper and build-test overlays"
```

---

## Task 8: Write the module README

**Files:**
- Replace: `README.md`

**Interfaces:**
- Consumes: the final DT/Kconfig/RPC names from Tasks 1-7.
- Produces: a user guide that lets an unfamiliar ZMK user wire, configure, flash, and tune the module.

- [ ] **Step 1: Write `README.md`**

```markdown
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
#include <tokyo2006/psp_joystick.dtsi>

&adc {
    status = "okay";
    #address-cells = <1>;
    #size-cells = <0>;

    channel@0 {
        reg = <0>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <0>; /* AIN0 */
        zephyr,resolution = <12>;
    };

    channel@1 {
        reg = <1>;
        zephyr,gain = "ADC_GAIN_1_6";
        zephyr,reference = "ADC_REF_INTERNAL";
        zephyr,acquisition-time = <ADC_ACQ_TIME_DEFAULT>;
        zephyr,input-positive = <1>; /* AIN1 */
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
processors, so the pointer does not move while scrolling. Increase the second
scaler cell (`24`) to scroll faster; decrease it to scroll slower.

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
before calibrating; if it is moving, the request fails with
`"joystick is moving; release it and retry"`.

## Calibration and Persistence

- On first boot without a persisted center, the driver collects a 500 ms
  window, rejects the measurement if the stick is moving or far from the DT
  expected center, and otherwise uses the median as the center.
- When custom settings are enabled and a center was persisted, boot-time
  auto-centering is skipped.
- `SetState(persist=true)` and `Calibrate(persist=true)` write every field.
- `Reset(factory_defaults=true)` erases the persisted records.

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
```

- [ ] **Step 2: Commit**

```bash
git add README.md
git commit -m "docs: add psp joystick module user guide"
```

---

## Task 9: Full verification and cleanup

**Files:**
- No new files. Fix any failures found.

- [ ] **Step 1: Run the host math tests**

```bash
python3 -m unittest test_math
```

Expected: PASS.

- [ ] **Step 2: Initialize the west workspace if needed**

```bash
bash scripts/setup_workspace.sh
```

Expected: west initializes and `west update --narrow` completes.

- [ ] **Step 3: Run the firmware build tests**

```bash
python3 -m unittest test
```

Expected: all four artifacts build and contain the expected `.config` lines
and `zmk.uf2`.

- [ ] **Step 4: Run formatting/lint**

```bash
python3 -m pre_commit run --all-files
```

If `pre_commit` is not installed, run the formatter the template ships
(`.clang-format`) with:

```bash
clang-format -i drivers/input/input_psp_joystick.c src/psp_joystick_math.c \
  src/psp_joystick_state.c src/studio/psp_joystick_handler.c \
  src/studio/psp_joystick_settings.c src/split/psp_joystick_relay.c \
  include/tokyo2006/psp-joystick/*.h
```

Expected: no formatting changes remain.

- [ ] **Step 5: Verify placeholders and repository cleanliness**

```bash
python3 scripts/init_module.py --verify-only
git status --short
```

Expected: `OK: no template placeholders left.` and a clean tree after the
final commit.

- [ ] **Step 6: Final commit**

```bash
git add -A
git commit -m "chore: finalize psp joystick module for v1"
```

---

## Spec Self-Review

| Spec section | Covered by |
|---|---|
| §4 Approach A (input driver + ZMK pointing) | Task 4 driver, Task 7 listener |
| §5 Data flow | Tasks 4, 7, 9 |
| §6 Module layout | Tasks 1-8 |
| §7 DT binding | Task 4 |
| §8.1 Sampling | Task 4 |
| §8.2 Auto-center | Task 4 |
| §8.3 Normalization | Task 2 |
| §8.4 Deadzone | Task 2 |
| §8.5 Curve/speed/remainder | Task 2 |
| §8.6 Axis transforms | Task 2, Task 4 |
| §8.7 Event emission | Task 4 |
| §9 Runtime state | Task 3 |
| §10 Studio RPC | Task 5 |
| §11 Scroll mode | Task 7 |
| §12 Persistence | Task 6 |
| §13 Split support | Task 6, Task 7 |
| §14 Error handling | Tasks 4, 5, 6 |
| §15 Testing | Tasks 2, 9 |
| §16 Kconfig | Task 4 |
| §17 Version compatibility | Task 1 (west manifests), README |
| §18 Deferred items | README, Global Constraints |

**Placeholder scan:** no `TBD`, `TODO`, "implement later", or "similar to
Task N" entries remain.

**Type consistency:** `psp_joystick_runtime_state` field names match the proto
fields and the getter/setter suffixes; `psp_joystick_driver_*` signatures match
the weak declarations in `psp_joystick_state.h`; the relay struct fields match
the getters used in `psp_joystick_relay.c`.
