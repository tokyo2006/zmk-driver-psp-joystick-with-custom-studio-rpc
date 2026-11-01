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
