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
