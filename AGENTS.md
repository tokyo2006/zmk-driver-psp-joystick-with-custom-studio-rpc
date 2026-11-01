This repository contains a ZMK module using the **unofficial** custom ZMK Studio RPC protocol.

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
