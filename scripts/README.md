# scripts

Helper scripts that run on the PC: parsers of the serial output, backup and restore commands,
calibration helpers. A script takes no interactive input, prints one JSON line, and exits non-zero
with a reason on failure, so an agent can run it.

The one exception is `freertos.py` in an STM32Cube project: it is a PlatformIO build hook that
PlatformIO runs through `extra_scripts` in `platformio.ini`, not a command for a person or an agent.

This project has one more exception, `scripts/eeprom-bridge/`: a PlatformIO project of the temporary
board firmware that `make eeprom-backup` flashes when the board firmware has no `eeprom` commands,
not a PC script.
