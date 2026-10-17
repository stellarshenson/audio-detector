---
name: audio-detector-device
description: Drive the audio-detector Arduino Nano over USB serial - find the port, build and flash, run the 9600 baud console, back up or restore the EEPROM, record IR codes and learn the threshold. Use when connecting to, flashing, backing up or driving the audio-detector board, when the device does not answer on its serial port, when the EEPROM must be preserved across a firmware change, or when the user says audio detector, detector board, EEPROM backup, flash the detector, record the remote or learn the threshold.
---

# Audio Detector Device

The device is an Arduino Nano (ATmega328P, old bootloader) running `src/audio-detector.ino`. The USB serial port has two uses at different speeds: PlatformIO uploads at 57600 baud, the console runs at 9600 baud. The wrong speed is the most common failure.

## Find the port

```bash
conda run -n firmware-avr pio device list
ls -l /dev/serial/by-id/ 2>/dev/null
```

- Use the `ttyUSB*` or `ttyACM*` path listed for the Nano; prefer `/dev/serial/by-id/` when several USB serial devices are attached
- Ask the user before opening a port when more than one candidate exists; a wrong port can be another instrument
- No port listed: ask the user to connect the board; `dmesg | tail` shows the attach
- `Permission denied`: add the login user to `dialout` and start a new session

## Build and test

```bash
make build            # conda run -n firmware-avr pio run
make test             # build, harness shim, pytest
```

- Run `make test` before and after a firmware change; a clean build has 0 warnings from `src/`
- The firmware runs on working hardware: keep the EEPROM layout - bytes 0-5 START code, 6-11 STOP code, 12-13 threshold, 14-15 unused
- Never run `pio run -t erase`; it wipes the EEPROM. `pio run -t upload` writes flash only and keeps the EEPROM

## Flash

```bash
conda run -n firmware-avr pio run --target upload --upload-port /dev/ttyUSB0
```

- Old bootloader: `board = nanoatmega328` in `platformio.ini`
- `avrdude: stk500_getsync()` or a timeout: the board has the new bootloader; switch to `nanoatmega328new` only with the user's word

## Serial console

The console runs at 9600 baud, 8N1. A line runs at `\n` or `\r`; lines over 64 characters are cut. The board resets when the port opens, so wait about 2 s before sending. The command is echoed, then its reply follows.

Non-interactive one-shot:

```bash
conda run -n firmware-avr python - <<'PY'
import serial, time
with serial.Serial("/dev/ttyUSB0", 9600, timeout=2) as s:
    time.sleep(2)
    s.reset_input_buffer()
    s.write(b"status\n")
    time.sleep(0.4)
    print(s.read_all().decode("ascii", "replace"))
PY
```

| Command | Reply |
|---|---|
| `help` | `[HELP] available commands` and the command list |
| `status` | threshold, off level, audio level, audio on/off, auto-standby, 12V trigger, debug plot |
| `set threshold <160-1023>` | `[CONFIG] setting Audiosense threshold to <n>`; saved to EEPROM |
| `set audio [0\|1]` | `[CONFIG] turning audio off` or `on` |
| `set debug_plot [0\|1]` | `[CONFIG] setting debug_plot off` or `on` |
| `reboot` | the device restarts |
| `eeprom dump <0-1008>` | `[EEPROM] <addr> <32 hex>` |
| `eeprom write <0-1023> <hex>` | `[EEPROM] wrote <n> bytes at <addr>` |

- `eeprom dump` returns the 16 bytes at the address as uppercase hex, prefixed by the address
- `eeprom write` takes 1 to 16 bytes; an address outside 0-1023, an odd hex length, non-hex text or more than 16 bytes is ignored with no reply
- Use `tools/eeprom_tool.py` for EEPROM work; use the raw console for `status`, `set` and `reboot`

## EEPROM backup and restore

```bash
make eeprom-backup PORT=/dev/ttyUSB0
make eeprom-restore PORT=/dev/ttyUSB0 FILE=eeprom.bin
make eeprom-restore PORT=/dev/ttyUSB0 FILE=eeprom.json
```

- `eeprom.json`: START and STOP codes (type, protocol, bits, value), threshold, the 1024-byte image as `raw_hex`, SHA-256
- `eeprom.bin`: 1024 raw bytes; `eeprom.hex`: Intel HEX of the same image
- Restore `.bin` or `.hex`: writes all 1024 bytes
- Restore `.json`: reads the current image, writes bytes 0-13 only, keeps 14-1023
- Both commands are non-interactive and print one JSON line; an agent can run them
- Take a backup before the first flash of a firmware that changes the EEPROM layout
- The backup files are in `.gitignore`; do not commit the user's codes unless asked
- The tool needs the `eeprom` commands in the firmware; a board with older firmware answers no `[EEPROM]` line

## Record IR codes and learn the threshold

Recording (button hold, longer than 600 ms):

1. Hold the button until the LED blinks fast; release
2. Point the remote at the receiver and press the START key; wait for the second blink pattern
3. Press the STOP key; the device saves both codes to EEPROM bytes 0-11
4. Only NEC and Sony codes are accepted; another protocol prints a rejection and recording keeps waiting
5. A repeat frame while a key is held is ignored

Learning (single press, shorter than 600 ms):

1. Play the music at the listening level
2. Press the button once; the device samples for 2.5 s and sets the threshold to half the average, at least 160
3. Pressing while audio is on turns audio off first

- The button is D8 to GND with no hardware filter; the firmware debounces 20 ms
- An erased EEPROM uses threshold 300; a threshold below 160 is read as 160
- Verify with `status`: `Audiosense threshold:` and `Audio off level:` (threshold/4)

## Troubleshooting

| Symptom | Cause and action |
|---|---|
| No reply at 9600 | wrong port or baud, board still in the bootloader (wait 2 s), or the port is held by PlatformIO |
| Garbled text | the port is open at the upload speed; close PlatformIO and reopen at 9600 |
| `[EEPROM]` absent | firmware without the eeprom commands; flash the current firmware |
| Upload fails | old/new bootloader mismatch; check `board` in `platformio.ini` |
| EEPROM values look wrong | restore the last `eeprom.bin` or `eeprom.json`; a JSON restore keeps bytes 14-1023 |

## References

- `README.md` - user-facing behaviour, wiring and build
- `src/audio-detector.ino` - state machine, commands and EEPROM access
- `src/AudioLevel.h`, `src/AudioLevel.cpp` - sampling, threshold and hysteresis
- `tools/eeprom_tool.py` - backup and restore CLI
- `test/harness/` - simavr board model used by `make test`

<!-- improved 2026-10-17 | body 0→946w / 0→125L | benchmark n/a (not-requested) | via improve-skill -->
