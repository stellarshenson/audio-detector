<!-- @import /home/lab/workspace/.claude/CLAUDE.md -->

# audio-detector

Audio detector

- Platform: Arduino
- Board: `nanoatmega328`
- Boot line: `[INIT] type 'help' for commands list`
- Console baud: 9600

## Structure

- `src/` - the firmware sketch and the audio level driver; the master source root
- `lib/vendor/` - the bundled Arduino libraries; `lib/README.md` lists their versions
- `test/host/` - host unit tests of the simulator harness decoders
- `test/harness/` - simulator wrapper and board helpers
- `test/functional/` - boot and behaviour tests
- `resources/` - graphics, external files, and `resources/envs/`, and `resources/eeprom/`
- `references/` - datasheets, notes and the board files in `references/schematics/`
- `docs/` - acceptance criteria and defects
- `scripts/` - `scripts/eeprom-bridge/`, the temporary bridge firmware of the EEPROM backup
- `host/` - the PC-side Python package `host/src/audiodetector_host/` and its tests in `host/tests/`; it talks to the board over serial and is never built for the board
- `Makefile` - the entry point for every development command
- `.agents/skills/audio-detector-device` - board facts, upload, console, EEPROM backup, IR recording and safety
- `.agents/skills/audio-detector-toolchain` - environment and simulator

## Commands

- `make help` - list the targets
- `make create_environment` (alias `make env`) - install the conda environment `firmware-avr`; `ENV_SCOPE=local` keeps it under `.conda/envs/` in the project, the default is `global`
- `make build` - build the firmware
- `make upload PORT=/dev/ttyUSB0` - flash the board
- `make monitor PORT=/dev/ttyUSB0 BAUD=9600` - open the serial console
- `make test` - run the host unit tests and the host application tests and the simulator tests
- `make sim` - run the functional tests against the emulator
- `make eeprom-backup PORT=/dev/ttyUSB0` - save the EEPROM under `resources/eeprom/` and print one JSON line on success
- `make eeprom-restore PORT=/dev/ttyUSB0 FILE=<backup>` - write a backup to the board and check the reply to each write
- `make host-run PORT=/dev/ttyUSB0` - read the EEPROM settings with the host application and print one JSON line on success
- `make host-app-test` - run the host application tests in `host/tests/`
- `make clean` - remove build output and caches
- `make increment_version` - increment the patch version in `VERSION`
- `RELEASING.md` - the release process, from the version increment to the push
- `CHANGELOG.md` - record changes under `[Unreleased]` until a release moves them under the version heading
- `/release` - run the release process as an agent workflow; it asks for approval before the commit, the tag and the push

Every command runs inside the conda environment through the Makefile. Do not call a toolchain program directly.

## Test tiers and emulator

| Tier | Where | Command |
|---|---|---|
| Host | `test/host/` and `host/tests/` | `make test` |
| Simulator | `test/functional/` with `test/harness/` | `make sim` |
| HIL | the board on the bench | `make upload`, then `make host-run` |

| Platform | Board / MCU | Host | Simulator | HIL |
|---|---|---|---|---|
| Arduino | nano, old or new bootloader (ATmega328P) | yes | simavr | yes |

## Board facts

- Boot line: `[INIT] type 'help' for commands list` on the console, the last line `setup()` prints
- Console baud: 9600
- Upload protocol: the board's USB serial bootloader; the old bootloader runs at 57600 baud and the new one at 115200
- Reset: the board resets when the host opens the serial port; wait about 2 s before the first command

## Safety

- Ask before flashing a board that holds user data.
- Run `make eeprom-backup` before the first upload of a new firmware, so the EEPROM can be restored with `make eeprom-restore`.

## Workspace configuration

This file imports workspace-level configuration from `/home/lab/workspace/.claude/CLAUDE.md`.
All workspace rules apply. Project-specific rules below strengthen or extend them.

## Mandatory bans (reinforced)

The following workspace rules are STRICTLY ENFORCED for this project:

- **No automatic git tags** - only create tags when user explicitly requests
- **No automatic version changes** - only modify version in package.json/pyproject.toml/etc. when user explicitly requests
- **No automatic publishing** - never run `make publish`, `npm publish`, `twine upload`, or similar without explicit user request
- **No manual package installs if Makefile exists** - use `make install` or equivalent Makefile targets, not direct `pip install`/`uv install`/`npm install`
- **No automatic git commits or pushes** - only when user explicitly requests

## Project context

Audio detector firmware for an Arduino Nano (ATmega328P). The device senses an audio signal at the RCA input through the ADC, turns the amplifier on with a recorded IR code and a relay output, and turns it off after 10 minutes of silence or when the amplifier's 12V trigger drops. Public repository: `https://github.com/stellarshenson/audio-detector`, default branch `master`.

- `src/audio-detector.ino` - pin map, finite state machine, IR record and send, EEPROM persistence of the codes, button interrupt driver, serial command line
- `src/AudioLevel.h`, `src/AudioLevel.cpp` - audio level driver: samples A1 every 51 ms in every state, 50-sample average, learning (threshold = half the playing level, at least 160), hysteresis (START above the threshold, audio on held above a quarter of it), threshold in EEPROM bytes 12-13; `restart()` on idle entry makes START wait for 50 new samples
- `notebooks/audio-level-simulation.ipynb` - behavioural model of the Mk10 front end (`notebooks/frontend_model.py`) and a replica of the firmware decisions, checked against simavr; quiet reads 4 to 35 counts, loud music saturates at 575 to 580
- `references/schematics/` - PCB design files (`.dch`, `.dip` from DipTrace) and the schematic image used by the README; `references/schematics/archive/` holds earlier board revisions (mk2 to mk9)
- `resources/graphics/` - README header banner and the board images used by the README
- Libraries, pinned by the version comment beside each `#include`: IRremote 2.6.1, arduino-fsm (Stellars Henson fork with timed-transition reset), JLed 4.5.2, EnableInterrupt 1.1.0, Smoothed 1.2.0, EEPROM (Arduino core)
- `lib/vendor/` - copies of the five non-core libraries taken from the shared `../libraries` folder, found by PlatformIO through `lib_extra_dirs` in `platformio.ini` and listed with their versions in `lib/README.md`; `lib/vendor/Smoothed` carries local fixes (`clear()` return value, array initialisation, sign-compare warnings)
- Build: PlatformIO, `platformio.ini` environment `nanoatmega328` (old Nano bootloader), platform pinned to `atmelavr@5.3.0`; the `firmware-avr` conda environment (installer `resources/envs/firmware-avr.py`, run by `make create_environment`; the lab copy `~/.local/lab-utils/envs/firmware-avr.py` is a symbolic link to `envs/firmware-avr.py` of the template repository; the project copy matched it at template v1.0.16, and copier update does not refresh the project copy) provides `pio` with that platform - `conda run -n firmware-avr pio run`
- States: `state_audio_sense` (idle), `state_ircode_record`, `state_audio_learn`, `state_audio_start`, `state_audio_enabled`
- EEPROM layout: bytes 0-5 audio start IR code (value, length, type), bytes 6-11 audio stop IR code, bytes 12-13 audio sense threshold
- Serial command line at 9600 baud: `set threshold <n>`, `set audio [0|1]`, `set debug_plot [0|1]`, `eeprom dump <addr>`, `eeprom write <addr> <hex>`, `status`, `reboot`, `help`; a line runs at `\n` or `\r`, at most 64 characters
- `host/src/audiodetector_host/` - backs up and restores the EEPROM over the serial console (`python -m audiodetector_host`; `eeprom_tool.py` holds the commands, `protocol.py` the EEPROM layout and the Intel HEX codec, `serial_client.py` the console exchange): `make eeprom-backup PORT=...` writes `resources/eeprom/eeprom.json`, `eeprom.bin` and `eeprom.hex`; `make eeprom-restore PORT=... FILE=...` writes it back; a JSON restore writes bytes 0-13 only and keeps bytes 14-1023; a board whose firmware has no `eeprom` commands is backed up through the temporary `scripts/eeprom-bridge` firmware, which the backup flashes and replaces with the saved `resources/eeprom/eeprom.firmware.hex` image
- Hardware facts the firmware relies on: button D8 to GND with no filter (software debounce 20 ms); ADC reference is AVCC 5 V; IR LED carrier from Timer2 on D3 (IRremote)
- Tests: `make test` (host tests, then the build, the simulator wrapper and pytest) inside `firmware-avr`; `test/harness/` runs `firmware.elf` on simavr and names the board pins, `test/host/` covers the harness decoders, `host/tests/` covers the host package, `test/functional/` covers every acceptance criterion; time-based tests move `timer0_millis` forward instead of waiting
- Trackers: `docs/acc-crit-audio-detector.md` and `docs/defects-audio-detector.md`, written only through `pm-tools` (author `@kj`); each functional test names its criterion or defect id in the docstring

## Journal rules (project-specific)

- **APPEND ONLY**: New journal entries MUST be appended at the end of the file, never inserted between existing entries
- Entries maintain strict chronological order by position - the last entry in the file is always the most recent work
- Never reorder, move, or insert entries out of sequence
- The Stellars **journal plugin** is the canonical tool for `.claude/JOURNAL.md`: create via `/journal:create`, append via `/journal:update`, archive via `/journal:archive`. The `journal:journal` skill auto-triggers on any mention of "journal" and runs `journal-tools check` after every write
- Direct edits to `JOURNAL.md` are a last resort - prefer the plugin so modus secundis format, continuous numbering and append-only order are enforced automatically

## Strengthened rules

- **Surgical changes** - the firmware is deployed to working hardware; touch only the lines the request reaches and match the existing comment style and `DEBUG_LEVEL`-guarded `Serial` output pattern
- **Keep library versions pinned** - IRremote 2.6.1 API differs from 3.x; do not port to a newer library version without the Star Colonel's word
- **Preserve the EEPROM layout** - a change to byte offsets invalidates codes already stored on deployed devices
- **Flash and RAM are scarce** - keep string literals in `F()`, avoid new `String` allocations in the loop
- **Commits** - conventional commits, no attribution trailers, approval required every time
- **Documents** - README follows modus primaris; the ASCII state diagram follows the `ascii-diagrams` skill
