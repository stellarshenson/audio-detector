<!-- @import /home/lab/workspace/.claude/CLAUDE.md -->

# Project-Specific Configuration

This file imports workspace-level configuration from `/home/lab/workspace/.claude/CLAUDE.md`.
All workspace rules apply. Project-specific rules below strengthen or extend them.

The workspace `/home/lab/workspace/.claude/` directory contains additional instruction files
(MERMAID.md, NOTEBOOK.md, DATASCIENCE.md, GIT.md, and others) referenced by CLAUDE.md.
Consult workspace CLAUDE.md and the .claude directory to discover all applicable standards.

## Mandatory Bans (Reinforced)

The following workspace rules are STRICTLY ENFORCED for this project:

- **No automatic git tags** - only create tags when user explicitly requests
- **No automatic version changes** - only modify version in package.json/pyproject.toml/etc. when user explicitly requests
- **No automatic publishing** - never run `make publish`, `npm publish`, `twine upload`, or similar without explicit user request
- **No manual package installs if Makefile exists** - use `make install` or equivalent Makefile targets, not direct `pip install`/`uv install`/`npm install`
- **No automatic git commits or pushes** - only when user explicitly requests

## Project Context

Audio detector firmware for an Arduino Nano (ATmega328P). The device senses an audio signal at the RCA input through the ADC, turns the amplifier on with a recorded IR code and a relay output, and turns it off after 10 minutes of silence or when the amplifier's 12V trigger drops. Public repository: `https://github.com/stellarshenson/audio-detector`, default branch `master`.

- `src/audio-detector.ino` - the whole firmware: pin map, finite state machine, IR record and send, EEPROM persistence, button interrupt driver, serial command line
- `misc/` - PCB design files (`.dch`, `.dip` from DipTrace), schematic and board images used by the README; `misc/archive/` holds earlier board revisions (mk2 to mk9)
- Libraries, pinned by the version comment beside each `#include`: IRremote 2.6.1, arduino-fsm (Stellars Henson fork with timed-transition reset), JLed 4.11, EnableInterrupt 1.1.0, Smoothed 1.2.0, EEPROM (Arduino core)
- `lib/` - copies of the five non-core libraries taken from the shared `../libraries` folder; the bundled JLed is 4.5.2, not the 4.11 named in the sketch comment
- Build: PlatformIO, `platformio.ini` environment `nanoatmega328` (old Nano bootloader), platform pinned to `atmelavr@5.3.0`; the `firmware-avr` conda environment (installer `~/.local/lab-utils/envs/firmware-avr.py`) provides `pio` with that platform - `conda run -n firmware-avr pio run`
- States: `state_audio_sense` (idle), `state_ircode_record`, `state_audio_learn`, `state_audio_start`, `state_audio_enabled`
- EEPROM layout: bytes 0-5 audio start IR code (value, length, type), bytes 6-11 audio stop IR code, bytes 12-13 audio sense threshold
- Serial command line at 9600 baud: `set threshold <n>`, `set audio [0|1]`, `set debug_plot [0|1]`, `status`, `reboot`, `help`

## Journal Rules (Project-Specific)

- **APPEND ONLY**: New journal entries MUST be appended at the end of the file, never inserted between existing entries
- Entries maintain strict chronological order by position - the last entry in the file is always the most recent work
- Never reorder, move, or insert entries out of sequence
- The Stellars **journal plugin** is the canonical tool for this file: create via `/journal:create`, append via `/journal:update`, archive via `/journal:archive`. The `journal:journal` skill auto-triggers on any mention of "journal" and runs `journal-tools check` after every write
- Direct edits to `JOURNAL.md` are a last resort - prefer the plugin so modus secundis format, continuous numbering and append-only order are enforced automatically

## Strengthened Rules

- **Surgical changes** - the firmware is a single file deployed to working hardware; touch only the lines the request reaches and match the existing comment style and `DEBUG_LEVEL`-guarded `Serial` output pattern
- **Keep library versions pinned** - IRremote 2.6.1 API differs from 3.x; do not port to a newer library version without the Star Colonel's word
- **Preserve the EEPROM layout** - a change to byte offsets invalidates codes already stored on deployed devices
- **Flash and RAM are scarce** - keep string literals in `F()`, avoid new `String` allocations in the loop
- **Commits** - conventional commits, no attribution trailers, approval required every time
- **Documents** - README follows modus primaris; the ASCII state diagram follows the `ascii-diagrams` skill
