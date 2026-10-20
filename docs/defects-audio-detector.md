# Audio Detector - Defects

Defects of the firmware in `src/audio-detector.ino` and its documentation, found by code review and confirmed by the simavr functional tests in `test/functional/`.

## Authors

- `@kj` Konrad Jelen

## Audio on `ENABLE`

Audio on state: relay, standby by trigger or silence timeout

- [x] `DEF-ENABLE-1-TRIGGER-TIMEOUT` **Trigger mode turns audio off after 10 min** - MAJOR; with the 12V trigger on, audio turns off 10 min after it turned on: relay LOW, STOP sent with auto-standby, then START again; fix: restart the timeout in trigger mode too; `src/audio-detector.ino`
  - evidence: test_trigger_mode_has_no_timeout passes, relay HIGH after 15 min with trigger on; make test 2026-10-17: 69 passed
  - related: ACC-ENABLE-34-NO-TIMEOUT - the criterion this violates
  - repro: trigger on within 3 s of START, audio playing, wait 10 min
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj `fsm.reset_timed_transition` runs only in the no-trigger branch of `on_audio_enabled_loop`, so the timer started on entry fires after `AUDIO_STANDBY_TIMEOUT`
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_trigger_mode_has_no_timeout fails, `[AUDIO ENABLED] Exit` after 10 min with trigger on
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: trigger mode restarts the standby timeout every loop
  - log: 2026-10-19T18:46:35Z @kj coded TRIGGER-TIMEOUT
- [x] `DEF-ENABLE-18-LOW-THRESHOLD` **Threshold below 140 keeps the amplifier on in silence** - MAJOR; in timeout mode a threshold below 140, from `set threshold` or stored by older firmware, puts the off level inside the quiet reading (up to 35), so the timeout never runs out and STOP is never sent; `src/audio-detector.ino`, `src/AudioLevel.cpp`
  - evidence: test_threshold_below_minimum_restored_as_minimum, test_invalid_threshold_ignored, test_set_threshold[160], test_set_threshold[1023] pass; the first two fail without the fix; make test 2026-10-17: 81 passed
  - related: ACC-ENABLE-54-OFF-LEVEL
  - repro: EEPROM threshold 100, audio on in timeout mode, then level 35 for 11 min: relay stays on
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T13:46:13Z @kj the 160 minimum that keeps the off level above quiet was applied only by learning
  - log: 2026-10-17T13:46:13Z @kj added
  - log: 2026-10-17T13:46:24Z @kj reproduced in simavr by review round 2 (bug-hunter, qa-engineer) and by the adjudicator: test_threshold_below_minimum_restored_as_minimum and test_invalid_threshold_ignored fail on the firmware without the fix
  - log: 2026-10-17T13:46:24Z @kj closed: fixed: set threshold accepts 160 to 1023, a stored value below 160 restores as 160
  - log: 2026-10-19T18:46:35Z @kj coded LOW-THRESHOLD

## IR recording `RECORD`

Recording START and STOP codes

- [x] `DEF-RECORD-2-NEC-REPEAT` **Held remote button saves NEC repeat as STOP** - MEDIUM; holding the remote button while recording saves STOP as NEC 0x0; fix: a repeat frame is not a code, the receiver resumes; `src/audio-detector.ino`
  - evidence: test_nec_repeat_not_saved passes, STOP is NEC B after 1.5 s of repeat frames; make test 2026-10-17: 69 passed
  - related: ACC-RECORD-18-NEC-REPEAT - the criterion this violates
  - repro: hold, send NEC A, repeat frames for 1.5 s, NEC B; EEPROM STOP is 0x0
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj `storeCode` sets `codeType` to NEC before it returns on `REPEAT`, so the record loop treats the repeat as a recorded code
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_nec_repeat_not_saved fails, STOP saved as (NEC, 255 bits, 0x0)
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: storeCode returns false on NEC repeat, receiver resumes
  - log: 2026-10-19T18:46:35Z @kj coded NEC-REPEAT
- [x] `DEF-RECORD-3-OTHER-PROTOCOL` **Codes of other protocols saved but never sent** - MEDIUM; a Samsung, RC5, JVC or other code completes recording, but the device never sends it; fix: accept only NEC and Sony, print a rejection for others; `src/audio-detector.ino`
  - evidence: test_unsupported_protocol_rejected passes, Samsung rejected, START A, STOP B; make test 2026-10-17: 69 passed
  - related: ACC-RECORD-19-UNSUPPORTED-PROTOCOL - the criterion this violates
  - repro: hold, send Samsung, then NEC A; START is the Samsung code
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj `storeCode` accepts every decoded type, `sendCode` sends only NEC and Sony; `UNKNOWN` (-1) in a `uint8_t` equals the 255 not-recorded marker
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_unsupported_protocol_rejected fails, Samsung code taken as START
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: only NEC and Sony stored, others rejected with a serial line
  - log: 2026-10-19T18:46:35Z @kj coded OTHER-PROTOCOL

## Power-on `BOOT`

Power-on: EEPROM restore and jumper read

- [x] `DEF-BOOT-4-ERASED-EEPROM` **Erased EEPROM disables audio detection** - MEDIUM; a board with erased EEPROM never detects audio until the threshold is learned; fix: keep `AUDIOLEVEL_INIT_THRESHOLD` when the stored value is above 1023; `src/audio-detector.ino`, `src/AudioLevel.cpp`
  - evidence: test_erased_eeprom_uses_default_threshold passes, audio 400 detected on erased EEPROM; make test 2026-10-17: 69 passed
  - related: ACC-BOOT-6-ERASED-THRESHOLD - the criterion this violates
  - repro: boot with erased EEPROM, audio 1023, nothing happens
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj erased bytes 12-13 read as 65535, above the ADC maximum 1023, and replace the default 300 unconditionally in `setup`
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_erased_eeprom_uses_default_threshold fails, threshold 65535, audio 400 never detected
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: stored threshold above 1023 keeps 300
  - log: 2026-10-17T14:03:19Z @kj amended text "a board with erased EEPROM never detects audio until the threshold is learned; fix: keep `AUDIOSENSE_INIT_THRESHOLD` when the stored value is above 1023; `src/audio-detector.ino`" -> "a board with erased EEPROM never detects audio until the threshold is learned; fix: keep `AUDIOLEVEL_INIT_THRESHOLD` when the stored value is above 1023; `src/audio-detector.ino`, `src/AudioLevel.cpp`"
  - log: 2026-10-19T18:46:35Z @kj coded ERASED-EEPROM
- [x] `DEF-BOOT-5-STANDBY-STATE` **Boot line shows wrong auto-standby state** - MINOR; `[INIT] Automatic standby is:` does not follow the D6 jumper; fix: print the flag; `src/audio-detector.ino`
  - evidence: test_auto_standby_reported passes for D6 LOW (ON) and open (OFF); make test 2026-10-17: 69 passed
  - related: ACC-BOOT-5-STANDBY-REPORT - the criterion this violates
  - repro: boot with D6 open; line says ON
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj `setup` prints `digitalRead(autoStandbyEnabled)`, which reads pin 0 or 1, not the flag
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_auto_standby_reported[False-OFF] fails, line says ON with D6 open
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: boot line prints the autoStandbyEnabled flag
  - log: 2026-10-19T18:46:35Z @kj coded STANDBY-STATE

## Timing `TIMING`

Interval checks across the millis() wrap

- [x] `DEF-TIMING-6-SAMPLING-WRAP` **Audio sampling can stop at the millis() wrap** - MEDIUM; after 49.7 days of uptime the audio level can stop updating until the next power-on; fix: compare `millis() - last` with the interval; `src/audio-detector.ino`, `src/AudioLevel.cpp`
  - evidence: test_audio_detected_across_wrap, test_button_press_across_wrap pass; test_audio_detected_across_wrap fails on the addition-form timer; make test 2026-10-17: 79 passed
  - related: ACC-TIMING-50-MILLIS-WRAP - the criterion this violates
  - repro: check written millis() > last + 50: a sample at exactly 2^32-51 waits for millis() > 2^32-1 forever; with that check, millis() set to 2^32-51 and audio 600 is never detected; on HEAD the first sample after that jump lands at 2^32-50 and detection comes 3.1 s later (DEF-TIMING-17-AUDIO-IGNORED)
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj interval checks written `millis() > last + interval` in `senseAudio`, `on_audio_learn_loop` and `on_button_update` never pass when `last + interval` is 2^32-1
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_audio_sampling_across_wrap fails, no detection for 10 s; test_button_press_across_wrap fails, 200 ms press read as hold
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: interval checks compare millis() - last
  - log: 2026-10-17T10:41:43Z @kj edited repro "set the last sample time to 2^32-51 and millis() to 2^32-40, audio 1023; never detected" -> "set millis() to 2^32-51 so the next sample is due at 2^32, audio 600; never detected"; evidence "test_audio_sampling_across_wrap and test_button_press_across_wrap pass; make test 2026-10-17: 69 passed" -> "test_audio_detected_across_wrap, test_button_press_across_wrap pass; test_audio_detected_across_wrap fails on the addition-form timer; make test 2026-10-17: 79 passed"
  - log: 2026-10-17T13:46:24Z @kj edited repro "set millis() to 2^32-51 so the next sample is due at 2^32, audio 600; never detected" -> "check written millis() > last + 50: a sample at exactly 2^32-51 waits for millis() > 2^32-1 forever; with that check, millis() set to 2^32-51 and audio 600 is never detected; on HEAD the first sample after that jump lands at 2^32-50 and detection comes 3.1 s later (DEF-TIMING-17)"; reason: the repro must state the exact sample alignment and the HEAD behaviour, as review round 2 found
  - log: 2026-10-17T14:03:19Z @kj amended text "after 49.7 days of uptime the audio level can stop updating for another 49.7 days; fix: compare `millis() - last` with the interval; `src/audio-detector.ino`" -> "after 49.7 days of uptime the audio level can stop updating for another 49.7 days, or until the next power-on; fix: compare `millis() - last` with the interval; `src/audio-detector.ino`, `src/AudioLevel.cpp`"
  - log: 2026-10-17T16:06:44Z @kj amended text "after 49.7 days of uptime the audio level can stop updating for another 49.7 days, or until the next power-on; fix: compare `millis() - last` with the interval; `src/audio-detector.ino`, `src/AudioLevel.cpp`" -> "after 49.7 days of uptime the audio level can stop updating until the next power-on; fix: compare `millis() - last` with the interval; `src/audio-detector.ino`, `src/AudioLevel.cpp`"
  - log: 2026-10-19T18:46:35Z @kj coded SAMPLING-WRAP
  - log: 2026-10-19T18:47:24Z @kj edited repro "check written millis() > last + 50: a sample at exactly 2^32-51 waits for millis() > 2^32-1 forever; with that check, millis() set to 2^32-51 and audio 600 is never detected; on HEAD the first sample after that jump lands at 2^32-50 and detection comes 3.1 s later (DEF-TIMING-17)" -> "check written millis() > last + 50: a sample at exactly 2^32-51 waits for millis() > 2^32-1 forever; with that check, millis() set to 2^32-51 and audio 600 is never detected; on HEAD the first sample after that jump lands at 2^32-50 and detection comes 3.1 s later (DEF-TIMING-17-AUDIO-IGNORED)"; reason: full id of the linked defect after migrate-codes
- [x] `DEF-TIMING-17-AUDIO-IGNORED` **Audio ignored for 3 s after the millis() wrap** - MINOR; every 49.7 days, for 3 s after millis() wraps, idle ignores audio as if the board had just powered on; `src/audio-detector.ino`
  - evidence: test_audio_detected_across_wrap passes, detection within 2 s; fails with the old check; make test 2026-10-17: 79 passed
  - related: ACC-TIMING-50-MILLIS-WRAP
  - repro: set millis() to 2^32-51, audio 600: detected 3 s after the wrap instead of about 1.3 s
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T10:41:43Z @kj on_audio_sense_loop tests millis() < STARTUP_STABILISE_DURATION, which is true again right after the wrap
  - log: 2026-10-17T10:41:43Z @kj added
  - log: 2026-10-17T10:41:49Z @kj reproduced in simavr: with the old check test_audio_detected_across_wrap fails, detection 3 s after the wrap
  - log: 2026-10-17T10:41:49Z @kj closed: fixed: the settling check latches once millis() reaches 3 s
  - log: 2026-10-19T18:46:35Z @kj coded AUDIO-IGNORED

## Serial commands `SERIAL`

Serial command line

- [x] `DEF-SERIAL-7-EMPTY-LINE` **Empty line repeats the previous command** - MINOR; an empty line runs `help`, `status` or `reboot` again; fix: no command when the line has no tokens; `src/audio-detector.ino`
  - evidence: test_empty_line_runs_nothing passes, one [HELP] after help and an empty line; make test 2026-10-17: 69 passed
  - related: ACC-SERIAL-47-EMPTY-LINE - the criterion this violates
  - repro: send `help`, then an empty line; `[HELP]` prints twice
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj `_argv` in `cmd_poll` is static and keeps the previous tokens, and `cmd_handler` reads `argv[0]` when `argc` is 0
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:16:04Z @kj reproduced in simavr: test_empty_line_runs_nothing fails, `[HELP]` printed twice
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: empty line runs nothing
  - log: 2026-10-19T18:46:35Z @kj coded EMPTY-LINE
- [x] `DEF-SERIAL-8-BLOCKING-INPUT` **Command input stops the device for 1 s** - MINOR; each reply comes 1 s after the command, and the state machine, LED and button stop for that second; fix: collect bytes into a line, run it at `\n` or `\r`; `src/audio-detector.ino`
  - evidence: test_reply_within_100_ms passes; make test 2026-10-17: 69 passed
  - related: ACC-SERIAL-48-NON-BLOCKING - the criterion this violates
  - repro: send `status` with newline; reply after 1000 ms
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:10:33Z @kj `serial.readString()` returns only after the 1000 ms Stream timeout with no new byte
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:10:41Z @kj amended text "each reply comes 1 s after the command, and the state machine, LED and button stop for that second; fix: collect bytes into a line, run it at `\\n` or `\\r`; `src/audio-detector.ino`" -> "each reply comes 1 s after the command, and the state machine, LED and button stop for that second; fix: collect bytes into a line, run it at `\n` or `\r`; `src/audio-detector.ino`"
  - log: 2026-10-17T09:16:05Z @kj reproduced in simavr: test_reply_within_100_ms fails, reply 1.02 s after line end
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: cmd_poll collects bytes without waiting, runs the line at \n or \r
  - log: 2026-10-19T18:46:35Z @kj coded BLOCKING-INPUT
- [x] `DEF-SERIAL-9-COMMAND-READER` **Undefined behaviour in the command reader** - MINOR; compiler warns `invalid conversion from 'void (*)(int, String*)' to 'void* (*)(int, String*)'` and `passing 'const String' as 'this' argument discards qualifiers`; fix: `void (*)(int, String*)` handler, non-const token array; `src/audio-detector.ino`
  - evidence: pio run shows no warning for src/audio-detector.ino; make test 2026-10-17: 69 passed
  - repro: `conda run -n firmware-avr pio run`, read the warnings at lines 220, 783, 790
  - test-tags: MANUAL
  - root-cause: 2026-10-17T09:10:33Z @kj `cmd_poll` declares `handler` as a function returning `void*`, and writes tokens into a `static const String` array
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: handler typed void (*)(int, String*), token array not const
  - log: 2026-10-17T10:43:32Z @kj edited test-tags "FUNCTIONAL" -> "MANUAL"
  - log: 2026-10-19T18:46:35Z @kj coded COMMAND-READER

## Button `BUTTON`

Button press and hold detection

- [x] `DEF-BUTTON-10-SHARED-STATE` **Button state shared with the interrupt unprotected** - MINOR; press and hold detection can read a half-updated 32-bit press time; Serial prints run inside the interrupt; fix: loop call with interrupts off, no prints in `on_button_update`; `src/audio-detector.ino`
  - evidence: code at loop() and on_button_update; button tests pass; make test 2026-10-17: 69 passed
  - repro: cannot reproduce on demand; race between the pin-change interrupt and loop(), found by code reading
  - test-tags: MANUAL
  - root-cause: 2026-10-17T09:10:33Z @kj `on_button_update` runs from the pin-change interrupt and from `loop()` and both change the same static variables
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:31:27Z @kj closed: fixed: loop() calls on_button_update with interrupts off; no Serial in it
  - log: 2026-10-19T18:46:35Z @kj coded SHARED-STATE
- [x] `DEF-BUTTON-15-CONTACT-BOUNCE` **Contact bounce turns a hold into a press** - MAJOR; the button shorts D8 to GND with no filter; the first bounce of a press counts as a release, so holding the button starts learning instead of IR recording; fix: software debounce, a level counts after 20 ms unchanged; `src/audio-detector.ino`
  - evidence: test_bouncing_hold_is_one_hold, test_bouncing_press_is_one_press, test_hold_starts_recording pass with 5 ms bounce; make test 2026-10-17: 69 passed
  - related: ACC-SENSE-53-CONTACT-BOUNCE - the criterion this violates
  - repro: hold the button 1 s with about 5 ms of contact bounce; `[AUDIOSENSE LEARN]` instead of `[IRCODE RECORD]`
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:22:53Z @kj `on_button_update` takes every level change as a press or release, and the pin-change interrupt runs it on each bounce
  - log: 2026-10-17T09:22:53Z @kj added
  - log: 2026-10-17T09:22:53Z @kj reproduced in simavr: test_hold_starts_recording and test_bouncing_hold_is_one_hold fail with bouncing presses
  - log: 2026-10-17T09:31:28Z @kj closed: fixed: software debounce, level counts after 20 ms unchanged
  - log: 2026-10-19T18:46:35Z @kj coded CONTACT-BOUNCE

## Build `BUILD`

Compiler output of the firmware build

- [x] `DEF-BUILD-11-SKETCH-WARNINGS` **Compiler warnings in the sketch** - MINOR; `pio run` warns sign-compare at lines 252, 516, 769, parentheses at 530, unused variable `count` at 629; fix: matching types, explicit conditions, remove `count`; `src/audio-detector.ino`
  - evidence: pio run shows 0 warnings for src/; make test 2026-10-17: 69 passed
  - repro: `conda run -n firmware-avr pio run`, read warnings for `src/`
  - test-tags: MANUAL
  - root-cause: 2026-10-17T09:10:33Z @kj `senseAudio` returns `int` compared with `uint16_t` (unsigned int on AVR), `int` loop index against `String::length()`, mixed `&&`/`||` without parentheses
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:31:28Z @kj closed: fixed: senseAudio returns uint16_t, unsigned index, explicit condition, count removed
  - log: 2026-10-17T10:43:32Z @kj edited test-tags "FUNCTIONAL" -> "MANUAL"
  - log: 2026-10-19T18:46:35Z @kj coded SKETCH-WARNINGS
- [x] `DEF-BUILD-14-SMOOTHED-CLEAR` **Smoothed library: clear() returns no value** - MINOR; `Smoothed::clear` falls off the end of a `bool` function (undefined behaviour), `begin` and `clear` never zero the array, `smoothReading` uninitialised; 4 compiler warnings; fix: return true, loop to `smoothReadingsFactor` with `uint16_t`, initialise members; `lib/vendor/Smoothed/src/Smoothed.h`
  - evidence: pio run shows 0 warnings for lib/Smoothed; make test 2026-10-17: 69 passed
  - repro: `conda run -n firmware-avr pio run`, warnings at `Smoothed.h` 56, 127, 184, 199
  - test-tags: MANUAL
  - root-cause: 2026-10-17T09:20:04Z @kj the loops compare an `int` with the `uint16_t` count `smoothReadingsNum`, which is 0 at `begin` and right after reset in `clear`; `clear` has no return after the switch
  - log: 2026-10-17T09:20:04Z @kj added; reason: the defect line names four separate faults in one library file that one fix covers
  - log: 2026-10-17T09:31:28Z @kj closed: fixed: clear returns true, loops to smoothReadingsFactor with uint16_t, members initialised
  - log: 2026-10-17T09:31:28Z @kj edited test-tags "UNIT" -> "FUNCTIONAL"
  - log: 2026-10-17T10:43:32Z @kj edited test-tags "FUNCTIONAL" -> "MANUAL"
  - log: 2026-10-19T18:39:46Z @kj edited text "`Smoothed::clear` falls off the end of a `bool` function (undefined behaviour), `begin` and `clear` never zero the array, `smoothReading` uninitialised; 4 compiler warnings; fix: return true, loop to `smoothReadingsFactor` with `uint16_t`, initialise members; `lib/Smoothed/src/Smoothed.h`" -> "`Smoothed::clear` falls off the end of a `bool` function (undefined behaviour), `begin` and `clear` never zero the array, `smoothReading` uninitialised; 4 compiler warnings; fix: return true, loop to `smoothReadingsFactor` with `uint16_t`, initialise members; `lib/vendor/Smoothed/src/Smoothed.h`"
  - log: 2026-10-19T18:46:35Z @kj coded SMOOTHED-CLEAR

## Documentation `DOCS`

README, project notes and source comments

- [x] `DEF-DOCS-12-JLED-VERSION` **JLed version stated as 4.11** - MINOR; sketch comment, README and the project instruction file (CLAUDE.md under .claude, now AGENTS.md) name JLed 4.11, the build uses `lib/vendor/JLed` 4.5.2; fix: state 4.5.2; `src/audio-detector.ino`, `README.md`
  - evidence: grep shows JLed 4.5.2 in src/audio-detector.ino, README.md, .claude/CLAUDE.md, matching lib/JLed/library.properties
  - repro: read `lib/vendor/JLed/library.properties`
  - test-tags: MANUAL
  - root-cause: 2026-10-17T09:10:33Z @kj the libraries were copied from the shared `../libraries` folder, which holds JLed 4.5.2
  - log: 2026-10-17T09:10:33Z @kj added
  - log: 2026-10-17T09:32:30Z @kj closed: fixed: sketch comment, README and .claude/CLAUDE.md state JLed 4.5.2
  - log: 2026-10-19T18:32:30Z @kj edited repro "read `lib/JLed/library.properties`" -> "read `lib/vendor/JLed/library.properties`"
  - log: 2026-10-19T18:39:46Z @kj edited text "sketch comment, README and `.claude/CLAUDE.md` name JLed 4.11, the build uses `lib/JLed` 4.5.2; fix: state 4.5.2; `src/audio-detector.ino`, `README.md`" -> "sketch comment, README and `.claude/CLAUDE.md` name JLed 4.11, the build uses `lib/vendor/JLed` 4.5.2; fix: state 4.5.2; `src/audio-detector.ino`, `README.md`"
  - log: 2026-10-19T18:46:35Z @kj coded JLED-VERSION
  - log: 2026-10-19T19:17:45Z @kj edited text "sketch comment, README and `.claude/CLAUDE.md` name JLed 4.11, the build uses `lib/vendor/JLed` 4.5.2; fix: state 4.5.2; `src/audio-detector.ino`, `README.md`" -> "sketch comment, README and the project instruction file (CLAUDE.md under .claude, now AGENTS.md) name JLed 4.11, the build uses `lib/vendor/JLed` 4.5.2; fix: state 4.5.2; `src/audio-detector.ino`, `README.md`"

## Idle `SENSE`

Idle state: audio detection

- [x] `DEF-SENSE-13-IDLE-SPIKE` **Spike after return to idle starts the amplifier** - MEDIUM; a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: idle entry clears the average and START waits for 50 new samples (DEF-SENSE-13-IDLE-SPIKE-1); `src/audio-detector.ino`, `src/AudioLevel.cpp`
  - evidence: test_spike_right_after_return_to_idle passes; fails when restart() clears the average without resetting the sample count; make test 2026-10-17: 81 passed
  - related: ACC-SENSE-10-SMOOTHING - the criterion this violates
  - repro: trigger mode, audio on, quiet 3 s, trigger off, then audio 1023 for 100 ms as soon as `[AUDIO SENSE] Start` prints: `[AUDIO START]` follows
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T09:20:04Z @kj `on_audio_sense_exit` and `on_audio_enabled_exit` clear `audioSenseMovingAvg`, so the next average holds one or two samples
  - log: 2026-10-17T09:20:04Z @kj added
  - log: 2026-10-17T09:20:04Z @kj reproduced in simavr: test_spike_ignored_after_return_to_idle fails, `[AUDIO START]` after the spike
  - log: 2026-10-17T09:31:28Z @kj closed: fixed: moving average kept across states
  - log: 2026-10-17T10:10:34Z @kj regressed as DEF-SENSE-13-1
  - log: 2026-10-17T10:41:43Z @kj reproduction record false: test_spike_ignored_after_return_to_idle passed on the defective firmware; the defect reproduces with a spike right after return to idle by trigger off
  - log: 2026-10-17T13:46:24Z @kj amended text "a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: keep the moving average across states; `src/audio-detector.ino`" -> "a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: idle entry clears the average and START waits for 50 new samples (DEF-SENSE-13-1); `src/audio-detector.ino`"
  - log: 2026-10-17T13:46:24Z @kj edited evidence "test_spike_ignored_after_return_to_idle passes; make test 2026-10-17: 69 passed" -> "test_spike_right_after_return_to_idle passes; fails when restart() clears the average without resetting the sample count; make test 2026-10-17: 81 passed"
  - log: 2026-10-17T14:03:19Z @kj amended text "a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: idle entry clears the average and START waits for 50 new samples (DEF-SENSE-13-1); `src/audio-detector.ino`" -> "a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: idle entry clears the average and START waits for 50 new samples (DEF-SENSE-13-1); `src/audio-detector.ino`, `src/AudioLevel.cpp`"
  - log: 2026-10-17T14:03:19Z @kj edited repro "silence timeout to idle, then audio 1023 for 100 ms; `[AUDIO START]` follows" -> "trigger mode, audio on, quiet 3 s, trigger off, then audio 1023 for 100 ms as soon as `[AUDIO SENSE] Start` prints: `[AUDIO START]` follows"
  - log: 2026-10-19T18:46:35Z @kj coded IDLE-SPIKE
  - log: 2026-10-19T18:47:25Z @kj amended text "a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: idle entry clears the average and START waits for 50 new samples (DEF-SENSE-13-1); `src/audio-detector.ino`, `src/AudioLevel.cpp`" -> "MEDIUM; a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: idle entry clears the average and START waits for 50 new samples (DEF-SENSE-13-IDLE-SPIKE-1); `src/audio-detector.ino`, `src/AudioLevel.cpp`"; reason: full id of the regression after migrate-codes
- [x] `DEF-SENSE-13-IDLE-SPIKE-1` **Stale average starts the amplifier after standby** - MAJOR; the kept moving average sends START again after standby within 1.25 s of the music stopping, after `set audio 0`, and after a silent recording from audio on; `src/audio-detector.ino`, `src/AudioLevel.cpp`
  - test-tags: FUNCTIONAL
  - evidence: test_standby_right_after_music_stops, test_set_audio_0_right_after_music_stops, test_spike_right_after_return_to_idle, test_silent_recording_from_audio_on pass; first two fail without restart(); spike test fails when restart() keeps the sample count; recording test fails only without restart() and sampling outside idle and audio on; make test 2026-10-17: 81 passed
  - related: ACC-SENSE-57-NEW-SAMPLES
  - root-cause: 2026-10-19T18:47:25Z @kj the DEF-SENSE-13-IDLE-SPIKE fix keeps the average across states: idle decides on entry on the last 2.5 s of loud audio; record, learn and start do not sample, so the average can be minutes old
  - repro: trigger mode, audio 600 then 0, trigger off 0.3 s later: STOP then START
  - log: 2026-10-17T10:10:34Z @kj regression of DEF-SENSE-13: review round 1: the kept average sends START again after standby
  - log: 2026-10-17T10:10:34Z @kj amended text "a 100 ms spike in the first samples after audio on or learning ends starts the amplifier again; fix: keep the moving average across states; `src/audio-detector.ino`" -> "the kept moving average sends START again after standby within 1.25 s of the music stopping, after `set audio 0`, and after a silent recording from audio on; `src/audio-detector.ino`"
  - log: 2026-10-17T10:10:34Z @kj edited severity "MEDIUM" -> "MAJOR"; repro added "trigger mode, audio 600 then 0, trigger off 0.3 s later: STOP then START"
  - log: 2026-10-17T10:10:38Z @kj amended title "Spike after return to idle starts the amplifier" -> "Stale average starts the amplifier after standby"
  - log: 2026-10-17T10:41:43Z @kj closed: fixed: AudioLevel samples in every state; idle entry clears the average and START waits for 50 new samples
  - log: 2026-10-17T10:41:52Z @kj edited test-tags added "FUNCTIONAL"
  - log: 2026-10-17T13:46:24Z @kj edited evidence "test_standby_right_after_music_stops, test_set_audio_0_right_after_music_stops, test_silent_recording_from_audio_on, test_spike_right_after_return_to_idle pass; each fails on a mutant without the fix; make test 2026-10-17: 79 passed" -> "test_standby_right_after_music_stops, test_set_audio_0_right_after_music_stops, test_spike_right_after_return_to_idle, test_silent_recording_from_audio_on pass; first two fail without restart(); spike test fails when restart() keeps the sample count; recording test fails only without restart() and sampling outside idle and audio on; make test 2026-10-17: 81 passed"; reason: one evidence line must name the mutant for each of the four tests, as review round 2 asked
  - log: 2026-10-17T14:03:19Z @kj amended text "the kept moving average sends START again after standby within 1.25 s of the music stopping, after `set audio 0`, and after a silent recording from audio on; `src/audio-detector.ino`" -> "the kept moving average sends START again after standby within 1.25 s of the music stopping, after `set audio 0`, and after a silent recording from audio on; `src/audio-detector.ino`, `src/AudioLevel.cpp`"
  - log: 2026-10-19T18:46:35Z @kj coded IDLE-SPIKE
  - log: 2026-10-19T18:47:25Z @kj root-cause updated "2026-10-17T10:10:34Z @kj the DEF-SENSE-13 fix keeps the average across states: idle decides on entry on the last 2.5 s of loud audio; record, learn and start do not sample, so the average can be minutes old" -> "the DEF-SENSE-13-IDLE-SPIKE fix keeps the average across states: idle decides on entry on the last 2.5 s of loud audio; record, learn and start do not sample, so the average can be minutes old"; reason: full id of the root defect after migrate-codes

## Threshold learning `LEARN`

learning the audio threshold from the playing level

- [x] `DEF-LEARN-16-UNREACHABLE-THRESHOLD` **Learned threshold unreachable by the same music** - MAJOR; learning on loud music sets the threshold to the saturated reading, about 579, so the same music starts the amplifier late or never; `src/audio-detector.ino`, `src/AudioLevel.cpp`
  - evidence: test_learned_on_loud_music_starts_again, test_threshold_half_the_playing_level pass; both fail on the mean rule; make test 2026-10-17: 79 passed
  - related: ACC-LEARN-22-HALF-LEVEL
  - repro: learn on audio 600, silence, then audio 600: no `[AUDIO START]`
  - test-tags: FUNCTIONAL
  - root-cause: 2026-10-17T10:41:43Z @kj learning stores the mean of the samples and START needs a level above it; the gain of 101 saturates every normal listening level at the same reading
  - log: 2026-10-17T10:41:43Z @kj added
  - log: 2026-10-17T10:41:49Z @kj reproduced in simavr: with the mean rule test_learned_on_loud_music_starts_again fails, no START; also in the front-end simulation, pop learned at 579 never starts
  - log: 2026-10-17T10:41:49Z @kj closed: fixed: learning sets half the average, at least 160
  - log: 2026-10-17T14:03:19Z @kj amended text "learning on loud music sets the threshold to the saturated reading, about 579, so the same music starts the amplifier late or never; `src/audio-detector.ino`" -> "learning on loud music sets the threshold to the saturated reading, about 579, so the same music starts the amplifier late or never; `src/audio-detector.ino`, `src/AudioLevel.cpp`"
  - log: 2026-10-19T18:46:35Z @kj coded UNREACHABLE-THRESHOLD

