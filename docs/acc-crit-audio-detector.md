# Audio Detector - Acceptance Criteria

Behaviour of the firmware in `src/audio-detector.ino` on the Arduino Nano. A five-state machine (idle, IR recording, threshold learning, audio start, audio on) switches the amplifier by IR code and relay from the RCA audio level and the amplifier's 12V trigger; settings live in EEPROM.

## Authors

- `@kj` Konrad Jelen

## Power-on `BOOT`

Power-on: settings restored from EEPROM, auto-standby jumper read

- [x] `ACC-BOOT-1` **Settings restored** - HIGH; at power-on the START code, STOP code and threshold are read from EEPROM bytes 0-13 and printed in the `[INIT] Restoring IR codes` line
  - evidence: test_settings_restored pass; make test 2026-10-17: 71 passed
  - test: boot with codes and threshold in EEPROM, assert the values in the `[INIT]` line
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:22Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-BOOT-2` **Relay off at power-on** - HIGH; relay output D4 is LOW after power-on
  - evidence: test_relay_off_at_power_on pass; make test 2026-10-17: 71 passed
  - test: boot, run 1 s, assert D4 LOW
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-BOOT-3` **Settling time** - MEDIUM; audio is ignored for the first 3 s after power-on
  - evidence: test_audio_ignored_while_settling pass; make test 2026-10-17: 71 passed
  - test: audio 1023 from power-on, assert no `Audio signal detected` before 3 s
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-BOOT-4` **Auto-standby jumper** - HIGH; D6 tied to GND at power-on enables auto-standby; D6 open disables it
  - evidence: test_stop_code_on_standby pass; make test 2026-10-17: 71 passed
  - test: boot with D6 LOW, then HIGH; compare STOP code sent on standby
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-BOOT-5` **Auto-standby report** - LOW; boot line `[INIT] Automatic standby is:` shows ON when D6 is LOW and OFF when D6 is open
  - evidence: test_auto_standby_reported pass; make test 2026-10-17: 71 passed
  - test: boot each way, assert the word
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-BOOT-6` **Edge: erased threshold** - HIGH; with EEPROM bytes 12-13 erased (0xFFFF) the threshold is 300
  - evidence: test_erased_eeprom_uses_default_threshold pass; make test 2026-10-17: 79 passed
  - test: boot with erased EEPROM, assert threshold 300 in `status`, audio 400 detected
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj edited test "boot with erased EEPROM, audio 400, assert audio detected" -> "boot with erased EEPROM, assert threshold 300 in `status`, audio 400 detected"; evidence "test_erased_eeprom_uses_default_threshold pass; make test 2026-10-17: 71 passed" -> "test_erased_eeprom_uses_default_threshold pass; make test 2026-10-17: 79 passed"
- [x] `ACC-BOOT-7` **Edge: no IR codes** - MEDIUM; with no IR codes in EEPROM the LED blinks fast, 250 ms on and 1000 ms off
  - evidence: test_fast_blink_without_codes pass; make test 2026-10-17: 71 passed
  - test: boot with erased EEPROM, measure LED on and off times
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-BOOT-51` **Edge: stored code that cannot be sent** - MEDIUM; a code in EEPROM whose type is not NEC or Sony counts as not recorded: `[INIT] IR codes not available`, LED blinks fast
  - evidence: test_stored_code_that_cannot_be_sent_is_not_recorded pass; make test 2026-10-17: 71 passed
  - test: boot with a Samsung START in EEPROM, assert `IR codes not available`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:20:07Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr

## Idle `SENSE`

Idle state: waiting for audio or the button

- [x] `ACC-SENSE-8` **Audio detected** - CRITICAL; in idle, a smoothed audio level above the threshold moves to audio start
  - evidence: test_audio_detected pass; make test 2026-10-17: 71 passed
  - test: threshold 300, audio 600, assert `[AUDIO START] Start`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-SENSE-9` **Audio below threshold** - HIGH; in idle, a smoothed audio level at or below the threshold never moves to audio start
  - evidence: test_audio_below_threshold_ignored[200], test_audio_below_threshold_ignored[290] pass; make test 2026-10-17: 79 passed
  - test: threshold 300, audio 200 and 290 for 30 s each, assert no `[AUDIO START]`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:33Z @kj edited test "threshold 300, audio 200 for 30 s, assert no `[AUDIO START]`" -> "threshold 300, audio 200 and 290 for 30 s each, assert no `[AUDIO START]`"; evidence "test_audio_below_threshold_ignored pass; make test 2026-10-17: 71 passed" -> "test_audio_below_threshold_ignored[200], test_audio_below_threshold_ignored[290] pass; make test 2026-10-17: 79 passed"
- [x] `ACC-SENSE-10` **Smoothing** - MEDIUM; the audio level is the average of the last 50 samples taken at least 51 ms apart; a 100 ms spike does not start the amplifier
  - evidence: test_spike_ignored, test_spike_right_after_return_to_idle pass; make test 2026-10-17: 79 passed
  - test: threshold 300, audio 1023 for 100 ms, assert no `[AUDIO START]`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj edited evidence "test_spike_ignored, test_spike_ignored_after_return_to_idle pass; make test 2026-10-17: 71 passed" -> "test_spike_ignored, test_spike_right_after_return_to_idle pass; make test 2026-10-17: 79 passed"
  - log: 2026-10-17T16:06:44Z @kj amended text "the audio level is the average of the last 50 samples taken 50 ms apart; a 100 ms spike does not start the amplifier" -> "the audio level is the average of the last 50 samples taken at least 51 ms apart; a 100 ms spike does not start the amplifier"
- [x] `ACC-SENSE-11` **LED with codes saved** - LOW; in idle with IR codes saved the LED blinks slowly, 1000 ms on and 2000 ms off
  - evidence: test_slow_blink_with_codes pass; make test 2026-10-17: 71 passed
  - test: boot with codes, measure LED on and off times
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-SENSE-12` **Button press starts learning** - HIGH; in idle, a button press shorter than 600 ms starts threshold learning
  - evidence: test_press_starts_learning pass; make test 2026-10-17: 71 passed
  - test: press 200 ms, assert `[AUDIOSENSE LEARN]`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-SENSE-13` **Button hold starts recording** - HIGH; in idle, holding the button 600 ms or longer starts IR code recording
  - evidence: test_hold_starts_recording pass; make test 2026-10-17: 71 passed
  - test: hold 1 s, assert `[IRCODE RECORD] Start`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-SENSE-53` **Edge: contact bounce** - HIGH; a press or release that bounces for up to 10 ms counts once; a held button with bounce starts IR recording
  - evidence: test_bouncing_hold_is_one_hold, test_bouncing_press_is_one_press pass; make test 2026-10-17: 71 passed
  - test: hold 1 s with 5 ms bounce, assert one `[IRCODE RECORD] Start` and no learning
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:22:53Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
- [x] `ACC-SENSE-57` **Start on new samples only** - HIGH; after every return to idle, START needs the average of 50 samples taken after the return (2.5 s); samples from before the return never count
  - evidence: test_standby_right_after_music_stops, test_set_audio_0_right_after_music_stops, test_silent_recording_from_audio_on pass; make test 2026-10-17: 79 passed
  - test: audio 600 then 0 and trigger off 0.3 s later, `set audio 0` after silence, silent recording from audio on: assert no `[AUDIO START]`
  - test-tags: FUNCTIONAL
  - mechanism: 2026-10-17T10:10:28Z @kj AudioLevel::restart() in on_audio_sense_enter; starts() is false until 50 new samples; poll() samples in every state
  - log: 2026-10-17T10:10:28Z @kj added
  - log: 2026-10-17T10:41:33Z @kj closed

## IR recording `RECORD`

IR recording: the START and STOP codes learned from the amplifier's remote

- [x] `ACC-RECORD-14` **LED while recording** - LOW; LED blinks 100 ms on, 100 ms off while waiting for START, and 500 ms on, 500 ms off while waiting for STOP
  - evidence: test_led_while_recording pass; make test 2026-10-17: 71 passed
  - test: hold, measure LED; send one code, measure again
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:04Z @kj closed: met in simavr
- [x] `ACC-RECORD-15` **First code is START** - CRITICAL; the first NEC or Sony code received is saved as START in EEPROM bytes 0-5
  - evidence: test_first_code_is_start pass; make test 2026-10-17: 71 passed
  - test: hold, send NEC 0x20DF10EF, assert EEPROM bytes 0-5
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-RECORD-16` **Second code is STOP** - CRITICAL; the second code received is saved as STOP in EEPROM bytes 6-11, then the device returns to idle
  - evidence: test_second_code_is_stop_and_back_to_idle pass; make test 2026-10-17: 71 passed
  - test: send two NEC codes, assert bytes 6-11 and `[AUDIO SENSE] Start`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:23Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-RECORD-17` **Sony codes** - MEDIUM; Sony codes are recorded and later sent like NEC codes
  - evidence: test_sony_codes_recorded_and_sent pass; make test 2026-10-17: 71 passed
  - test: record Sony 12-bit codes, play audio, decode the Sony frame on D3
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-RECORD-18` **Edge: NEC repeat ignored** - HIGH; NEC repeat frames from a held remote button are never saved as a code
  - evidence: test_nec_repeat_not_saved pass; make test 2026-10-17: 71 passed
  - test: send NEC A, repeat frames for 1.5 s, then NEC B; assert STOP is B
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-RECORD-19` **Edge: unsupported protocol** - HIGH; a code of a protocol the device cannot send (not NEC or Sony) is not saved; recording keeps waiting and serial prints that the code was rejected
  - evidence: test_unsupported_protocol_rejected pass; make test 2026-10-17: 71 passed
  - test: send Samsung, then NEC A, NEC B; assert START A, STOP B
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-RECORD-20` **Recording from audio on** - MEDIUM; holding the button while audio is on also starts IR code recording
  - evidence: test_recording_from_audio_on pass; make test 2026-10-17: 71 passed
  - test: audio on, hold 1 s, assert `[IRCODE RECORD] Start`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-RECORD-21` **Codes kept after power cycle** - HIGH; recorded codes are sent after a power cycle
  - evidence: test_codes_kept_after_power_cycle pass; make test 2026-10-17: 71 passed
  - test: record, power on a new board with that EEPROM, play audio, decode START on D3
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr

## Threshold learning `LEARN`

Threshold learning: the audio threshold taken from the playing level

- [x] `ACC-LEARN-22` **Threshold half the playing level** - HIGH; learning takes 50 samples at least 51 ms apart and sets the threshold to half their average
  - evidence: test_threshold_half_the_playing_level, test_learned_on_loud_music_starts_again pass; make test 2026-10-17: 79 passed
  - test: audio 500 while learning, assert threshold 250 +-3; learn on audio 600, then audio 600 starts the amplifier
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:33Z @kj reopened: learning sets half the average: the mean of saturated music cannot be exceeded by the same music (DEF-LEARN-16); evidence retired: test_threshold_from_average pass; make test 2026-10-17: 71 passed
  - log: 2026-10-17T10:41:33Z @kj amended title "Threshold from average" -> "Threshold half the playing level"; text "learning takes 50 samples 50 ms apart and sets the threshold to their average" -> "learning takes 50 samples 50 ms apart and sets the threshold to half their average"
  - log: 2026-10-17T10:41:33Z @kj edited test "audio 500 while learning, assert threshold 500 +-5 in `status`" -> "audio 500 while learning, assert threshold 250 +-3; learn on audio 600, then audio 600 starts the amplifier"
  - log: 2026-10-17T10:41:33Z @kj closed
  - log: 2026-10-17T16:06:44Z @kj amended text "learning takes 50 samples 50 ms apart and sets the threshold to half their average" -> "learning takes 50 samples at least 51 ms apart and sets the threshold to half their average"
- [x] `ACC-LEARN-23` **Minimum threshold 160** - LOW; a learned threshold is never below 160, so its off level (40) stays above the quiet reading, at most 35
  - evidence: test_minimum_threshold[0], test_minimum_threshold[300] pass; make test 2026-10-17: 79 passed
  - test: silence and audio 300 while learning, assert threshold 160
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:33Z @kj reopened: minimum raised so the off level of a learned threshold stays above the quiet reading; evidence retired: test_minimum_threshold_10 pass; make test 2026-10-17: 71 passed
  - log: 2026-10-17T10:41:33Z @kj amended title "Minimum threshold 10" -> "Minimum threshold 160"; text "a learned threshold is never below 10" -> "a learned threshold is never below 160, so its off level (40) stays above the quiet reading, at most 35"
  - log: 2026-10-17T10:41:33Z @kj edited test "silence while learning, assert threshold 10" -> "silence and audio 300 while learning, assert threshold 160"
  - log: 2026-10-17T10:41:33Z @kj closed
- [x] `ACC-LEARN-24` **Threshold saved** - HIGH; the learned threshold is saved in EEPROM bytes 12-13 and used after a power cycle
  - evidence: test_threshold_saved pass; make test 2026-10-17: 71 passed
  - test: learn, assert bytes 12-13, boot a new board with that EEPROM
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-LEARN-25` **LED while learning** - LOW; LED blinks 750 ms on, 250 ms off while learning
  - evidence: test_led_while_learning pass; make test 2026-10-17: 71 passed
  - test: press, measure LED
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-LEARN-26` **Back to idle** - MEDIUM; after learning the device is in idle
  - evidence: test_back_to_idle pass; make test 2026-10-17: 71 passed
  - test: press, assert `[AUDIO SENSE] Start` after the learn exit line
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-LEARN-27` **Learning from audio on** - MEDIUM; a button press while audio is on ends audio on (relay LOW, STOP sent when auto-standby is enabled), then learns
  - evidence: test_learning_from_audio_on pass; make test 2026-10-17: 71 passed
  - test: audio on with auto-standby, press, assert D4 LOW, STOP decoded, learning starts
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
  - log: 2026-10-17T10:43:32Z @kj amended text "a button press while audio is on leaves audio on (relay LOW, STOP sent when auto-standby is enabled), then learns" -> "a button press while audio is on ends audio on (relay LOW, STOP sent when auto-standby is enabled), then learns"
- [-] `ACC-LEARN-55` **Sigma learned and saved** - HIGH; learning sets sigma to the standard deviation of the same 50 samples, saves it in EEPROM bytes 14-15 and uses it after a power cycle; `status` shows it
  - test: learn on a 400/600 square wave, assert sigma 95-101 in `status` and bytes 14-15, boot a new board with that EEPROM
  - test-tags: FUNCTIONAL
  - mechanism: 2026-10-17T10:10:28Z @kj AudioLevel::poll() sums the samples and their squares while learning; sigma = sqrt(n*squares - sum^2) / n
  - log: 2026-10-17T10:10:28Z @kj added
  - log: 2026-10-17T10:41:33Z @kj rejected: superseded by ACC-ENABLE-54: music saturates the front end, so the learn samples spread 0 to 7 counts and a 1-sigma band changes nothing (notebooks/audio-level-simulation.ipynb)
- [-] `ACC-LEARN-56` **Default sigma** - MEDIUM; while EEPROM bytes 14-15 are erased, sigma is 10% of the threshold; `set threshold` keeps a learned sigma
  - test: erased bytes 14-15, threshold 300: `status` shows sigma 30; set threshold 500: sigma 50; after a learn, set threshold keeps the learned sigma
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T10:10:28Z @kj added
  - log: 2026-10-17T10:41:33Z @kj rejected: superseded by ACC-ENABLE-54: no sigma is kept, the off level is 25% of the threshold

## Audio start `START`

Audio start: the amplifier switched on

- [x] `ACC-START-28` **START code sent once** - CRITICAL; entering audio start sends the START code once
  - evidence: test_start_code_sent_once pass; make test 2026-10-17: 71 passed
  - test: audio on, decode D3, assert one frame equal to START
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-START-29` **Trigger mode** - HIGH; if the 12V trigger turns on (D7 LOW) within 3 s of START, audio on follows the trigger
  - evidence: test_trigger_mode pass; make test 2026-10-17: 71 passed
  - test: audio, trigger on after 1 s, assert `Audio trigger detected`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:05Z @kj closed: met in simavr
- [x] `ACC-START-30` **Timeout mode** - HIGH; without the 12V trigger within 3 s, audio on uses the 10 min silence timeout
  - evidence: test_timeout_mode pass; make test 2026-10-17: 71 passed
  - test: audio, no trigger, assert `Trigger not available` after 3 s
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr

## Audio on `ENABLE`

Audio on: relay on, waiting for standby

- [x] `ACC-ENABLE-31` **Relay on** - CRITICAL; relay output D4 is HIGH while audio is on
  - evidence: test_relay_on pass; make test 2026-10-17: 71 passed
  - test: audio on, assert D4 HIGH
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:24Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-ENABLE-32` **LED on** - LOW; the LED is on without blinking while audio is on
  - evidence: test_led_on pass; make test 2026-10-17: 71 passed
  - test: audio on, assert LED HIGH and no LED edges for 5 s
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-ENABLE-33` **Trigger off turns audio off** - HIGH; in trigger mode, the trigger turning off (D7 HIGH) turns audio off: relay LOW, back to idle
  - evidence: test_trigger_off_turns_audio_off pass; make test 2026-10-17: 71 passed
  - test: trigger mode, trigger off, assert D4 LOW and `[AUDIO SENSE] Start`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-ENABLE-34` **Trigger mode has no timeout** - HIGH; in trigger mode audio stays on while the trigger stays on, also after 10 min
  - evidence: test_trigger_mode_has_no_timeout pass; make test 2026-10-17: 71 passed
  - test: trigger mode, audio playing, 15 min, assert D4 HIGH and no IR frame sent
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-ENABLE-35` **Silence timeout** - HIGH; in timeout mode, 10 min with no level above the off level turns audio off: relay LOW, back to idle
  - evidence: test_silence_timeout, test_level_below_off_level_turns_audio_off pass; make test 2026-10-17: 79 passed
  - test: timeout mode, silence, 11 min, assert D4 LOW and `[AUDIO SENSE] Start`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:13:17Z @kj amended text "in timeout mode, 10 min with no audio above the threshold turns audio off: relay LOW, LED off, back to idle" -> "in timeout mode, 10 min with no audio above the threshold turns audio off: relay LOW, back to idle"
  - log: 2026-10-17T09:13:17Z @kj edited test "timeout mode, silence, 11 min, assert D4 LOW" -> "timeout mode, silence, 11 min, assert D4 LOW and `[AUDIO SENSE] Start`"
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
  - log: 2026-10-17T10:10:28Z @kj reopened: reopened: hysteresis, the off level replaces the threshold; evidence retired: test_silence_timeout pass; make test 2026-10-17: 71 passed
  - log: 2026-10-17T10:10:28Z @kj amended text "in timeout mode, 10 min with no audio above the threshold turns audio off: relay LOW, back to idle" -> "in timeout mode, 10 min with no level above the off level turns audio off: relay LOW, back to idle"
  - log: 2026-10-17T10:41:33Z @kj closed
- [x] `ACC-ENABLE-36` **Audio keeps audio on** - HIGH; in timeout mode, a level above the off level restarts the 10 min timeout, checked at most once per minute
  - evidence: test_audio_keeps_audio_on, test_quiet_music_keeps_audio_on pass, no relay edge in 15 min; make test 2026-10-17: 79 passed
  - test: timeout mode, level between off level and threshold, 15 min, assert D4 HIGH with no relay edge
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
  - log: 2026-10-17T10:10:28Z @kj reopened: reopened: hysteresis, the off level replaces the threshold; evidence retired: test_audio_keeps_audio_on pass; make test 2026-10-17: 71 passed
  - log: 2026-10-17T10:10:28Z @kj amended text "in timeout mode, audio above the threshold restarts the 10 min timeout, checked at most once per minute" -> "in timeout mode, a level above the off level restarts the 10 min timeout, checked at most once per minute"
  - log: 2026-10-17T10:10:28Z @kj edited test "timeout mode, audio playing, 15 min, assert D4 HIGH" -> "timeout mode, level between off level and threshold, 15 min, assert D4 HIGH with no relay edge"
  - log: 2026-10-17T10:41:33Z @kj closed
  - log: 2026-10-17T10:41:34Z @kj test_audio_keeps_audio_on asserts no relay edge; fails when audio does not restart the timeout (review round 1, mutant M1)
- [x] `ACC-ENABLE-37` **STOP with auto-standby** - HIGH; with auto-standby enabled, leaving audio on sends the STOP code once
  - evidence: test_stop_code_on_standby pass; make test 2026-10-17: 71 passed
  - test: auto-standby, timeout, decode one STOP frame
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-ENABLE-38` **No STOP without auto-standby** - HIGH; with auto-standby disabled, leaving audio on sends no IR code
  - evidence: test_stop_code_on_standby pass; make test 2026-10-17: 71 passed
  - test: no auto-standby, timeout, assert no IR frame
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-ENABLE-54` **Off level** - HIGH; the off level is 25% of the threshold; only audio on uses it, START still needs a level above the threshold
  - evidence: test_quiet_music_keeps_audio_on, test_level_below_off_level_turns_audio_off, test_threshold_below_minimum_restored_as_minimum, test_audio_below_threshold_ignored[290] pass; make test 2026-10-17: 81 passed
  - test: threshold 300: level 150 keeps audio on 15 min with no relay edge, level 60 turns it off after 10 min
  - test-tags: FUNCTIONAL
  - mechanism: 2026-10-17T10:41:32Z @kj AudioLevel::offLevel() returns threshold / 4; on_audio_enabled_loop asks holds(), on_audio_sense_loop asks starts()
  - mechanism: 2026-10-17T10:10:28Z @kj AudioLevel::offLevel(); on_audio_enabled_loop asks holds(), on_audio_sense_loop asks starts()
  - log: 2026-10-17T10:10:28Z @kj added
  - log: 2026-10-17T10:41:32Z @kj amended text "the off level is the threshold minus sigma, at least 10 and never above the threshold; only audio on uses it, START still needs a level above the threshold" -> "the off level is 25% of the threshold; only audio on uses it, START still needs a level above the threshold"
  - log: 2026-10-17T10:41:32Z @kj edited test "threshold 50 with sigma 100: level 20 keeps audio on 15 min, level 5 turns it off" -> "threshold 300: level 150 keeps audio on 15 min with no relay edge, level 60 turns it off after 10 min"
  - log: 2026-10-17T10:41:33Z @kj closed
  - log: 2026-10-17T13:46:25Z @kj edited evidence "test_quiet_music_keeps_audio_on, test_level_below_off_level_turns_audio_off, test_audio_below_threshold_ignored[290] pass; make test 2026-10-17: 79 passed" -> "test_quiet_music_keeps_audio_on, test_level_below_off_level_turns_audio_off, test_threshold_below_minimum_restored_as_minimum, test_audio_below_threshold_ignored[290] pass; make test 2026-10-17: 81 passed"

## Serial commands `SERIAL`

Serial command line at 9600 baud

- [x] `ACC-SERIAL-39` **help** - LOW; `help` lists the commands
  - evidence: test_help pass; make test 2026-10-17: 90 passed
  - test: send `help`, assert the header and the seven command lines
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
  - log: 2026-10-17T16:06:44Z @kj edited test "send `help`, assert `[HELP] available commands`" -> "send `help`, assert the header and the five command lines"; evidence "test_help pass; make test 2026-10-17: 71 passed" -> "test_help pass; make test 2026-10-17: 81 passed"
  - log: 2026-10-17T16:26:14Z @kj edited test "send `help`, assert the header and the five command lines" -> "send `help`, assert the header and the seven command lines"; evidence "test_help pass; make test 2026-10-17: 81 passed" -> "test_help pass; make test 2026-10-17: 90 passed"
- [x] `ACC-SERIAL-40` **status** - MEDIUM; `status` prints threshold, off level, audio level, audio on or off, auto-standby, 12V trigger and debug plot
  - evidence: test_status pass; make test 2026-10-17: 79 passed
  - test: send `status`, assert the seven lines
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj amended text "`status` prints threshold, audio level, audio on or off, auto-standby, 12V trigger and debug plot" -> "`status` prints threshold, off level, audio level, audio on or off, auto-standby, 12V trigger and debug plot"
  - log: 2026-10-17T10:41:34Z @kj edited test "send `status`, assert the six lines" -> "send `status`, assert the seven lines"; evidence "test_status pass; make test 2026-10-17: 71 passed" -> "test_status pass; make test 2026-10-17: 79 passed"
- [x] `ACC-SERIAL-41` **set threshold** - MEDIUM; `set threshold <n>` with n from 160 to 1023 sets the threshold and saves it in EEPROM bytes 12-13
  - evidence: test_set_threshold[160], test_set_threshold[1023] pass; make test 2026-10-17: 81 passed
  - test: send `set threshold` 160 and 1023, assert status, bytes 12-13 and the threshold after a power cycle
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj amended text "`set threshold <n>` with n above 0 sets the threshold and saves it in EEPROM bytes 12-13" -> "`set threshold <n>` with n from 1 to 1023 sets the threshold and saves it in EEPROM bytes 12-13"
  - log: 2026-10-17T10:41:34Z @kj edited evidence "test_set_threshold pass; make test 2026-10-17: 71 passed" -> "test_set_threshold pass; make test 2026-10-17: 79 passed"
  - log: 2026-10-17T13:46:25Z @kj amended text "`set threshold <n>` with n from 1 to 1023 sets the threshold and saves it in EEPROM bytes 12-13" -> "`set threshold <n>` with n from 160 to 1023 sets the threshold and saves it in EEPROM bytes 12-13"
  - log: 2026-10-17T13:46:25Z @kj edited test "send `set threshold 450`, assert status and bytes 12-13" -> "send `set threshold` 160 and 1023, assert status, bytes 12-13 and the threshold after a power cycle"; evidence "test_set_threshold pass; make test 2026-10-17: 79 passed" -> "test_set_threshold[160], test_set_threshold[1023] pass; make test 2026-10-17: 81 passed"
- [x] `ACC-SERIAL-42` **Edge: invalid threshold** - LOW; `set threshold` with a value below 160, a value above 1023 or a value that is not a number leaves the threshold unchanged
  - evidence: test_invalid_threshold_ignored pass; make test 2026-10-17: 81 passed
  - test: send `set threshold` 0, abc, 159, 1024 and -5, assert threshold unchanged
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj amended text "`set threshold 0` or a value that is not a number leaves the threshold unchanged" -> "`set threshold` with 0, a negative number, a value above 1023 or a value that is not a number leaves the threshold unchanged"
  - log: 2026-10-17T10:41:34Z @kj edited test "send `set threshold 0` and `set threshold abc`, assert threshold unchanged" -> "send `set threshold` 0, abc, 1024 and -5, assert threshold unchanged"; evidence "test_invalid_threshold_ignored pass; make test 2026-10-17: 71 passed" -> "test_invalid_threshold_ignored pass; make test 2026-10-17: 79 passed"
  - log: 2026-10-17T13:46:25Z @kj amended text "`set threshold` with 0, a negative number, a value above 1023 or a value that is not a number leaves the threshold unchanged" -> "`set threshold` with a value below 160, a value above 1023 or a value that is not a number leaves the threshold unchanged"
  - log: 2026-10-17T13:46:25Z @kj edited test "send `set threshold` 0, abc, 1024 and -5, assert threshold unchanged" -> "send `set threshold` 0, abc, 159, 1024 and -5, assert threshold unchanged"; evidence "test_invalid_threshold_ignored pass; make test 2026-10-17: 79 passed" -> "test_invalid_threshold_ignored pass; make test 2026-10-17: 81 passed"
- [x] `ACC-SERIAL-43` **set audio 1** - MEDIUM; `set audio 1` in idle moves to audio start, as detected audio does
  - evidence: test_set_audio_1 pass; make test 2026-10-17: 71 passed
  - test: idle, send `set audio 1`, assert START frame on D3
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-SERIAL-44` **set audio 0** - MEDIUM; `set audio 0` while audio is on turns audio off
  - evidence: test_set_audio_0 pass; make test 2026-10-17: 71 passed
  - test: audio on, send `set audio 0`, assert D4 LOW
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:06Z @kj closed: met in simavr
- [x] `ACC-SERIAL-45` **set debug_plot** - LOW; `set debug_plot 1` prints an `[ADC]` line per audio sample with the sample, average, threshold and off level; `set debug_plot 0` stops them
  - evidence: test_debug_plot pass; make test 2026-10-17: 79 passed
  - test: send each, count `[ADC]` lines over 1 s
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj amended text "`set debug_plot 1` prints an `[ADC]` line per audio sample; `set debug_plot 0` stops them" -> "`set debug_plot 1` prints an `[ADC]` line per audio sample with the sample, average, threshold and off level; `set debug_plot 0` stops them"
  - log: 2026-10-17T10:41:34Z @kj edited evidence "test_debug_plot pass; make test 2026-10-17: 71 passed" -> "test_debug_plot pass; make test 2026-10-17: 79 passed"
- [x] `ACC-SERIAL-46` **reboot** - LOW; `reboot` restarts the firmware and the boot lines print again
  - evidence: test_reboot pass; make test 2026-10-17: 71 passed
  - test: send `reboot`, assert a second `[INIT] Restoring` line
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:25Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
- [x] `ACC-SERIAL-47` **Edge: empty line** - LOW; an empty line runs no command
  - evidence: test_empty_line_runs_nothing pass; make test 2026-10-17: 71 passed
  - test: send `help`, then an empty line, assert one `[HELP]` line
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:26Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
- [x] `ACC-SERIAL-48` **Commands do not stop the device** - MEDIUM; a command runs within 100 ms of its line end, and the device keeps running while a command is typed
  - evidence: test_reply_within_100_ms, test_device_runs_while_command_is_typed pass; make test 2026-10-17: 71 passed
  - test: send `stat` with no line end, audio 600, assert START within 5 s; send `status` and newline, reply within 100 ms
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:26Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
  - log: 2026-10-17T10:43:33Z @kj edited test "send `status` with no line end, LED keeps blinking; send newline, reply within 100 ms" -> "send `stat` with no line end, audio 600, assert START within 5 s; send `status` and newline, reply within 100 ms"
- [x] `ACC-SERIAL-49` **Line ending** - LOW; a command runs when its line ends with `\n` or `\r`
  - evidence: test_carriage_return_ends_line pass; make test 2026-10-17: 71 passed
  - test: send `help` ending in `\r`, assert `[HELP]`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:26Z @kj added
  - log: 2026-10-17T09:09:43Z @kj amended text "a command runs when its line ends with `\\n` or `\\r`" -> "a command runs when its line ends with `\n` or `\r`"
  - log: 2026-10-17T09:09:43Z @kj edited test "send `help\\r`, assert `[HELP]`" -> "send `help` ending in `\r`, assert `[HELP]`"
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
- [x] `ACC-SERIAL-52` **Edge: long line** - LOW; bytes of a command line past 64 are dropped; the device keeps running and the next command works
  - evidence: test_long_line_is_cut_and_device_keeps_running pass; make test 2026-10-17: 71 passed
  - test: send 300 bytes with no line end, then `help`, assert `[HELP]`
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:20:07Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr

## Timing `TIMING`

Behaviour when millis() wraps past 2^32 after 49.7 days of uptime

- [x] `ACC-TIMING-50` **Edge: millis wrap** - MEDIUM; audio sampling, button timing and the silence timeout keep working when millis() wraps past 2^32
  - evidence: test_audio_detected_across_wrap, test_button_press_across_wrap, test_silence_timeout_across_wrap pass; make test 2026-10-17: 79 passed
  - test: set millis() to 2^32-51 with audio 600, assert detected within 2 s; silence timeout started 5 min before the wrap ends 10 min later
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T09:09:26Z @kj added
  - log: 2026-10-17T09:34:07Z @kj closed: met in simavr
  - log: 2026-10-17T10:41:34Z @kj edited test "set millis() and the sample time just before the wrap, run across it, assert audio detected" -> "set millis() to 2^32-51 with audio 600, assert detected within 2 s; silence timeout started 5 min before the wrap ends 10 min later"; evidence "test_audio_sampling_across_wrap, test_button_press_across_wrap, test_silence_timeout_across_wrap pass; make test 2026-10-17: 71 passed" -> "test_audio_detected_across_wrap, test_button_press_across_wrap, test_silence_timeout_across_wrap pass; make test 2026-10-17: 79 passed"
  - log: 2026-10-17T10:41:34Z @kj test_audio_detected_across_wrap fails on the wrap-unsafe sample timer and on the settling check; test_silence_timeout_across_wrap fails on a wrap-unsafe Fsm timer (review round 1, mutant M8)
  - log: 2026-10-17T10:43:33Z @kj amended text "audio sampling, learning, button timing and the silence timeout keep working when millis() wraps past 2^32" -> "audio sampling, button timing and the silence timeout keep working when millis() wraps past 2^32"

## EEPROM backup and restore `EEPROM`

Serial download and upload of the stored settings and the raw EEPROM image

- [x] `ACC-EEPROM-58` **Download a block** - HIGH; `eeprom dump <addr>` returns 16 bytes at `addr` as 32 hex characters prefixed by the address, for an address from 0 to 1008
  - evidence: test_eeprom_dump_first_block, test_eeprom_dump_last_block pass; make test 2026-10-17: 90 passed
  - test: boot with a known EEPROM image, send `eeprom dump 0`, assert the line equals the image bytes
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T16:22:17Z @kj added
  - log: 2026-10-17T16:26:13Z @kj closed: verified: eeprom dump returns each 16-byte block
- [x] `ACC-EEPROM-59` **Upload a block** - HIGH; `eeprom write <addr> <hex>` writes 1 to 16 bytes at `addr`, confirms the count, and leaves every other byte unchanged
  - evidence: test_eeprom_write_block passes; make test 2026-10-17: 90 passed
  - test: send `eeprom write 12 2C01`, assert bytes 12-13 and the reply; assert bytes 14-15 unchanged
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T16:22:17Z @kj added
  - log: 2026-10-17T16:26:14Z @kj closed: verified: eeprom write writes the given bytes
- [x] `ACC-EEPROM-60` **Reject invalid writes** - MEDIUM; a write with an address outside 0 to 1023, an odd number of hex characters, more than 16 bytes, or a non-hex character writes nothing and prints no confirmation
  - evidence: test_eeprom_write_rejects_invalid passes; make test 2026-10-17: 90 passed
  - test: send each invalid form, assert the EEPROM image is unchanged
  - test-tags: FUNCTIONAL
  - log: 2026-10-17T16:22:17Z @kj added
  - log: 2026-10-17T16:26:14Z @kj closed: verified: invalid writes change nothing
- [x] `ACC-EEPROM-61` **JSON backup** - HIGH; the host tool writes a JSON file with the START and STOP codes (type, bits, value), the threshold and the full raw image as hex; the JSON decodes back to the same 1024 bytes
  - evidence: test_decode_image, test_encode_image_round_trip pass; make test 2026-10-17: 90 passed
  - test: unit test encode and decode round trip on a known image
  - test-tags: UNIT
  - log: 2026-10-17T16:22:17Z @kj added
  - log: 2026-10-17T16:26:14Z @kj closed: verified: JSON decode and encode round trip
- [x] `ACC-EEPROM-62` **Restore from JSON preserves the rest** - HIGH; restoring a JSON file writes only bytes 0 to 13 and leaves bytes 14 to 1023 as they were
  - evidence: test_overlay_preserves_rest passes; make test 2026-10-17: 90 passed
  - test: unit test overlay on a known current image
  - test-tags: UNIT
  - log: 2026-10-17T16:22:17Z @kj added
  - log: 2026-10-17T16:26:14Z @kj closed: verified: JSON restore keeps bytes 14-1023
- [ ] `ACC-EEPROM-63` **Agent command** - MEDIUM; `make eeprom-backup PORT=<dev>` and `make eeprom-restore PORT=<dev> FILE=<file>` run without prompts, write the files and print one JSON line with the paths and decoded settings; restore requires `--yes`
  - test: run `make eeprom-backup` against a board with the new firmware, assert the three files and the JSON line
  - test-tags: MANUAL
  - log: 2026-10-17T16:22:17Z @kj added
  - log: 2026-10-17T16:26:14Z @kj unit Intel HEX round trip passes; live restore waits for the board
  - log: 2026-10-17T16:26:24Z @kj correction: the previous line belongs to ACC-EEPROM-64 (raw backup and restore)
- [ ] `ACC-EEPROM-64` **Raw backup and restore** - HIGH; the host tool writes a 1024-byte `.bin` and an Intel HEX `.hex`; restoring either writes all 1024 bytes
  - test: unit test Intel HEX round trip; live restore is manual
  - test-tags: UNIT, MANUAL
  - log: 2026-10-17T16:22:41Z @kj added
  - log: 2026-10-17T16:26:14Z @kj host tool decode and dry-run verified offline; live backup waits for the board
  - log: 2026-10-17T16:26:24Z @kj correction: the previous line belongs to ACC-EEPROM-63 (agent command)

