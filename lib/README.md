# lib


The project libraries. PlatformIO compiles every folder under `lib/`.


## Library register

| Library | Version | Notes |
|---|---|---|
| `vendor/IRremote` | 2.6.1 | IR code record and send; the 2.x API differs from 3.x |
| `vendor/arduino-fsm` | Stellars Henson fork | `https://github.com/stellarshenson/arduino-fsm`, the fork with timed-transition reset |
| `vendor/JLed` | 4.5.2 | non-blocking LED blinking patterns |
| `vendor/EnableInterrupt` | 1.1.0 | button interrupt driver |
| `vendor/Smoothed` | 1.2.0 | carries local fixes: `clear()` return value, array initialisation, sign-compare warnings |

Pin a library version in this table and keep the full library in the repository, so the firmware
builds alone.
