# firmware-avr toolchain

The stages of the project's installer and the programs each one leaves in the environment. Versions
are pins at the top of `resources/envs/firmware-avr.py`; read them there, not here.

| Stage | Source | Leaves |
|---|---|---|
| C compiler, pyserial | conda-forge | the host compiler, `python`, `pyserial` |
| Rust with rust-src | static.rust-lang.org | `cargo`, `rustc`, the Rust source for the AVR core |
| avr-gcc | downloads.arduino.cc | `avr-gcc`, `avr-objcopy`, `avr-size` |
| avrdude | downloads.arduino.cc | `avrdude` |
| PlatformIO with atmelavr | registry.platformio.org | `pio`, the AVR toolchain, the Arduino AVR core, `tool-simavr`, `pytest` |
| ngspice | conda-forge | `ngspice`, unused until the project adopts the ltspice component |

Checks: `cargo --version`, `avr-gcc --version`, `avrdude -v`, `pio --version`, `pytest --version`,
`ngspice --version`.
