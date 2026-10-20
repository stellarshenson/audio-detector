#!/usr/bin/env -S PYTHONDONTWRITEBYTECODE=1 uv run --quiet --script
## Firmware toolchain for AVR (Arduino, ATtiny) - nightly Rust, avr-gcc, avrdude, PlatformIO with the Arduino AVR core, simavr, pytest
# /// script
# requires-python = ">=3.12"
# dependencies = ["rich>=13"]
# ///
"""Install the conda environment firmware-avr: nightly Rust with its source, the AVR GNU toolchain
that links the firmware, avrdude for upload through a serial bootloader or a programmer, and for
Arduino C/C++ sketches PlatformIO with the atmelavr platform and the Arduino AVR core, and simavr
with pytest for tests that run the firmware on a simulated microcontroller."""

import shutil

from firmware_env import Stage, install

NIGHTLY = "nightly-2026-10-06"  # AVR builds need nightly Rust; this date is tested with atmega328p
AVR_GCC = "7.3.0-atmel3.6.1-arduino7"  # with avr-binutils and avr-libc, as the Arduino IDE installs it
AVRDUDE = "8.0-arduino.1"
PLATFORMIO = "6.2.0"
ATMELAVR = "5.3.0"  # PlatformIO platform; a project pins the same version in platformio.ini
PYTEST = "9.1.1"


def conda_packages(env):
    env.create("pyserial")


def rust(env):
    env.rustup(NIGHTLY, "--component", "rust-src,clippy,rustfmt")  # rust-src: the AVR library is built from source


def avr_gcc(env):
    (archive,) = env.fetch(f"https://downloads.arduino.cc/tools/avr-gcc-{AVR_GCC}-x86_64-pc-linux-gnu.tar.bz2")
    env.ui.tail("unpacking avr-gcc")
    shutil.unpack_archive(archive, env.tmp, filter="data")
    shutil.copytree(env.tmp / "avr", env.prefix / "avr-gcc", symlinks=True, dirs_exist_ok=True)
    env.link(env.prefix / "avr-gcc/bin")


def avrdude(env):
    (archive,) = env.fetch(f"https://downloads.arduino.cc/tools/avrdude_{AVRDUDE}_Linux_64bit.tar.gz")
    shutil.unpack_archive(archive, env.tmp, filter="data")
    shutil.copytree(env.tmp / "avrdude_Linux_64bit", env.prefix / "avrdude", dirs_exist_ok=True)
    env.link(env.prefix / "avrdude/bin")


def platformio(env):
    """PlatformIO keeps the platform, its toolchain, the Arduino core and simavr (program, library and
    headers) in <prefix>/platformio."""
    env.platformio(PLATFORMIO, f"atmelavr@{ATMELAVR}", "nanoatmega328", "tool-avrdude", "tool-simavr")
    env.run(env.prefix / "bin/python", "-m", "pip", "install", f"pytest=={PYTEST}")


def circuit(env):
    """ngspice runs the netlist of the opt-in ltspice component in batch mode."""
    env.circuit_simulator()


install(
    name="firmware-avr",
    summary="Firmware toolchain for AVR",
    variables={"PLATFORMIO_CORE_DIR": "{prefix}/platformio"},
    stages=[
        Stage("C compiler, pyserial", "conda-forge", conda_packages),
        Stage(f"Rust {NIGHTLY} with rust-src", "static.rust-lang.org", rust),
        Stage(f"avr-gcc {AVR_GCC}", "downloads.arduino.cc", avr_gcc),
        Stage(f"avrdude {AVRDUDE}", "downloads.arduino.cc", avrdude),
        Stage(f"PlatformIO {PLATFORMIO}, atmelavr {ATMELAVR}, simavr, pytest {PYTEST}", "registry.platformio.org", platformio),
        Stage("ngspice for the circuit simulation", "conda-forge", circuit),
    ],
    tools=[
        ("cargo", ["cargo", "--version"]),
        ("rustc", ["rustc", "--version"]),
        ("avr-gcc", ["avr-gcc", "--version"]),
        ("avrdude", ["avrdude", "-v"]),
        ("pio", ["pio", "--version"]),
        ("PlatformIO platform", f"atmelavr {ATMELAVR} with framework-arduino-avr and simavr"),
        ("pytest", ["pytest", "--version"]),
        ("ngspice", ["ngspice", "--version"]),
        ("target", "avr-none"),
    ],
    usage=[
        "conda activate firmware-avr",
        'RUSTFLAGS="-C target-cpu=atmega328p" cargo build --release -Z build-std=core --target avr-none',
        "avrdude -c arduino -p atmega328p -P <serial device> -U flash:w:<firmware>.elf",
        "pio run --target upload --upload-port <serial device>    (in a PlatformIO project)",
        "ngspice -b simulations/circuit.cir    (the opt-in ltspice component runs the surrounding circuit)",
    ],
)
