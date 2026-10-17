"""One simulated audio detector: firmware.elf on simavr, with the board's pins named

The tests drive the inputs a real installation drives (RCA audio level, 12V trigger, button,
IR receiver, auto-standby jumper, serial console) and read the outputs (IR LED, relay, status
LED, serial console).
"""

import ctypes
import subprocess
from pathlib import Path

import ir
from eeprom import EEPROM_SIZE

ROOT = Path(__file__).resolve().parents[2]
FIRMWARE = ROOT / ".pio/build/nanoatmega328/firmware.elf"
LIBRARY = ROOT / ".pio/simharness/libsimharness.so"

# Arduino Nano pin -> (AVR port, bit)
PINS = {
    "ir_in": ("D", 2),  # IR receiver output, LOW during a mark
    "relay": ("D", 4),  # audio-enabled output
    "autostandby": ("D", 6),  # jumper to GND enables auto-standby
    "trigger": ("D", 7),  # optocoupler pulls LOW while the amplifier's 12V trigger is on
    "button": ("B", 0),  # pulls LOW while pressed
    "led": ("B", 1),  # status LED
}
CARRIER = "ir_out"  # IR LED (D3): 38 kHz carrier on while Timer2 drives OC2B, see simharness.c
CARRIER_PIN = 24
AUDIO_ADC_CHANNEL = 1  # A1
# contact bounce of the push button: (level, microseconds) before the level settles, about 5 ms
PRESS_BOUNCE = [(0, 300), (1, 200), (0, 700), (1, 400), (0, 1100), (1, 300), (0, 2000)]
RELEASE_BOUNCE = [(1, 400), (0, 300), (1, 900), (0, 200), (1, 1300), (0, 400), (1, 1500)]
CPU_DONE, CPU_CRASHED = 6, 7  # simavr cpu states: the firmware stopped or crashed


class Edge(ctypes.Structure):
    _fields_ = [("t_us", ctypes.c_uint64), ("pin", ctypes.c_uint8), ("level", ctypes.c_uint8)]


_lib = ctypes.CDLL(str(LIBRARY))
_lib.sim_new.restype = ctypes.c_void_p
_lib.sim_new.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
_lib.sim_free.argtypes = [ctypes.c_void_p]
_lib.sim_run_us.argtypes = [ctypes.c_void_p, ctypes.c_uint64]
_lib.sim_now_us.restype = ctypes.c_uint64
_lib.sim_now_us.argtypes = [ctypes.c_void_p]
_lib.sim_set_input.argtypes = [ctypes.c_void_p, ctypes.c_char, ctypes.c_int, ctypes.c_int]
_lib.sim_set_adc_mv.argtypes = [ctypes.c_void_p, ctypes.c_int, ctypes.c_int]
_lib.sim_level.argtypes = [ctypes.c_void_p, ctypes.c_char, ctypes.c_int]
_lib.sim_take_edges.restype = ctypes.c_size_t
_lib.sim_take_edges.argtypes = [ctypes.c_void_p, ctypes.POINTER(Edge), ctypes.c_size_t]
_lib.sim_take_uart.restype = ctypes.c_size_t
_lib.sim_take_uart.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
_lib.sim_send_uart.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_size_t]
_lib.sim_read_eeprom.argtypes = [ctypes.c_void_p, ctypes.c_char_p]
_lib.sim_add_u32.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint32]
_lib.sim_write_u32.argtypes = [ctypes.c_void_p, ctypes.c_uint16, ctypes.c_uint32]
_lib.sim_read_u32.restype = ctypes.c_uint32
_lib.sim_read_u32.argtypes = [ctypes.c_void_p, ctypes.c_uint16]

_symbols = {}


def symbol(name):
    """data-space address of a firmware variable, from the ELF symbol table"""
    if not _symbols:
        out = subprocess.run(["avr-nm", "-C", str(FIRMWARE)], capture_output=True, text=True, check=True).stdout
        for line in out.splitlines():
            parts = line.split(maxsplit=2)
            if len(parts) == 3:
                _symbols[parts[2]] = int(parts[0], 16) & 0xFFFF
    return _symbols[name]


class Board:
    def __init__(self, eeprom=None, autostandby=False, firmware=FIRMWARE):
        self._sim = _lib.sim_new(str(firmware).encode(), eeprom)
        assert self._sim, f"simavr could not load {firmware}"
        self.serial = ""
        self.edges = {name: [] for name in [*PINS, CARRIER]}
        self._buffer = (Edge * 65536)()
        self.set_input("ir_in", 1)
        self.set_input("trigger", 1)
        self.set_input("button", 1)
        self.set_input("autostandby", 0 if autostandby else 1)
        self.set_audio(0)

    def close(self):
        _lib.sim_free(self._sim)

    @property
    def now(self):
        """simulated seconds since power-on"""
        return _lib.sim_now_us(self._sim) / 1e6

    def set_input(self, name, level):
        port, bit = PINS[name]
        _lib.sim_set_input(self._sim, port.encode(), bit, level)

    def set_audio(self, adc):
        """RCA input level as the ADC reading 0..1023 it produces against the 5 V AVCC reference"""
        _lib.sim_set_adc_mv(self._sim, AUDIO_ADC_CHANNEL, adc * 5000 // 1024)

    def set_trigger(self, on):
        self.set_input("trigger", 0 if on else 1)

    def level(self, name):
        port, bit = PINS[name]
        return _lib.sim_level(self._sim, port.encode(), bit)

    def run(self, seconds):
        state = _lib.sim_run_us(self._sim, int(seconds * 1e6))
        assert state not in (CPU_DONE, CPU_CRASHED), "firmware stopped"
        self._collect()

    def _collect(self):
        index = {"B": 0, "C": 1, "D": 2}
        names = {index[port] * 8 + bit: name for name, (port, bit) in PINS.items()} | {CARRIER_PIN: CARRIER}
        while n := _lib.sim_take_edges(self._sim, self._buffer, len(self._buffer)):
            for edge in self._buffer[:n]:
                if edge.pin in names:
                    self.edges[names[edge.pin]].append((edge.t_us, edge.level))
        chunk = ctypes.create_string_buffer(65536)
        while n := _lib.sim_take_uart(self._sim, chunk, len(chunk)):
            self.serial += chunk.raw[:n].decode("latin-1")

    def wait_for(self, text, timeout, start=0):
        """runs until text appears in the serial output after position start; returns its position"""
        deadline = self.now + timeout
        while (found := self.serial.find(text, start)) < 0:
            assert self.now < deadline, f"no {text!r} within {timeout} s; serial tail:\n{self.serial[-600:]}"
            self.run(0.01)
        return found

    def elapse(self, seconds, step=20):
        """moves firmware time forward by seconds: millis() jumps in steps, with 0.2 s of real
        running after each jump, so periodic checks in the firmware still run between jumps"""
        while seconds > 0:
            jump = min(step, seconds)
            _lib.sim_add_u32(self._sim, symbol("timer0_millis"), int(jump * 1000))
            self.run(0.2)
            seconds -= jump

    def press(self, seconds):
        """holds the button for seconds; press and release both bounce as a real contact does"""
        self._bounce(PRESS_BOUNCE)
        self.run(seconds)
        self._bounce(RELEASE_BOUNCE)
        self.run(0.1)

    def _bounce(self, steps):
        for level, duration in steps:
            self.set_input("button", level)
            self.run(duration / 1e6)

    def send_ir(self, frame):
        for level, duration in ir.receiver_segments(frame):
            self.set_input("ir_in", level)
            self.run(duration / 1e6)

    def write(self, text):
        data = text.encode()
        _lib.sim_send_uart(self._sim, data, len(data))

    def command(self, line, reply, timeout=2):
        """sends one command line and waits for reply in the output that follows"""
        start = len(self.serial)
        self.write(line + "\n")
        return self.wait_for(reply, timeout, start)

    def eeprom(self):
        image = ctypes.create_string_buffer(EEPROM_SIZE)
        _lib.sim_read_eeprom(self._sim, image)
        return image.raw

    def ir_sent(self, since=0.0):
        """IR frames the board sent after simulated second since, as (protocol, value, bits)"""
        return ir.decode_edges([e for e in self.edges[CARRIER] if e[0] >= since * 1e6])

    def write_u32(self, name, value):
        """sets a 32-bit firmware variable, applied once interrupts are enabled"""
        _lib.sim_write_u32(self._sim, symbol(name), value)

    def read_u32(self, name):
        return _lib.sim_read_u32(self._sim, symbol(name))
