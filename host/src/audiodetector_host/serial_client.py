"""The serial console of the board: the EEPROM read and written 16 bytes at a time.

Firmware commands used:
    eeprom dump <addr>         -> [EEPROM] <addr> <32 hex>
    eeprom write <addr> <hex>  -> [EEPROM] wrote <n> bytes at <addr>
"""

from __future__ import annotations

import time

from .protocol import BLOCK, EEPROM_SIZE

BAUD = 9600


class NoEepromCommands(RuntimeError):
    """the firmware on the board does not answer eeprom dump"""


def _open(port: str, baud: int):
    """open the port, wait for the console after the board reset and return (serial, help banner)"""
    import serial

    ser = serial.Serial(port, baud, timeout=2, write_timeout=2)
    ser.reset_input_buffer()
    deadline = time.time() + 8
    while time.time() < deadline:
        ser.write(b"help\n")
        end = time.time() + 0.5
        while time.time() < end:
            line = ser.readline().decode("ascii", "replace").strip()
            if line.startswith("[HELP]"):
                return ser, line
    ser.close()
    raise RuntimeError("no [HELP] reply; check the port and the 9600 baud console")


def _line(ser, prefix: str, timeout: float = 3) -> str:
    deadline = time.time() + timeout
    while time.time() < deadline:
        line = ser.readline().decode("ascii", "replace").strip()
        if line.startswith(prefix):
            return line
    raise RuntimeError(f"no reply starting {prefix!r}")


def dump_board(port: str, baud: int = BAUD) -> tuple[bytes, str]:
    """read all 1024 EEPROM bytes, 16 per command, and return (image, help banner);
    NoEepromCommands when the first block is not answered"""
    image = bytearray(EEPROM_SIZE)
    ser, banner = _open(port, baud)
    try:
        for address in range(0, EEPROM_SIZE, BLOCK):
            ser.write(f"eeprom dump {address}\n".encode())
            try:
                line = _line(ser, f"[EEPROM] {address} ", timeout=3 if address else 2)
            except RuntimeError:
                if address == 0:
                    raise NoEepromCommands("the firmware did not answer eeprom dump")
                raise
            payload = line.split(" ", 2)[2]
            if len(payload) != 2 * BLOCK:
                raise RuntimeError(f"short dump line: {line!r}")
            image[address:address + BLOCK] = bytes.fromhex(payload)
    finally:
        ser.close()
    return bytes(image), banner


def write_board(port: str, image: bytes, chunks, baud: int = BAUD) -> None:
    """write (address, length) chunks and check each reply"""
    ser, _ = _open(port, baud)
    try:
        for address, length in chunks:
            payload = image[address:address + length].hex().upper()
            ser.write(f"eeprom write {address} {payload}\n".encode())
            expected = f"[EEPROM] wrote {length} bytes at {address}"
            line = _line(ser, "[EEPROM] wrote ")
            if line != expected:
                raise RuntimeError(f"unexpected reply: {line!r}")
    finally:
        ser.close()
