"""The EEPROM layout shared with the firmware and the Intel HEX form of an image.

The firmware keeps the START code in bytes 0-5 (value little-endian, then bits,
then the IRremote type), the STOP code in bytes 6-11 and the threshold in
bytes 12-13.
"""

from __future__ import annotations

import hashlib

EEPROM_SIZE = 1024
BLOCK = 16
SETTINGS_SIZE = 14

NEC, SONY, SAMSUNG = 3, 4, 7
NOT_RECORDED = 255
PROTOCOLS = {NEC: "NEC", SONY: "SONY", SAMSUNG: "SAMSUNG", NOT_RECORDED: "NOT_RECORDED"}


def decode_image(data: bytes) -> dict:
    """the settings and the raw image as JSON-ready data"""
    if len(data) != EEPROM_SIZE:
        raise ValueError(f"expected {EEPROM_SIZE} bytes, got {len(data)}")

    def code(offset: int) -> dict:
        value = int.from_bytes(data[offset:offset + 4], "little")
        kind = data[offset + 5]
        return {
            "type": kind,
            "protocol": PROTOCOLS.get(kind, f"UNKNOWN_{kind}"),
            "bits": data[offset + 4],
            "value": value,
            "value_hex": f"0x{value:08X}",
        }

    threshold = int.from_bytes(data[12:14], "little")
    return {
        "layout": 1,
        "start": code(0),
        "stop": code(6),
        "threshold": threshold,
        "threshold_set": threshold <= 1023,
        "raw_size": len(data),
        "raw_hex": data.hex(),
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def encode_image(settings: dict) -> bytes:
    """a 1024-byte image from JSON settings; bytes 14-1023 are erased"""
    data = bytearray(b"\xFF" * EEPROM_SIZE)
    for offset, key in ((0, "start"), (6, "stop")):
        code = settings[key]
        value, bits, kind = int(code["value"]), int(code["bits"]), int(code["type"])
        if not 0 <= value <= 0xFFFFFFFF:
            raise ValueError(f"{key} value out of range")
        if not 0 <= bits <= 0xFF or not 0 <= kind <= 0xFF:
            raise ValueError(f"{key} bits or type out of range")
        data[offset:offset + 6] = value.to_bytes(4, "little") + bytes([bits, kind])
    threshold = int(settings["threshold"])
    if not 0 <= threshold <= 0xFFFF:
        raise ValueError("threshold out of range")
    data[12:14] = threshold.to_bytes(2, "little")
    return bytes(data)


def overlay_settings(current: bytes, settings: dict) -> bytes:
    """replace bytes 0-13 of current and keep bytes 14-1023"""
    if len(current) != EEPROM_SIZE:
        raise ValueError("current image must be 1024 bytes")
    image = bytearray(current)
    image[0:SETTINGS_SIZE] = encode_image(settings)[0:SETTINGS_SIZE]
    return bytes(image)


def to_hex(data: bytes) -> str:
    """Intel HEX image, 16 bytes per record"""
    lines = []
    for offset in range(0, len(data), BLOCK):
        chunk = data[offset:offset + BLOCK]
        record = bytes([len(chunk)]) + offset.to_bytes(2, "big") + b"\x00" + chunk
        checksum = (-sum(record)) & 0xFF
        lines.append(":" + (record + bytes([checksum])).hex().upper())
    lines.append(":00000001FF")
    return "\n".join(lines) + "\n"


def from_hex(text: str) -> bytes:
    """read an Intel HEX image and require all 1024 bytes"""
    data = bytearray(b"\xFF" * EEPROM_SIZE)
    seen = bytearray(EEPROM_SIZE)
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if not line.startswith(":"):
            raise ValueError(f"not an Intel HEX record: {line!r}")
        record = bytes.fromhex(line[1:])
        if sum(record) & 0xFF:
            raise ValueError(f"bad Intel HEX checksum: {line!r}")
        count, address, kind = record[0], int.from_bytes(record[1:3], "big"), record[3]
        if kind == 0x01:
            break
        if kind != 0x00:
            continue
        payload = record[4:4 + count]
        if address + len(payload) > EEPROM_SIZE:
            raise ValueError("Intel HEX record outside the 1024-byte EEPROM")
        data[address:address + len(payload)] = payload
        seen[address:address + len(payload)] = b"\x01" * len(payload)
    if not all(seen):
        raise ValueError("Intel HEX file does not cover all 1024 bytes")
    return bytes(data)


def settings_of(image: bytes) -> dict:
    decoded = decode_image(image)
    return {key: decoded[key] for key in ("start", "stop", "threshold")}
