"""Back up and restore the audio-detector EEPROM over the serial console.

The firmware keeps the START code in bytes 0-5 (value little-endian, then bits,
then the IRremote type), the STOP code in bytes 6-11 and the threshold in
bytes 12-13. The other bytes are kept as they are by a JSON restore.

Firmware commands used:
    eeprom dump <addr>         -> [EEPROM] <addr> <32 hex>
    eeprom write <addr> <hex>  -> [EEPROM] wrote <n> bytes at <addr>
"""

from __future__ import annotations

import argparse
import hashlib
import json
import time
from pathlib import Path

EEPROM_SIZE = 1024
BLOCK = 16
SETTINGS_SIZE = 14
BAUD = 9600

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

    return {
        "layout": 1,
        "start": code(0),
        "stop": code(6),
        "threshold": int.from_bytes(data[12:14], "little"),
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


def _open(port: str, baud: int):
    """open the port and wait for the console after the board reset"""
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
                return ser
    ser.close()
    raise RuntimeError("no [HELP] reply; check the port and that the firmware has the eeprom commands")


def _line(ser, prefix: str) -> str:
    deadline = time.time() + 3
    while time.time() < deadline:
        line = ser.readline().decode("ascii", "replace").strip()
        if line.startswith(prefix):
            return line
    raise RuntimeError(f"no reply starting {prefix!r}")


def dump_board(port: str, baud: int = BAUD) -> bytes:
    """read all 1024 EEPROM bytes, 16 per command"""
    image = bytearray(EEPROM_SIZE)
    ser = _open(port, baud)
    try:
        for address in range(0, EEPROM_SIZE, BLOCK):
            ser.write(f"eeprom dump {address}\n".encode())
            line = _line(ser, f"[EEPROM] {address} ")
            payload = line.split(" ", 2)[2]
            if len(payload) != 2 * BLOCK:
                raise RuntimeError(f"short dump line: {line!r}")
            image[address:address + BLOCK] = bytes.fromhex(payload)
    finally:
        ser.close()
    return bytes(image)


def write_board(port: str, image: bytes, chunks, baud: int = BAUD) -> None:
    """write (address, length) chunks and check each reply"""
    ser = _open(port, baud)
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


def load_image(path: Path) -> tuple[bytes, str]:
    """(image, mode); mode is "settings" for JSON and "raw" for bin and hex"""
    suffix = path.suffix.lower()
    if suffix == ".json":
        return encode_image(json.loads(path.read_text())), "settings"
    if suffix == ".bin":
        data = path.read_bytes()
        if len(data) != EEPROM_SIZE:
            raise ValueError(f"{path} is {len(data)} bytes, expected {EEPROM_SIZE}")
        return data, "raw"
    if suffix == ".hex":
        return from_hex(path.read_text()), "raw"
    raise ValueError(f"unsupported file type: {path.suffix}")


def settings_of(image: bytes) -> dict:
    decoded = decode_image(image)
    return {key: decoded[key] for key in ("start", "stop", "threshold")}


def backup(port: str, out: Path) -> dict:
    image = dump_board(port)
    settings = decode_image(image)
    paths = {
        "bin": out.with_suffix(".bin"),
        "hex": out.with_suffix(".hex"),
        "json": out.with_suffix(".json"),
    }
    paths["bin"].write_bytes(image)
    paths["hex"].write_text(to_hex(image))
    paths["json"].write_text(json.dumps(settings, indent=2) + "\n")
    return {
        "backup": {key: str(path) for key, path in paths.items()},
        "settings": {key: settings[key] for key in ("start", "stop", "threshold")},
    }


def restore(port: str, path: Path, yes: bool, dry_run: bool) -> dict:
    image, mode = load_image(path)
    chunks = [(0, SETTINGS_SIZE)] if mode == "settings" else [(a, BLOCK) for a in range(0, EEPROM_SIZE, BLOCK)]
    result = {"file": str(path), "mode": mode, "settings": settings_of(image), "chunks": chunks}
    if dry_run:
        result["dry_run"] = True
        return result
    if not yes:
        raise SystemExit("refusing to write without --yes")
    if mode == "settings":
        image = overlay_settings(dump_board(port), json.loads(path.read_text()))
    write_board(port, image, chunks)
    result["written"] = True
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("backup", help="read the EEPROM and write .bin, .hex and .json")
    p.add_argument("--port", required=True)
    p.add_argument("--out", default="eeprom", help="output prefix without a suffix")

    p = sub.add_parser("restore", help="write .bin, .hex or .json to the EEPROM")
    p.add_argument("--port", required=True)
    p.add_argument("file", type=Path)
    p.add_argument("--yes", action="store_true", help="write without asking")
    p.add_argument("--dry-run", action="store_true", help="print the plan without writing")

    p = sub.add_parser("decode", help="print the settings in a .bin, .hex or .json file")
    p.add_argument("file", type=Path)

    p = sub.add_parser("verify", help="read the EEPROM and print the settings")
    p.add_argument("--port", required=True)

    args = parser.parse_args(argv)
    if args.command == "backup":
        result = backup(args.port, Path(args.out))
    elif args.command == "restore":
        result = restore(args.port, args.file, args.yes, args.dry_run)
    elif args.command == "decode":
        image, _ = load_image(args.file)
        result = decode_image(image)
    else:
        result = settings_of(dump_board(args.port))
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
