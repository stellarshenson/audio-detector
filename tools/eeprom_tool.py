"""Back up and restore the audio-detector EEPROM over the serial console.

The firmware keeps the START code in bytes 0-5 (value little-endian, then bits,
then the IRremote type), the STOP code in bytes 6-11 and the threshold in
bytes 12-13. The other bytes are kept as they are by a JSON restore.

Firmware commands used:
    eeprom dump <addr>         -> [EEPROM] <addr> <32 hex>
    eeprom write <addr> <hex>  -> [EEPROM] wrote <n> bytes at <addr>

A board that runs firmware without those commands is backed up through the
temporary bridge firmware in tools/eeprom-bridge: the tool saves the whole
flash image, flashes the bridge, dumps the EEPROM, writes the saved image
back and reads it back to verify it. The flash is restored in every case.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import subprocess
import tempfile
import time
from pathlib import Path

EEPROM_SIZE = 1024
BLOCK = 16
SETTINGS_SIZE = 14
BAUD = 9600

NEC, SONY, SAMSUNG = 3, 4, 7
NOT_RECORDED = 255
PROTOCOLS = {NEC: "NEC", SONY: "SONY", SAMSUNG: "SAMSUNG", NOT_RECORDED: "NOT_RECORDED"}
BRIDGE_BANNER = "[HELP] eeprom bridge"


class NoEepromCommands(RuntimeError):
    """the firmware on the board does not answer eeprom dump"""


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


def _avrdude_paths() -> tuple[str, str]:
    core = Path(os.environ.get("PLATFORMIO_CORE_DIR", str(Path.home() / ".platformio")))
    package = core / "packages" / "tool-avrdude"
    binary, conf = package / "avrdude", package / "avrdude.conf"
    if not binary.exists() or not conf.exists():
        raise RuntimeError(f"avrdude is not installed under {package}; run inside the firmware-avr environment")
    return str(binary), str(conf)


def _avrdude(port: str, *args: str) -> subprocess.CompletedProcess:
    binary, conf = _avrdude_paths()
    command = [binary, "-C", conf, "-p", "atmega328p", "-c", "arduino", "-P", port, "-b", "57600", *args]
    return subprocess.run(command, capture_output=True, text=True, timeout=180)


def _output(result: subprocess.CompletedProcess) -> str:
    return (result.stdout + result.stderr).strip()


def read_flash(port: str, path: Path) -> None:
    """save the whole flash, bootloader included, as Intel HEX"""
    result = _avrdude(port, "-U", f"flash:r:{path}:i")
    if result.returncode != 0 or not path.exists():
        raise RuntimeError(f"reading the firmware failed:\n{_output(result)}")


def flash_bridge(port: str, project: Path) -> None:
    """build and upload the temporary bridge firmware"""
    try:
        result = subprocess.run(
            ["pio", "run", "-d", str(project), "-t", "upload", "--upload-port", port],
            capture_output=True, text=True, timeout=600,
        )
    except FileNotFoundError:
        raise RuntimeError("pio is not on PATH; run inside the firmware-avr environment") from None
    if result.returncode != 0:
        raise RuntimeError(f"flashing the bridge firmware failed:\n{_output(result)[-2000:]}")


def restore_flash(port: str, path: Path) -> subprocess.CompletedProcess:
    """write the saved image back; -D keeps avrdude from erasing the EEPROM"""
    return _avrdude(port, "-D", "-U", f"flash:w:{path}:i")


def verify_flash(port: str, path: Path) -> None:
    """read the flash back and require it to equal the saved image"""
    with tempfile.TemporaryDirectory() as tmp:
        readback = Path(tmp) / "readback.hex"
        result = _avrdude(port, "-U", f"flash:r:{readback}:i")
        if result.returncode != 0 or readback.read_text() != path.read_text():
            raise RuntimeError(f"the firmware read back does not match {path}:\n{_output(result)}")


def restore_saved(port: str, path: Path) -> None:
    """write the saved image back and verify the read-back"""
    result = restore_flash(port, path)
    verify_flash(port, path)
    if result.returncode != 0:
        raise RuntimeError(f"writing the firmware back reported an error:\n{_output(result)}")


def bridge_backup(port: str, out: Path, project: Path) -> tuple[bytes, Path]:
    """dump the EEPROM through the bridge firmware and restore the saved flash image

    The saved image is read from the board once and kept, so a run that is
    interrupted after the bridge is flashed restores the same image next time.
    """
    original = out.with_name(out.name + ".firmware.hex")
    if not original.exists():
        read_flash(port, original)
    try:
        flash_bridge(port, project)
        image, _ = dump_board(port)
    finally:
        restore_saved(port, original)
    return image, original


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


def backup(port: str, out: Path, bridge: Path | None = None) -> dict:
    source, firmware = "console", None
    try:
        image, banner = dump_board(port)
    except NoEepromCommands:
        if bridge is None:
            raise SystemExit("the firmware has no eeprom commands; pass --bridge <project> to flash the temporary bridge firmware")
        image, original = bridge_backup(port, out, bridge)
        source, firmware = "bridge", str(original)
    else:
        if banner == BRIDGE_BANNER:
            original = out.with_name(out.name + ".firmware.hex")
            if not original.exists():
                raise SystemExit(f"the board runs the bridge firmware and {original} is missing; flash the original image back with avrdude")
            restore_saved(port, original)
            source, firmware = "bridge", str(original)
    settings = decode_image(image)
    paths = {
        "bin": out.with_suffix(".bin"),
        "hex": out.with_suffix(".hex"),
        "json": out.with_suffix(".json"),
    }
    paths["bin"].write_bytes(image)
    paths["hex"].write_text(to_hex(image))
    paths["json"].write_text(json.dumps(settings, indent=2) + "\n")
    result = {
        "backup": {key: str(path) for key, path in paths.items()},
        "settings": {key: settings[key] for key in ("start", "stop", "threshold")},
        "source": source,
    }
    if firmware:
        result["firmware"] = firmware
    return result


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
        current, _ = dump_board(port)
        image = overlay_settings(current, json.loads(path.read_text()))
    write_board(port, image, chunks)
    result["written"] = True
    return result


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("backup", help="read the EEPROM and write .bin, .hex and .json")
    p.add_argument("--port", required=True)
    p.add_argument("--out", default="eeprom", help="output prefix without a suffix")
    p.add_argument("--bridge", type=Path, metavar="DIR",
                   help="bridge firmware project, flashed when the board firmware has no eeprom commands")

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
        result = backup(args.port, Path(args.out), args.bridge)
    elif args.command == "restore":
        result = restore(args.port, args.file, args.yes, args.dry_run)
    elif args.command == "decode":
        image, _ = load_image(args.file)
        result = decode_image(image)
    else:
        result = settings_of(dump_board(args.port)[0])
    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
