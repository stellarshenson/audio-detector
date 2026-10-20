"""Back up and restore the audio-detector EEPROM over the serial console.

The firmware keeps the START code in bytes 0-5 (value little-endian, then bits,
then the IRremote type), the STOP code in bytes 6-11 and the threshold in
bytes 12-13. The other bytes are kept as they are by a JSON restore.

Firmware commands used:
    eeprom dump <addr>         -> [EEPROM] <addr> <32 hex>
    eeprom write <addr> <hex>  -> [EEPROM] wrote <n> bytes at <addr>

A board that runs firmware without those commands is backed up through the
temporary bridge firmware in scripts/eeprom-bridge: the tool saves the whole
flash image, flashes the bridge, dumps the EEPROM, writes the saved image
back and reads it back to verify it. The saved image is written back after
a failed bridge flash or dump too; an interrupted write-back can leave the
board without working firmware until the next upload.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import tempfile
from pathlib import Path

from .protocol import (
    BLOCK,
    EEPROM_SIZE,
    SETTINGS_SIZE,
    decode_image,
    encode_image,
    from_hex,
    overlay_settings,
    settings_of,
    to_hex,
)
from .serial_client import NoEepromCommands, dump_board, write_board

EEPROM_DIR = Path("resources") / "eeprom"
BRIDGE_BANNER = "[HELP] eeprom bridge"


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


def backup(port: str, out: Path, bridge: Path | None = None) -> dict:
    out.parent.mkdir(parents=True, exist_ok=True)
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
    p.add_argument("--out", default=str(EEPROM_DIR / "eeprom"), help="output prefix without a suffix")
    p.add_argument("--bridge", type=Path, metavar="DIR",
                   help="bridge firmware project, flashed when the board firmware has no eeprom commands")

    p = sub.add_parser("restore", help="write .bin, .hex or .json to the EEPROM")
    p.add_argument("--port", required=True)
    p.add_argument("file", type=Path)
    p.add_argument("--yes", action="store_true", help="required to write; without it restore refuses")
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
    print(json.dumps(result))
    return 0
