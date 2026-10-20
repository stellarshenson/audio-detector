"""The host EEPROM tool's image, JSON and Intel HEX codecs and the bridge recovery flow"""

import json
import subprocess

import eeprom
from audiodetector_host import eeprom_tool


def sample():
    return eeprom.image(start=(eeprom.NEC, 32, 0x20DF10EF),
                        stop=(eeprom.NEC, 32, 0x20DF906F), threshold=300)


def test_decode_image():
    """ACC-EEPROM-61"""
    decoded = eeprom_tool.decode_image(sample())
    assert decoded["start"]["protocol"] == "NEC"
    assert decoded["start"]["type"] == eeprom.NEC
    assert decoded["start"]["bits"] == 32
    assert decoded["start"]["value"] == 0x20DF10EF
    assert decoded["stop"]["value"] == 0x20DF906F
    assert decoded["threshold"] == 300
    assert decoded["threshold_set"] is True
    assert bytes.fromhex(decoded["raw_hex"]) == sample()


def test_decode_erased_threshold():
    """ACC-EEPROM-61 an erased EEPROM keeps the firmware's initial threshold and reports it as not set"""
    decoded = eeprom_tool.decode_image(bytes(b"\xff" * 1024))
    assert decoded["threshold"] == 0xFFFF
    assert decoded["threshold_set"] is False
    assert decoded["start"]["protocol"] == "NOT_RECORDED"


def test_encode_image_round_trip():
    """ACC-EEPROM-61"""
    decoded = json.loads(json.dumps(eeprom_tool.decode_image(sample())))
    assert eeprom_tool.encode_image(decoded)[0:14] == sample()[0:14]


def test_overlay_preserves_rest():
    """ACC-EEPROM-62"""
    current = bytes(b"\xAA" * 1024)
    restored = eeprom_tool.overlay_settings(current, eeprom_tool.decode_image(sample()))
    assert restored[0:14] == sample()[0:14]
    assert restored[14:] == current[14:]


def test_intel_hex_round_trip():
    """ACC-EEPROM-64"""
    assert eeprom_tool.from_hex(eeprom_tool.to_hex(sample())) == sample()


def test_intel_hex_rejects_bad_checksum():
    """ACC-EEPROM-64"""
    text = eeprom_tool.to_hex(sample()).replace(":10000000", ":10001000", 1)
    try:
        eeprom_tool.from_hex(text)
    except ValueError:
        pass
    else:
        raise AssertionError("bad checksum accepted")


def test_avrdude_reads_and_writes_flash(monkeypatch, tmp_path):
    """ACC-EEPROM-65 the flash commands name the port, the old bootloader and Intel HEX; the write keeps the EEPROM"""
    commands = []

    def run(command, **kwargs):
        commands.append(command)
        return subprocess.CompletedProcess(command, 0, "", "")

    monkeypatch.setattr(eeprom_tool.subprocess, "run", run)
    monkeypatch.setattr(eeprom_tool, "_avrdude_paths", lambda: ("avrdude", "avrdude.conf"))
    image = tmp_path / "firmware.hex"
    image.write_text(":00000001FF\n")

    eeprom_tool.read_flash("/dev/ttyUSB0", image)
    eeprom_tool.restore_flash("/dev/ttyUSB0", image)

    assert commands[0][-1] == f"flash:r:{image}:i"
    assert commands[1][-1] == f"flash:w:{image}:i"
    assert "-D" in commands[1]
    for command in commands:
        assert command[command.index("-P") + 1] == "/dev/ttyUSB0"
        assert command[command.index("-b") + 1] == "57600"
        assert command[command.index("-p") + 1] == "atmega328p"


def test_backup_flashes_the_bridge_and_restores_the_firmware(monkeypatch, tmp_path):
    """ACC-EEPROM-65 the backup saves the flash, flashes the bridge, dumps, restores and verifies in that order"""
    image = sample()
    calls, dumps = [], [eeprom_tool.NoEepromCommands("no commands"), (image, "[HELP] available commands")]

    def dump(port):
        calls.append("dump")
        result = dumps.pop(0)
        if isinstance(result, Exception):
            raise result
        return result

    def read(port, path):
        calls.append("read")
        path.write_text("original")

    def restore(port, path):
        calls.append("restore")
        return subprocess.CompletedProcess([], 0, "", "")

    monkeypatch.setattr(eeprom_tool, "dump_board", dump)
    monkeypatch.setattr(eeprom_tool, "read_flash", read)
    monkeypatch.setattr(eeprom_tool, "flash_bridge", lambda port, project: calls.append("flash"))
    monkeypatch.setattr(eeprom_tool, "restore_flash", restore)
    monkeypatch.setattr(eeprom_tool, "verify_flash", lambda port, path: calls.append("verify"))

    result = eeprom_tool.backup("port", tmp_path / "eeprom", tmp_path)

    assert calls == ["dump", "read", "flash", "dump", "restore", "verify"]
    assert result["source"] == "bridge"
    assert result["firmware"] == str(tmp_path / "eeprom.firmware.hex")
    assert (tmp_path / "eeprom.firmware.hex").read_text() == "original"
    assert (tmp_path / "eeprom.bin").read_bytes() == image


def test_backup_reuses_the_saved_firmware(monkeypatch, tmp_path):
    """ACC-EEPROM-65 a saved image is not read again, so a re-run after a failed restore still writes the original back"""
    (tmp_path / "eeprom.firmware.hex").write_text("original")
    calls, dumps = [], [eeprom_tool.NoEepromCommands("no commands"), (sample(), "[HELP] available commands")]

    def dump(port):
        calls.append("dump")
        result = dumps.pop(0)
        if isinstance(result, Exception):
            raise result
        return result

    monkeypatch.setattr(eeprom_tool, "dump_board", dump)
    monkeypatch.setattr(eeprom_tool, "read_flash", lambda port, path: calls.append("read"))
    monkeypatch.setattr(eeprom_tool, "flash_bridge", lambda port, project: calls.append("flash"))
    monkeypatch.setattr(eeprom_tool, "restore_flash", lambda port, path: calls.append("restore") or subprocess.CompletedProcess([], 0, "", ""))
    monkeypatch.setattr(eeprom_tool, "verify_flash", lambda port, path: calls.append("verify"))

    eeprom_tool.backup("port", tmp_path / "eeprom", tmp_path)

    assert calls == ["dump", "flash", "dump", "restore", "verify"]


def test_backup_finishes_a_failed_restore(monkeypatch, tmp_path):
    """ACC-EEPROM-65 a board left on the bridge is dumped and the saved firmware is written back without flashing again"""
    (tmp_path / "eeprom.firmware.hex").write_text("original")
    calls = []
    monkeypatch.setattr(eeprom_tool, "dump_board", lambda port: (sample(), eeprom_tool.BRIDGE_BANNER))
    monkeypatch.setattr(eeprom_tool, "flash_bridge", lambda port, project: calls.append("flash"))
    monkeypatch.setattr(eeprom_tool, "restore_flash", lambda port, path: calls.append("restore") or subprocess.CompletedProcess([], 0, "", ""))
    monkeypatch.setattr(eeprom_tool, "verify_flash", lambda port, path: calls.append("verify"))

    result = eeprom_tool.backup("port", tmp_path / "eeprom", tmp_path)

    assert calls == ["restore", "verify"]
    assert result["source"] == "bridge"
    assert result["firmware"] == str(tmp_path / "eeprom.firmware.hex")
    assert (tmp_path / "eeprom.bin").read_bytes() == sample()


def test_backup_without_bridge_names_the_option(monkeypatch, tmp_path):
    """ACC-EEPROM-65 a firmware without the commands is not flashed unless the bridge project is named"""
    def no_commands(port):
        raise eeprom_tool.NoEepromCommands("no commands")

    monkeypatch.setattr(eeprom_tool, "dump_board", no_commands)
    try:
        eeprom_tool.backup("port", tmp_path / "eeprom")
    except SystemExit as error:
        assert "--bridge" in str(error)
    else:
        raise AssertionError("backup without --bridge did not stop")


def test_backup_writes_under_resources_eeprom(monkeypatch, tmp_path, capsys):
    """ACC-LAYOUT-84 a backup without --out creates resources/eeprom/ and writes the three files there"""
    monkeypatch.chdir(tmp_path)
    monkeypatch.setattr(eeprom_tool, "dump_board", lambda port: (sample(), "[HELP] available commands"))

    assert eeprom_tool.main(["backup", "--port", "port"]) == 0

    (line,) = capsys.readouterr().out.splitlines()
    assert json.loads(line)["backup"] == {kind: f"resources/eeprom/eeprom.{kind}" for kind in ("bin", "hex", "json")}
    assert (tmp_path / "resources" / "eeprom" / "eeprom.bin").read_bytes() == sample()
