"""The temporary bridge firmware that reads the EEPROM of a board whose firmware has no eeprom commands

Every test names the acceptance criterion (ACC-...) it checks.
"""

import pytest

import eeprom
import sim

START = (eeprom.NEC, 32, 0x20DF10EF)
STOP = (eeprom.NEC, 32, 0x20DF906F)
THRESHOLD = 300
BRIDGE = sim.ROOT / "tools/eeprom-bridge/.pio/build/nanoatmega328/firmware.elf"


@pytest.fixture
def bridge(make_board):
    board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD), firmware=BRIDGE)
    board.run(0.1)
    return board


def test_bridge_help(bridge):
    """ACC-EEPROM-65 the bridge names itself and its two commands"""
    bridge.command("help", "[HELP] eeprom bridge")
    bridge.wait_for("eeprom write <0-1023> <hex>", 2)
    assert "eeprom dump <0-1008>" in bridge.serial
    assert "eeprom write <0-1023> <hex>" in bridge.serial


def test_bridge_dump(bridge):
    """ACC-EEPROM-65 the bridge prints the same 16-byte dump line as the main firmware"""
    image = eeprom.image(START, STOP, THRESHOLD)
    expected = f"[EEPROM] 0 {image[:16].hex().upper()}"
    start = bridge.command("eeprom dump 0", expected)
    assert bridge.serial[start:].splitlines()[0] == expected


def test_bridge_write(bridge):
    """ACC-EEPROM-65 the bridge writes 1 to 16 bytes, confirms the count and leaves the rest alone"""
    bridge.command("eeprom write 12 2C01", "[EEPROM] wrote 2 bytes at 12")
    assert bridge.eeprom()[12:16] == b"\x2C\x01\xFF\xFF"


def test_bridge_rejects_invalid_write(bridge):
    """ACC-EEPROM-65 an address above 1023, odd hex or more than 16 bytes writes nothing"""
    before = bridge.eeprom()
    for line in ("eeprom write 1024 00", "eeprom write 12 2C0", "eeprom write 0 " + "00" * 17, "eeprom write 12 2G"):
        bridge.write(line + "\n")
    bridge.run(0.1)
    assert "[EEPROM] wrote" not in bridge.serial
    assert bridge.eeprom() == before
