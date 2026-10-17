"""The host EEPROM tool's image, JSON and Intel HEX codecs"""

import json

import eeprom
import eeprom_tool


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
    assert bytes.fromhex(decoded["raw_hex"]) == sample()


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
    """ACC-EEPROM-63"""
    assert eeprom_tool.from_hex(eeprom_tool.to_hex(sample())) == sample()


def test_intel_hex_rejects_bad_checksum():
    """ACC-EEPROM-63"""
    text = eeprom_tool.to_hex(sample()).replace(":10000000", ":10001000", 1)
    try:
        eeprom_tool.from_hex(text)
    except ValueError:
        pass
    else:
        raise AssertionError("bad checksum accepted")
