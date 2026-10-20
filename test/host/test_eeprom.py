"""The EEPROM layout helper against the byte offsets the firmware reads in setup()"""

import eeprom


def test_codes_and_threshold_at_their_offsets():
    data = eeprom.image(start=(eeprom.NEC, 32, 0x20DF10EF), stop=(eeprom.SONY, 12, 0xA90), threshold=0x1234)
    assert data[0:6] == bytes([0xEF, 0x10, 0xDF, 0x20, 32, 3])
    assert data[6:12] == bytes([0x90, 0x0A, 0, 0, 12, 4])
    assert data[12:14] == bytes([0x34, 0x12])
    assert set(data[14:]) == {0xFF}


def test_decode_reverses_image():
    start, stop = (eeprom.NEC, 32, 0x20DF10EF), (eeprom.NEC, 32, 0x20DF906F)
    assert eeprom.decode(eeprom.image(start, stop, 300)) == (start, stop, 300)


def test_erased_image_reads_not_recorded():
    start, stop, threshold = eeprom.decode(eeprom.image())
    assert start[0] == stop[0] == eeprom.NOT_RECORDED
    assert threshold == 0xFFFF
