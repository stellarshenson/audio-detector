"""The firmware's EEPROM layout: START code at bytes 0-5, STOP code at 6-11, threshold at 12-13

A code is (type, bits, value): value as 4 bytes little-endian, then bits, then the IRremote
decode type. Erased bytes read 0xFF.
"""

EEPROM_SIZE = 1024
NEC, SONY, SAMSUNG = 3, 4, 7  # IRremote 2.6.1 decode_type_t
NOT_RECORDED = 255


def image(start=None, stop=None, threshold=None):
    data = bytearray(b"\xff" * EEPROM_SIZE)
    for offset, code in ((0, start), (6, stop)):
        if code:
            kind, bits, value = code
            data[offset:offset + 6] = value.to_bytes(4, "little") + bytes([bits, kind])
    if threshold is not None:
        data[12:14] = threshold.to_bytes(2, "little")
    return bytes(data)


def decode(data):
    """(start, stop, threshold)"""
    def code(offset):
        return (data[offset + 5], data[offset + 4], int.from_bytes(data[offset:offset + 4], "little"))
    return code(0), code(6), int.from_bytes(data[12:14], "little")
