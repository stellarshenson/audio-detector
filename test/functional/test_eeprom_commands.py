"""EEPROM download and upload over the serial console"""

import eeprom
from conftest import SETTLED, START, STOP, THRESHOLD


def dump_line(address, image):
    return f"[EEPROM] {address} {image[address:address + 16].hex().upper()}"


def test_eeprom_dump_first_block(make_board):
    """ACC-EEPROM-58"""
    board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD))
    board.run(SETTLED)
    expected = dump_line(0, board.eeprom())
    board.command("eeprom dump 0", expected)
    assert expected in board.serial


def test_eeprom_dump_last_block(make_board):
    """ACC-EEPROM-58"""
    board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD))
    board.run(SETTLED)
    expected = dump_line(1008, board.eeprom())
    board.command("eeprom dump 1008", expected)
    assert expected in board.serial


def test_eeprom_write_block(board):
    """ACC-EEPROM-59"""
    before = board.eeprom()
    board.command("eeprom write 12 2C01", "[EEPROM] wrote 2 bytes at 12")
    after = board.eeprom()
    assert after[12:14] == b"\x2c\x01"
    assert after[14:16] == before[14:16]


def test_eeprom_write_rejects_invalid(board):
    """ACC-EEPROM-60"""
    before = board.eeprom()
    for line in ("eeprom write 1024 00",
                 "eeprom write 0 001",
                 "eeprom write 0 00112233445566778899AABBCCDDEEFF00",
                 "eeprom write 0 GG",
                 "eeprom write 1016 00112233445566778899AABBCCDDEEFF"):
        board.write(line + "\n")
    board.run(0.5)
    assert board.eeprom() == before
    assert "[EEPROM] wrote" not in board.serial
