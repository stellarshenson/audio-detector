"""Power-on: settings restored from EEPROM, auto-standby jumper read"""

import pytest

import eeprom
from conftest import SETTLED, START, STOP, THRESHOLD, blink, threshold


def test_settings_restored(board):
    """ACC-BOOT-1"""
    assert "[INIT] Restoring IR codes, AUDIO START: 20DF10EF , AUDIO STOP: 20DF906F , SENSE THRESHOLD: 300" in board.serial


@pytest.mark.parametrize("configured_eeprom", [True, False])
def test_relay_off_at_power_on(make_board, configured_eeprom):
    """ACC-BOOT-2"""
    board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD) if configured_eeprom else None)
    board.run(1)
    assert board.level("relay") == 0


def test_audio_ignored_while_settling(make_board):
    """ACC-BOOT-3"""
    board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD))
    board.set_audio(1023)
    board.wait_for("Audio signal detected", 6)
    assert 3.0 <= board.now < 3.5


@pytest.mark.parametrize("jumper, word", [(True, "ON"), (False, "OFF")])
def test_auto_standby_reported(make_board, jumper, word):
    """ACC-BOOT-5, DEF-BOOT-5"""
    board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD), autostandby=jumper)
    board.wait_for("[INIT] type 'help'", 2)
    assert f"[INIT] Automatic standby is: {word}" in board.serial


def test_erased_eeprom_uses_default_threshold(make_board):
    """ACC-BOOT-6, DEF-BOOT-4"""
    board = make_board()
    board.run(SETTLED)
    assert threshold(board) == 300
    board.set_audio(400)
    board.wait_for("[AUDIO SENSE] Audio signal detected", 5)


def test_fast_blink_without_codes(make_board):
    """ACC-BOOT-7"""
    board = make_board()
    on, off = blink(board, 4)
    assert on == pytest.approx(250, rel=0.05) and off == pytest.approx(1000, rel=0.05)


def test_stored_code_that_cannot_be_sent_is_not_recorded(make_board):
    """ACC-BOOT-51: a Samsung START left in EEPROM, for example by an older firmware"""
    board = make_board(eeprom=eeprom.image((eeprom.SAMSUNG, 32, 0xE0E040BF), STOP, THRESHOLD))
    board.wait_for("[INIT] IR codes not available", 2)
    on, off = blink(board, 4)
    assert on == pytest.approx(250, rel=0.05) and off == pytest.approx(1000, rel=0.05)
