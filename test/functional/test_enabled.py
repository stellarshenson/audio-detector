"""Audio on: relay on, waiting for standby by trigger or silence timeout"""

import pytest

import eeprom
import ir
from conftest import SETTLED, START, STOP, audio_on, silence, threshold


def test_relay_on(board):
    """ACC-ENABLE-31"""
    audio_on(board)
    assert board.level("relay") == 1


def test_led_on(board):
    """ACC-ENABLE-32"""
    audio_on(board)
    board.run(0.2)
    board.edges["led"].clear()
    board.run(5)
    assert board.level("led") == 1 and board.edges["led"] == []


def test_trigger_off_turns_audio_off(board):
    """ACC-ENABLE-33"""
    audio_on(board, trigger=True)
    position = len(board.serial)
    board.set_trigger(False)
    board.wait_for("[AUDIO SENSE] Start", 1, position)
    assert board.level("relay") == 0


def test_trigger_mode_has_no_timeout(configured):
    """ACC-ENABLE-34, DEF-ENABLE-1"""
    board = configured(autostandby=True)
    audio_on(board, trigger=True)
    since, position = board.now, len(board.serial)
    board.elapse(15 * 60)
    assert board.level("relay") == 1
    assert "[AUDIO ENABLED] Exit" not in board.serial[position:]
    assert board.ir_sent(since) == []


def test_silence_timeout(board):
    """ACC-ENABLE-35"""
    audio_on(board)
    silence(board)
    position = len(board.serial)
    board.elapse(9 * 60)
    assert board.level("relay") == 1
    board.elapse(2 * 60)
    assert board.level("relay") == 0
    assert "[AUDIO SENSE] Start" in board.serial[position:]


def test_audio_keeps_audio_on(board):
    """ACC-ENABLE-36"""
    audio_on(board)
    board.edges["relay"].clear()
    board.elapse(15 * 60)
    assert board.level("relay") == 1 and board.edges["relay"] == []


@pytest.mark.parametrize("jumper, sent", [(True, [(ir.NEC, STOP[2], 32)]), (False, [])])
def test_stop_code_on_standby(configured, jumper, sent):
    """ACC-ENABLE-37, ACC-ENABLE-38, ACC-BOOT-4"""
    board = configured(autostandby=jumper)
    audio_on(board)
    since = board.now
    silence(board)
    board.elapse(11 * 60)
    assert board.level("relay") == 0
    assert board.ir_sent(since) == sent


def test_quiet_music_keeps_audio_on(board):
    """ACC-ENABLE-36, ACC-ENABLE-54: 150 is below the threshold 300 and above the off level 75"""
    audio_on(board)
    board.set_audio(150)
    board.run(3)
    board.edges["relay"].clear()
    board.elapse(15 * 60)
    assert board.level("relay") == 1 and board.edges["relay"] == []


def test_level_below_off_level_turns_audio_off(board):
    """ACC-ENABLE-35, ACC-ENABLE-54: 60 is below the off level 75"""
    audio_on(board)
    board.set_audio(60)
    board.run(3)
    board.elapse(9 * 60)
    assert board.level("relay") == 1
    board.elapse(2 * 60)
    assert board.level("relay") == 0


def test_threshold_below_minimum_restored_as_minimum(make_board):
    """ACC-ENABLE-54, DEF-ENABLE-18: a threshold of 100 stored by older firmware restores as 160, so quiet at 35
    is below the off level 40"""
    board = make_board(eeprom=eeprom.image(START, STOP, 100))
    board.run(SETTLED)
    assert threshold(board) == 160
    audio_on(board)
    board.set_audio(35)
    board.run(3)
    board.elapse(11 * 60)
    assert board.level("relay") == 0
