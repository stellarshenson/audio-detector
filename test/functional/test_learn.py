"""Threshold learning: the audio threshold taken from the playing level"""

import pytest

import eeprom
import ir
from conftest import SETTLED, STOP, audio_on, threshold, times_at


def learn(board, level):
    board.set_audio(level)
    board.press(0.2)
    position = board.wait_for("[AUDIOSENSE LEARN] Exit", 5)
    board.set_audio(0)
    return position


def test_threshold_half_the_playing_level(board):
    """ACC-LEARN-22"""
    learn(board, 500)
    assert threshold(board) == pytest.approx(250, abs=3)


@pytest.mark.parametrize("level", [0, 300])
def test_minimum_threshold(board, level):
    """ACC-LEARN-23"""
    learn(board, level)
    assert threshold(board) == 160


def test_learned_on_loud_music_starts_again(board):
    """ACC-LEARN-22, DEF-LEARN-16: the front end saturates, so the music a threshold was learned on reads the same"""
    learn(board, 600)
    board.run(3)
    position = len(board.serial)
    board.set_audio(600)
    board.wait_for("[AUDIO START] Start", 5, position)


def test_threshold_saved(board, make_board):
    """ACC-LEARN-24"""
    learn(board, 500)
    learned = threshold(board)
    assert eeprom.decode(board.eeprom())[2] == learned
    rebooted = make_board(eeprom=board.eeprom())
    rebooted.run(SETTLED)
    assert f"SENSE THRESHOLD: {learned}" in rebooted.serial


def test_led_while_learning(board):
    """ACC-LEARN-25"""
    board.press(0.2)
    board.edges["led"].clear()
    board.run(2.0)
    on, off = times_at(board.edges["led"], 1), times_at(board.edges["led"], 0)
    assert on[-1] == pytest.approx(750, rel=0.05) and off[-1] == pytest.approx(250, rel=0.1)


def test_back_to_idle(board):
    """ACC-LEARN-26"""
    position = learn(board, 0)
    board.wait_for("[AUDIO SENSE] Start", 1, position)


def test_learning_from_audio_on(configured):
    """ACC-LEARN-27"""
    board = configured(autostandby=True)
    audio_on(board)
    since = board.now
    board.press(0.2)
    board.wait_for("[AUDIOSENSE LEARN] Entering state", 1)
    assert board.level("relay") == 0
    assert board.ir_sent(since) == [(ir.NEC, STOP[2], 32)]
