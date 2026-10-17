"""Idle state: waiting for audio or the button"""

import pytest

import ir
from conftest import LOUD, QUIET, audio_on, blink


def test_audio_detected(board):
    """ACC-SENSE-8"""
    board.set_audio(LOUD)
    board.wait_for("[AUDIO START] Start", 5)


@pytest.mark.parametrize("level", [200, 290])
def test_audio_below_threshold_ignored(board, level):
    """ACC-SENSE-9, ACC-ENABLE-54: 290 is above the off level 75, which only audio on uses"""
    board.set_audio(level)
    board.run(30)
    assert "[AUDIO START]" not in board.serial


def test_spike_ignored(board):
    """ACC-SENSE-10"""
    board.set_audio(1023)
    board.run(0.1)
    board.set_audio(0)
    board.run(5)
    assert "[AUDIO START]" not in board.serial


def test_standby_right_after_music_stops(board):
    """ACC-SENSE-57, DEF-SENSE-13-1: standby 0.3 s after the music stops does not start the amplifier again"""
    audio_on(board, trigger=True)
    board.run(5)
    board.set_audio(QUIET)
    board.run(0.3)
    position = len(board.serial)
    board.set_trigger(False)
    board.wait_for("[AUDIO SENSE] Start", 1, position)
    board.run(3)
    assert "[AUDIO START]" not in board.serial[position:]



def test_spike_right_after_return_to_idle(board):
    """ACC-SENSE-10, DEF-SENSE-13: a spike in the first samples after standby by trigger"""
    audio_on(board, trigger=True)
    board.set_audio(QUIET)
    board.run(3)
    position = len(board.serial)
    board.set_trigger(False)
    board.wait_for("[AUDIO SENSE] Start", 1, position)
    board.set_audio(1023)
    board.run(0.1)
    board.set_audio(QUIET)
    board.run(5)
    assert "[AUDIO START]" not in board.serial[position:]


def test_set_audio_0_right_after_music_stops(board):
    """ACC-SENSE-57, DEF-SENSE-13-1"""
    audio_on(board)
    board.set_audio(QUIET)
    board.run(0.5)
    position = board.command("set audio 0", "[AUDIO SENSE] Start")
    board.run(3)
    assert "[AUDIO START]" not in board.serial[position:]


def test_silent_recording_from_audio_on(board):
    """ACC-SENSE-57, DEF-SENSE-13-1: the source is silent while the owner records codes from audio on"""
    audio_on(board)
    board.set_audio(QUIET)
    board.press(1.0)
    board.wait_for("[IRCODE RECORD] Start", 1)
    for value in (0x20DF10EF, 0x20DF906F):
        board.send_ir(ir.nec_frame(value))
        board.run(1)
    position = board.wait_for("[AUDIO SENSE] Start", 2, board.serial.index("[IRCODE RECORD] Start"))
    board.run(3)
    assert "[AUDIO START]" not in board.serial[position:]

def test_slow_blink_with_codes(board):
    """ACC-SENSE-11"""
    on, off = blink(board, 7)
    assert on == pytest.approx(1000, rel=0.05) and off == pytest.approx(2000, rel=0.05)


def test_press_starts_learning(board):
    """ACC-SENSE-12"""
    board.press(0.2)
    board.wait_for("[AUDIOSENSE LEARN] Entering state", 1)
    assert "[IRCODE RECORD]" not in board.serial


def test_hold_starts_recording(board):
    """ACC-SENSE-13"""
    board.press(1.0)
    board.wait_for("[IRCODE RECORD] Start", 1)
    assert "[AUDIOSENSE LEARN]" not in board.serial


def test_bouncing_hold_is_one_hold(board):
    """ACC-SENSE-53, DEF-BUTTON-15: the press bounce must not end the hold"""
    board.press(1.0)
    board.run(3)
    assert board.serial.count("[IRCODE RECORD] Start") == 1
    assert "[AUDIOSENSE LEARN]" not in board.serial


def test_bouncing_press_is_one_press(board):
    """ACC-SENSE-53, DEF-BUTTON-15"""
    board.press(0.2)
    board.wait_for("[AUDIOSENSE LEARN] Exit", 5)
    board.run(3)
    assert board.serial.count("[AUDIOSENSE LEARN] Entering state") == 1
