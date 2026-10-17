"""Behaviour when millis() wraps past 2^32 after 49.7 days of uptime"""

from conftest import LOUD, audio_on, silence

WRAP = 2**32


def test_audio_detected_across_wrap(board):
    """ACC-TIMING-50, DEF-TIMING-6, DEF-TIMING-17: the jump makes the next sample land at 2^32-51, so the one
    after it is due at 2^32; the average passes the threshold about 1.3 s later, after the wrap"""
    since = board.now
    board.write_u32("timer0_millis", WRAP - 51)
    board.set_audio(LOUD)
    board.wait_for("[AUDIO SENSE] Audio signal detected", 10)
    assert board.now - since < 2.0


def test_button_press_across_wrap(board):
    """ACC-TIMING-50, DEF-TIMING-6: a 200 ms press that starts 300 ms before the wrap is a press"""
    board.write_u32("timer0_millis", WRAP - 300)
    board.press(0.2)
    board.wait_for("[AUDIOSENSE LEARN] Entering state", 1)
    assert "[IRCODE RECORD]" not in board.serial


def test_silence_timeout_across_wrap(board):
    """ACC-TIMING-50"""
    board.write_u32("timer0_millis", WRAP - 5 * 60 * 1000)
    audio_on(board)
    silence(board)
    board.elapse(9 * 60)
    assert board.level("relay") == 1
    board.elapse(2 * 60)
    assert board.level("relay") == 0
