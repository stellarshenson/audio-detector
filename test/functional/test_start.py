"""Audio start: the amplifier switched on"""

import ir
from conftest import START, audio_on


def test_start_code_sent_once(board):
    """ACC-START-28"""
    audio_on(board)
    board.run(2)
    assert board.ir_sent() == [(ir.NEC, START[2], 32)]


def test_trigger_mode(board):
    """ACC-START-29"""
    start = audio_on(board, trigger=True)
    assert "[AUDIO START] Audio trigger detected" in board.serial[start:]
    assert "Trigger not available" not in board.serial[start:]


def test_timeout_mode(board):
    """ACC-START-30"""
    start = audio_on(board)
    assert "[AUDIO START] Timeout. Trigger not available" in board.serial[start:]
