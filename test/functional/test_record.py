"""IR recording: the START and STOP codes learned from the amplifier's remote"""

import pytest

import eeprom
import ir
from conftest import SETTLED, THRESHOLD, audio_on, blink

A, B = 0x20DF10EF, 0x20DF906F


@pytest.fixture
def recording(make_board):
    """a board with no IR codes in EEPROM, in IR recording"""
    board = make_board(eeprom=eeprom.image(threshold=THRESHOLD))
    board.run(SETTLED)
    board.press(1.0)
    board.wait_for("[IRCODE RECORD] Start", 1)
    return board


def send(board, frame):
    board.send_ir(frame)
    board.run(1)


def test_led_while_recording(recording):
    """ACC-RECORD-14"""
    on, off = blink(recording, 1)
    assert on == pytest.approx(100, rel=0.1) and off == pytest.approx(100, rel=0.1)
    send(recording, ir.nec_frame(A))
    on, off = blink(recording, 3)
    assert on == pytest.approx(500, rel=0.05) and off == pytest.approx(500, rel=0.05)


def test_first_code_is_start(recording):
    """ACC-RECORD-15"""
    send(recording, ir.nec_frame(A))
    assert eeprom.decode(recording.eeprom())[0] == (eeprom.NEC, 32, A)
    assert "[AUDIO SENSE] Start" not in recording.serial.split("[IRCODE RECORD] Start")[-1]


def test_second_code_is_stop_and_back_to_idle(recording):
    """ACC-RECORD-16"""
    send(recording, ir.nec_frame(A))
    send(recording, ir.nec_frame(B))
    start, stop, _ = eeprom.decode(recording.eeprom())
    assert (start, stop) == ((eeprom.NEC, 32, A), (eeprom.NEC, 32, B))
    recording.wait_for("[AUDIO SENSE] Start", 1, recording.serial.index("[2/2]"))


def test_sony_codes_recorded_and_sent(recording):
    """ACC-RECORD-17"""
    send(recording, ir.sony_frame(0xA90))
    send(recording, ir.sony_frame(0xA91))
    assert eeprom.decode(recording.eeprom())[:2] == ((eeprom.SONY, 12, 0xA90), (eeprom.SONY, 12, 0xA91))
    since = recording.now
    audio_on(recording)
    assert recording.ir_sent(since) == [(ir.SONY, 0xA90, 12)]


def test_nec_repeat_not_saved(recording):
    """ACC-RECORD-18, DEF-RECORD-2"""
    recording.send_ir(ir.nec_frame(A))
    end = recording.now + 1.5
    while recording.now < end:
        recording.send_ir(ir.nec_repeat())
    recording.run(1)
    send(recording, ir.nec_frame(B))
    start, stop, _ = eeprom.decode(recording.eeprom())
    assert (start, stop) == ((eeprom.NEC, 32, A), (eeprom.NEC, 32, B))


def test_unsupported_protocol_rejected(recording):
    """ACC-RECORD-19, DEF-RECORD-3"""
    send(recording, ir.samsung_frame(0xE0E040BF))
    assert "rejected" in recording.serial
    assert eeprom.decode(recording.eeprom())[0][0] == eeprom.NOT_RECORDED
    send(recording, ir.nec_frame(A))
    send(recording, ir.nec_frame(B))
    start, stop, _ = eeprom.decode(recording.eeprom())
    assert (start, stop) == ((eeprom.NEC, 32, A), (eeprom.NEC, 32, B))


def test_recording_from_audio_on(board):
    """ACC-RECORD-20"""
    audio_on(board)
    board.press(1.0)
    board.wait_for("[IRCODE RECORD] Start", 1)


def test_codes_kept_after_power_cycle(recording, make_board):
    """ACC-RECORD-21"""
    send(recording, ir.nec_frame(A))
    send(recording, ir.nec_frame(B))
    board = make_board(eeprom=recording.eeprom())
    board.run(SETTLED)
    audio_on(board)
    assert board.ir_sent() == [(ir.NEC, A, 32)]
