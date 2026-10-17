"""Serial command line at 9600 baud"""

import pytest

import eeprom
import ir
from conftest import SETTLED, START, THRESHOLD, audio_on, threshold

UART_BYTE_S = 0.001042


def test_help(board):
    """ACC-SERIAL-39"""
    start = len(board.serial)
    board.command("help", "[HELP] available commands")
    board.wait_for("reboot - to reboot the device", 2, start)
    reply = board.serial[start:]
    for line in ("set threshold <160-1023> - to set the audio sense threshold",
                 "set audio [0|1] - to enable or disable audio control",
                 "set debug_plot [0|1] - to enable audio signal plot (use arduino tools->serial plotter)",
                 "eeprom dump <0-1008> - to print 16 EEPROM bytes as hex",
                 "eeprom write <0-1023> <hex> - to write 1 to 16 EEPROM bytes",
                 "status - to print device status",
                 "reboot - to reboot the device"):
        assert line in reply


def test_status(board):
    """ACC-SERIAL-40"""
    start = len(board.serial)
    board.command("status", "Debug plot: off")
    reply = board.serial[start:]
    for line in ("Audiosense threshold: 300", "Audio off level: 75", "Audio level is ", "Audio is off", "Autostandby config: off",
                 "External 12V Trigger: off", "Debug plot: off"):
        assert line in reply


@pytest.mark.parametrize("value", [160, 1023])
def test_set_threshold(board, make_board, value):
    """ACC-SERIAL-41"""
    board.command(f"set threshold {value}", f"[CONFIG] setting Audiosense threshold to {value}")
    assert threshold(board) == value
    assert eeprom.decode(board.eeprom())[2] == value
    rebooted = make_board(eeprom=board.eeprom())
    rebooted.run(SETTLED)
    assert f"SENSE THRESHOLD: {value}" in rebooted.serial


def test_invalid_threshold_ignored(board):
    """ACC-SERIAL-42"""
    for value in ("0", "abc", "159", "1024", "-5"):
        board.write(f"set threshold {value}\n")
    board.run(1)
    assert threshold(board) == THRESHOLD
    assert "[CONFIG]" not in board.serial


def test_set_audio_1(board):
    """ACC-SERIAL-43"""
    since = board.now
    board.command("set audio 1", "[AUDIO START] Start")
    board.run(0.5)
    assert board.ir_sent(since) == [(ir.NEC, START[2], 32)]


def test_set_audio_0(board):
    """ACC-SERIAL-44"""
    audio_on(board)
    board.command("set audio 0", "[AUDIO SENSE] Start")
    assert board.level("relay") == 0


def test_debug_plot(board):
    """ACC-SERIAL-45"""
    board.command("set debug_plot 1", "[CONFIG] setting debug_plot on")
    start = len(board.serial)
    board.run(1)
    assert board.serial[start:].count("[ADC]") >= 5
    board.command("set debug_plot 0", "[CONFIG] setting debug_plot off")
    board.run(0.5)
    start = len(board.serial)
    board.run(1)
    assert "[ADC]" not in board.serial[start:]


def test_reboot(board):
    """ACC-SERIAL-46"""
    first = board.serial.index("[INIT] Restoring")
    board.write("reboot\n")
    board.wait_for("[INIT] Restoring", 2, first + 1)


def test_empty_line_runs_nothing(board):
    """ACC-SERIAL-47, DEF-SERIAL-7"""
    board.command("help", "[HELP] available commands")
    board.write("\n")
    board.run(2)
    assert board.serial.count("[HELP] available commands") == 1


def test_reply_within_100_ms(board):
    """ACC-SERIAL-48, DEF-SERIAL-8"""
    line = "status\n"
    line_end = board.now + len(line) * UART_BYTE_S
    board.write(line)
    board.wait_for("[STATUS]", 2)
    assert board.now - line_end < 0.1


def test_device_runs_while_command_is_typed(board):
    """ACC-SERIAL-48"""
    board.write("stat")
    board.set_audio(600)
    board.wait_for("[AUDIO START] Start", 5)


def test_carriage_return_ends_line(board):
    """ACC-SERIAL-49"""
    board.write("help\r")
    board.wait_for("[HELP] available commands", 2)


def test_long_line_is_cut_and_device_keeps_running(board):
    """ACC-SERIAL-52"""
    board.write("x" * 300)
    board.run(0.5)
    board.write("\n")
    board.run(0.2)
    board.command("help", "[HELP] available commands")
    board.set_audio(600)
    board.wait_for("[AUDIO START] Start", 5)
