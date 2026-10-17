"""Boards and flows shared by the functional tests

Every test names the acceptance criterion (ACC-...) or defect (DEF-...) it checks.
"""

import pytest

import eeprom
import sim

START = (eeprom.NEC, 32, 0x20DF10EF)
STOP = (eeprom.NEC, 32, 0x20DF906F)
THRESHOLD = 300
LOUD, QUIET = 600, 0  # ADC readings of the RCA input
SETTLED = 3.5  # seconds after power-on; the firmware ignores audio for the first 3 s


@pytest.fixture
def make_board():
    boards = []

    def make(**options):
        board = sim.Board(**options)
        boards.append(board)
        return board

    yield make
    for board in boards:
        board.close()


@pytest.fixture
def configured(make_board):
    """factory for a board with START, STOP and threshold 300 in EEPROM, past the settling time"""
    def make(autostandby=False):
        board = make_board(eeprom=eeprom.image(START, STOP, THRESHOLD), autostandby=autostandby)
        board.run(SETTLED)
        return board
    return make


@pytest.fixture
def board(configured):
    return configured()


def audio_on(board, trigger=False):
    """plays audio until the board is in audio on; with trigger, the amplifier's 12V trigger turns on
    1 s after START, otherwise audio on starts in timeout mode"""
    board.set_audio(LOUD)
    start = board.wait_for("[AUDIO START] Start", 5)
    if trigger:
        board.run(1)
        board.set_trigger(True)
    board.wait_for("[AUDIO ENABLED] Start", 5, start)
    board.run(0.2)  # the relay switches after the state's entry messages are sent
    return start


def silence(board):
    """stops the audio and runs 3 s, so the 50 samples of the moving average (2.5 s) all hold the silence
    before a test moves time forward in jumps"""
    board.set_audio(QUIET)
    board.run(3)


def times_at(edges, level):
    """durations in ms of the complete intervals the pin spent at level"""
    return [(t2 - t1) / 1000 for (t1, l1), (t2, _) in zip(edges, edges[1:]) if l1 == level]


def blink(board, seconds):
    """(on_ms, off_ms) of the LED over the next seconds, from the last complete intervals"""
    board.edges["led"].clear()
    board.run(seconds)
    edges = board.edges["led"]
    return times_at(edges, 1)[-1], times_at(edges, 0)[-1]


def threshold(board):
    """the threshold the status command reports"""
    start = len(board.serial)
    board.command("status", "Audio level is")
    line = board.serial[board.serial.index("Audiosense threshold: ", start):].splitlines()[0]
    return int(line.split(":")[1])
