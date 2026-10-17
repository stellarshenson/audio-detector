"""IR protocol timings: frames to drive the receiver pin, and decoding of the IR LED pin

A frame is a list of (mark_us, space_us) pairs; the last space is the idle gap after the frame.
The receiver output (D2) is LOW during a mark. The board's IR LED (D3) is recorded as its carrier
envelope: level 1 while the 38 kHz carrier is on, which is a mark.
"""

NEC, SONY, SAMSUNG = "NEC", "SONY", "SAMSUNG"
FRAME_GAP_US = 40000
TOLERANCE = 0.25


def _pulse_distance(value, bits, header, bit_mark, one_space, zero_space):
    pairs = [header]
    for i in reversed(range(bits)):
        pairs.append((bit_mark, one_space if value >> i & 1 else zero_space))
    pairs.append((bit_mark, FRAME_GAP_US))
    return pairs


def nec_frame(value, bits=32):
    return _pulse_distance(value, bits, (9000, 4500), 560, 1690, 560)


def nec_repeat():
    return [(9000, 2250), (560, FRAME_GAP_US)]


def samsung_frame(value, bits=32):
    return _pulse_distance(value, bits, (4500, 4500), 560, 1600, 560)


def sony_frame(value, bits=12):
    pairs = [(2400, 600)]
    for i in reversed(range(bits)):
        pairs.append((1200 if value >> i & 1 else 600, 600))
    pairs[-1] = (pairs[-1][0], FRAME_GAP_US)
    return pairs


def receiver_segments(frame):
    """(level, duration_us) steps for the receiver pin, starting and ending idle HIGH"""
    steps = []
    for mark, space in frame:
        steps += [(0, mark), (1, space)]
    return steps


def marks(edges):
    """marks as (start_us, end_us) from the carrier envelope edges (t_us, level)"""
    out, start = [], None
    for t, level in edges:
        if level:
            start = t
        elif start is not None:
            out.append((start, t))
            start = None
    return out


def _near(measured, expected):
    return abs(measured - expected) <= expected * TOLERANCE + 30


def frames(marks, gap_us=20000):
    """splits marks into frames of (mark_us, space_us) pairs at idle gaps longer than gap_us"""
    out, frame = [], []
    for i, (start, end) in enumerate(marks):
        space = marks[i + 1][0] - end if i + 1 < len(marks) else None
        if space is None or space > gap_us:
            frame.append((end - start, None))
            out.append(frame)
            frame = []
        else:
            frame.append((end - start, space))
    return out


def decode(frame):
    """(protocol, value, bits) for one frame of (mark_us, space_us) pairs, or None"""
    head_mark, head_space = frame[0]
    if _near(head_mark, 9000) and head_space is not None and _near(head_space, 2250):
        return (NEC, 0xFFFFFFFF, 0)
    if _near(head_mark, 9000) and head_space is not None and _near(head_space, 4500):
        value = 0
        for mark, space in frame[1:-1]:
            if not _near(mark, 560):
                return None
            value = value << 1 | (1 if _near(space, 1690) else 0)
        return (NEC, value, len(frame) - 2)
    if _near(head_mark, 2400):
        value = 0
        for mark, _space in frame[1:]:
            value = value << 1 | (1 if _near(mark, 1200) else 0)
        return (SONY, value, len(frame) - 1)
    return None


def decode_edges(edges):
    """every decodable frame sent on the IR LED pin, as (protocol, value, bits)"""
    return [d for d in (decode(f) for f in frames(marks(edges))) if d]
