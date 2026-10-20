"""IR frame encoders and the carrier decoder the functional tests rely on"""

import ir


def carrier_edges(frame, start_us=1000):
    """carrier envelope edges a board sending this frame would produce"""
    edges, t = [], start_us
    for mark, space in frame:
        edges += [(t, 1), (t + mark, 0)]
        t += mark + space
    return edges


def test_nec_frame_decodes_to_its_value():
    assert ir.decode_edges(carrier_edges(ir.nec_frame(0x20DF10EF))) == [(ir.NEC, 0x20DF10EF, 32)]


def test_nec_frame_has_header_32_bits_and_stop_mark():
    frame = ir.nec_frame(0x80000001)
    assert len(frame) == 34
    assert frame[0] == (9000, 4500)
    assert frame[1] == (560, 1690) and frame[2] == (560, 560) and frame[32] == (560, 1690)
    assert frame[33][0] == 560


def test_nec_repeat_decodes_as_repeat():
    assert ir.decode_edges(carrier_edges(ir.nec_repeat())) == [(ir.NEC, 0xFFFFFFFF, 0)]


def test_sony_frame_decodes_to_its_value():
    assert ir.decode_edges(carrier_edges(ir.sony_frame(0xA90))) == [(ir.SONY, 0xA90, 12)]


def test_samsung_frame_is_not_decoded_as_nec_or_sony():
    assert ir.decode_edges(carrier_edges(ir.samsung_frame(0xE0E040BF))) == []


def test_two_frames_split_at_the_gap():
    edges = carrier_edges(ir.nec_frame(0x1) + ir.nec_frame(0x2))
    assert ir.decode_edges(edges) == [(ir.NEC, 0x1, 32), (ir.NEC, 0x2, 32)]


def test_decode_tolerates_20_percent_longer_timings():
    stretched = [(int(m * 1.2), int(s * 1.2)) for m, s in ir.nec_frame(0x20DF10EF)]
    assert ir.decode_edges(carrier_edges(stretched)) == [(ir.NEC, 0x20DF10EF, 32)]


def test_marks_ignore_a_leading_falling_edge():
    assert ir.marks([(5, 0), (10, 1), (20, 0)]) == [(10, 20)]


def test_receiver_segments_alternate_low_mark_high_space():
    steps = ir.receiver_segments([(9000, 4500), (560, 40000)])
    assert steps == [(0, 9000), (1, 4500), (0, 560), (1, 40000)]
