from lucia_tof import protocol
from lucia_tof.tof_check import check, tof_node_running


def test_check_counts_frames_glitches_and_prints_grids():
    clock = [0.0]
    sent = [0]

    def read():
        clock[0] += 0.05
        sent[0] += 1
        glitch = sent[0] % 4 == 0                       # middle sensor: a 0 mm zone now and then
        middle = [600] * 16
        if glitch:
            middle[5] = 0
        return (protocol.encode(protocol.Packet(0, 0, sent[0], 1000, 330, tuple([330] * 16)))
                + protocol.encode(protocol.Packet(1, 1 if glitch else 0, sent[0], 1000,
                                                  0 if glitch else 600, tuple(middle))))

    lines = []
    counts = check(read, 2.0, clock=lambda: clock[0], out=lines.append)
    assert counts['frames'][0] == counts['frames'][1] == 40
    assert counts['frames'][2] == 0
    assert counts['glitches'] == [0, 10, 0]
    text = '\n'.join(lines)
    assert 'middle: 600 mm' in text and 'NEAR' not in text     # the glitches never show as NEAR
    assert 'right: 0 frames/s  <- nothing from this sensor' in text
    assert '10 frames with a glitch' in text
    # grids are each zone's median: the 0 mm glitches don't show in the middle sensor's
    assert 'middle: 20 frames/s, 16 of 16 zones see something' in text
    assert '      600    600    600    600' in text


def test_tof_node_running(tmp_path):
    (tmp_path / '12').mkdir()
    (tmp_path / '12' / 'cmdline').write_bytes(b'python3\x00/ros2_ws/install/lucia_tof/lib/lucia_tof/tof_node\x00')
    (tmp_path / 'self').mkdir()
    assert tof_node_running(str(tmp_path))
    (tmp_path / '12' / 'cmdline').write_bytes(b'python3\x00tof_check\x00')
    assert not tof_node_running(str(tmp_path))
