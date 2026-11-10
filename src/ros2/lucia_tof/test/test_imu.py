import math

from lucia_tof import imu, protocol
from lucia_tof.protocol import ImuPacket, Packet, Parser, encode

LEVEL = (1.0, 0.0, 0.0, 0.0)                        # w, x, y, z: no rotation
QUARTER_LEFT = (math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5))  # 90 deg about z


def imu_packet(orientation=LEVEL, gyro=(0.0, 0.0, 0.5), accel=(0.0, 0.0, 9.75)):
    return ImuPacket(0b111111, 7, 1000, orientation, gyro, accel)


def test_packet_reading():
    r = imu.from_packet(imu_packet())
    assert r == imu.Reading(LEVEL, (0.0, 0.0, 0.5), (0.0, 0.0, 9.75))


def test_packet_with_bad_orientation_is_dropped():
    assert imu.from_packet(imu_packet(orientation=(0.0, 0.0, 0.0, 0.0))) is None
    assert imu.from_packet(imu_packet(orientation=(math.nan, 0.0, 0.0, 1.0))) is None
    assert imu.from_packet(imu_packet(gyro=(0.0, math.inf, 0.0))) is None


def test_packet_with_bad_accel_keeps_heading():
    r = imu.from_packet(imu_packet(accel=(math.nan, 0.0, 9.8)))
    assert r.orientation == LEVEL and r.accel is None


def test_stream_main_line():
    r = imu.from_line('I,0.70711,0.00000,0.00000,0.70711,0.0100,-0.0200,0.5000')
    assert r.orientation == (0.70711, 0.0, 0.0, 0.70711)
    assert r.gyro == (0.01, -0.02, 0.5)
    assert r.accel is None


def test_lines_that_are_not_readings():
    assert imu.from_line('# streaming') is None
    assert imu.from_line('01477,0.00000,0.70711,0.0100,-0.0200,0.5000') is None  # tail of a line
    assert imu.from_line('I,1.0,0.0,0.0') is None                                  # cut short
    assert imu.from_line('I,1.0,0.0,0.0,0.0,0.0,0.0,abc') is None
    assert imu.from_line('I,0.5,0.0,0.0,0.0,0.0,0.0,0.0') is None                  # not a unit quaternion
    assert imu.from_line('I,nan,0.0,0.0,1.0,0.0,0.0,0.0') is None


def test_is_line():
    assert imu.is_line('I,1.0,0.0,0.0,0.0,0.0,0.0,0.0')
    assert not imu.is_line('# left sensor at 0x32')


def test_yaw():
    assert imu.yaw_rad(imu.Reading(LEVEL, (0, 0, 0), None)) == 0.0
    assert math.isclose(imu.yaw_rad(imu.Reading(QUARTER_LEFT, (0, 0, 0), None)), math.pi / 2)


def test_both_pico_programs_through_the_parser():
    # What tof_node sees: stream_main.py's text lines and main.py's packets
    # (from either program, depending on what's loaded) come out as items it can sort.
    tof = Packet(1, 0, 3, 2000, 420, tuple(range(protocol.ZONES)))
    stream = (b'# starting\nI,1.00000,0.00000,0.00000,0.00000,0.0000,0.0000,0.2500\n'
              + encode(imu_packet()) + encode(tof))
    parser, items = Parser(3), []
    for i in range(0, len(stream), 5):          # arrives in small chunks
        items += parser.feed(stream[i:i + 5])
    assert items[0] == '# starting'
    assert imu.is_line(items[1]) and imu.from_line(items[1]).gyro == (0.0, 0.0, 0.25)
    assert isinstance(items[2], ImuPacket) and imu.from_packet(items[2]).orientation == LEVEL
    assert items[3] == tof
    assert len(items) == 4


# --- Boards mounted upside down -------------------------------------------

# The IMU reading captured on the robot on 2026-11-10 (that board is chip side down)
ROBOT_PACKET = ImuPacket(35, 53359, 579310,
                         (0.00872802734375, 0.00030517578125, 0.99993896484375, -0.00384521484375),
                         (0.0, 0.0, 0.0), (-0.171875, -0.078125, -9.8671875))


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2)


def about(axis, angle):
    s = math.sin(angle / 2)
    return (math.cos(angle / 2),) + tuple(s * a for a in axis)


def in_board_frame(q, v):
    """A vector given in world axes, in the axes of a board with orientation q."""
    w, x, y, z = q
    return qmul(qmul((w, -x, -y, -z), (0.0,) + tuple(v)), q)[1:]


def board_reading(heading, turn_rate, mount):
    """What a board mounted on a level robot (mount: board -> robot rotation)
    reads while the robot faces `heading` and turns left at `turn_rate` rad/s."""
    q = qmul(about((0, 0, 1), heading), mount)
    return imu.Reading(q, in_board_frame(q, (0.0, 0.0, turn_rate)),
                       in_board_frame(q, (0.0, 0.0, 9.81)))     # standing still: +g, "up"


def heading_change(a, b):
    d = imu.yaw_rad(b) - imu.yaw_rad(a)
    return math.atan2(math.sin(d), math.cos(d))


def test_the_robots_board_reads_upside_down_and_turned_over_reads_level():
    raw = imu.from_packet(ROBOT_PACKET)
    assert imu.upside_down(raw) is True
    fixed = imu.turned_over(raw)
    assert imu.upside_down(fixed) is False
    assert fixed.accel[2] > 9.8
    w, x, y, z = fixed.orientation
    assert 1 - 2 * (x * x + y * y) > 0.999          # its z axis points straight up again
    assert math.isclose(math.sqrt(w * w + x * x + y * y + z * z), 1.0, abs_tol=1e-3)


def test_turning_left_reads_left_once_turned_over():
    # Whichever way the board was turned over (about its x or its y axis)
    for mount in (about((1, 0, 0), math.pi), about((0, 1, 0), math.pi)):
        start = board_reading(0.3, 0.0, mount)
        turning = board_reading(0.8, 0.5, mount)
        assert math.isclose(turning.gyro[2], -0.5, abs_tol=1e-9)   # raw: turn rate backwards
        a, b = imu.turned_over(start), imu.turned_over(turning)
        assert math.isclose(heading_change(a, b), 0.5, abs_tol=1e-9)
        assert math.isclose(b.gyro[2], 0.5, abs_tol=1e-9)
        assert math.isclose(b.accel[2], 9.81, abs_tol=1e-9)


def test_right_way_up_board_needs_nothing():
    level = about((0, 0, 1), 0.0)
    start, turning = board_reading(0.3, 0.0, level), board_reading(0.8, 0.5, level)
    assert imu.upside_down(turning) is False
    assert math.isclose(heading_change(start, turning), 0.5, abs_tol=1e-9)
    assert math.isclose(turning.gyro[2], 0.5, abs_tol=1e-9)


def test_upside_down_needs_gravity():
    assert imu.upside_down(imu.Reading(LEVEL, (0.0, 0.0, 0.0), None)) is None
    assert imu.upside_down(imu.Reading(LEVEL, (0.0, 0.0, 0.0), (9.8, 0.0, 0.3))) is None  # on its side
    assert imu.turned_over(imu.Reading(LEVEL, (0.0, 0.0, 0.1), None)).accel is None
