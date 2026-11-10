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
