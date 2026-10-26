from lucia_tof import protocol
from lucia_tof.protocol import ClockSync, ImuPacket, Packet, Parser, encode


def make_packet(sensor=0, seq=0, ticks=1000, flags=0, nearest=500, distances=None):
    if distances is None:
        distances = tuple(500 + i for i in range(protocol.ZONES))
    return Packet(sensor, flags, seq, ticks, nearest, tuple(distances))


def make_imu(seq=0, ticks=1000, flags=0b111111):
    # Values exactly representable as float32, so they round trip unchanged.
    return ImuPacket(flags, seq, ticks, (0.5, -0.5, 0.25, 0.75),
                     (0.125, -1.5, 2.0), (0.0, 0.5, 9.75))


def test_crc_check_value():
    assert protocol.crc16(b'123456789') == 0x29B1


def test_packet_length():
    assert len(encode(make_packet())) == protocol.TOF_PACKET_LEN == 47


def test_round_trip():
    pkt = make_packet(sensor=2, seq=65535, ticks=(1 << 30) - 1, flags=protocol.FLAG_NEAR,
                      nearest=-1, distances=[-1] * protocol.ZONES)
    out = Parser(3).feed(encode(pkt))
    assert out == [pkt]
    assert out[0].near


def test_text_and_corruption_in_odd_chunks():
    good = [make_packet(sensor=i % 3, seq=i) for i in range(5)]
    bad = bytearray(encode(make_packet()))
    bad[30] ^= 0xFF
    stream = b'# ranging started\n' + bytes(bad) + b''.join(encode(p) for p in good)

    parser = Parser(3)
    out = []
    for i in range(0, len(stream), 7):
        out += parser.feed(stream[i:i + 7])
    assert out == ['# ranging started'] + good
    assert parser.crc_errors == 1


def test_imu_packet_length():
    assert len(encode(make_imu())) == protocol.IMU_PACKET_LEN == 53


def test_imu_round_trip():
    pkt = make_imu(seq=65535, ticks=(1 << 30) - 1, flags=0b100111)
    out = Parser(3).feed(encode(pkt))
    assert out == [pkt]
    assert (out[0].orientation_accuracy, out[0].gyro_accuracy,
            out[0].accel_accuracy) == (3, 1, 2)


def test_mixed_tof_and_imu_with_corrupt_imu():
    good = [make_packet(sensor=0), make_imu(seq=1), make_packet(sensor=2), make_imu(seq=2)]
    bad = bytearray(encode(make_imu(seq=9)))
    bad[20] ^= 0xFF
    stream = encode(good[0]) + encode(good[1]) + bytes(bad) + b''.join(encode(p) for p in good[2:])

    parser = Parser(3)
    out = []
    for i in range(0, len(stream), 5):
        out += parser.feed(stream[i:i + 5])
    assert out == good
    assert parser.crc_errors == 1


def test_rejects_old_protocol_version():
    old = bytearray(encode(make_packet()))
    old[2] = 1
    assert Parser(3).feed(bytes(old)) == []


def test_rejects_unknown_sensor():
    assert Parser(2).feed(encode(make_packet(sensor=2))) == []


def test_clock_sync_takes_least_delayed_offset():
    sync = ClockSync()
    base = 10_000_000_000
    # Frames 67 ms apart, received with 1-5 ms of delay.
    stamps = [sync.stamp(t, base + t * 1_000_000 + d * 1_000_000)
              for t, d in [(0, 5), (67, 1), (134, 3), (201, 2)]]
    assert stamps[1:] == [base + t * 1_000_000 + 1_000_000 for t in (67, 134, 201)]


def test_clock_sync_handles_ticks_wrap():
    sync = ClockSync()
    period = protocol.TICKS_PERIOD_MS
    t0 = sync.stamp(period - 10, 0 + 10**12)
    t1 = sync.stamp(20, 30_000_000 + 10**12)  # 30 ms later, after the wrap
    assert t1 - t0 == 30_000_000


def test_clock_sync_never_stamps_after_receive():
    sync = ClockSync()
    sync.stamp(0, 10**12)
    assert sync.stamp(100, 10**12) == 10**12
