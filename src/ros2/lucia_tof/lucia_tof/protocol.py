"""Parser for the packet stream sent by the Pico 2 firmware, src/pico2/main.py.

No ROS imports, so it can be used and tested on its own. The packet layout is
documented at the top of main.py; the constants here must match it.
"""
import binascii
import struct
from collections import namedtuple

SYNC = b'\xAA\x55'
PROTO_VERSION = 2
SOURCE_IMU = 0x80
FLAG_NEAR = 0x01
SIDE = 4
ZONES = SIDE * SIDE
INVALID = -1

# sync, version, source, flags, seq, ticks_ms
HEADER_FMT = '<BBBBBHI'
HEADER_LEN = 11

# header + nearest mm
TOF_HEADER_FMT = HEADER_FMT + 'h'
TOF_HEADER_LEN = HEADER_LEN + 2
TOF_PACKET_LEN = TOF_HEADER_LEN + ZONES * 2 + 2

# header + quaternion w x y z, gyro x y z, accel x y z
IMU_FMT = HEADER_FMT + '10f'
IMU_PACKET_LEN = HEADER_LEN + 10 * 4 + 2

# The Pico's time.ticks_ms() wraps at this value.
TICKS_PERIOD_MS = 1 << 30

Packet = namedtuple('Packet', 'sensor flags seq ticks_ms nearest_mm distances')
Packet.__doc__ = """One ToF sensor frame.

distances: ZONES (16) ints in mm, row major, INVALID (-1) for zones with no valid
target. nearest_mm: smallest valid distance, or INVALID if there is none.
"""
Packet.near = property(lambda p: bool(p.flags & FLAG_NEAR))

ImuPacket = namedtuple('ImuPacket', 'flags seq ticks_ms orientation gyro accel')
ImuPacket.__doc__ = """One IMU sample.

orientation: game rotation vector quaternion (w, x, y, z), no magnetometer.
gyro: (x, y, z) rad/s. accel: (x, y, z) m/s^2, gravity included.
ticks_ms: Pico time of the orientation sample.
"""
# Report accuracy 0 (unreliable) .. 3 (high).
ImuPacket.orientation_accuracy = property(lambda p: p.flags & 0x03)
ImuPacket.gyro_accuracy = property(lambda p: (p.flags >> 2) & 0x03)
ImuPacket.accel_accuracy = property(lambda p: (p.flags >> 4) & 0x03)


def crc16(data):
    """CRC-16/CCITT-FALSE, as computed by the Pico."""
    return binascii.crc_hqx(data, 0xFFFF)


def encode(packet):
    """Build the bytes for a Packet or ImuPacket, as the Pico sends them
    (for tests)."""
    if isinstance(packet, ImuPacket):
        body = struct.pack(IMU_FMT, 0xAA, 0x55, PROTO_VERSION, SOURCE_IMU,
                           packet.flags, packet.seq, packet.ticks_ms,
                           *packet.orientation, *packet.gyro, *packet.accel)
    else:
        body = struct.pack(TOF_HEADER_FMT, 0xAA, 0x55, PROTO_VERSION,
                           packet.sensor, packet.flags, packet.seq,
                           packet.ticks_ms, packet.nearest_mm)
        body += struct.pack('<{}h'.format(ZONES), *packet.distances)
    return body + struct.pack('<H', crc16(body[2:]))


class Parser:
    """Splits a raw serial byte stream into packets and text lines.

    feed() returns a list of Packet, ImuPacket and str items in stream order.
    Strings are the Pico's log lines (setup messages, tracebacks) without the
    trailing newline. Corrupted packets are skipped and counted in
    crc_errors.
    """

    def __init__(self, num_sensors):
        self.num_sensors = num_sensors
        self.crc_errors = 0
        self._buf = bytearray()
        self._text = bytearray()
        # After a bad packet, bytes up to the next good one are its remains,
        # not text.
        self._discarding = False

    def _packet_len(self, version, source):
        """Length of the packet a header announces, or None if it is not a
        known packet."""
        if version != PROTO_VERSION:
            return None
        if source < self.num_sensors:
            return TOF_PACKET_LEN
        if source == SOURCE_IMU:
            return IMU_PACKET_LEN
        return None

    def feed(self, data):
        out = []
        buf = self._buf
        buf += data
        while True:
            start = buf.find(SYNC)
            if start < 0:
                # Keep a trailing 0xAA in case it starts the next sync.
                end = len(buf) - (1 if buf.endswith(SYNC[:1]) else 0)
                self._take_text(buf[:end], out)
                del buf[:end]
                break
            self._take_text(buf[:start], out)
            del buf[:start]
            if len(buf) < 4:
                break

            length = self._packet_len(buf[2], buf[3])
            if length is not None:
                if len(buf) < length:
                    break
                pkt = bytes(buf[:length])
                crc, = struct.unpack_from('<H', pkt, length - 2)
                if crc16(pkt[2:-2]) == crc:
                    del buf[:length]
                    self._discarding = False
                    self._text.clear()
                    out.append(self._decode(pkt))
                    continue
                self.crc_errors += 1
            # Not a real packet; skip this sync and resync.
            del buf[:1]
            self._discarding = True
        return out

    @staticmethod
    def _decode(pkt):
        if pkt[3] == SOURCE_IMU:
            (_, _, _, _, flags, seq, ticks,
             *values) = struct.unpack_from(IMU_FMT, pkt)
            return ImuPacket(flags, seq, ticks, tuple(values[0:4]),
                             tuple(values[4:7]), tuple(values[7:10]))
        (_, _, _, sensor, flags, seq, ticks,
         nearest) = struct.unpack_from(TOF_HEADER_FMT, pkt)
        distances = struct.unpack_from('<{}h'.format(ZONES), pkt, TOF_HEADER_LEN)
        return Packet(sensor, flags, seq, ticks, nearest, distances)

    def _take_text(self, chunk, out):
        if self._discarding:
            return
        self._text += chunk
        while b'\n' in self._text:
            line, _, rest = self._text.partition(b'\n')
            self._text = bytearray(rest)
            line = line.decode('utf-8', 'replace').strip()
            # Skip leftovers of corrupted binary packets.
            if line and line.isprintable():
                out.append(line)


class ClockSync:
    """Maps Pico ticks_ms to host time in nanoseconds.

    The host receive time is the Pico time plus a constant offset plus a
    variable USB / scheduling delay. The smallest offset seen over a recent
    window has the least delay, so it is used for every packet. The window
    lets the estimate follow clock drift. Call reset() when the Pico may
    have restarted.
    """

    def __init__(self, window=200):
        self.window = window
        self.reset()

    def reset(self):
        self._last_raw = None
        self._ticks = 0
        self._offsets = []

    def stamp(self, ticks_ms, recv_ns):
        if self._last_raw is None:
            self._ticks = ticks_ms
        else:
            # Signed difference modulo the wrap period, so slightly out of
            # order packets (from different sensors) step back, not forward.
            delta = (ticks_ms - self._last_raw) % TICKS_PERIOD_MS
            if delta >= TICKS_PERIOD_MS // 2:
                delta -= TICKS_PERIOD_MS
            self._ticks += delta
        self._last_raw = ticks_ms

        pico_ns = self._ticks * 1_000_000
        self._offsets.append(recv_ns - pico_ns)
        if len(self._offsets) > self.window:
            del self._offsets[0]
        # Never stamp a frame later than it was received.
        return min(pico_ns + min(self._offsets), recv_ns)
