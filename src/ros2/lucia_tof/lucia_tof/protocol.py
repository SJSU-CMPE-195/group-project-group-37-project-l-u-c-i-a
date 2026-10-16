"""Parser for the ToF packet stream sent by src/tof_pico/heatmap/pico_stream.py.

No ROS imports, so it can be used and tested on its own. The packet layout is
documented at the top of pico_stream.py; the constants here must match it.
"""
import binascii
import struct
from collections import namedtuple

SYNC = b'\xAA\x55'
PROTO_VERSION = 1
FLAG_NEAR = 0x01
SIDE = 8
ZONES = SIDE * SIDE
INVALID = -1

# sync, version, sensor index, flags, seq, ticks_ms, nearest mm
HEADER_FMT = '<BBBBBHIh'
HEADER_LEN = 13
PACKET_LEN = HEADER_LEN + ZONES * 2 + 2

# The Pico's time.ticks_ms() wraps at this value.
TICKS_PERIOD_MS = 1 << 30

Packet = namedtuple('Packet', 'sensor flags seq ticks_ms nearest_mm distances')
Packet.__doc__ = """One sensor frame.

distances: 64 ints in mm, row major, INVALID (-1) for zones with no valid
target. nearest_mm: smallest valid distance, or INVALID if there is none.
"""
Packet.near = property(lambda p: bool(p.flags & FLAG_NEAR))


def crc16(data):
    """CRC-16/CCITT-FALSE, as computed by the Pico."""
    return binascii.crc_hqx(data, 0xFFFF)


def encode(packet):
    """Build the bytes for a packet, as the Pico sends them (for tests)."""
    body = struct.pack(HEADER_FMT, 0xAA, 0x55, PROTO_VERSION, packet.sensor,
                       packet.flags, packet.seq, packet.ticks_ms,
                       packet.nearest_mm)
    body += struct.pack('<64h', *packet.distances)
    return body + struct.pack('<H', crc16(body[2:]))


class Parser:
    """Splits a raw serial byte stream into packets and text lines.

    feed() returns a list of Packet and str items in stream order. Strings
    are the Pico's log lines (setup messages, tracebacks) without the
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
            if len(buf) < PACKET_LEN:
                break

            pkt = bytes(buf[:PACKET_LEN])
            (_, _, version, sensor, flags, seq, ticks,
             nearest) = struct.unpack_from(HEADER_FMT, pkt)
            crc, = struct.unpack_from('<H', pkt, PACKET_LEN - 2)
            if (version != PROTO_VERSION or sensor >= self.num_sensors
                    or crc16(pkt[2:PACKET_LEN - 2]) != crc):
                # Not a real packet; skip this sync and resync.
                if version == PROTO_VERSION:
                    self.crc_errors += 1
                del buf[:1]
                self._discarding = True
                continue
            del buf[:PACKET_LEN]
            self._discarding = False
            self._text.clear()
            distances = struct.unpack_from('<64h', pkt, HEADER_LEN)
            out.append(Packet(sensor, flags, seq, ticks, nearest, distances))
        return out

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
