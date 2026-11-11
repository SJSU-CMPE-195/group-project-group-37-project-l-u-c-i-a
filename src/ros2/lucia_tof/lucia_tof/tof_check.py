"""tof_check: read the sensor Pico for a few seconds and show what the ToF
sensors see. No ROS needed; stop mapping first (tof_node holds the port while
mapping runs).

    ros2 run lucia_tof tof_check                  # 10 s on /dev/imu
    ros2 run lucia_tof tof_check --seconds 20

Twice a second: each sensor's nearest distance, and NEAR the way tof_node
decides it (near_filter.py). At the end: frames per second, readings too close
to be real (glitches the filter drops), each sensor's 4x4 grid in mm (-1 = no
reading, row 0 at the top), the IMU's rate and the Pico's own messages.

Hand test: hold a hand about 15 cm in front of one sensor at a time; only that
sensor should drop and say NEAR. Names are the robot's own left / middle /
right (checked on the robot 2026-11-10).
"""
import argparse
import os
import time

from lucia_tof import protocol
from lucia_tof.near_filter import COVERED_ZONES, INVALID, NearFilter, too_close

NAMES = ['left', 'middle', 'right']


def _show(nearest_mm, near):
    if nearest_mm == INVALID:
        text = 'nothing'
    else:
        text = '{} mm'.format(nearest_mm)
    return text + (' NEAR' if near else '')


def check(read, seconds, clock=time.monotonic, out=print, min_mm=20):
    """Read packets with read() (bytes) for `seconds`; print as it goes and a
    summary. Returns the counts, for tests."""
    parser = protocol.Parser(len(NAMES))
    filters = [NearFilter(min_mm=min_mm) for _ in NAMES]
    frames = [0] * len(NAMES)
    glitches = [0] * len(NAMES)
    last = [None] * len(NAMES)          # (packet, nearest_mm, near)
    imu = 0
    texts = []
    start = tick = clock()
    while clock() - start < seconds:
        for item in parser.feed(read()):
            if isinstance(item, protocol.Packet):
                i = item.sensor
                frames[i] += 1
                if 0 < too_close(item.distances, min_mm) < COVERED_ZONES:
                    glitches[i] += 1
                last[i] = (item,) + filters[i].update(item.distances)
            elif isinstance(item, protocol.ImuPacket):
                imu += 1
            else:
                texts.append(item)
        if clock() - tick >= 0.5:
            tick += 0.5
            out('   '.join('{:>6}: {:14}'.format(n, '-' if l is None else _show(l[1], l[2]))
                           for n, l in zip(NAMES, last)))
    elapsed = max(clock() - start, 1e-9)
    out('')
    for n, f, g, l in zip(NAMES, frames, glitches, last):
        line = '{}: {:.0f} frames/s'.format(n, f / elapsed)
        if l is None:
            out(line + '  <- nothing from this sensor')
            continue
        valid = sum(1 for d in l[0].distances if d >= min_mm)
        out(line + ', {} of 16 zones see something, {} frames with a glitch'.format(valid, g))
        for r in range(protocol.SIDE):
            row = l[0].distances[protocol.SIDE * r:protocol.SIDE * (r + 1)]
            out('   ' + ' '.join('{:6}'.format(d) for d in row))
    out('IMU: {:.0f} readings/s   damaged packets: {}'.format(imu / elapsed, parser.crc_errors))
    for t in texts[:10]:
        out('pico says: ' + t)
    return {'frames': frames, 'glitches': glitches, 'imu': imu}


def tof_node_running(proc='/proc'):
    """True if tof_node (mapping) is running: it would share the port, and each
    program would get only part of the data."""
    try:
        pids = [p for p in os.listdir(proc) if p.isdigit()]
    except OSError:
        return False
    for pid in pids:
        try:
            with open(os.path.join(proc, pid, 'cmdline'), 'rb') as f:
                if b'tof_node' in f.read():
                    return True
        except OSError:
            pass
    return False


def main(argv=None):
    ap = argparse.ArgumentParser(description='Show what the ToF sensors see (stop mapping first).')
    ap.add_argument('--port', default='/dev/imu', help='the sensor Pico (default /dev/imu)')
    ap.add_argument('--seconds', type=float, default=10.0)
    args = ap.parse_args(argv)

    if tof_node_running():
        print('Mapping is running (tof_node has the Pico). Stop mapping on the page first.')
        return 1
    import serial
    try:
        port = serial.Serial(args.port, 115200, timeout=0.1)
    except serial.SerialException as e:
        print('Cannot open {}: {}'.format(args.port, e))
        print('Is the Pico plugged in? (ls -la /dev/imu)')
        return 1
    try:
        check(lambda: port.read(port.in_waiting or 1), args.seconds)
    except KeyboardInterrupt:
        pass
    finally:
        port.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
