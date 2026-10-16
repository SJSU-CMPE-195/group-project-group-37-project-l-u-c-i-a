"""
imu_line.py

Parses one line of the Pico's IMU stream (src/imu/stream_main.py):

    I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>

Pure Python, no ROS, so it can be unit-tested anywhere.
"""

import math
from collections import namedtuple

ImuReading = namedtuple('ImuReading', 'qw qx qy qz gx gy gz')

PREFIX = 'I,'
FIELD_COUNT = 7

# A unit quaternion has length 1. Anything this far off is a corrupted line.
MAX_QUATERNION_NORM_ERROR = 0.1


def parse_line(line):
    """
    Turn one line from the Pico into an ImuReading.

    Returns None for anything that isn't a valid data line: startup text,
    error messages, half-received lines, or numbers that can't be right.
    """
    line = line.strip()
    if not line.startswith(PREFIX):
        return None

    fields = line[len(PREFIX):].split(',')
    if len(fields) != FIELD_COUNT:
        return None
    try:
        values = [float(f) for f in fields]
    except ValueError:
        return None
    if not all(math.isfinite(v) for v in values):
        return None

    reading = ImuReading(*values)
    norm = math.sqrt(reading.qw ** 2 + reading.qx ** 2 + reading.qy ** 2 + reading.qz ** 2)
    if abs(norm - 1.0) > MAX_QUATERNION_NORM_ERROR:
        return None
    return reading


def yaw_rad(reading):
    """Heading (rotation about the vertical axis) from the quaternion, in radians."""
    w, x, y, z = reading.qw, reading.qx, reading.qy, reading.qz
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
