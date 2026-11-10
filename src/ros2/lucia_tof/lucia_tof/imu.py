"""IMU readings from the sensor Pico, ready to go into a sensor_msgs/Imu.

No ROS imports, so it can be used and tested on its own.

The robot has one Pico 2 for the IMU and the ToF sensors, and tof_node is the
only program that opens it. The Pico sends IMU readings in one of two forms,
depending on which program is loaded on it:

- src/pico2/main.py (ToF + IMU, Noel's): binary IMU packets, which
  protocol.py turns into ImuPacket (orientation, gyro and acceleration).
- src/imu/stream_main.py (IMU only, Ling's): one text line per reading,
  "I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>", without acceleration.

tof_node accepts both, so the robot keeps its heading whichever is loaded.
"""
import math
from collections import namedtuple

Reading = namedtuple('Reading', 'orientation gyro accel')
Reading.__doc__ = """One IMU reading.

orientation: unit quaternion (w, x, y, z), the game rotation vector (gyro +
accelerometer, no magnetometer). gyro: (x, y, z) in rad/s. accel: (x, y, z)
in m/s^2 with gravity, or None when the Pico doesn't send it.
"""

LINE_PREFIX = 'I,'
LINE_FIELDS = 7

# A unit quaternion has length 1. Anything this far off is corrupted or not
# set yet. Same limit as lucia_imu's imu_line.py.
MAX_QUATERNION_NORM_ERROR = 0.1


def _finite(values):
    return all(math.isfinite(v) for v in values)


def _good_orientation(q):
    norm = math.sqrt(sum(v * v for v in q))
    return abs(norm - 1.0) <= MAX_QUATERNION_NORM_ERROR


def from_packet(pkt):
    """protocol.ImuPacket -> Reading, or None if the numbers can't be right."""
    orientation, gyro = tuple(pkt.orientation), tuple(pkt.gyro)
    if not (_finite(orientation + gyro) and _good_orientation(orientation)):
        return None
    accel = tuple(pkt.accel) if _finite(pkt.accel) else None
    return Reading(orientation, gyro, accel)


def is_line(text):
    """True for stream_main.py's data lines, which shouldn't be logged as Pico messages."""
    return text.startswith(LINE_PREFIX)


def from_line(text):
    """One stream_main.py line -> Reading, or None if it isn't a valid data line."""
    text = text.strip()
    if not text.startswith(LINE_PREFIX):
        return None
    fields = text[len(LINE_PREFIX):].split(',')
    if len(fields) != LINE_FIELDS:
        return None
    try:
        values = tuple(float(f) for f in fields)
    except ValueError:
        return None
    orientation, gyro = values[:4], values[4:]
    if not (_finite(values) and _good_orientation(orientation)):
        return None
    return Reading(orientation, gyro, None)


def yaw_rad(reading):
    """Heading (rotation about the vertical axis) from the quaternion, in radians."""
    w, x, y, z = reading.orientation
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


# A board lying flat reads gravity as about +9.8 m/s^2 on its z axis when it's
# the right way up, -9.8 when it's upside down. Anything nearer 0 than this
# (the board on its side) can't tell.
GRAVITY_CHECK = 5.0


def upside_down(reading):
    """True / False from the reading's gravity, None if it can't tell (no
    acceleration in it, or the board isn't lying flat)."""
    if reading.accel is None:
        return None
    az = reading.accel[2]
    if az < -GRAVITY_CHECK:
        return True
    if az > GRAVITY_CHECK:
        return False
    return None


def turned_over(reading):
    """A reading from a board mounted upside down (chip side down), as the same
    board would give it the right way up: turned 180 degrees about its x axis.

    Orientation: q * (0, 1, 0, 0), i.e. (w, x, y, z) -> (-x, w, z, -y). Turn
    rates and accelerations: y and z change sign. On the floor only the heading
    and the turn rate are used, and those come out right whichever way the
    board was turned over: a different way only adds a fixed offset to the
    heading, and the EKF counts the starting heading as 0 anyway.
    """
    w, x, y, z = reading.orientation
    gx, gy, gz = reading.gyro
    accel = reading.accel
    if accel is not None:
        accel = (accel[0], -accel[1], -accel[2])
    return Reading((-x, w, z, -y), (gx, -gy, -gz), accel)
