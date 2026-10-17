"""
logic.py

Pure helpers for the web UI, kept free of ROS so they can be tested anywhere.
"""

import math
import re

MAP_NAME_PATTERN = re.compile(r'^[A-Za-z0-9_-]{1,40}$')

# How old the last message on a topic can be before its status light changes.
FRESH_S = 1.0
STALE_S = 5.0


def drive_to_twist(linear_frac, angular_frac, max_linear_mps, max_angular_rad_s):
    """
    Turn the drive pad's -1..1 values into (linear m/s, angular rad/s).

    Values outside -1..1 are clamped; NaN/inf (or anything that isn't a
    number) means stop.
    """
    try:
        linear_frac = float(linear_frac)
        angular_frac = float(angular_frac)
    except (TypeError, ValueError):
        return 0.0, 0.0
    if not (math.isfinite(linear_frac) and math.isfinite(angular_frac)):
        return 0.0, 0.0
    linear_frac = max(-1.0, min(1.0, linear_frac))
    angular_frac = max(-1.0, min(1.0, angular_frac))
    return linear_frac * max_linear_mps, angular_frac * max_angular_rad_s


def valid_map_name(name):
    """Map names become file names: letters, digits, '-' and '_' only."""
    return isinstance(name, str) and bool(MAP_NAME_PATTERN.match(name))


def topic_status(age_s):
    """'ok', 'stale' or 'missing' for a topic whose last message is age_s old (None = never)."""
    if age_s is None or age_s > STALE_S:
        return 'missing'
    if age_s > FRESH_S:
        return 'stale'
    return 'ok'


def encode_grid(data):
    """
    OccupancyGrid data (-1 unknown, 0..100 occupied %) as one byte per cell,
    with unknown as 255, so the browser can unpack it from base64.
    """
    return bytes(255 if v < 0 else min(int(v), 100) for v in data)


def yaw_from_quaternion(x, y, z, w):
    """Rotation about the vertical axis, in radians."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def clamp_speed(value, lowest, highest):
    """A speed asked for by the page, in m/s, kept within lowest..highest. None if it isn't a number."""
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if not math.isfinite(value):
        return None
    return max(lowest, min(highest, value))


def scan_to_radar(ranges, angle_min, angle_increment, range_min, range_max,
                  yaw_offset=0.0, slices=180):
    """
    Squeeze a LaserScan into `slices` equal slices around the robot for the
    radar view, keeping the nearest reading in each, in whole centimeters
    (0 = no reading in that slice).

    Slice 0 is centered straight ahead and the rest go counterclockwise (to the
    robot's left), the ROS convention. yaw_offset is how the lidar is turned on
    the robot (the base_link->laser yaw, pi on LUCIA), so slices are relative
    to the robot, not the lidar. Readings outside range_min..range_max, NaN
    and inf count as no reading.
    """
    out = [0] * slices
    width = 2 * math.pi / slices
    for i, r in enumerate(ranges):
        if not (math.isfinite(r) and range_min <= r <= range_max):
            continue
        angle = angle_min + i * angle_increment + yaw_offset
        k = int(math.floor(angle / width + 0.5)) % slices
        cm = max(1, int(round(r * 100)))
        if out[k] == 0 or cm < out[k]:
            out[k] = cm
    return out


def radar_nearest(slices_cm):
    """
    Closest reading in scan_to_radar() output, as {'m': meters, 'deg': bearing},
    bearing in -180..180 with positive = to the left. None if nothing was seen.
    """
    best = None
    for k, cm in enumerate(slices_cm):
        if cm and (best is None or cm < slices_cm[best]):
            best = k
    if best is None:
        return None
    deg = best * 360.0 / len(slices_cm)
    if deg > 180:
        deg -= 360
    return {'m': slices_cm[best] / 100.0, 'deg': deg}


MISSION_WORDS = ('explore', 'home', 'stop')
MAP_LIMIT_M = 1000.0        # sanity bound for a clicked goal


def mission_text(body):
    """
    The /mission/command text (see lucia_nav/commands.py) for a page request
    like {'cmd': 'explore'} or {'cmd': 'goto', 'x': 1.2, 'y': -0.5}. None if
    the request isn't valid.
    """
    if not isinstance(body, dict):
        return None
    cmd = body.get('cmd')
    if cmd in MISSION_WORDS:
        return cmd
    if cmd == 'goto':
        try:
            x, y = float(body.get('x')), float(body.get('y'))
        except (TypeError, ValueError):
            return None
        if all(math.isfinite(v) and abs(v) < MAP_LIMIT_M for v in (x, y)):
            return f'goto {x:.3f} {y:.3f}'
    return None


def thin_path(points, max_points=120):
    """Every n-th (x, y) of a planned path so at most max_points are sent; always keeps the end."""
    points = list(points)
    if len(points) <= max_points:
        return points
    step = (len(points) - 1) / (max_points - 1)
    return [points[round(i * step)] for i in range(max_points)]
