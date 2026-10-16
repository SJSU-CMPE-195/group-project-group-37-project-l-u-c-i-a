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
