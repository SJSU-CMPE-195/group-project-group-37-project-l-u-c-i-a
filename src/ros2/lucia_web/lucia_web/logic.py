"""
logic.py

Pure helpers for the web UI, kept free of ROS so they can be tested anywhere.
"""

import math
import os
import re
from collections import deque

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


DRIVING_MODES = ('explore', 'home', 'goto')     # mission modes where it drives itself


class AutoResume:
    """
    Driving by hand while the robot drives itself pauses the mission (lucia_nav's
    'pause'); once nobody has driven for after_s seconds it carries on
    ('resume'). STOP or any other mission command from the page calls cancel().
    after_s <= 0: never carry on by itself (press Explore again).
    """

    def __init__(self, after_s):
        self.after_s = after_s
        self.at = None              # when to send resume (time.monotonic())

    def drive(self, mission_mode, moving, now):
        """
        A drive command from the page. Returns 'pause' (pause the mission, then
        drive), 'ignore' (drop this command: a key let go while the mission
        drives) or None (just drive).
        """
        if mission_mode in DRIVING_MODES:
            if not moving:
                return 'ignore'
            self._restart(now)
            return 'pause'
        if mission_mode == 'paused' and moving:
            self._restart(now)
        return None

    def cancel(self):
        self.at = None

    def due(self, mission_mode, now):
        """True once, when it's time to send resume. Forgets it if the mission isn't paused any more."""
        if self.at is None or now < self.at:
            return False
        self.at = None
        return mission_mode == 'paused'

    def seconds_left(self, now):
        return None if self.at is None else max(0.0, self.at - now)

    def _restart(self, now):
        self.at = now + self.after_s if self.after_s > 0 else None


def thin_path(points, max_points=120):
    """Every n-th (x, y) of a planned path so at most max_points are sent; always keeps the end."""
    points = list(points)
    if len(points) <= max_points:
        return points
    step = (len(points) - 1) / (max_points - 1)
    return [points[round(i * step)] for i in range(max_points)]


# ----------------------------------------------------------------------
# Metrics for the page's sidebar
# ----------------------------------------------------------------------

FREE_MAX = 25               # OccupancyGrid values 0..25 count as open floor (as in map_saver_cli)
LOG_LEVELS = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


class RateMeter:
    """How many messages per second arrived over the last window_s seconds."""

    def __init__(self, window_s=2.0):
        self._window_s = window_s
        self._times = deque()

    def tick(self, now):
        self._times.append(now)
        self._trim(now)

    def hz(self, now):
        self._trim(now)
        return len(self._times) / self._window_s

    def _trim(self, now):
        while self._times and now - self._times[0] > self._window_s:
            self._times.popleft()


class Trail:
    """
    Where the robot has been (map frame), how far it has driven, and how fast
    it's going over the ground.

    A point is added when the robot is at least min_step_m from the last one.
    Steps faster than max_speed_mps aren't counted as driving: those are
    slam_toolbox correcting the robot's position, not the robot moving.
    Over max_points, every other point is dropped; that, like reset(), changes
    `session`, which tells the page to fetch the whole trail again.
    """

    def __init__(self, min_step_m=0.05, max_speed_mps=1.0, max_points=3000):
        self.min_step_m = min_step_m
        self.max_speed_mps = max_speed_mps
        self.max_points = max_points
        self._run = os.urandom(3).hex()     # differs after a web_ui restart
        self._count = 0
        self.reset()

    def reset(self):
        self._count += 1
        self.session = f'{self._run}-{self._count}'
        self.points = []
        self.distance_m = 0.0
        self._last = None                   # (x, y, t) of the last point added
        self._recent = deque(maxlen=8)      # (x, y, t) of the last few positions, for speed

    def add(self, x, y, t):
        self._recent.append((x, y, t))
        if self._last is None:
            self.points.append((x, y))
            self._last = (x, y, t)
            return
        lx, ly, lt = self._last
        step = math.hypot(x - lx, y - ly)
        if step < self.min_step_m:
            return
        if t > lt and step / (t - lt) <= self.max_speed_mps:
            self.distance_m += step
        self.points.append((x, y))
        self._last = (x, y, t)
        if len(self.points) > self.max_points:
            self.points = self.points[::2] + ([self.points[-1]] if len(self.points) % 2 == 0 else [])
            self._count += 1
            self.session = f'{self._run}-{self._count}'

    def ground_speed(self):
        """m/s over the last second or so of positions; None if unknown or a position jump."""
        if len(self._recent) < 2:
            return None
        dt = self._recent[-1][2] - self._recent[0][2]
        if dt < 0.5:
            return None
        pts = list(self._recent)
        length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        speed = length / dt
        return speed if speed <= self.max_speed_mps else None


def mapped_area_m2(data, resolution):
    """Open floor in the map, in square meters."""
    return sum(1 for v in data if 0 <= v <= FREE_MAX) * resolution * resolution


def parse_temp(text):
    """CPU temperature in deg C from /sys/class/thermal/thermal_zone0/temp (millidegrees)."""
    try:
        return int(text.strip()) / 1000.0
    except (AttributeError, ValueError):
        return None


def parse_loadavg(text):
    """1-minute load average from /proc/loadavg."""
    try:
        return float(text.split()[0])
    except (AttributeError, IndexError, ValueError):
        return None


def parse_meminfo(text):
    """Memory in use, in percent, from /proc/meminfo."""
    try:
        values = {}
        for line in text.splitlines():
            key, _, rest = line.partition(':')
            if key in ('MemTotal', 'MemAvailable'):
                values[key] = int(rest.split()[0])
        total, available = values['MemTotal'], values['MemAvailable']
        return 100.0 * (total - available) / total
    except (AttributeError, KeyError, IndexError, ValueError, ZeroDivisionError):
        return None


def cpu_times(text):
    """(idle, total) jiffies from the first line of /proc/stat."""
    try:
        line = text.splitlines()[0].split()
        if line[0] != 'cpu':
            return None
        fields = [int(v) for v in line[1:9]]
        return fields[3] + fields[4], sum(fields)      # idle + iowait, everything
    except (AttributeError, IndexError, ValueError):
        return None


def cpu_percent(previous, current):
    """CPU in use, in percent, between two cpu_times() readings."""
    if previous is None or current is None:
        return None
    idle = current[0] - previous[0]
    total = current[1] - previous[1]
    if total <= 0:
        return None
    return 100.0 * (1.0 - idle / total)


def log_level_name(level):
    """rcl_interfaces/msg/Log level number as a word."""
    return LOG_LEVELS.get(level, str(level))
