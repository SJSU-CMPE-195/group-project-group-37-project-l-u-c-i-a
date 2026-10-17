"""
motion_check.py

Is the robot really moving the way its wheels were told to? Answered from the
lidar and the IMU, not the wheels: /odom is open-loop (commanded speed, see
TODO.md Phase 1 Step B), so it keeps counting distance while the robot is
stuck, and so does Nav2's own progress check, which watches /odom.

- Turning: the IMU's heading should change about as much as the commanded turn.
- Driving: two lidar scans about a second apart should line up once the newer
  one is shifted by the distance actually driven. A few shifts between 0
  (didn't move) and a bit more than the commanded distance are tried; the one
  that fits best is how far it really went. When the room gives no clue (a long,
  bare hallway, where every shift fits about as well) the answer is "can't
  tell", never "stuck".

Several "not moving" checks in a row while the wheels are being driven = stuck.
That catches what the lidar can't see (a low ledge, a cable, a chair leg under
the lidar's height), wheels spinning in place, and the Roomba stopping itself
(Safe mode: a cliff sensor or wheel drop).

Pure Python (no numpy, no ROS) so it's tested on any machine. mission.py feeds
it /scan, /odom and /imu/data and runs check() twice a second.
"""

import math
from collections import deque

FRACTIONS = (0.0, 0.25, 0.5, 0.75, 1.0, 1.25)   # shifts tried, as a share of the commanded distance


# ----------------------------------------------------------------------------
# Scans
# ----------------------------------------------------------------------------

def scan_to_points(ranges, angle_min, angle_increment, sensor_pose=(0.0, 0.0, 0.0),
                   min_range=0.0, max_range=float('inf'), max_points=360, max_spread=None):
    """
    Lidar ranges -> (x, y) points in the robot's frame (base_link), in scan
    order. sensor_pose is where the lidar sits on the robot (x, y, yaw). Only
    readings between min_range and max_range are kept, and at most about
    max_points of them (every n-th reading).

    max_spread (for lining scans up): also drop points whose nearest neighbour
    in the scan is more than max_spread times as far away as on a wall facing
    the lidar head-on. That's a wall seen at a shallow angle (beams far apart
    along it, so a scan that didn't move fits itself best just because the beams
    land in the same spots again), or a stray point.
    """
    sx, sy, syaw = sensor_pose
    stride = max(1, -(-len(ranges) // max_points))
    kept = []                                   # (beam index, range, x, y)
    for i in range(0, len(ranges), stride):
        r = ranges[i]
        if not (math.isfinite(r) and min_range <= r <= max_range):
            continue
        a = syaw + angle_min + i * angle_increment
        kept.append((i, r, sx + r * math.cos(a), sy + r * math.sin(a)))
    if max_spread is None:
        return [(x, y) for _, _, x, y in kept]

    points = []
    beam_gap = abs(stride * angle_increment)
    for k, (i, r, x, y) in enumerate(kept):
        nearest = math.inf
        for n in (k - 1, k + 1):
            if 0 <= n < len(kept) and abs(kept[n][0] - i) == stride:
                nearest = min(nearest, math.hypot(kept[n][2] - x, kept[n][3] - y))
        if nearest <= max_spread * r * beam_gap:
            points.append((x, y))
    return points


def densify(points, max_gap=0.2, step=0.02):
    """
    Fill in neighbouring scan points (closer than max_gap, so the same wall)
    every `step` meters. Lining a scan up against a solid outline instead of
    separate dots stops a scan from fitting itself best just because the
    lidar's beams hit the same spots again.
    """
    out = []
    for i, (x, y) in enumerate(points):
        out.append((x, y))
        if i + 1 < len(points):
            nx, ny = points[i + 1]
            gap = math.hypot(nx - x, ny - y)
            if step < gap < max_gap:
                n = int(gap / step)
                for k in range(1, n):
                    t = k / n
                    out.append((x + t * (nx - x), y + t * (ny - y)))
    return out


class _PointGrid:
    """Points bucketed into square cells, for quick nearest-point lookups up to one cell away."""

    def __init__(self, points, cell_m):
        self.cell = cell_m
        self.cells = {}
        for x, y in points:
            self.cells.setdefault((math.floor(x / cell_m), math.floor(y / cell_m)), []).append((x, y))

    def distance(self, x, y, cap):
        """Distance to the nearest point, or cap if none is that close (cap <= cell size)."""
        cx, cy = math.floor(x / self.cell), math.floor(y / self.cell)
        best = cap * cap
        for i in (cx - 1, cx, cx + 1):
            for j in (cy - 1, cy, cy + 1):
                for px, py in self.cells.get((i, j), ()):
                    d = (px - x) ** 2 + (py - y) ** 2
                    if d < best:
                        best = d
        return math.sqrt(best)


def fit_error(grid, points, dx, dy, dyaw, cap):
    """
    Mean distance (each capped at `cap`) from `points`, moved by (dx, dy, dyaw),
    to the nearest point in `grid`. Low = the two scans line up.
    """
    c, s = math.cos(dyaw), math.sin(dyaw)
    total = 0.0
    for x, y in points:
        total += grid.distance(c * x - s * y + dx, s * x + c * y + dy, cap)
    return total / len(points)


def driven_fraction(ref_points, cur_points, commanded_xy, turned_rad,
                    cap=0.1, min_contrast=0.012, min_points=30):
    """
    How much of the commanded distance the robot really drove between two scans.

    ref_points, cur_points: the two scans, each in the robot's frame when it was taken
    commanded_xy: where the wheels were told to take it, in the first scan's frame
    turned_rad:   how far it really turned meanwhile (IMU; else the commanded turn)

    Returns (fraction, errors): the best fit among FRACTIONS (0 = didn't move,
    1 = drove the full commanded distance), or None when the scans can't tell
    (too few points, or every shift fits about as well: the best and worst fit
    differ by less than min_contrast meters). errors maps each tried fraction to
    its fit error.
    """
    if len(ref_points) < min_points or len(cur_points) < min_points:
        return None, {}
    grid = _PointGrid(densify(ref_points), cap)
    cx, cy = commanded_xy
    errors = {f: fit_error(grid, cur_points, f * cx, f * cy, turned_rad, cap) for f in FRACTIONS}
    best = min(errors, key=errors.get)
    if max(errors.values()) - errors[best] < min_contrast:
        return None, errors
    return best, errors


def clear_behind(points, robot_radius, distance, margin=0.05):
    """
    True if nothing the lidar sees is in the strip the robot would sweep
    backing up `distance` meters. Points inside the robot's own outline (its
    arm, if the lidar sees it) don't count.
    """
    half_width = robot_radius + margin
    reach = robot_radius + distance + margin
    for x, y in points:
        if math.hypot(x, y) <= robot_radius:
            continue
        if -reach <= x <= 0.0 and abs(y) <= half_width:
            return False
    return True


# ----------------------------------------------------------------------------
# The running check
# ----------------------------------------------------------------------------

def wrap_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class _Unwrapper:
    """Turns a heading that jumps from +pi to -pi into one that keeps counting (rad)."""

    def __init__(self):
        self._last = None
        self.total = 0.0

    def add(self, yaw):
        if self._last is not None:
            self.total += wrap_angle(yaw - self._last)
        self._last = yaw
        return self.total


class _Snapshot:
    __slots__ = ('t', 'points', 'x', 'y', 'yaw', 'imu_yaw')

    def __init__(self, t, points, x, y, yaw, imu_yaw):
        self.t, self.points, self.x, self.y, self.yaw, self.imu_yaw = t, points, x, y, yaw, imu_yaw


class MotionCheck:
    """
    Keeps the last few seconds of scans with the commanded position (/odom) and
    the IMU heading at each, and answers "is it moving as commanded?".

    Feed it with odom(), imu() and scan(); call check() a couple of times a
    second. stuck turns true after `checks_to_stuck` "not moving" answers in a
    row while the wheels were being driven; reset() starts over (after
    backing out, say).
    """

    def __init__(self, window_s=3.0, min_travel_m=0.12, min_turn_rad=0.35,
                 still_fraction=0.3, checks_to_stuck=3, imu_fresh_s=0.5):
        self.window_s = window_s
        self.min_travel_m = min_travel_m
        self.min_turn_rad = min_turn_rad
        self.still_fraction = still_fraction
        self.checks_to_stuck = checks_to_stuck
        self.imu_fresh_s = imu_fresh_s

        self._history = deque()
        self._odom = None               # (x, y, unwrapped yaw)
        self._odom_yaw = _Unwrapper()
        self._imu = None                # (t, unwrapped yaw)
        self._imu_yaw = _Unwrapper()
        self.still_checks = 0
        self.last = {'moving': None, 'driven': None, 'turned': None}

    @property
    def stuck(self):
        return self.still_checks >= self.checks_to_stuck

    def reset(self):
        self._history.clear()
        self.still_checks = 0
        self.last = {'moving': None, 'driven': None, 'turned': None}

    def odom(self, x, y, yaw):
        """Where /odom (the commanded motion) says the robot is."""
        self._odom = (x, y, self._odom_yaw.add(yaw))

    def imu(self, t, yaw):
        self._imu = (t, self._imu_yaw.add(yaw))

    def scan(self, t, points):
        """A new scan (points in the robot's frame), stamped with the latest odom and IMU."""
        if self._odom is None:
            return
        imu_yaw = None
        if self._imu is not None and t - self._imu[0] <= self.imu_fresh_s:
            imu_yaw = self._imu[1]
        self._history.append(_Snapshot(t, points, *self._odom, imu_yaw))
        while self._history and t - self._history[0].t > self.window_s:
            self._history.popleft()

    def check(self):
        """
        Compare the newest scan with older ones. Returns True if the robot is
        moving as commanded, False if it isn't, None if there's nothing to go
        on (the wheels weren't driven, or the room gives no clue). Also updates
        stuck, and `last` (what the web page shows).
        """
        if len(self._history) < 2:
            return None
        cur = self._history[-1]
        driven, commanded = self._check_distance(cur)
        turned, commanded_turn = self._check_turn(cur)
        self.last = {'moving': None, 'driven': driven, 'turned': turned}
        if not (commanded or commanded_turn):
            self.still_checks = 0           # not being driven: can't be stuck
            return None
        results = [r for r in (driven, turned) if r is not None]
        if not results:
            return None                     # can't tell: keep the count as it is
        moving = all(r >= self.still_fraction for r in results)
        self.still_checks = 0 if moving else self.still_checks + 1
        self.last['moving'] = moving
        return moving

    def _check_distance(self, cur):
        """(fraction of the commanded distance driven or None, whether enough was commanded)."""
        for ref in reversed(self._history):
            if ref is cur:
                continue
            # Commanded motion since ref, in the robot's frame at ref
            dx, dy = cur.x - ref.x, cur.y - ref.y
            if math.hypot(dx, dy) < self.min_travel_m:
                continue
            c, s = math.cos(-ref.yaw), math.sin(-ref.yaw)
            commanded = (c * dx - s * dy, s * dx + c * dy)
            if ref.imu_yaw is not None and cur.imu_yaw is not None:
                turned = cur.imu_yaw - ref.imu_yaw
            else:
                turned = cur.yaw - ref.yaw
            fraction, _ = driven_fraction(ref.points, cur.points, commanded, turned)
            return fraction, True
        return None, False

    def _check_turn(self, cur):
        """(fraction of the commanded turn the IMU saw or None, whether enough turning was commanded)."""
        if cur.imu_yaw is None:
            return None, False
        for ref in reversed(self._history):
            if ref is cur or ref.imu_yaw is None:
                continue
            commanded = cur.yaw - ref.yaw
            if abs(commanded) < self.min_turn_rad:
                continue
            return (cur.imu_yaw - ref.imu_yaw) / commanded, True
        return None, False
