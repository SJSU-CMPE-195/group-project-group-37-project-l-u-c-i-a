"""Tests for motion_check.py. Run from src/ros2/lucia_nav:  python3 -m pytest test"""

import math
import random
import time

import pytest

from lucia_nav.motion_check import (
    MotionCheck, clear_behind, driven_fraction, scan_to_points, yaw_from_quaternion)

# A 4 x 3 m room with a desk and a box in it (wall segments, meters)
ROOM = [
    ((0, 0), (4, 0)), ((4, 0), (4, 3)), ((4, 3), (0, 3)), ((0, 3), (0, 0)),
    ((2.5, 2.0), (3.3, 2.0)), ((3.3, 2.0), (3.3, 2.6)), ((3.3, 2.6), (2.5, 2.6)), ((2.5, 2.6), (2.5, 2.0)),
    ((0.6, 2.2), (1.0, 2.2)), ((1.0, 2.2), (1.0, 2.6)),
]
# A long, bare hallway: both ends are past the lidar's 12 m range
HALLWAY = [((-30, -0.6), (30, -0.6)), ((-30, 0.6), (30, 0.6))]

BEAMS = 360
ANGLE_MIN = -math.pi
ANGLE_INC = 2 * math.pi / BEAMS


def cast(walls, pose, noise=0.01, max_range=12.0, rng=None):
    """Pretend lidar at pose (x, y, yaw): one range per beam, inf when nothing is hit."""
    rng = rng or random.Random(1)
    px, py, pyaw = pose
    ranges = []
    for i in range(BEAMS):
        a = pyaw + ANGLE_MIN + i * ANGLE_INC
        dx, dy = math.cos(a), math.sin(a)
        best = math.inf
        for (x1, y1), (x2, y2) in walls:
            ex, ey = x2 - x1, y2 - y1
            denom = dx * ey - dy * ex
            if abs(denom) < 1e-12:
                continue
            t = ((x1 - px) * ey - (y1 - py) * ex) / denom      # along the beam
            u = ((x1 - px) * dy - (y1 - py) * dx) / denom      # along the wall
            if t > 0 and 0 <= u <= 1:
                best = min(best, t)
        ranges.append(best + rng.gauss(0, noise) if best <= max_range else math.inf)
    return ranges


def points(walls, pose, rng=None, noise=0.01):
    # The settings mission.py uses
    return scan_to_points(cast(walls, pose, noise, rng=rng), ANGLE_MIN, ANGLE_INC,
                          min_range=0.3, max_range=4.0, max_spread=2.5)


def ahead(pose, d, turn=0.0):
    x, y, yaw = pose
    return (x + d * math.cos(yaw), y + d * math.sin(yaw), yaw + turn)


START = (1.2, 1.0, 0.4)


# ---- driven_fraction -----------------------------------------------------

@pytest.mark.parametrize('really_drove, expected', [(0.15, 1.0), (0.0, 0.0), (0.075, 0.5)])
def test_finds_how_far_it_really_drove(really_drove, expected):
    rng = random.Random(2)
    ref = points(ROOM, START, rng)
    cur = points(ROOM, ahead(START, really_drove), rng)
    fraction, _ = driven_fraction(ref, cur, (0.15, 0.0), 0.0)
    assert fraction == expected


def test_drove_while_turning():
    rng = random.Random(3)
    end = (START[0] + 0.15 * math.cos(START[2] + 0.1), START[1] + 0.15 * math.sin(START[2] + 0.1), START[2] + 0.2)
    ref, cur = points(ROOM, START, rng), points(ROOM, end, rng)
    commanded = (0.15 * math.cos(0.1), 0.15 * math.sin(0.1))
    assert driven_fraction(ref, cur, commanded, 0.2)[0] == 1.0
    # Same turn but stuck in place (the IMU says it turned, the wheels didn't get it anywhere)
    cur_stuck = points(ROOM, (START[0], START[1], START[2] + 0.2), rng)
    assert driven_fraction(ref, cur_stuck, commanded, 0.2)[0] == 0.0


def test_bare_hallway_cant_tell():
    rng = random.Random(4)
    ref = points(HALLWAY, (0, 0, 0), rng)
    for really_drove in (0.0, 0.15):
        cur = points(HALLWAY, (really_drove, 0, 0), rng)
        assert driven_fraction(ref, cur, (0.15, 0.0), 0.0)[0] is None


def test_bare_hallway_with_noisy_lidar_never_says_it_stopped():
    # Driving down it, slightly off straight, with 3 cm of noise: must not look like "didn't move"
    for seed in range(20):
        rng = random.Random(seed)
        yaw = rng.uniform(-0.03, 0.03)
        ref = points(HALLWAY, (0, 0, yaw), rng, noise=0.03)
        cur = points(HALLWAY, (0.15 * math.cos(yaw), 0.15 * math.sin(yaw), yaw), rng, noise=0.03)
        assert driven_fraction(ref, cur, (0.15, 0.0), 0.0)[0] != 0.0


def test_random_spots_in_the_room():
    rng = random.Random(7)
    for _ in range(40):
        pose = (rng.uniform(0.4, 3.6), rng.uniform(0.4, 1.7), rng.uniform(-math.pi, math.pi))
        ref = points(ROOM, pose, rng, noise=0.02)
        assert driven_fraction(ref, points(ROOM, pose, rng, noise=0.02), (0.15, 0.0), 0.0)[0] == 0.0
        assert driven_fraction(ref, points(ROOM, ahead(pose, 0.15), rng, noise=0.02), (0.15, 0.0), 0.0)[0] >= 0.75


def test_too_few_points_cant_tell():
    assert driven_fraction([(1, 0)] * 5, [(1, 0)] * 5, (0.15, 0.0), 0.0)[0] is None


def test_fast_enough_for_the_pi():
    rng = random.Random(5)
    ref, cur = points(ROOM, START, rng), points(ROOM, ahead(START, 0.15), rng)
    t0 = time.perf_counter()
    driven_fraction(ref, cur, (0.15, 0.0), 0.0)
    assert time.perf_counter() - t0 < 0.1        # ~10-20 ms on a laptop; checks run twice a second


# ---- scan_to_points / clear_behind / helpers ---------------------------------

def test_scan_to_points_uses_where_the_lidar_sits():
    # One reading straight ahead of a lidar mounted backwards (yaw pi) lands behind the robot
    pts = scan_to_points([1.0], 0.0, 0.1, sensor_pose=(0.0, 0.0, math.pi))
    assert pts[0] == pytest.approx((-1.0, 0.0))


def test_scan_to_points_drops_walls_seen_edge_on():
    # Down a hallway the far parts of the walls are seen at a shallow angle: dropped with max_spread
    ranges = cast(HALLWAY, (0, 0, 0), noise=0.0)
    every = scan_to_points(ranges, ANGLE_MIN, ANGLE_INC, max_range=4.0)
    facing = scan_to_points(ranges, ANGLE_MIN, ANGLE_INC, max_range=4.0, max_spread=2.5)
    assert len(facing) < len(every)
    assert max(abs(x) for x, _ in facing) < 2.0 < max(abs(x) for x, _ in every)


def test_scan_to_points_drops_out_of_range_and_thins():
    ranges = [0.1, 1.0, math.inf, float('nan'), 5.0, 1.0]
    assert len(scan_to_points(ranges, 0.0, 0.1, min_range=0.3, max_range=4.0)) == 2
    assert len(scan_to_points([1.0] * 720, 0.0, 0.01, max_points=360)) == 360


def test_clear_behind():
    r, d = 0.23, 0.15
    assert clear_behind([(1.0, 0.0), (-0.3, 0.6)], r, d)        # ahead, and off to the side
    assert not clear_behind([(-0.35, 0.05)], r, d)              # right behind
    assert clear_behind([(-0.1, 0.05)], r, d)                   # inside its own outline (the arm)
    assert clear_behind([(-0.6, 0.0)], r, d)                    # farther than it would back up


def test_yaw_from_quaternion():
    a = 0.7
    assert yaw_from_quaternion(0, 0, math.sin(a / 2), math.cos(a / 2)) == pytest.approx(a)


# ---- MotionCheck: a timeline at 10 scans a second ---------------------------

class Run:
    """Drives a pretend robot: odom follows the commands, the real pose only while not stuck."""

    def __init__(self, walls=ROOM, pose=START, imu=True):
        self.walls, self.real, self.odom = walls, pose, (0.0, 0.0, 0.0)
        self.imu, self.t, self.rng = imu, 0.0, random.Random(6)
        self.check = MotionCheck()
        self.answers = []

    def drive(self, seconds, v=0.15, w=0.0, stuck=False, turns_anyway=False):
        """Command v/w for `seconds`; returns whether it was flagged stuck at the end."""
        for _ in range(int(round(seconds * 10))):
            self.t += 0.1
            x, y, yaw = self.odom
            self.odom = (x + v * 0.1 * math.cos(yaw), y + v * 0.1 * math.sin(yaw), yaw + w * 0.1)
            if not stuck:
                self.real = ahead(self.real, v * 0.1, w * 0.1)
            elif turns_anyway:
                self.real = (self.real[0], self.real[1], self.real[2] + w * 0.1)
            self.check.odom(*self.odom)
            if self.imu:
                self.check.imu(self.t, math.atan2(math.sin(self.real[2]), math.cos(self.real[2])))
            self.check.scan(self.t, points(self.walls, self.real, self.rng))
            if round(self.t * 10) % 5 == 0:             # check() twice a second
                self.answers.append(self.check.check())
        return self.check.stuck


def test_driving_normally_is_never_stuck():
    run = Run()
    assert not run.drive(4.0)
    assert not run.drive(3.0, v=0.05, w=0.3)           # slow curve
    assert True in run.answers and False not in run.answers


def test_stuck_driving_forward_is_caught_within_a_few_seconds():
    run = Run()
    run.drive(2.0)
    seconds = 0.0
    while not run.drive(0.5, stuck=True):
        seconds += 0.5
        assert seconds < 4.0, 'took too long to notice'
    assert run.check.last['moving'] is False and run.check.last['driven'] == 0.0


def test_back_to_moving_clears_the_count():
    run = Run()
    run.drive(2.0, stuck=True)
    assert run.check.still_checks > 0
    run.drive(2.0)
    assert run.check.still_checks == 0 and not run.check.stuck


def test_spinning_wheels_without_turning_is_caught_by_the_imu():
    run = Run()
    assert run.drive(4.0, v=0.0, w=0.6, stuck=True)
    assert run.check.last['turned'] == pytest.approx(0.0, abs=0.01)


def test_turning_in_place_normally_is_fine():
    run = Run()
    assert not run.drive(4.0, v=0.0, w=0.6)
    assert run.check.last['turned'] == pytest.approx(1.0, abs=0.05)


def test_turning_in_place_without_the_imu_cant_tell():
    run = Run(imu=False)
    assert not run.drive(4.0, v=0.0, w=0.6, stuck=True)


def test_standing_still_is_never_stuck():
    run = Run()
    assert not run.drive(4.0, v=0.0)
    assert set(run.answers) == {None}


def test_bare_hallway_is_never_called_stuck():
    run = Run(walls=HALLWAY, pose=(0.0, 0.0, 0.0))
    assert not run.drive(4.0, stuck=True)


def test_reset_starts_over():
    run = Run()
    run.drive(4.0, stuck=True)
    assert run.check.stuck
    run.check.reset()
    assert not run.check.stuck and run.check.check() is None
