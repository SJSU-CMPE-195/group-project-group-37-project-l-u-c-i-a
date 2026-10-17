"""Tests for safety_logic.py. Run from src/ros2/lucia_bringup:  python3 -m pytest test"""

import math

import pytest

from lucia_bringup.safety_logic import (clearances, gate, limit_forward, scan_points,
                                        tight_space_cap, tof_blocked)


def test_clear_when_fresh_and_nothing_near():
    assert not tof_blocked(False, 0.1, 0.5)


def test_blocked_when_near():
    assert tof_blocked(True, 0.1, 0.5)


def test_blocked_when_silent_or_stale():
    assert tof_blocked(None, None, 0.5)        # never heard from
    assert tof_blocked(False, 0.8, 0.5)        # stopped talking


def test_gate_passes_everything_when_clear():
    assert gate(0.2, 0.5, False) == (0.2, 0.5)


def test_gate_blocks_forward_only():
    assert gate(0.2, 0.5, True) == (0.0, 0.5)      # arc -> turn in place
    assert gate(0.0, -0.6, True) == (0.0, -0.6)    # spin allowed
    assert gate(-0.1, 0.0, True) == (-0.1, 0.0)    # backing up allowed


# ---- Tight spaces ----------------------------------------------------------

R = 0.23                    # robot radius
MIN, MAX = 0.06, 0.30


def cap_for(points):
    return tight_space_cap(*clearances(points, R), MIN, MAX)


def wall(x0, y0, x1, y1, step=0.02):
    n = max(1, int(math.hypot(x1 - x0, y1 - y0) / step))
    return [(x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n) for i in range(n + 1)]


def test_open_room_is_full_speed():
    room = wall(2, -2, 2, 2) + wall(-2, 2, 2, 2) + wall(-2, -2, 2, -2)
    assert cap_for(room) == MAX
    assert cap_for([]) == MAX                                   # nothing in range at all


def test_doorway_is_slow():
    # 0.7 m doorway, the robot in the middle of it
    jambs = wall(-0.1, 0.35, 0.1, 0.35) + wall(-0.1, -0.35, 0.1, -0.35)
    assert cap_for(jambs) == pytest.approx(MIN + (MAX - MIN) * (0.12 - 0.05) / 0.30)
    assert cap_for(jambs) < 0.13


def test_hugging_a_wall_is_in_between():
    side_wall = wall(-1, 0.43, 2, 0.43)                         # 0.2 m from the robot's edge
    assert 0.15 < cap_for(side_wall) < 0.25


def test_wall_coming_up_ahead_slows_it_down():
    caps = [cap_for(wall(R + gap, -1, R + gap, 1)) for gap in (1.0, 0.6, 0.4, 0.25, 0.1)]
    assert caps[0] == MAX and caps[-1] == MIN
    assert caps == sorted(caps, reverse=True)


def test_things_behind_and_its_own_arm_dont_count():
    behind = wall(-0.5, -1, -0.5, 1)
    arm = [(0.1, 0.1), (0.15, 0.0)]                             # inside its 0.23 m outline
    assert cap_for(behind + arm) == MAX


def test_limit_keeps_the_curve():
    assert limit_forward(0.3, 0.6, 0.15) == pytest.approx((0.15, 0.3))
    assert limit_forward(0.1, 0.6, 0.15) == (0.1, 0.6)          # already slow enough
    assert limit_forward(-0.1, 0.0, 0.06) == (-0.1, 0.0)        # backing up untouched
    assert limit_forward(0.0, 1.0, 0.06) == (0.0, 1.0)          # turning in place untouched


def test_scan_points_lidar_mounted_backwards():
    pts = scan_points([1.0, math.inf, 0.1], 0.0, math.pi / 2, (0.0, 0.0, math.pi), min_range=0.15)
    assert len(pts) == 1 and pts[0] == pytest.approx((-1.0, 0.0))
