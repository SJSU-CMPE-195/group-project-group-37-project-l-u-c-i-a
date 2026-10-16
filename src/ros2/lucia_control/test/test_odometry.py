"""Tests for odometry.py. Run from src/ros2/lucia_control:  python3 -m pytest test"""

import math

import pytest

from lucia_control.odometry import Odometry

WHEEL_BASE = 235.0


def new_odom():
    return Odometry(WHEEL_BASE)


def test_starts_at_origin_facing_zero():
    o = new_odom()
    assert o.pose() == (0.0, 0.0, 0.0)


def test_zero_dt_does_nothing():
    o = new_odom()
    o.update(100, 100, 0.0)
    assert o.pose() == (0.0, 0.0, 0.0)


def test_straight_line_forward():
    o = new_odom()
    o.update(100, 100, 2.0)  # 100 mm/s for 2 s
    x, y, heading = o.pose()
    assert x == pytest.approx(200.0)
    assert y == pytest.approx(0.0)
    assert heading == pytest.approx(0.0)


def test_straight_line_backward():
    o = new_odom()
    o.update(-50, -50, 2.0)
    x, y, heading = o.pose()
    assert x == pytest.approx(-100.0)
    assert y == pytest.approx(0.0, abs=1e-9)


def test_spin_in_place_changes_heading_not_position():
    o = new_odom()
    # Same magnitude, opposite sign -> pure rotation, no translation
    o.update(-100, 100, 1.0)
    x, y, heading = o.pose()
    assert x == pytest.approx(0.0, abs=1e-9)
    assert y == pytest.approx(0.0, abs=1e-9)
    expected = (100 - (-100)) / WHEEL_BASE * 1.0
    assert heading == pytest.approx(expected)


def test_quarter_circle_arc_ends_up_perpendicular():
    o = new_odom()
    # Left wheel still, right wheel moving: turns left (CCW) around the left wheel.
    # Pick a duration that sweeps exactly 90 degrees and check the geometry,
    # not just that it moved.
    left_mm_s, right_mm_s = 0.0, 100.0
    turn_rad_s = (right_mm_s - left_mm_s) / WHEEL_BASE
    dt_s = (math.pi / 2) / turn_rad_s
    o.update(left_mm_s, right_mm_s, dt_s)
    x, y, heading = o.pose()
    radius_mm = ((left_mm_s + right_mm_s) / 2.0) / turn_rad_s
    assert heading == pytest.approx(math.pi / 2)
    assert x == pytest.approx(radius_mm, rel=1e-6)
    assert y == pytest.approx(radius_mm, rel=1e-6)


def test_heading_stays_in_range():
    o = new_odom()
    # Spin almost 3 full turns' worth of heading change; should wrap, not grow forever.
    o.heading_rad = 0.0
    o.update(-1000, 1000, 100.0)  # a large, deliberately unrealistic turn
    _, _, heading = o.pose()
    assert -math.pi < heading <= math.pi


def test_rejects_non_positive_wheel_base():
    with pytest.raises(ValueError):
        Odometry(0.0)
    with pytest.raises(ValueError):
        Odometry(-10.0)


def test_many_small_steps_match_one_big_step_for_a_straight_line():
    # Sanity check that repeated ticks accumulate correctly.
    o_ticked = new_odom()
    for _ in range(20):
        o_ticked.update(150, 150, 0.1)  # 20 ticks of 0.1 s, matching drive_period_s
    o_once = new_odom()
    o_once.update(150, 150, 2.0)
    assert o_ticked.pose() == pytest.approx(o_once.pose())
