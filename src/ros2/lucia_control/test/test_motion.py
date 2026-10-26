"""Tests for motion.py. Run from src/ros2/lucia_control:  python3 -m pytest test"""

import pytest

from lucia_control.motion import ramp_toward, ramp_twist, twist_to_wheel_speeds

WHEEL_BASE = 235.0
MAX_SPEED = 300.0


def speeds(linear, angular):
    return twist_to_wheel_speeds(linear, angular, WHEEL_BASE, MAX_SPEED)


def test_stopped():
    assert speeds(0.0, 0.0) == (0, 0)


def test_straight_forward():
    assert speeds(0.2, 0.0) == (200, 200)


def test_straight_backward():
    assert speeds(-0.1, 0.0) == (-100, -100)


def test_spin_in_place_counter_clockwise():
    # Right wheel forward, left wheel backward, each at (turn rate * wheel_base / 2)
    assert speeds(0.0, 2.0) == (-235, 235)


def test_spin_in_place_clockwise():
    assert speeds(0.0, -2.0) == (235, -235)


def test_arc_left_while_moving():
    left, right = speeds(0.2, 0.5)
    assert (left, right) == (141, 259)
    assert right > left


def test_arc_right_while_moving():
    left, right = speeds(0.2, -0.5)
    assert left > right


def test_exactly_at_limit_is_not_scaled():
    assert speeds(0.3, 0.0) == (300, 300)


def test_too_fast_straight_is_capped():
    assert speeds(0.5, 0.0) == (300, 300)
    assert speeds(-0.5, 0.0) == (-300, -300)


def test_too_fast_arc_keeps_its_shape():
    # Unscaled this would be (282.5, 517.5); both wheels are scaled down together
    left, right = speeds(0.4, 1.0)
    assert right == 300
    assert abs(left / right - 282.5 / 517.5) < 0.01


def test_non_finite_input_stops():
    nan, inf = float('nan'), float('inf')
    assert speeds(nan, 0.0) == (0, 0)
    assert speeds(0.2, nan) == (0, 0)
    assert speeds(inf, 0.0) == (0, 0)
    assert speeds(0.0, -inf) == (0, 0)


# ---- ramp_toward -----------------------------------------------------------

ACCEL, DECEL, DT = 0.3, 0.6, 0.1        # bridge defaults: m/s^2, m/s^2, s


def ramp(current, target):
    return ramp_toward(current, target, ACCEL, DECEL, DT)


def steps_to_reach(current, target, limit=100):
    for n in range(1, limit):
        current = ramp(current, target)
        if current == target:
            return n
    raise AssertionError('never reached the target')


def test_speeds_up_gradually_from_a_stop():
    assert ramp(0.0, 0.15) == pytest.approx(0.03)
    assert steps_to_reach(0.0, 0.15) == 5            # 0.5 s to 15 cm/s


def test_never_overshoots():
    assert ramp(0.14, 0.15) == 0.15
    assert ramp(0.01, 0.0) == 0.0


def test_slows_down_faster_than_it_speeds_up():
    assert ramp(0.15, 0.0) == pytest.approx(0.09)
    assert steps_to_reach(0.15, 0.0) == 3            # 0.3 s to stop


def test_backward_ramps_the_same_way():
    assert ramp(0.0, -0.1) == pytest.approx(-0.03)
    assert ramp(-0.1, 0.0) == pytest.approx(-0.04)


def test_reversing_stops_at_zero_first():
    assert ramp(0.03, -0.15) == 0.0                  # would cross zero: stop exactly
    assert ramp(0.0, -0.15) == pytest.approx(-0.03)  # then speed up the other way


def test_lower_target_same_direction_is_slowing_down():
    assert ramp(0.3, 0.15) == pytest.approx(0.24)


def test_bad_target_ramps_down_to_stop():
    assert ramp(0.15, float('nan')) == pytest.approx(0.09)


def test_no_time_passed_no_change():
    assert ramp_toward(0.1, 0.3, ACCEL, DECEL, 0.0) == 0.1


# ---- ramp_twist: forward speed and turn rate together ------------------------

LIMITS = ((0.3, 0.6), (1.5, 3.0))       # bridge defaults: (accel, decel) m/s^2, rad/s^2


def run_twist(current, target, ticks):
    out = []
    for _ in range(ticks):
        current = ramp_twist(current, target, LIMITS, DT)
        out.append(current)
    return out


def test_stopping_from_a_curve_keeps_the_curve():
    # A 0.5 m radius arc, then stop: every step on the way down is still on that arc
    steps = run_twist((0.30, 0.6), (0.0, 0.0), 10)
    assert steps[-1] == (0.0, 0.0)
    for v, w in steps:
        if w:
            assert v / w == pytest.approx(0.5)


def test_starting_into_a_curve_keeps_the_curve():
    steps = run_twist((0.0, 0.0), (0.2, 0.4), 20)
    assert steps[-1] == (0.2, 0.4)
    for v, w in steps:
        assert v / w == pytest.approx(0.5)


def test_slowest_speed_sets_the_pace():
    # Forward speed limits this one (0.03 m/s per step), so the turn takes as many steps
    v, w = ramp_twist((0.0, 0.0), (0.3, 0.3), LIMITS, DT)
    assert v == pytest.approx(0.03) and w == pytest.approx(0.03)


def test_turn_in_place_reverses_through_zero():
    v, w = ramp_twist((0.0, 0.6), (0.0, -0.6), LIMITS, DT)
    assert (v, w) == (0.0, pytest.approx(0.3))          # slowing down at 3 rad/s^2
    v, w = ramp_twist((0.0, 0.2), (0.0, -0.6), LIMITS, DT)
    assert (v, w) == (0.0, 0.0)                         # stops at 0 first
    v, w = ramp_twist((0.0, 0.0), (0.0, -0.6), LIMITS, DT)
    assert w == pytest.approx(-0.15)                    # then speeds up the other way


def test_reaches_the_target_exactly():
    assert ramp_twist((0.29, 0.1), (0.3, 0.1), LIMITS, DT) == (0.3, 0.1)


def test_bad_target_means_stop():
    v, w = ramp_twist((0.3, 0.0), (float('nan'), float('inf')), LIMITS, DT)
    assert v == pytest.approx(0.24) and w == 0.0
