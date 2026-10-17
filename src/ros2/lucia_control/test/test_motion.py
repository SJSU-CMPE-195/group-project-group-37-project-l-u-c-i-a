"""Tests for motion.py. Run from src/ros2/lucia_control:  python3 -m pytest test"""

import pytest

from lucia_control.motion import ramp_toward, twist_to_wheel_speeds

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
