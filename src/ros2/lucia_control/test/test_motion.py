"""Tests for motion.py. Run from src/ros2/lucia_control:  python3 -m pytest test"""

from lucia_control.motion import twist_to_wheel_speeds

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
