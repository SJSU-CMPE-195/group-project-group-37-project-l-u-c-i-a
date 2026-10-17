"""Tests for safety_logic.py. Run from src/ros2/lucia_bringup:  python3 -m pytest test"""

from lucia_bringup.safety_logic import gate, tof_blocked


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
