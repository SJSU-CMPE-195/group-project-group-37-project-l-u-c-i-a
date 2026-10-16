"""Tests for imu_line.py. Run from src/ros2/lucia_imu:  python3 -m pytest test"""

import math

import pytest

from lucia_imu.imu_line import ImuReading, parse_line, yaw_rad


def test_parses_a_valid_line():
    r = parse_line('I,1.00000,0.00000,0.00000,0.00000,0.0100,-0.0200,0.5000\n')
    assert r == ImuReading(1.0, 0.0, 0.0, 0.0, 0.01, -0.02, 0.5)


def test_ignores_startup_and_error_text():
    assert parse_line('# streaming') is None
    assert parse_line('Traceback (most recent call last):') is None
    assert parse_line('') is None


def test_rejects_half_received_line():
    assert parse_line('I,1.00000,0.00000,0.000') is None


def test_rejects_too_many_fields():
    assert parse_line('I,1,0,0,0,0,0,0,0') is None


def test_rejects_garbage_numbers():
    assert parse_line('I,1.0,abc,0,0,0,0,0') is None
    assert parse_line('I,nan,0,0,0,0,0,0') is None


def test_rejects_quaternion_that_isnt_unit_length():
    assert parse_line('I,0.5,0,0,0,0,0,0') is None


def test_accepts_slightly_off_unit_quaternion_from_rounding():
    assert parse_line('I,0.99999,0.00001,0,0,0,0,0') is not None


def test_yaw_zero_for_identity():
    assert yaw_rad(ImuReading(1, 0, 0, 0, 0, 0, 0)) == pytest.approx(0.0)


@pytest.mark.parametrize('yaw_deg', [30, 90, 179, -45, -120])
def test_yaw_matches_rotation_about_vertical_axis(yaw_deg):
    half = math.radians(yaw_deg) / 2
    r = ImuReading(math.cos(half), 0, 0, math.sin(half), 0, 0, 0)
    assert math.degrees(yaw_rad(r)) == pytest.approx(yaw_deg)
