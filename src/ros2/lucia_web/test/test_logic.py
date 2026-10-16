"""Tests for logic.py. Run from src/ros2/lucia_web:  python3 -m pytest test"""

import base64
import math

import pytest

from lucia_web.logic import (drive_to_twist, encode_grid, topic_status,
                             valid_map_name, yaw_from_quaternion)


def test_drive_scales_to_max_speeds():
    assert drive_to_twist(1, -1, 0.15, 0.6) == (0.15, -0.6)
    assert drive_to_twist(0.5, 0, 0.2, 1.0) == (0.1, 0.0)


def test_drive_clamps_out_of_range():
    assert drive_to_twist(5, -9, 0.15, 0.6) == (0.15, -0.6)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), 'fast', None])
def test_drive_bad_input_means_stop(bad):
    assert drive_to_twist(bad, 0, 0.15, 0.6) == (0.0, 0.0)
    assert drive_to_twist(0, bad, 0.15, 0.6) == (0.0, 0.0)


@pytest.mark.parametrize('name', ['room3', 'Room_2', 'lab-a', 'a' * 40])
def test_valid_map_names(name):
    assert valid_map_name(name)


@pytest.mark.parametrize('name', ['', '../etc', 'a/b', 'room 3', 'a' * 41, 'map.png', None])
def test_invalid_map_names(name):
    assert not valid_map_name(name)


def test_topic_status():
    assert topic_status(None) == 'missing'
    assert topic_status(0.2) == 'ok'
    assert topic_status(2.0) == 'stale'
    assert topic_status(10.0) == 'missing'


def test_encode_grid_unknown_is_255():
    assert encode_grid([-1, 0, 50, 100]) == bytes([255, 0, 50, 100])
    assert base64.b64decode(base64.b64encode(encode_grid([-1, 100]))) == bytes([255, 100])


def test_yaw_from_quaternion():
    half = math.radians(90) / 2
    assert math.degrees(yaw_from_quaternion(0, 0, math.sin(half), math.cos(half))) == pytest.approx(90)
