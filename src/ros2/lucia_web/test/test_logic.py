"""Tests for logic.py. Run from src/ros2/lucia_web:  python3 -m pytest test"""

import base64
import math

import pytest

from lucia_web.logic import (clamp_speed, drive_to_twist, encode_grid,
                             mission_text, radar_nearest, scan_to_radar,
                             thin_path, topic_status, valid_map_name,
                             yaw_from_quaternion)


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


def test_clamp_speed():
    assert clamp_speed(0.2, 0.05, 0.3) == 0.2
    assert clamp_speed('0.1', 0.05, 0.3) == 0.1
    assert clamp_speed(9, 0.05, 0.3) == 0.3
    assert clamp_speed(-1, 0.05, 0.3) == 0.05


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), 'fast', None, [0.1]])
def test_clamp_speed_rejects_non_numbers(bad):
    assert clamp_speed(bad, 0.05, 0.3) is None


def _scan(readings, count=360):
    """A full-circle scan, one reading per degree starting at 0 deg, all empty except `readings`."""
    ranges = [float('inf')] * count
    for deg, r in readings.items():
        ranges[deg] = r
    return dict(ranges=ranges, angle_min=0.0, angle_increment=2 * math.pi / count,
                range_min=0.15, range_max=12.0)


def test_radar_slices_ahead_and_left():
    out = scan_to_radar(**_scan({0: 1.0, 90: 2.5}), slices=4)
    assert out == [100, 250, 0, 0]          # ahead, left, behind, right


def test_radar_keeps_nearest_in_slice_and_centers_slice_zero():
    # 359 deg and 1 deg both fall in the slice centered straight ahead
    out = scan_to_radar(**_scan({359: 0.8, 1: 1.2}), slices=4)
    assert out == [80, 0, 0, 0]


def test_radar_turns_by_lidar_yaw():
    # LUCIA's lidar faces backwards: its 180 deg reading is straight ahead of the robot
    out = scan_to_radar(**_scan({180: 0.5}), yaw_offset=math.pi, slices=4)
    assert out == [50, 0, 0, 0]


def test_radar_drops_out_of_range_and_nan():
    out = scan_to_radar(**_scan({0: 0.05, 90: float('nan'), 180: 20.0}), slices=4)
    assert out == [0, 0, 0, 0]


def test_radar_nearest():
    assert radar_nearest([0, 0, 0, 0]) is None
    assert radar_nearest([100, 40, 0, 0]) == {'m': 0.4, 'deg': 90.0}
    assert radar_nearest([0, 0, 0, 30]) == {'m': 0.3, 'deg': -90.0}


@pytest.mark.parametrize('body, text', [
    ({'cmd': 'explore'}, 'explore'),
    ({'cmd': 'home'}, 'home'),
    ({'cmd': 'stop'}, 'stop'),
    ({'cmd': 'goto', 'x': 1.25, 'y': -0.5}, 'goto 1.250 -0.500'),
    ({'cmd': 'goto', 'x': '2', 'y': 0}, 'goto 2.000 0.000'),
])
def test_mission_text(body, text):
    assert mission_text(body) == text


@pytest.mark.parametrize('body', [
    None, [], {}, {'cmd': 'dance'}, {'cmd': 'goto'}, {'cmd': 'goto', 'x': 1},
    {'cmd': 'goto', 'x': 'nan', 'y': 0}, {'cmd': 'goto', 'x': 1e9, 'y': 0},
    {'cmd': 'explore; rm -rf /'},
])
def test_mission_text_rejects(body):
    assert mission_text(body) is None


def test_thin_path():
    pts = [(i, 0) for i in range(1000)]
    out = thin_path(pts, 100)
    assert len(out) == 100
    assert out[0] == (0, 0) and out[-1] == (999, 0)
    assert thin_path(pts[:5], 100) == pts[:5]
