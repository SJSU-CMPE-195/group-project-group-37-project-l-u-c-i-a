import struct

import pytest

from lucia_tof.obstacles import FLOOR_MM, classify, pack_xyz

Z = 0.254   # sensors' height

# tof_check on open floor, 2026-11-10 (medians)
OPEN_FLOOR = {
    'left': [-1, 1474, 1840, 1626, 1542, 1075, 1793, 1669, -1, 1079, 1306, 1259, 713, 699, 698, 711],
    'middle': [-1, 1956, -1, 1593, 1550, -1, 1663, 1594, -1, -1, -1, -1, 633, 606, 648, 643],
    'right': [1052, 1121, 925, 985, 1033, -1, -1, -1, 847, -1, -1, 865, 550, 534, 550, 560],
}


def test_the_open_floor_is_not_an_obstacle():
    for name in ('left', 'middle'):
        obstacles, free = classify(OPEN_FLOOR[name], FLOOR_MM[name], Z)
        assert obstacles == [], name
        assert len(free) == 16
    # the right sensor saw something about 85 cm away, low enough to hit: those stay.
    # Its top-row readings at 93-99 cm are about 53 cm up, higher than the robot: free
    obstacles, _ = classify(OPEN_FLOOR['right'], FLOOR_MM['right'], Z)
    assert [round(p[0], 3) for p in obstacles] == [0.847, 0.865]


def test_without_the_floor_distances_the_floor_would_be_an_obstacle():
    obstacles, _ = classify(OPEN_FLOOR['middle'], None, Z)
    assert len(obstacles) == 4


def test_something_low_in_front_is_an_obstacle():
    grid = list(OPEN_FLOOR['middle'])
    grid[13] = grid[14] = 450                     # bottom row, middle two zones
    obstacles, _ = classify(grid, FLOOR_MM['middle'], Z)
    assert len(obstacles) == 2
    x, y, z = obstacles[0]
    assert x == pytest.approx(0.45) and Z + z < 0.15   # low, 45 cm out


def test_the_book_test_marks_the_book():
    # middle sensor, book on the robot's left half (2026-11-10)
    grid = [194, 183, 412, 255, 201, 183, 287, 285, 202, 183, 293, 274, 201, 184, -1, 261]
    obstacles, free = classify(grid, FLOOR_MM['middle'], Z)
    assert len(obstacles) == 15 and len(free) == 1


def test_higher_than_the_robot_and_far_away_are_free():
    grid = [-1] * 16
    grid[0] = 900                                 # top row: about 53 cm up at 90 cm out
    grid[5] = 1400                                # beyond 1 m
    obstacles, free = classify(grid, None, Z)
    assert obstacles == [] and len(free) == 16
    assert free[5][0] == pytest.approx(1.4)
    assert free[3][0] == pytest.approx(1.5)       # nothing in range: free up to 1.5 m


def test_pack_xyz():
    data = pack_xyz([(1.0, 2.0, 3.0), (-0.5, 0.25, 0.0)])
    assert len(data) == 24
    assert struct.unpack('<6f', data) == (1.0, 2.0, 3.0, -0.5, 0.25, 0.0)
    assert pack_xyz([]) == b''
