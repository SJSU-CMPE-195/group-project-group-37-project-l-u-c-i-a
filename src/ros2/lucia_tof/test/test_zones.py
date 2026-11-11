import math

import pytest

from lucia_tof import zones
from lucia_tof.near_filter import NearFilter
from lucia_tof.zones import MOUNTS, StopBox, in_box, in_the_way, zone_angles, zone_point

BOX = StopBox(front=zones.ROOMBA_FRONT_M, ahead=0.15, half_width=0.25, z_min=0.02, z_max=0.45)


def wall_along_left(y_wall, mount, radial=True):
    """Each zone's reading (mm) with a wall parallel to the robot, y_wall to its left."""
    out = []
    for i in range(16):
        az, el = (math.radians(a) for a in zone_angles(i))
        heading = math.radians(mount.yaw_deg) + az
        sideways = math.cos(el) * math.sin(heading)
        if sideways <= 0:
            out.append(-1)
            continue
        d = (y_wall - mount.y) / sideways
        out.append(round(d * 1000) if d < 4.0 else -1)
    return out


def test_zone_angles_in_image_order():
    assert zone_angles(0) == (16.875, 16.875)        # top row, sensor's left
    assert zone_angles(15) == (-16.875, -16.875)     # bottom row, sensor's right


def test_left_sensors_front_corner_zone_lands_in_front_of_the_left_bumper():
    # left sensor, column 3 (its right: toward the front), row 2, 25 cm
    x, y, z = zone_point(11, 0.25, MOUNTS['left'])
    assert x == pytest.approx(0.2225, abs=0.001)
    assert y == pytest.approx(0.2216, abs=0.001)
    assert z == pytest.approx(0.2295, abs=0.001)
    assert in_box((x, y, z), BOX)


def test_left_and_right_mirror_each_other():
    lx, ly, lz = zone_point(5, 0.4, MOUNTS['left'])
    rx, ry, rz = zone_point(6, 0.4, MOUNTS['right'])
    assert (lx, lz) == pytest.approx((rx, rz)) and ly == pytest.approx(-ry)


def test_something_straight_ahead_within_the_stop_distance_is_in_the_way():
    assert in_the_way([200] * 16, MOUNTS['middle'], BOX)       # 20 cm from the sensor
    assert in_the_way([160] * 16, MOUNTS['middle'], BOX)       # the hand test
    assert not in_the_way([300] * 16, MOUNTS['middle'], BOX)   # beyond 15 cm past the front
    assert in_the_way([300] * 16, MOUNTS['middle'], BOX._replace(ahead=0.25))


def test_driving_along_a_wall_is_not_in_the_way():
    # A wall 27 cm from the middle of the robot (4 cm from its edge), on either side
    for name in ('left', 'middle', 'right'):
        m = MOUNTS[name]
        assert not in_the_way(wall_along_left(0.27, m), m, BOX), name
        mirrored = m._replace(y=-m.y, yaw_deg=-m.yaw_deg)
        assert not in_the_way(wall_along_left(0.27, mirrored), mirrored, BOX), name


def test_what_the_robot_saw_with_nothing_in_its_path_is_not_in_the_way():
    # Grids from tof_check on the robot, 2026-11-10 (left saw something beside it at ~35 cm)
    seen = {
        'left': [361, 334, 429, 393, 365, 337, 428, 392, 367, 336, 425, 393, 356, 332, 416, 385],
        'middle': [-1, -1, 692, -1, -1, -1, 531, 680, -1, -1, 2835, 687, 640, 606, 650, 654],
        'right': [1936, -1, -1, -1, 1940, -1, -1, -1, -1, 748, -1, 854, 560, 537, 548, 553],
    }
    for name, grid in seen.items():
        assert not in_the_way(grid, MOUNTS[name], BOX), name


def test_the_floor_never_counts():
    # Bottom row seeing the floor about 55-65 cm away
    grid = [-1] * 12 + [550, 600, 620, 650]
    for name in MOUNTS:
        assert not in_the_way(grid, MOUNTS[name], BOX)


def test_glitches_and_far_readings_are_ignored():
    assert not in_the_way([0, 5, 19] + [-1] * 13, MOUNTS['middle'], BOX)


def test_perpendicular_distances_also_work():
    assert in_the_way([200] * 16, MOUNTS['middle'], BOX, radial=False)
    assert not in_the_way([320] * 16, MOUNTS['middle'], BOX, radial=False)


def test_filter_goes_by_the_in_the_way_answer_but_covered_always_counts():
    f = NearFilter(frames=1)
    assert f.update([150] * 16, frame_near=False) == (150, False)   # close but beside the robot
    assert f.update([5] * 16, frame_near=False)[1] is True          # covered: can't see
