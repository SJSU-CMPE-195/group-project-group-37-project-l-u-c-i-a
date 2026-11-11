"""Where each ToF zone's reading is on the robot, and whether it's in the
robot's way. No ROS imports, so it can be tested on its own.

Robot frame (base_link): x forward, y to the robot's left, z up from the floor,
origin at the middle of the Roomba (between its wheels).

Each VL53L5CX sees 45 x 45 degrees in 4 x 4 zones of 11.25 degrees. Zones are
in image order (row 0 at the top, column 0 on the sensor's own left), the
order tof_node publishes after flip_h / flip_v.

Where the sensors are (Christian, 2026-11-10: bracket drawing and tape
measure): all three 25.4 cm (10 in) up, level. The middle one faces straight
ahead, 7.6 cm (3 in) behind the front of the Roomba (34 cm across), on the
centerline. The bracket is a trapezoid: 40 mm at the back, a 25 mm front face
(middle sensor) and two 25 mm side faces at 72.5 degrees from straight ahead
(left and right sensors), so those sit 1.6 cm to the side of the middle one
and 1.2 cm behind it.
"""
import math
from collections import namedtuple

FOV_DEG = 45.0
SIDE = 4
ZONE_DEG = FOV_DEG / SIDE

Mount = namedtuple('Mount', 'x y z yaw_deg')
Mount.__doc__ = """Where a sensor is on the robot (m, base_link) and which way
it faces (degrees, + = toward the robot's left)."""

ROOMBA_FRONT_M = 0.17           # front of the Roomba from its middle (34 cm across)
_MIDDLE_X = ROOMBA_FRONT_M - 0.076
_SIDE_BACK = 0.0119             # side faces' middles, behind the front face (bracket drawing)
_SIDE_OUT = 0.01625

MOUNTS = {
    'left': Mount(_MIDDLE_X - _SIDE_BACK, _SIDE_OUT, 0.254, 72.5),
    'middle': Mount(_MIDDLE_X, 0.0, 0.254, 0.0),
    'right': Mount(_MIDDLE_X - _SIDE_BACK, -_SIDE_OUT, 0.254, -72.5),
}

StopBox = namedtuple('StopBox', 'front ahead half_width z_min z_max')
StopBox.__doc__ = """The space the robot would drive into next (m, base_link):
x from `front` to `front + ahead`, |y| up to `half_width`, z from `z_min` (above
that is not the floor) to `z_max` (higher than the robot, so it passes under)."""


def zone_angles(index):
    """(azimuth, elevation) in degrees of a zone's middle, in image order:
    + azimuth = the sensor's left, + elevation = up."""
    row, col = divmod(index, SIDE)
    return (1.5 - col) * ZONE_DEG, (1.5 - row) * ZONE_DEG


def sensor_point(index, distance_m, radial=False):
    """The point a zone's reading came from, in the sensor's own frame
    (x straight out of its face, y to its left, z up).

    radial: the reading is the distance along the zone's line of sight;
    otherwise (the default) it's measured straight out from the sensor's face.
    On the robot (2026-11-10, facing a wall) the corner zones read no longer
    than the middle ones, so straight out; it matters little this close.
    """
    az, el = (math.radians(a) for a in zone_angles(index))
    if radial:
        return (distance_m * math.cos(el) * math.cos(az),
                distance_m * math.cos(el) * math.sin(az),
                distance_m * math.sin(el))
    return distance_m, distance_m * math.tan(az), distance_m * math.tan(el)


def zone_point(index, distance_m, mount, radial=True):
    """The point a zone's reading came from, in base_link (x, y, z)."""
    fwd, side, up = sensor_point(index, distance_m, radial)
    yaw = math.radians(mount.yaw_deg)
    return (mount.x + fwd * math.cos(yaw) - side * math.sin(yaw),
            mount.y + fwd * math.sin(yaw) + side * math.cos(yaw),
            mount.z + up)


def in_box(point, box):
    x, y, z = point
    return (box.front <= x <= box.front + box.ahead and abs(y) <= box.half_width
            and box.z_min <= z <= box.z_max)


def in_the_way(distances_mm, mount, box, min_mm=20, radial=True):
    """True if any believable reading (>= min_mm) of this frame is in the box.
    distances_mm in image order, -1 = no reading."""
    for i, d in enumerate(distances_mm):
        if d >= min_mm and in_box(zone_point(i, d / 1000.0, mount, radial), box):
            return True
    return False
