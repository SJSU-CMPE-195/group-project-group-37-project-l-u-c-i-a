"""Which ToF readings are obstacles for Nav2's map, and which show free space.
No ROS imports, so it can be tested on its own.

Why: the lidar (30.5 cm up) can't see anything lower, and the ToF sensors
(25.4 cm up, level) lose a low object from view as the robot gets close
(zones.py). Put on Nav2's map in a layer of their own (nav2.yaml `tof_layer`),
a low object seen further out stays marked after it drops out of view, and
Nav2 plans around it.

Each zone gives one point, in the sensor's own frame (zones.sensor_point):
- an obstacle: something within mark_max_mm that isn't the floor and isn't
  higher than z_max_m (it would pass under);
- free space up to that point: the floor, anything further away, or nothing
  in range (then free up to free_max_mm). Nav2 clears its map along the line
  to these, but only from raytrace_min_range (0.7 m) out, so a low object that
  dropped out of view close to the robot isn't wiped.

The floor: on the robot each sensor sees it only in its bottom row (tof_check
on open floor, 2026-11-10): left about 70 cm, middle 63, right 55 (the bracket
isn't perfectly level). A bottom-row reading at least floor_margin_mm shorter
than that zone's floor is something on the floor; otherwise it's the floor.
"""
from lucia_tof import zones

# Bottom-row floor distances (mm, image order), medians from tof_check on open
# floor, 2026-11-10. tof.yaml has the same; re-measure if the bracket moves.
FLOOR_MM = {
    'left': [713, 699, 698, 711],
    'middle': [633, 606, 648, 643],
    'right': [550, 534, 550, 560],
}


def zone_kind(index, d, floor_row_mm, mount_z, min_mm=20, mark_max_mm=1000,
              floor_margin_mm=100, z_max_m=0.45, radial=False):
    """What one zone's reading (mm, image order) is:
    'none' (nothing in range, or a glitch), 'floor', 'obstacle' (within
    mark_max_mm and low enough to hit), 'high' (higher than the robot: it
    passes under) or 'far' (further than mark_max_mm)."""
    if d < min_mm:
        return 'none'
    bottom = (zones.SIDE - 1) * zones.SIDE
    if index >= bottom and floor_row_mm is not None and d >= floor_row_mm[index - bottom] - floor_margin_mm:
        return 'floor'
    if d > mark_max_mm:
        return 'far'
    if mount_z + zones.sensor_point(index, d / 1000.0, radial)[2] > z_max_m:
        return 'high'
    return 'obstacle'


def classify(image_mm, floor_row_mm, mount_z, min_mm=20, mark_max_mm=1000,
             free_max_mm=1500, floor_margin_mm=100, z_max_m=0.45, radial=False):
    """One frame (16 distances in mm, image order, -1 = no reading) ->
    (obstacles, free): lists of (x, y, z) in the sensor's frame.
    floor_row_mm: the bottom row's floor distances, or None if unknown (then
    the bottom row is treated like the others)."""
    obstacles, free = [], []
    for i, d in enumerate(image_mm):
        kind = zone_kind(i, d, floor_row_mm, mount_z, min_mm, mark_max_mm, floor_margin_mm, z_max_m, radial)
        if kind == 'obstacle':
            obstacles.append(zones.sensor_point(i, d / 1000.0, radial))
        elif kind == 'none':                 # free up to free_max_mm
            free.append(zones.sensor_point(i, free_max_mm / 1000.0, radial))
        else:                                # floor, far, high: free up to the reading
            free.append(zones.sensor_point(i, min(d, free_max_mm) / 1000.0, radial))
    return obstacles, free


def radar_points(image_mm, floor_row_mm, mount, min_mm=20, mark_max_mm=1000,
                 floor_margin_mm=100, z_max_m=0.45, radial=False):
    """For the web page's radar: [(x, y, kind)] in base_link (m) for every
    zone with a reading, kind as zone_kind."""
    out = []
    for i, d in enumerate(image_mm):
        kind = zone_kind(i, d, floor_row_mm, mount.z, min_mm, mark_max_mm, floor_margin_mm, z_max_m, radial)
        if kind != 'none':
            x, y, _ = zones.zone_point(i, d / 1000.0, mount, radial)
            out.append((round(x, 3), round(y, 3), kind))
    return out


def pack_xyz(points):
    """Points as the bytes of a PointCloud2 with float32 x, y, z fields
    (little-endian, 12 bytes a point)."""
    import struct
    flat = [c for p in points for c in p]
    return struct.pack('<{}f'.format(len(flat)), *flat)
