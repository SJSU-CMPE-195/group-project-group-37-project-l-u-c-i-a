"""
frontier.py

Finds frontiers in the map: the edges between floor the lidar has seen to be
open and space it hasn't seen yet. Driving up to a frontier lets the lidar see
past it, so exploring is: drive to the best frontier, repeat, until none are
left.

Only frontiers the robot can actually drive to count. A search spreads out
from the robot over open floor wide enough for it (clearance from walls), the
way water would flood the map; a frontier the flood doesn't reach (behind a
wall, past a gap too narrow, or only reachable through unseen space) is left
out, and distances are along the flood, not in a straight line through walls.

Plain Python on the map's own list of cells (no ROS, no numpy), so it can be
tested anywhere. Cell values follow nav_msgs/OccupancyGrid: -1 unknown,
0..100 how likely the cell is occupied. Cells are (col, row), row 0 at the
map's origin.
"""

import math
from array import array
from collections import deque, namedtuple

# Same split map_saver_cli uses (free_thresh 0.25, occupied_thresh 0.65).
FREE_MAX = 25
OCCUPIED_MIN = 65

# x, y: where to send the robot (a frontier cell it can reach), in map meters.
# length_m: how long the frontier is, roughly how much new space is behind it.
# distance_m: how far the robot drives to get there (None if not measured).
Frontier = namedtuple('Frontier', 'x y length_m cells distance_m', defaults=(None,))

_NEIGHBORS_4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
_NEIGHBORS_8 = _NEIGHBORS_4 + ((1, 1), (1, -1), (-1, 1), (-1, -1))


class Grid:
    """An OccupancyGrid's cells plus where they sit in the map frame."""

    def __init__(self, data, width, height, resolution, origin_x, origin_y):
        if len(data) != width * height:
            raise ValueError(f'grid has {len(data)} cells, expected {width}x{height}')
        self.data = data
        self.width = width
        self.height = height
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y

    def value(self, col, row):
        return self.data[row * self.width + col]

    def inside(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def to_world(self, col, row):
        """Center of a cell, in map meters."""
        return (self.origin_x + (col + 0.5) * self.resolution,
                self.origin_y + (row + 0.5) * self.resolution)

    def to_cell(self, x, y):
        """The cell containing map point (x, y), or None if it's off the map."""
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        return (col, row) if self.inside(col, row) else None


def _is_free(v):
    return 0 <= v <= FREE_MAX


def frontier_cells(grid):
    """Open cells with at least one unknown cell directly beside them."""
    out = []
    w, h, data = grid.width, grid.height, grid.data
    for row in range(h):
        base = row * w
        for col in range(w):
            if not _is_free(data[base + col]):
                continue
            for dc, dr in _NEIGHBORS_4:
                c, r = col + dc, row + dr
                if 0 <= c < w and 0 <= r < h and data[r * w + c] < 0:
                    out.append((col, row))
                    break
    return out


def cluster(cells):
    """Split cells into groups that touch (including diagonally)."""
    remaining = set(cells)
    groups = []
    while remaining:
        start = remaining.pop()
        group, queue = [start], deque([start])
        while queue:
            col, row = queue.popleft()
            for dc, dr in _NEIGHBORS_8:
                n = (col + dc, row + dr)
                if n in remaining:
                    remaining.remove(n)
                    group.append(n)
                    queue.append(n)
        groups.append(group)
    return groups


def near_walls(grid, radius_cells):
    """
    bytearray, one entry per cell: 1 if an occupied cell lies within
    radius_cells of it (the robot's middle can't go there), else 0.
    """
    w, h, data = grid.width, grid.height, grid.data
    out = bytearray(w * h)
    r2 = radius_cells * radius_cells
    disk = [(dc, dr) for dr in range(-radius_cells, radius_cells + 1)
            for dc in range(-radius_cells, radius_cells + 1) if dc * dc + dr * dr <= r2]
    for i, v in enumerate(data):
        if v < OCCUPIED_MIN:
            continue
        col, row = i % w, i // w
        for dc, dr in disk:
            c, r = col + dc, row + dr
            if 0 <= c < w and 0 <= r < h:
                out[r * w + c] = 1
    return out


def reachable_steps(grid, start, radius_cells):
    """
    Flood out from the start cell over open floor the robot fits on (no wall
    within radius_cells), moving up/down/left/right. Returns, per cell, how
    many steps it took to get there, or -1 if it can't be reached.

    Right around the start (where the robot already is), cells that aren't
    walls are allowed even if they're unknown or close to a wall, so a robot
    parked near a wall can still get out.
    """
    w, h, data = grid.width, grid.height, grid.data
    blocked = near_walls(grid, radius_cells)
    steps = array('l', [-1]) * (w * h)
    sc, sr = start
    escape2 = (radius_cells + 1) ** 2
    steps[sr * w + sc] = 0
    queue = deque([sr * w + sc])
    while queue:
        i = queue.popleft()
        col, row = i % w, i // w
        for dc, dr in _NEIGHBORS_4:
            c, r = col + dc, row + dr
            if not (0 <= c < w and 0 <= r < h):
                continue
            j = r * w + c
            if steps[j] >= 0:
                continue
            v = data[j]
            if (c - sc) ** 2 + (r - sr) ** 2 <= escape2:
                ok = v < OCCUPIED_MIN
            else:
                ok = 0 <= v <= FREE_MAX and not blocked[j]
            if ok:
                steps[j] = steps[i] + 1
                queue.append(j)
    return steps


def find_frontiers(grid, robot_xy, min_length_m=0.3, clearance_m=0.2):
    """
    Frontiers the robot can drive to from robot_xy, as Frontier tuples.

    min_length_m drops tiny ones, which are mostly speckle: single cells the
    lidar happened to miss in the middle of open floor. clearance_m is how far
    the robot's middle must stay from walls (its radius plus a margin), both
    on the way and at the goal. Each frontier's goal is the reachable cell of
    the frontier nearest its middle.
    """
    start = grid.to_cell(*robot_xy)
    if start is None:
        return []
    res, w = grid.resolution, grid.width
    min_cells = max(1, int(math.ceil(min_length_m / res - 1e-9)))
    steps = reachable_steps(grid, start, max(0, int(math.ceil(clearance_m / res - 1e-9))))
    out = []
    for group in cluster(frontier_cells(grid)):
        if len(group) < min_cells:
            continue
        reached = [(col, row) for col, row in group if steps[row * w + col] >= 0]
        if not reached:
            continue
        mid_c = sum(c for c, _ in group) / len(group)
        mid_r = sum(r for _, r in group) / len(group)
        col, row = min(reached, key=lambda cell: (cell[0] - mid_c) ** 2 + (cell[1] - mid_r) ** 2)
        x, y = grid.to_world(col, row)
        out.append(Frontier(x, y, len(group) * res, len(group), steps[row * w + col] * res))
    return out


def choose_frontier(frontiers, robot_xy, avoid=(), avoid_radius_m=0.5, gain=0.5):
    """
    The frontier to explore next: the cheapest by
        driving distance  -  gain * frontier length
    so nearby frontiers win, but a long one (a doorway into a new room) can win
    over a closer scrap. Straight-line distance is used if a frontier has no
    driving distance. Frontiers within avoid_radius_m of a point in `avoid`
    (goals that already failed) are skipped. None if nothing is left.
    """
    rx, ry = robot_xy
    best, best_cost = None, None
    for f in frontiers:
        if any(math.hypot(f.x - ax, f.y - ay) < avoid_radius_m for ax, ay in avoid):
            continue
        distance = f.distance_m if f.distance_m is not None else math.hypot(f.x - rx, f.y - ry)
        cost = distance - gain * f.length_m
        if best_cost is None or cost < best_cost:
            best, best_cost = f, cost
    return best


def near_any(point, frontiers, radius_m):
    """True if some frontier's goal is within radius_m of point (x, y)."""
    px, py = point
    return any(math.hypot(f.x - px, f.y - py) < radius_m for f in frontiers)
