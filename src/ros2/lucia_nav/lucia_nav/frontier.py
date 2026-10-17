"""
frontier.py

Finds frontiers in the map: the edges between floor the lidar has seen to be
open and space it hasn't seen yet. Driving up to a frontier lets the lidar see
past it, so exploring is: drive to the best frontier, repeat, until none are
left.

Plain Python on the map's own list of cells (no ROS, no numpy), so it can be
tested anywhere. Cell values follow nav_msgs/OccupancyGrid: -1 unknown,
0..100 how likely the cell is occupied. Cells are (col, row), row 0 at the
map's origin.
"""

import math
from collections import deque, namedtuple

# Same split map_saver_cli uses (free_thresh 0.25, occupied_thresh 0.65).
FREE_MAX = 25
OCCUPIED_MIN = 65

# x, y: where to send the robot (a frontier cell with room around it), in map meters.
# length_m: how long the frontier is, roughly how much new space is behind it.
Frontier = namedtuple('Frontier', 'x y length_m cells')

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


def has_clearance(grid, col, row, radius_cells):
    """True if no occupied cell lies within radius_cells of (col, row)."""
    r2 = radius_cells * radius_cells
    for dr in range(-radius_cells, radius_cells + 1):
        for dc in range(-radius_cells, radius_cells + 1):
            if dc * dc + dr * dr > r2:
                continue
            c, r = col + dc, row + dr
            if grid.inside(c, r) and grid.value(c, r) >= OCCUPIED_MIN:
                return False
    return True


def goal_cell(grid, group, clearance_cells):
    """
    Where to send the robot for one frontier: of the frontier's cells that are
    clear of walls, the one nearest the frontier's middle. None if every cell
    is too close to something (e.g. a gap too narrow for the robot).
    """
    mid_c = sum(c for c, _ in group) / len(group)
    mid_r = sum(r for _, r in group) / len(group)
    best, best_d2 = None, None
    for col, row in group:
        d2 = (col - mid_c) ** 2 + (row - mid_r) ** 2
        if best_d2 is not None and d2 >= best_d2:
            continue
        if has_clearance(grid, col, row, clearance_cells):
            best, best_d2 = (col, row), d2
    return best


def find_frontiers(grid, min_length_m=0.3, clearance_m=0.25):
    """
    All frontiers worth driving to, as Frontier tuples.

    min_length_m drops tiny ones, which are mostly speckle: single cells the
    lidar happened to miss in the middle of open floor. clearance_m keeps goals
    away from walls (robot radius plus a margin).
    """
    min_cells = max(1, int(math.ceil(min_length_m / grid.resolution)))
    clearance_cells = max(0, int(math.ceil(clearance_m / grid.resolution)))
    out = []
    for group in cluster(frontier_cells(grid)):
        if len(group) < min_cells:
            continue
        cell = goal_cell(grid, group, clearance_cells)
        if cell is None:
            continue
        x, y = grid.to_world(*cell)
        out.append(Frontier(x, y, len(group) * grid.resolution, len(group)))
    return out


def choose_frontier(frontiers, robot_xy, avoid=(), avoid_radius_m=0.5, gain=0.5):
    """
    The frontier to explore next: the cheapest by
        distance from the robot  -  gain * frontier length
    so nearby frontiers win, but a long one (a doorway into a new room) can win
    over a closer scrap. Frontiers within avoid_radius_m of a point in `avoid`
    (goals that already failed) are skipped. None if nothing is left.
    """
    rx, ry = robot_xy
    best, best_cost = None, None
    for f in frontiers:
        if any(math.hypot(f.x - ax, f.y - ay) < avoid_radius_m for ax, ay in avoid):
            continue
        cost = math.hypot(f.x - rx, f.y - ry) - gain * f.length_m
        if best_cost is None or cost < best_cost:
            best, best_cost = f, cost
    return best


def near_any(point, frontiers, radius_m):
    """True if some frontier's goal is within radius_m of point (x, y)."""
    px, py = point
    return any(math.hypot(f.x - px, f.y - py) < radius_m for f in frontiers)
