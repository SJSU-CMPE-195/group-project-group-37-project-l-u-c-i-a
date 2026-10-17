"""Tests for frontier.py and commands.py. Run from src/ros2/lucia_nav:  python3 -m pytest test"""

import pytest

from lucia_nav.commands import parse_command
from lucia_nav.frontier import (Frontier, Grid, choose_frontier, cluster,
                                find_frontiers, frontier_cells, has_clearance,
                                near_any)

U, F, X = -1, 0, 100     # unknown, free, occupied


def grid_from(rows, resolution=0.1, origin=(0.0, 0.0)):
    """Build a Grid from rows written top-down (as they read on screen)."""
    rows = [list(r) for r in reversed(rows)]   # OccupancyGrid row 0 is the bottom
    data = [v for r in rows for v in r]
    return Grid(data, len(rows[0]), len(rows), resolution, *origin)


def room_with_door():
    """
    A 12 x 9 room seen from inside: walls all round except a 3-cell door on
    the right, unknown past the door. One frontier: the door.
    """
    w = 14
    rows = [[X] * 12 + [U, U]]
    for r in range(7):
        door = 2 <= r <= 4
        rows.append([X] + [F] * 10 + ([F, U, U] if door else [X, U, U]))
    rows.append([X] * 12 + [U, U])
    assert all(len(r) == w for r in rows)
    return grid_from(rows)


def test_grid_cell_world_round_trip():
    g = Grid([F] * 20, 5, 4, 0.05, -1.0, 2.0)
    x, y = g.to_world(3, 2)
    assert (x, y) == pytest.approx((-1.0 + 3.5 * 0.05, 2.0 + 2.5 * 0.05))
    assert g.to_cell(x, y) == (3, 2)
    assert g.to_cell(-5.0, 0.0) is None


def test_grid_rejects_wrong_size():
    with pytest.raises(ValueError):
        Grid([F] * 5, 3, 2, 0.05, 0, 0)


def test_frontier_cells_are_free_cells_beside_unknown():
    g = grid_from([
        [U, U, U],
        [F, F, F],
        [X, F, F],
    ])
    # only the middle row touches unknown (above it)
    assert sorted(frontier_cells(g)) == [(0, 1), (1, 1), (2, 1)]


def test_walls_beside_unknown_are_not_frontiers():
    g = grid_from([
        [U, U, U],
        [X, X, X],
        [F, F, F],
    ])
    assert frontier_cells(g) == []


def test_cluster_joins_diagonal_neighbors():
    groups = cluster([(0, 0), (1, 1), (2, 2), (5, 5)])
    assert sorted(len(g) for g in groups) == [1, 3]


def test_has_clearance():
    g = grid_from([
        [F, F, F, F, F],
        [F, F, F, F, F],
        [F, F, F, F, X],
    ])
    assert has_clearance(g, 0, 2, 2)
    assert not has_clearance(g, 4, 1, 1)     # right above the wall cell
    assert has_clearance(g, 3, 1, 1)         # diagonal is 1.4 cells away, outside radius 1
    assert not has_clearance(g, 3, 1, 2)


def test_door_is_the_one_frontier():
    g = room_with_door()
    fs = find_frontiers(g, min_length_m=0.3, clearance_m=0.1)
    assert len(fs) == 1
    f = fs[0]
    assert f.cells == 3
    assert f.length_m == pytest.approx(0.3)
    # the door is the gap in the wall column (col 11), rows 3..5 from the bottom;
    # the goal is its middle cell
    assert g.to_cell(f.x, f.y) == (11, 4)


def test_small_frontiers_are_dropped():
    g = room_with_door()
    assert find_frontiers(g, min_length_m=0.5, clearance_m=0.1) == []


def test_frontier_too_narrow_for_the_robot_is_dropped():
    g = room_with_door()
    # 0.2 m clearance: every door cell is within 2 cells of the door frame
    assert find_frontiers(g, min_length_m=0.3, clearance_m=0.2) == []


def test_fully_known_room_has_no_frontiers():
    g = grid_from([[X] * 5] + [[X, F, F, F, X]] * 3 + [[X] * 5])
    assert find_frontiers(g) == []


def test_choose_prefers_near_then_long():
    near_small = Frontier(1.0, 0.0, 0.3, 6)
    far_long = Frontier(3.0, 0.0, 2.0, 40)
    far_small = Frontier(3.0, 1.0, 0.3, 6)
    # cost: near_small 1 - 0.15 = 0.85 ; far_long 3 - 1.0 = 2.0
    assert choose_frontier([far_long, near_small, far_small], (0, 0)) == near_small
    # with a strong pull toward long frontiers the doorway wins
    assert choose_frontier([far_long, near_small], (0, 0), gain=2.0) == far_long


def test_choose_skips_failed_goals_and_handles_none_left():
    a = Frontier(1.0, 0.0, 0.3, 6)
    b = Frontier(4.0, 0.0, 0.3, 6)
    assert choose_frontier([a, b], (0, 0), avoid=[(1.1, 0.1)]) == b
    assert choose_frontier([a], (0, 0), avoid=[(1.0, 0.0)]) is None
    assert choose_frontier([], (0, 0)) is None


def test_near_any():
    fs = [Frontier(1.0, 1.0, 0.3, 6)]
    assert near_any((1.2, 1.0), fs, 0.5)
    assert not near_any((2.0, 1.0), fs, 0.5)


@pytest.mark.parametrize('text, expected', [
    ('explore', ('explore',)),
    ('  HOME ', ('home',)),
    ('stop', ('stop',)),
    ('goto 1.5 -0.25', ('goto', 1.5, -0.25)),
])
def test_parse_command(text, expected):
    assert parse_command(text) == expected


@pytest.mark.parametrize('text', ['', 'dance', 'goto 1', 'goto a b', 'goto nan 0',
                                  'goto inf 1', 'explore now', None, 42])
def test_parse_command_rejects(text):
    assert parse_command(text) is None
