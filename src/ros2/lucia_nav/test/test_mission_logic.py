"""Tests for mission_logic.py. Run from src/ros2/lucia_nav:  python3 -m pytest test"""

import math

import pytest

from lucia_nav.frontier import Grid
from lucia_nav.mission_logic import MissionLogic

U, F, X = -1, 0, 100
RES = 0.1


class World:
    """
    Two 3 x 3 m rooms side by side with a 0.6 m doorway between them, and a
    pretend lidar (2 m range, so one spot can't see everything) that fills in
    the robot's map as it moves. The robot starts at the map origin, like
    slam_toolbox's map frame.
    """
    W, H = 64, 32                    # cells: 6.4 x 3.2 m
    ORIGIN = (-1.0, -1.5)            # so the start cell's corner is at (0, 0)

    def __init__(self):
        truth = [[F] * self.W for _ in range(self.H)]
        for c in range(self.W):
            truth[0][c] = truth[self.H - 1][c] = X
        for r in range(self.H):
            truth[r][0] = truth[r][self.W - 1] = X
            if not 13 <= r <= 18:    # doorway in the middle wall
                truth[r][31] = X
        self.truth = truth
        self.known = [U] * (self.W * self.H)
        self.robot = (0.0, 0.0)
        self.scan()

    def grid(self):
        return Grid(list(self.known), self.W, self.H, RES, *self.ORIGIN)

    def cell(self, x, y):
        return (int(math.floor((x - self.ORIGIN[0]) / RES)),
                int(math.floor((y - self.ORIGIN[1]) / RES)))

    def scan(self, max_range=2.0):
        c0, r0 = self.cell(*self.robot)
        for i in range(360):
            a = math.radians(i)
            for step in range(1, int(max_range / (RES / 2))):
                d = step * RES / 2
                c = int(math.floor(c0 + 0.5 + d / RES * math.cos(a)))
                r = int(math.floor(r0 + 0.5 + d / RES * math.sin(a)))
                if not (0 <= c < self.W and 0 <= r < self.H):
                    break
                self.known[r * self.W + c] = self.truth[r][c]
                if self.truth[r][c] == X:
                    break

    def known_fraction(self, cols):
        cells = [(c, r) for r in range(1, self.H - 1) for c in cols]
        return sum(self.known[r * self.W + c] != U for c, r in cells) / len(cells)


def run(logic, world, actions, reach=lambda goal_id: True, limit=60):
    """Carry out actions like mission.py + Nav2 would: the robot jumps to each goal."""
    goals_sent = []
    now = 0.0
    queue = list(actions)
    while queue and limit:
        limit -= 1
        act = queue.pop(0)
        if act[0] != 'send':
            continue
        _, goal_id, x, y, _yaw = act
        goals_sent.append((x, y))
        now += 10.0
        ok = reach(goal_id)
        if ok:
            world.robot = (x, y)
            world.scan()
            logic.map_updated(world.grid())
        queue += logic.goal_finished(goal_id, ok, now, world.robot)
    return goals_sent


def make_logic(**kw):
    kw.setdefault('min_frontier_m', 0.3)
    kw.setdefault('clearance_m', 0.2)
    return MissionLogic(**kw)


def test_explores_both_rooms_then_goes_home():
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    assert world.known_fraction(range(33, 63)) < 0.2         # second room unseen at the start

    goals = run(logic, world, logic.command('explore', 0.0, world.robot, nav_ready=True))

    assert world.known_fraction(range(1, 31)) > 0.95
    assert world.known_fraction(range(33, 63)) > 0.95
    assert logic.mode == 'idle' and logic.message == 'Home'
    assert goals[-1] == (0.0, 0.0)
    assert world.robot == (0.0, 0.0)
    assert len(goals) < 20


def test_no_frontiers_means_straight_home_or_stop():
    world = World()
    world.known = [world.truth[r][c] for r in range(world.H) for c in range(world.W)]
    logic = make_logic()
    logic.map_updated(world.grid())
    acts = logic.command('explore', 0.0, (1.0, 0.5), nav_ready=True)
    assert acts[0][0] == 'send' and acts[0][2:4] == (0.0, 0.0)
    assert logic.mode == 'home'

    logic = make_logic(return_home_when_done=False)
    logic.map_updated(world.grid())
    assert logic.command('explore', 0.0, (1.0, 0.5), nav_ready=True) == [('cancel',)]
    assert logic.mode == 'idle' and logic.message == 'Exploration finished'


def test_unreachable_frontiers_are_skipped():
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    # Nav2 fails every goal except home: each frontier gets tried once, then home
    first = logic.command('explore', 0.0, world.robot, nav_ready=True)
    goals = run(logic, world, first, reach=lambda _id: logic.mode == 'home')
    frontier_goals = goals[:-1]
    assert len(frontier_goals) == len(set(frontier_goals))   # never retried
    assert len(logic.failed) == len(frontier_goals)
    assert goals[-1] == (0.0, 0.0) and logic.message == 'Home'


def test_explore_waits_for_map_and_pose():
    logic = make_logic()
    assert logic.command('explore', 0.0, None, nav_ready=True) == []
    assert logic.mode == 'explore' and 'waiting' in logic.message
    world = World()
    logic.map_updated(world.grid())
    acts = logic.tick(3.0, world.robot)
    assert acts and acts[0][0] == 'send'


def test_needs_nav2_except_stop():
    logic = make_logic()
    for cmd in ('explore', 'home', 'goto 1 1'):
        assert logic.command(cmd, 0.0, (0, 0), nav_ready=False) == []
        assert logic.mode == 'idle' and 'Nav2' in logic.message
    assert logic.command('stop', 0.0, (0, 0), nav_ready=False) == [('cancel',), ('stop_wheels',)]


def test_unknown_command_does_nothing():
    logic = make_logic()
    assert logic.command('dance', 0.0, (0, 0), nav_ready=True) == []
    assert logic.mode == 'idle' and 'dance' in logic.message


def test_goto_faces_direction_of_travel_and_reports():
    logic = make_logic()
    (act,) = logic.command('goto 1 1', 0.0, (0.0, 0.0), nav_ready=True)
    assert act[0] == 'send' and act[2:4] == (1.0, 1.0)
    assert act[4] == pytest.approx(math.pi / 4)
    assert logic.goal_finished(act[1], False, 5.0, (0, 0)) == []
    assert logic.mode == 'idle' and logic.message == 'Could not reach that spot'


def test_home_success():
    logic = make_logic()
    (act,) = logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    assert act[2:] == (0.0, 0.0, 0.0)
    logic.goal_finished(act[1], True, 5.0, (0, 0))
    assert logic.mode == 'idle' and logic.message == 'Home'


def test_stop_ignores_results_that_arrive_late():
    logic = make_logic()
    (act,) = logic.command('goto 2 0', 0.0, (0, 0), nav_ready=True)
    assert logic.command('stop', 1.0, (0, 0), nav_ready=True) == [('cancel',), ('stop_wheels',)]
    assert logic.mode == 'idle' and logic.message == 'Stopped'
    assert logic.goal_finished(act[1], True, 2.0, (0, 0)) == []
    assert logic.message == 'Stopped'


def test_replaced_goal_result_is_ignored():
    logic = make_logic()
    (old,) = logic.command('goto 2 0', 0.0, (0, 0), nav_ready=True)
    (new,) = logic.command('goto 0 2', 1.0, (0, 0), nav_ready=True)
    assert logic.goal_finished(old[1], False, 2.0, (0, 0)) == []
    assert logic.mode == 'goto' and logic.goal == (0.0, 2.0)
    logic.goal_finished(new[1], True, 3.0, (0, 2))
    assert logic.message == 'Arrived'


def test_goal_timeout():
    world = World()
    logic = make_logic(goal_timeout_s=30.0)
    logic.map_updated(world.grid())
    (first,) = logic.command('explore', 0.0, world.robot, nav_ready=True)
    assert logic.tick(10.0, world.robot) == []
    acts = logic.tick(31.0, world.robot)
    assert logic.failed == [first[2:4]]
    assert acts and acts[0][0] == 'send' and acts[0][2:4] != first[2:4]

    logic = make_logic(goal_timeout_s=30.0)
    logic.command('home', 0.0, (1, 1), nav_ready=True)
    assert logic.tick(31.0, (1, 1)) == [('cancel',), ('stop_wheels',)]
    assert logic.mode == 'idle'


def test_moves_on_when_target_frontier_is_seen_on_the_way():
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    (first,) = logic.command('explore', 0.0, world.robot, nav_ready=True)
    # The robot gets most of the way there and the lidar sees past the target
    gx, gy = first[2:4]
    world.robot = (gx * 0.8, gy * 0.8)
    world.scan()
    world.robot = (gx, gy)
    world.scan()
    logic.map_updated(world.grid())
    acts = logic.tick(5.0, world.robot)
    assert acts and acts[0][0] == 'send'
    assert math.hypot(acts[0][2] - gx, acts[0][3] - gy) > 0.5


def test_state_is_plain_data():
    import json
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    logic.command('explore', 0.0, world.robot, nav_ready=True)
    state = json.loads(json.dumps(logic.state()))
    assert state['mode'] == 'explore'
    assert state['goal'] is not None and state['frontiers']
    assert state['home'] == {'x': 0.0, 'y': 0.0}


# ---- Stuck ---------------------------------------------------------------

def kinds(actions):
    return [a[0] for a in actions]


def test_stuck_exploring_backs_up_then_tries_another_frontier():
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    (first,) = logic.command('explore', 0.0, world.robot, nav_ready=True)
    acts = logic.stuck(5.0, (0.3, 0.1), can_back_up=True)
    assert acts == [('cancel',), ('sound', 'stuck'), ('back_up', 0.15)]
    assert logic.recovering and logic.mode == 'explore' and logic.goal is None
    assert logic.failed == [first[2:4]] and logic.stuck_spots == [(0.3, 0.1)]
    assert 'backing up' in logic.message
    # Nothing happens until it's done backing up, and the canceled goal's result is ignored
    assert logic.tick(7.0, world.robot) == []
    assert logic.goal_finished(first[1], False, 7.0, world.robot) == []
    assert logic.stuck(7.5, (0.3, 0.1), can_back_up=True) == []
    (nxt,) = logic.backup_done(8.0, world.robot)
    assert nxt[0] == 'send' and nxt[2:4] != first[2:4]
    assert not logic.recovering


def test_stuck_with_no_room_behind_moves_on_at_once():
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    (first,) = logic.command('explore', 0.0, world.robot, nav_ready=True)
    acts = logic.stuck(5.0, world.robot, can_back_up=False)
    assert kinds(acts) == ['cancel', 'sound', 'stop_wheels', 'send']
    assert acts[-1][2:4] != first[2:4] and not logic.recovering


def test_stuck_going_home_tries_again():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.stuck(5.0, (1.5, 1.0), can_back_up=True)
    (retry,) = logic.backup_done(8.0, (1.4, 1.0))
    assert retry[0] == 'send' and retry[2:] == (0.0, 0.0, 0.0)
    assert logic.mode == 'home' and 'try 2' in logic.message


def test_stuck_on_a_goto_tries_the_same_spot_again():
    logic = make_logic()
    logic.command('goto 2 0', 0.0, (0.0, 0.0), nav_ready=True)
    logic.stuck(5.0, (0.5, 0.0), can_back_up=True)
    (retry,) = logic.backup_done(8.0, (0.4, 0.0))
    assert retry[2:4] == (2.0, 0.0) and logic.mode == 'goto'


def test_stuck_three_times_in_a_row_asks_for_help():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    for n in (1, 2):
        logic.stuck(5.0 * n, (1.5, 1.0), can_back_up=True)
        logic.backup_done(5.0 * n + 2, (1.4, 1.0))
    acts = logic.stuck(20.0, (1.5, 1.0), can_back_up=True)
    assert acts == [('cancel',), ('stop_wheels',), ('sound', 'help')]
    assert logic.mode == 'idle' and 'needs help' in logic.message
    assert len(logic.stuck_spots) == 3


def test_reaching_a_goal_resets_the_count():
    logic = make_logic()
    logic.command('goto 2 0', 0.0, (0.0, 0.0), nav_ready=True)
    logic.stuck(5.0, (0.5, 0.0), can_back_up=True)
    (retry,) = logic.backup_done(8.0, (0.4, 0.0))
    assert logic.stuck_in_a_row == 1
    logic.goal_finished(retry[1], True, 20.0, (2.0, 0.0))
    assert logic.stuck_in_a_row == 0 and logic.message == 'Arrived'


def test_stop_while_backing_up():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.stuck(5.0, (1.5, 1.0), can_back_up=True)
    assert logic.command('stop', 6.0, (1.5, 1.0), nav_ready=True) == [('cancel',), ('stop_wheels',)]
    assert not logic.recovering and logic.mode == 'idle'
    assert logic.backup_done(7.0, (1.4, 1.0)) == []


def test_new_command_while_backing_up_takes_over():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.stuck(5.0, (1.5, 1.0), can_back_up=True)
    (act,) = logic.command('goto 1 1', 6.0, (1.5, 1.0), nav_ready=True)
    assert act[0] == 'send' and not logic.recovering and logic.stuck_in_a_row == 0
    assert logic.backup_done(7.0, (1.4, 1.0)) == []


def test_stuck_while_driving_by_hand_only_says_so():
    logic = make_logic()
    assert logic.stuck(5.0, (1.0, 1.0), can_back_up=True) == []
    assert logic.mode == 'idle' and 'Stuck?' in logic.message
    assert logic.stuck_spots == [] and logic.stuck_in_a_row == 0


def test_stuck_spots_are_in_the_state_and_cleared_by_a_new_explore():
    import json
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    logic.command('explore', 0.0, world.robot, nav_ready=True)
    logic.stuck(5.0, (0.3, 0.1), can_back_up=True)
    state = json.loads(json.dumps(logic.state()))
    assert state['stuck_spots'] == [{'x': 0.3, 'y': 0.1}] and state['stuck_in_a_row'] == 1
    logic.command('explore', 9.0, world.robot, nav_ready=True)
    assert logic.stuck_spots == [] and logic.failed == []


# ---- Pause / resume (driving by hand takes over, then it carries on) -------

def test_commands_pause_and_resume_parse():
    from lucia_nav.commands import parse_command
    assert parse_command(' Pause ') == ('pause',)
    assert parse_command('resume') == ('resume',)


def test_pause_explore_then_resume_keeps_what_it_learned():
    world = World()
    logic = make_logic()
    logic.map_updated(world.grid())
    logic.command('explore', 0.0, world.robot, nav_ready=True)
    logic.failed.append((5.0, 1.0))                              # a frontier that failed earlier
    assert logic.command('pause', 6.0, world.robot, nav_ready=True) == [('cancel',)]
    assert logic.mode == 'paused' and logic.goal is None and 'driving by hand' in logic.message
    assert logic.state()['paused_from'] == 'explore'
    assert logic.tick(8.0, world.robot) == []                    # stays put while paused
    (act,) = logic.command('resume', 9.0, world.robot, nav_ready=True)
    assert act[0] == 'send' and logic.mode == 'explore'
    assert len(logic.failed) == 1                                # not started over
    assert logic.state()['paused_from'] is None


def test_pause_home_and_goto_resume_the_same_goal():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.command('pause', 1.0, (2.0, 1.0), nav_ready=True)
    (act,) = logic.command('resume', 4.0, (1.0, 1.0), nav_ready=True)
    assert act[2:] == (0.0, 0.0, 0.0) and logic.mode == 'home'

    logic = make_logic()
    logic.command('goto 2 0', 0.0, (0.0, 0.0), nav_ready=True)
    logic.command('pause', 1.0, (0.5, 0.0), nav_ready=True)
    (act,) = logic.command('resume', 4.0, (0.0, 1.0), nav_ready=True)
    assert act[2:4] == (2.0, 0.0) and logic.mode == 'goto'
    assert act[4] == pytest.approx(math.atan2(-1.0, 2.0))       # faces the goal from where it is now


def test_old_goal_result_after_pause_is_ignored():
    logic = make_logic()
    (act,) = logic.command('goto 2 0', 0.0, (0.0, 0.0), nav_ready=True)
    logic.command('pause', 1.0, (0.5, 0.0), nav_ready=True)
    assert logic.goal_finished(act[1], False, 2.0, (0.5, 0.0)) == []
    assert logic.mode == 'paused'


def test_stop_while_paused_forgets_it():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.command('pause', 1.0, (2.0, 1.0), nav_ready=True)
    logic.command('stop', 2.0, (2.0, 1.0), nav_ready=True)
    assert logic.command('resume', 4.0, (2.0, 1.0), nav_ready=True) == []
    assert logic.mode == 'idle' and logic.message == 'Nothing to resume'


def test_pause_when_idle_or_already_paused_does_nothing():
    logic = make_logic()
    assert logic.command('pause', 0.0, (0, 0), nav_ready=True) == []
    assert logic.mode == 'idle'
    logic.command('home', 1.0, (2.0, 1.0), nav_ready=True)
    logic.command('pause', 2.0, (2.0, 1.0), nav_ready=True)
    assert logic.command('pause', 3.0, (2.0, 1.0), nav_ready=True) == []
    assert logic.state()['paused_from'] == 'home'


def test_new_command_while_paused_replaces_it():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.command('pause', 1.0, (2.0, 1.0), nav_ready=True)
    logic.command('goto 1 1', 2.0, (2.0, 1.0), nav_ready=True)
    assert logic.mode == 'goto' and logic.state()['paused_from'] is None


def test_pause_while_backing_up_then_resume():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.stuck(5.0, (1.5, 1.0), can_back_up=True)
    logic.command('pause', 6.0, (1.5, 1.0), nav_ready=True)
    assert not logic.recovering and logic.backup_done(7.0, (1.4, 1.0)) == []
    (act,) = logic.command('resume', 9.0, (1.0, 1.0), nav_ready=True)
    assert act[2:] == (0.0, 0.0, 0.0) and logic.stuck_in_a_row == 0


def test_stuck_while_paused_only_says_so():
    logic = make_logic()
    logic.command('home', 0.0, (2.0, 1.0), nav_ready=True)
    logic.command('pause', 1.0, (2.0, 1.0), nav_ready=True)
    assert logic.stuck(5.0, (2.0, 1.0), can_back_up=True) == []
    assert logic.mode == 'paused' and logic.stuck_spots == []
