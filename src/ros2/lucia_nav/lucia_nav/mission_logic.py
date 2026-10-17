"""
mission_logic.py

The mission node's decisions, without ROS, so they can be tested anywhere.
mission.py feeds this events (a command, a new map, a goal finished, a timer
tick) and carries out the actions it returns:

    ('send', goal_id, x, y, yaw)   send a Nav2 NavigateToPose goal (map frame)
    ('cancel',)                    cancel every Nav2 goal this node sent
    ('stop_wheels',)               publish one zero /cmd_vel

Modes:
    idle      nothing to do
    goto      driving to one spot (the web page's map click)
    home      driving back to where mapping started
    explore   frontier exploration: drive to the best frontier, repeat; when
              none are left, go home (or stop, if return_home_when_done is off)

Home is the map frame's origin: slam_toolbox starts the map where the robot is
when mapping starts, facing the same way.
"""

import math

from lucia_nav.commands import parse_command
from lucia_nav.frontier import choose_frontier, find_frontiers, near_any

HOME = (0.0, 0.0, 0.0)      # x, y, yaw in the map frame


class MissionLogic:
    def __init__(self, min_frontier_m=0.3, clearance_m=0.2, gain=0.5,
                 avoid_radius_m=0.5, goal_timeout_s=90.0, return_home_when_done=True):
        self.min_frontier_m = min_frontier_m
        self.clearance_m = clearance_m
        self.gain = gain
        self.avoid_radius_m = avoid_radius_m
        self.goal_timeout_s = goal_timeout_s
        self.return_home_when_done = return_home_when_done

        self.mode = 'idle'
        self.message = 'Ready'
        self.goal = None            # (x, y) of the goal being driven to, or None
        self.frontiers = []
        self.failed = []            # frontier goals that didn't work out this run
        self._goal_id = 0
        self._goal_started = None   # when driving to the current target began
        self._grid = None
        self._map_changed = False

    @property
    def goal_id(self):
        """Id of the goal being driven to, None if none. Results for other ids are stale."""
        return self._goal_id if self.goal is not None else None

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def map_updated(self, grid):
        self._grid = grid
        self._map_changed = True

    def command(self, text, now, robot_xy, nav_ready):
        cmd = parse_command(text)
        if cmd is None:
            self.message = f'Unknown command: {text!r}'
            return []
        if cmd[0] == 'stop':
            was = self.mode
            self._idle('Stopped' if was != 'idle' else self.message)
            return [('cancel',), ('stop_wheels',)]
        if not nav_ready:
            self.message = "Navigation (Nav2) isn't running yet"
            return []
        if cmd[0] == 'home':
            self.mode = 'home'
            self.message = 'Going home'
            return self._send(now, *HOME)
        if cmd[0] == 'goto':
            _, x, y = cmd
            self.mode = 'goto'
            self.message = f'Going to ({x:.2f}, {y:.2f})'
            return self._send(now, x, y, _heading(robot_xy, (x, y)))
        # explore
        self.mode = 'explore'
        self.failed = []
        self.goal = None
        self._map_changed = True    # recompute from where the robot is now
        return self._next_frontier(now, robot_xy)

    def goal_finished(self, goal_id, succeeded, now, robot_xy):
        if goal_id != self._goal_id or self.goal is None:
            return []           # an old goal (replaced or stopped); ignore
        goal = self.goal
        self.goal = None
        if self.mode == 'goto':
            self._idle('Arrived' if succeeded else 'Could not reach that spot')
        elif self.mode == 'home':
            self._idle('Home' if succeeded else 'Could not get home')
        elif self.mode == 'explore':
            if not succeeded:
                self.failed.append(goal)
            return self._next_frontier(now, robot_xy)
        return []

    def tick(self, now, robot_xy):
        """Call every few seconds."""
        changed = self._refresh_frontiers(robot_xy)
        if self.mode == 'explore':
            if self.goal is None:
                return self._next_frontier(now, robot_xy)
            if now - self._goal_started > self.goal_timeout_s:
                self.failed.append(self.goal)
                return self._next_frontier(now, robot_xy)
            if changed and not near_any(self.goal, self.frontiers, self.avoid_radius_m):
                # The lidar already saw past this frontier on the way; move on.
                return self._next_frontier(now, robot_xy)
        elif self.mode in ('home', 'goto') and self.goal is not None:
            if now - self._goal_started > self.goal_timeout_s:
                self._idle('Took too long; stopped')
                return [('cancel',), ('stop_wheels',)]
        return []

    def state(self):
        """What the web page shows, as plain data (sent as JSON)."""
        return {
            'mode': self.mode,
            'message': self.message,
            'goal': None if self.goal is None else {'x': self.goal[0], 'y': self.goal[1]},
            'home': {'x': HOME[0], 'y': HOME[1]},
            'frontiers': [{'x': f.x, 'y': f.y, 'length_m': f.length_m} for f in self.frontiers[:100]],
            'failed': [{'x': x, 'y': y} for x, y in self.failed],
        }

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _idle(self, message):
        self.mode = 'idle'
        self.message = message
        self.goal = None
        self._goal_id += 1      # any result still on its way is now stale

    def _send(self, now, x, y, yaw):
        # Keep the timer running if this is only a nudged version of the same
        # target (frontiers shift a little as the map fills in), so a target
        # that keeps moving still times out.
        same_target = (self.goal is not None
                       and math.hypot(x - self.goal[0], y - self.goal[1]) < self.avoid_radius_m)
        if not same_target:
            self._goal_started = now
        self._goal_id += 1
        self.goal = (x, y)
        return [('send', self._goal_id, x, y, yaw)]

    def _refresh_frontiers(self, robot_xy):
        """Recompute frontiers if the map changed. They depend on where the robot is (reachability)."""
        if not self._map_changed or self._grid is None or robot_xy is None:
            return False
        self._map_changed = False
        self.frontiers = find_frontiers(self._grid, robot_xy, self.min_frontier_m, self.clearance_m)
        return True

    def _next_frontier(self, now, robot_xy):
        if self._grid is None or robot_xy is None:
            self.goal = None
            self.message = 'Exploring: waiting for the map and the robot position'
            return []
        self._refresh_frontiers(robot_xy)
        target = choose_frontier(self.frontiers, robot_xy, self.failed,
                                 self.avoid_radius_m, self.gain)
        if target is None:
            if self.return_home_when_done:
                self.mode = 'home'
                self.message = 'Exploration finished. Going home'
                return self._send(now, *HOME)
            self._idle('Exploration finished')
            return [('cancel',)]
        left = sum(1 for f in self.frontiers
                   if not any(math.hypot(f.x - x, f.y - y) < self.avoid_radius_m for x, y in self.failed))
        self.message = f'Exploring: {left} frontier{"s" if left != 1 else ""} left'
        return self._send(now, target.x, target.y, _heading(robot_xy, (target.x, target.y)))


def _heading(from_xy, to_xy):
    """Yaw pointing from one point to another (0 if unknown or the same point)."""
    if from_xy is None:
        return 0.0
    dx, dy = to_xy[0] - from_xy[0], to_xy[1] - from_xy[1]
    if abs(dx) < 1e-9 and abs(dy) < 1e-9:
        return 0.0
    return math.atan2(dy, dx)
