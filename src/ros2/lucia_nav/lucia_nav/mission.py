"""
mission.py

ROS2 node that gives the robot something to do, by sending goals to Nav2:
- explore: frontier exploration. Drive to the edge of the known map, again and
  again, until nothing is left unexplored; then drive home.
- home: drive back to where mapping started (the map frame's origin).
- goto X Y: drive to one spot on the map (test tool: the web page's map click).
- stop: cancel whatever it's doing and stop the wheels.

Commands come in on /mission/command (std_msgs/String, the words above):
    ros2 topic pub --once /mission/command std_msgs/String "data: explore"
State goes out on /mission/state (std_msgs/String holding JSON) twice a
second, for the web page:
    ros2 topic echo /mission/state

The decisions are in mission_logic.py (tested without ROS). This file only
connects them to ROS: the map (/map), the robot's position (TF map->base_link)
and Nav2's navigate_to_pose action. Nav2 does the driving itself: it plans a
path and steers around what the lidar sees.

Stuck check (motion_check.py, stuck_check:=false turns it off): twice a second
it compares /odom (what the wheels were told) with /scan and /imu/data (what
really happened). Stuck = the wheels are driven but the robot isn't moving, for
about 2-3 s. Then mission_logic decides: back up 15 cm (straight back on
/cmd_vel, only if the lidar sees nothing behind), skip that frontier or try
the goal again; after 3 in a row, stop and play the 'help' tune on the Roomba.
The result is also in /mission/state ('motion') for the web page.

Usage (normally started by lucia_nav's navigation.launch.py):
    ros2 run lucia_nav mission

Runs on the Pi: Explore worked on the robot 2026-10-17.
"""

import json
import math
import time

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from lucia_nav.frontier import Grid
from lucia_nav.mission_logic import MissionLogic
from lucia_nav.motion_check import MotionCheck, clear_behind, scan_to_points, yaw_from_quaternion

# Tunes for the Roomba's speaker (roomba_bridge's ~/song): MIDI note, duration in 1/64 s
SOUNDS = {
    'stuck': [[67, 10], [60, 18]],                                  # "uh-oh"
    'help': [[72, 12], [67, 12], [64, 12], [60, 28], [0, 16]] * 3,  # falling, three times
}

STATUS_NAMES = {
    GoalStatus.STATUS_SUCCEEDED: 'succeeded',
    GoalStatus.STATUS_CANCELED: 'canceled',
    GoalStatus.STATUS_ABORTED: 'aborted',
}


class Mission(Node):
    def __init__(self):
        super().__init__('mission')

        self.declare_parameter('min_frontier_m', 0.3)       # shorter frontiers are mostly speckle
        self.declare_parameter('clearance_m', 0.25)         # robot's middle stays this far from walls (0.23 radius + margin)
        self.declare_parameter('frontier_gain', 0.5)        # how much a long frontier beats a near one
        self.declare_parameter('avoid_radius_m', 0.5)       # skip frontiers this close to failed goals
        self.declare_parameter('goal_timeout_s', 90.0)
        self.declare_parameter('return_home_when_done', True)
        self.declare_parameter('replan_period_s', 2.0)
        self.declare_parameter('stuck_check', True)
        self.declare_parameter('stuck_checks', 3)            # "not moving" answers in a row (2 a second)
        self.declare_parameter('max_stuck', 3)               # in a row, before giving up and calling for help
        self.declare_parameter('backup_m', 0.15)
        self.declare_parameter('backup_speed_mps', 0.08)
        self.declare_parameter('robot_radius_m', 0.23)       # for "is there room to back up?"

        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self._logic = MissionLogic(
            min_frontier_m=param('min_frontier_m'),
            clearance_m=param('clearance_m'),
            gain=param('frontier_gain'),
            avoid_radius_m=param('avoid_radius_m'),
            goal_timeout_s=param('goal_timeout_s'),
            return_home_when_done=param('return_home_when_done'),
            max_stuck=param('max_stuck'),
            backup_m=param('backup_m'),
        )
        self._stuck_check = param('stuck_check')
        self._motion = MotionCheck(checks_to_stuck=param('stuck_checks'))
        self._backup_speed = param('backup_speed_mps')
        self._robot_radius = param('robot_radius_m')
        self._backup_until = None       # time.monotonic() when backing up ends, while backing up
        self._laser_pose = None         # where the lidar sits on the robot (x, y, yaw), from TF
        self._last_scan = None

        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._handles = {}          # goal_id -> goal handle, while Nav2 is working on it

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        self.create_subscription(String, 'mission/command', self._on_command, 10)
        self._state_pub = self.create_publisher(String, 'mission/state', 10)
        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._song_pub = self.create_publisher(String, 'roomba_bridge/song', 10)

        self.create_timer(param('replan_period_s'), self._on_tick)
        self.create_timer(0.5, self._publish_state)
        self.create_timer(0.1, self._backup_tick)
        if self._stuck_check:
            self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
            self.create_subscription(Odometry, 'odom', self._on_odom, 10)
            self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)
            self.create_timer(0.5, self._check_motion)

        self.get_logger().info('Mission ready: explore / home / goto X Y / stop on /mission/command'
                               + ('' if self._stuck_check else ' (stuck check off)'))

    # ------------------------------------------------------------------
    # Inputs
    # ------------------------------------------------------------------

    def _robot_xy(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            return None
        return (t.transform.translation.x, t.transform.translation.y)

    def _on_map(self, msg):
        info = msg.info
        try:
            grid = Grid(msg.data, info.width, info.height, info.resolution,
                        info.origin.position.x, info.origin.position.y)
        except ValueError as e:
            self.get_logger().warn(f'Ignoring bad map: {e}')
            return
        self._logic.map_updated(grid)

    def _on_odom(self, msg):
        q = msg.pose.pose.orientation
        self._motion.odom(msg.pose.pose.position.x, msg.pose.pose.position.y,
                          yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_imu(self, msg):
        q = msg.orientation
        if q.x == q.y == q.z == q.w == 0.0:
            return                  # no orientation in this message
        self._motion.imu(time.monotonic(), yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _on_scan(self, msg):
        if self._laser_pose is None:
            try:
                t = self._tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
            except TransformException:
                return
            r = t.transform.rotation
            self._laser_pose = (t.transform.translation.x, t.transform.translation.y,
                                yaw_from_quaternion(r.x, r.y, r.z, r.w))
        self._last_scan = msg
        # For lining scans up: leave out the robot's own parts (and anything touching
        # it), far readings, and walls seen edge-on (motion_check.scan_to_points)
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, self._laser_pose,
                                min_range=0.3, max_range=4.0, max_spread=2.5)
        self._motion.scan(time.monotonic(), points)

    def _check_motion(self):
        if self._backup_until is not None:
            return                  # backing up on purpose; checked again from scratch afterwards
        moving = self._motion.check()
        if moving is False:
            self.get_logger().debug(f'Not moving as commanded: {self._motion.last}')
        if not self._motion.stuck:
            return
        last = self._motion.last
        self.get_logger().warn(
            "Stuck: the wheels are driven but the robot isn't moving "
            f"(drove {_pct(last['driven'])}, turned {_pct(last['turned'])} of what was commanded)")
        self._motion.reset()
        before = self._logic.message
        self._do(self._logic.stuck(time.monotonic(), self._robot_xy(), self._room_behind()))
        if self._logic.message != before:
            self.get_logger().info(self._logic.message)
        self._publish_state()

    def _room_behind(self):
        msg = self._last_scan
        if msg is None or self._laser_pose is None:
            return False
        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment, self._laser_pose,
                                min_range=msg.range_min, max_range=1.0, max_points=720)
        return clear_behind(points, self._robot_radius, self._logic.backup_m)

    def _backup_tick(self):
        if self._backup_until is None:
            return
        if time.monotonic() < self._backup_until:
            msg = Twist()
            msg.linear.x = -self._backup_speed
            self._cmd_pub.publish(msg)
            return
        self._backup_until = None
        self._cmd_pub.publish(Twist())
        self._motion.reset()
        before = self._logic.message
        self._do(self._logic.backup_done(time.monotonic(), self._robot_xy()))
        if self._logic.message != before:
            self.get_logger().info(self._logic.message)
        self._publish_state()

    def _on_command(self, msg):
        self.get_logger().info(f'Command: {msg.data!r}')
        before = self._logic.message
        self._do(self._logic.command(msg.data, time.monotonic(), self._robot_xy(),
                                     self._nav.server_is_ready()))
        if self._logic.message != before:
            self.get_logger().info(self._logic.message)
        self._publish_state()

    def _on_tick(self):
        before = self._logic.message
        self._do(self._logic.tick(time.monotonic(), self._robot_xy()))
        if self._logic.message != before:
            self.get_logger().info(self._logic.message)

    # ------------------------------------------------------------------
    # Carrying out the logic's actions
    # ------------------------------------------------------------------

    def _do(self, actions):
        for action in actions:
            if action[0] in ('send', 'cancel', 'stop_wheels'):
                self._backup_until = None       # whatever comes next replaces backing up
            if action[0] == 'send':
                self._send_goal(*action[1:])
            elif action[0] == 'cancel':
                for handle in self._handles.values():
                    handle.cancel_goal_async()
                self._handles.clear()
            elif action[0] == 'stop_wheels':
                self._cmd_pub.publish(Twist())
            elif action[0] == 'back_up':
                self._motion.reset()
                self._backup_until = time.monotonic() + action[1] / self._backup_speed
            elif action[0] == 'sound':
                self._song_pub.publish(String(data=json.dumps({'notes': SOUNDS[action[1]]})))

    def _send_goal(self, goal_id, x, y, yaw):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.get_logger().info(f'Goal {goal_id}: ({x:.2f}, {y:.2f}), facing {math.degrees(yaw):.0f} deg')
        future = self._nav.send_goal_async(goal)
        future.add_done_callback(lambda f, gid=goal_id: self._on_goal_response(gid, f))

    def _on_goal_response(self, goal_id, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn(f'Nav2 rejected goal {goal_id}')
            self._finished(goal_id, False)
            return
        if goal_id != self._logic.goal_id:
            # Stopped or replaced while the goal was on its way to Nav2.
            # (A replaced goal is already over on Nav2's side; canceling it is harmless.)
            handle.cancel_goal_async()
            return
        self._handles[goal_id] = handle
        handle.get_result_async().add_done_callback(
            lambda f, gid=goal_id: self._on_result(gid, f))

    def _on_result(self, goal_id, future):
        self._handles.pop(goal_id, None)
        status = future.result().status
        if goal_id == self._logic.goal_id:
            self.get_logger().info(f'Goal {goal_id} {STATUS_NAMES.get(status, status)}')
        self._finished(goal_id, status == GoalStatus.STATUS_SUCCEEDED)

    def _finished(self, goal_id, succeeded):
        before = self._logic.message
        self._do(self._logic.goal_finished(goal_id, succeeded, time.monotonic(), self._robot_xy()))
        if self._logic.message != before:
            self.get_logger().info(self._logic.message)
        self._publish_state()

    def _publish_state(self):
        state = self._logic.state()
        state['nav_ready'] = self._nav.server_is_ready()
        state['motion'] = dict(self._motion.last, still_checks=self._motion.still_checks,
                               enabled=self._stuck_check)
        self._state_pub.publish(String(data=json.dumps(state)))

    def shutdown(self):
        # On Ctrl+C rclpy may already be shut down; then Nav2 is going down too.
        try:
            self._do([('cancel',), ('stop_wheels',)])
        except Exception:  # noqa: BLE001
            pass


def _pct(fraction):
    return '-' if fraction is None else f'{fraction * 100:.0f}%'


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Mission()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
