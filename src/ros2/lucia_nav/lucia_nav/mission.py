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
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from lucia_nav.frontier import Grid
from lucia_nav.mission_logic import MissionLogic

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

        param = lambda name: self.get_parameter(name).value  # noqa: E731
        self._logic = MissionLogic(
            min_frontier_m=param('min_frontier_m'),
            clearance_m=param('clearance_m'),
            gain=param('frontier_gain'),
            avoid_radius_m=param('avoid_radius_m'),
            goal_timeout_s=param('goal_timeout_s'),
            return_home_when_done=param('return_home_when_done'),
        )

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

        self.create_timer(param('replan_period_s'), self._on_tick)
        self.create_timer(0.5, self._publish_state)

        self.get_logger().info('Mission ready: explore / home / goto X Y / stop on /mission/command')

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
            if action[0] == 'send':
                self._send_goal(*action[1:])
            elif action[0] == 'cancel':
                for handle in self._handles.values():
                    handle.cancel_goal_async()
                self._handles.clear()
            elif action[0] == 'stop_wheels':
                self._cmd_pub.publish(Twist())

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
        self._state_pub.publish(String(data=json.dumps(state)))

    def shutdown(self):
        # On Ctrl+C rclpy may already be shut down; then Nav2 is going down too.
        try:
            self._do([('cancel',), ('stop_wheels',)])
        except Exception:  # noqa: BLE001
            pass


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
