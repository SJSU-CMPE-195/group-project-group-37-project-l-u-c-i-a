"""
safety_gate.py

ROS2 node that sits between everything that drives the robot (the web page's
drive pad, Nav2, the mission node) and roomba_bridge:

    /cmd_vel  ->  safety_gate  ->  /cmd_vel_safe  ->  roomba_bridge

With use_tof on, it blocks forward motion while the ToF sensors (Noel's
lucia_tof, topic tof/near) see something close, or while their signal is
missing or stale. Turning and backing up still go through. This stands in for
the removed bump sensor: it catches things below the lidar that Nav2 can't see.

With use_tof off (the default until the ToF sensors are mounted), commands
pass through unchanged.

mapping.launch.py starts it and points roomba_bridge at /cmd_vel_safe:
    ros2 launch lucia_bringup mapping.launch.py use_tof:=true
lucia_tof's own node must be running for tof/near to exist; without it,
use_tof:=true blocks all forward driving (missing = danger).

Runs on the Pi in pass-through (use_tof:=false) as part of mapping.launch.py since
2026-10-16. The blocking path (use_tof:=true) is not yet tried.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool

from lucia_bringup.safety_logic import gate, tof_blocked


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('use_tof', False)
        self.declare_parameter('stale_s', 0.5)      # lucia_tof sends tof/near at least 10 Hz

        self._use_tof = self.get_parameter('use_tof').value
        self._stale_s = self.get_parameter('stale_s').value
        self._near = None
        self._near_time = None
        self._blocked = self._use_tof           # nothing heard yet: blocked
        self._last_angular = 0.0

        self._pub = self.create_publisher(Twist, 'cmd_vel_safe', 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd, 10)
        if self._use_tof:
            self.create_subscription(Bool, 'tof/near', self._on_near, 10)
            self.create_timer(0.05, self._check)
            self.get_logger().info('ToF safety stop ON: forward driving stops while tof/near is true or silent')
        else:
            self.get_logger().info('ToF safety stop OFF (use_tof:=false): passing drive commands through')

    def _on_near(self, msg):
        self._near = msg.data
        self._near_time = time.monotonic()
        self._check()

    def _check(self):
        age = None if self._near_time is None else time.monotonic() - self._near_time
        blocked = tof_blocked(self._near, age, self._stale_s)
        if blocked and not self._blocked:
            # Don't wait for the next drive command: stop now (turning is kept).
            msg = Twist()
            msg.angular.z = self._last_angular
            self._pub.publish(msg)
            self.get_logger().warn('Safety stop: something close in front (or ToF silent); forward blocked')
        elif self._blocked and not blocked:
            self.get_logger().info('Safety stop cleared')
        self._blocked = blocked

    def _on_cmd(self, msg):
        blocked = self._use_tof and self._blocked
        linear, angular = gate(msg.linear.x, msg.angular.z, blocked)
        self._last_angular = angular
        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SafetyGate()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
