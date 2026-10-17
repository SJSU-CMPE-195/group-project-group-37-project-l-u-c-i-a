"""
safety_gate.py

ROS2 node that sits between everything that drives the robot (the web page's
drive pad, Nav2, the mission node) and roomba_bridge:

    /cmd_vel  ->  safety_gate  ->  /cmd_vel_safe  ->  roomba_bridge

With use_tof on, it blocks forward motion while the ToF sensors (Noel's
lucia_tof, topic tof/near) see something close, or while their signal is
missing or stale. Turning and backing up still go through. This stands in for
the removed bump sensor: it catches things below the lidar that Nav2 can't see.

With use_tof off (the default until the ToF sensors are mounted), forward
motion is never blocked.

Tight spaces (tight_space, on by default): full speed in the open, slower close
to things. From each lidar scan it measures the room around the robot's front
half and the free run straight ahead (safety_logic.clearances) and caps forward
speed between max_speed_mps (0.30, roomba_bridge's own limit: 0.35 m of room
around it and 0.8 m clear ahead) and min_speed_mps (0.06: within 5 cm, or 0.2 m
ahead). In a 0.7 m doorway that's about 12 cm/s; 20 cm from a wall about
18 cm/s. Turning is slowed in proportion so the robot keeps to its curve;
backing up and turning in place aren't touched. It never stops the robot. With
no fresh scan it allows only min_speed_mps (can't see: creep).

mapping.launch.py starts it and points roomba_bridge at /cmd_vel_safe:
    ros2 launch lucia_bringup mapping.launch.py use_tof:=true
lucia_tof's own node must be running for tof/near to exist; without it,
use_tof:=true blocks all forward driving (missing = danger).

Runs on the Pi in pass-through (use_tof:=false) as part of mapping.launch.py since
2026-10-16. The blocking path (use_tof:=true) is not yet tried.
"""

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformException, TransformListener

from lucia_bringup.safety_logic import (clearances, gate, limit_forward, scan_points,
                                        tight_space_cap, tof_blocked)


class SafetyGate(Node):
    def __init__(self):
        super().__init__('safety_gate')
        self.declare_parameter('use_tof', False)
        self.declare_parameter('stale_s', 0.5)      # lucia_tof sends tof/near at least 10 Hz
        self.declare_parameter('tight_space', True)
        self.declare_parameter('max_speed_mps', 0.30)
        self.declare_parameter('min_speed_mps', 0.06)
        self.declare_parameter('robot_radius_m', 0.23)     # 18 in across (nav2.yaml has the same)
        self.declare_parameter('scan_stale_s', 0.5)        # the lidar sends ~12 scans a second

        self._use_tof = self.get_parameter('use_tof').value
        self._stale_s = self.get_parameter('stale_s').value
        self._tight_space = self.get_parameter('tight_space').value
        self._max_speed = self.get_parameter('max_speed_mps').value
        self._min_speed = self.get_parameter('min_speed_mps').value
        self._robot_radius = self.get_parameter('robot_radius_m').value
        self._scan_stale_s = self.get_parameter('scan_stale_s').value
        self._cap = None                        # (time, m/s) from the newest scan
        self._laser_pose = None                 # where the lidar sits on the robot, from TF
        self._blind = False
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
            self.get_logger().info('ToF safety stop OFF (use_tof:=false): forward driving never blocked')
        if self._tight_space:
            self._tf_buffer = Buffer()
            self._tf_listener = TransformListener(self._tf_buffer, self)
            self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
            self.get_logger().info(
                f'Tight spaces: forward speed {self._max_speed:.2f} m/s in the open, '
                f'down to {self._min_speed:.2f} m/s close to things (from the lidar)')

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

    def _on_scan(self, msg):
        if self._laser_pose is None:
            try:
                t = self._tf_buffer.lookup_transform('base_link', msg.header.frame_id, Time())
            except TransformException:
                return
            q = t.transform.rotation
            self._laser_pose = (t.transform.translation.x, t.transform.translation.y,
                                math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        points = scan_points(msg.ranges, msg.angle_min, msg.angle_increment, self._laser_pose,
                             min_range=msg.range_min)
        cap = tight_space_cap(*clearances(points, self._robot_radius), self._min_speed, self._max_speed)
        self._cap = (time.monotonic(), cap)

    def _speed_cap(self):
        """Forward speed allowed right now, from the newest scan; min speed if there isn't a fresh one."""
        fresh = self._cap is not None and time.monotonic() - self._cap[0] <= self._scan_stale_s
        if fresh == self._blind:
            self._blind = not fresh
            if self._blind:
                self.get_logger().warn(
                    f'Tight spaces: no fresh lidar scan, forward speed held to {self._min_speed:.2f} m/s')
            else:
                self.get_logger().info('Tight spaces: lidar scans arriving again')
        return self._cap[1] if fresh else self._min_speed

    def _on_cmd(self, msg):
        blocked = self._use_tof and self._blocked
        linear, angular = gate(msg.linear.x, msg.angular.z, blocked)
        if self._tight_space:
            linear, angular = limit_forward(linear, angular, self._speed_cap())
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
