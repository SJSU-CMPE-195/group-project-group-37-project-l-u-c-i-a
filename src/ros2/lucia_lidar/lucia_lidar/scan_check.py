"""
scan_check.py

Prints the closest thing the lidar sees, once a second, as a distance and an
angle in the lidar's own frame (0 deg = that frame's forward, positive =
counter-clockwise / to the left).

Used to find out which way the lidar faces on the robot: put a box about 30 cm
straight in front of the robot, with nothing else that close.
- Reads about +/-180 deg: keep the default laser_yaw (pi) in lidar_and_map.launch.py
- Reads about 0 deg: launch with laser_yaw:=0.0
- Anything else: tell Claude the angle

Usage (with the lidar running):
    ros2 run lucia_lidar scan_check
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanCheck(Node):
    def __init__(self):
        super().__init__('scan_check')
        self._last_print_s = 0.0
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)

    def _on_scan(self, msg):
        now_s = self.get_clock().now().nanoseconds / 1e9
        if now_s - self._last_print_s < 1.0:
            return
        self._last_print_s = now_s

        closest = None
        for i, r in enumerate(msg.ranges):
            if msg.range_min < r < msg.range_max and (closest is None or r < closest[0]):
                closest = (r, i)
        if closest is None:
            self.get_logger().info('no valid readings')
            return

        r, i = closest
        angle_deg = math.degrees(msg.angle_min + i * msg.angle_increment)
        angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
        self.get_logger().info(f'closest: {r:.2f} m at {angle_deg:+.0f} deg')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ScanCheck()
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
