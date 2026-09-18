"""
roomba_bridge.py

ROS2 node that owns the Roomba's serial port (nothing else may open it).

- Listens on /cmd_vel and turns it into left/right wheel speeds
- Keeps the Roomba in Safe mode by re-sending the mode command on a timer, so it
  recovers on its own after a wheel-drop / cliff stop (no restart needed)
- Stops the wheels if /cmd_vel goes quiet, since the Roomba would otherwise keep
  driving at the last speed forever

Wheel sensors (/odom) and battery are not read yet.

Usage:
    ros2 run lucia_control roomba_bridge --ros-args -p port:=/dev/ttyUSB0
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from lucia_control.motion import twist_to_wheel_speeds
from lucia_control.roomba_oi import RoombaOI


class RoombaBridge(Node):
    def __init__(self):
        super().__init__('roomba_bridge')

        self.declare_parameter('port', '/dev/roomba')
        self.declare_parameter('oi_mode', 'safe')           # 'full' = bench testing only
        self.declare_parameter('wheel_base_mm', 235.0)      # same value the old scripts use
        self.declare_parameter('max_speed_mm_s', 300.0)
        self.declare_parameter('cmd_vel_timeout_s', 0.5)
        self.declare_parameter('drive_period_s', 0.1)
        self.declare_parameter('heartbeat_period_s', 1.0)

        port = self.get_parameter('port').value
        self._oi_mode = self.get_parameter('oi_mode').value
        self._wheel_base_mm = self.get_parameter('wheel_base_mm').value
        self._max_speed_mm_s = self.get_parameter('max_speed_mm_s').value
        self._cmd_vel_timeout_s = self.get_parameter('cmd_vel_timeout_s').value
        drive_period_s = self.get_parameter('drive_period_s').value
        heartbeat_period_s = self.get_parameter('heartbeat_period_s').value

        # Check settings before touching the serial port
        if self._oi_mode not in ('safe', 'full'):
            raise ValueError(f"oi_mode must be 'safe' or 'full', got '{self._oi_mode}'")
        if self._wheel_base_mm <= 0:
            raise ValueError('wheel_base_mm must be greater than 0')
        if not 0 < self._max_speed_mm_s <= 500:
            raise ValueError('max_speed_mm_s must be between 0 and 500 (Roomba limit)')
        if min(self._cmd_vel_timeout_s, drive_period_s, heartbeat_period_s) <= 0:
            raise ValueError('timeout and period settings must be greater than 0')

        self._left = 0
        self._right = 0
        self._last_cmd_time = None
        self._timed_out = False

        self.get_logger().info(f'Opening Roomba on {port} (takes a few seconds)')
        self._roomba = RoombaOI(port)
        self._roomba.start()
        self._enter_mode()
        if self._oi_mode == 'full':
            self.get_logger().warning(
                'Full mode: the Roomba will NOT stop by itself when lifted. Bench testing only.')

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(drive_period_s, self._drive_tick)
        self.create_timer(heartbeat_period_s, self._enter_mode)

        self.get_logger().info(
            f'Ready: mode={self._oi_mode}, wheel_base={self._wheel_base_mm} mm, '
            f'max_speed={self._max_speed_mm_s} mm/s, cmd_vel_timeout={self._cmd_vel_timeout_s} s')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_cmd_vel(self, msg):
        """Store the newest command. Only linear.x and angular.z are used."""
        self._left, self._right = twist_to_wheel_speeds(
            msg.linear.x, msg.angular.z, self._wheel_base_mm, self._max_speed_mm_s)
        self._last_cmd_time = self.get_clock().now()

    def _drive_tick(self):
        """Send the current wheel command, or zero if /cmd_vel has gone quiet."""
        stale = self._command_is_stale()
        if stale and not self._timed_out and self._last_cmd_time is not None:
            self.get_logger().info('No /cmd_vel recently - stopping wheels')
        self._timed_out = stale

        left, right = (0, 0) if stale else (self._left, self._right)
        self._roomba.drive_direct(left, right)

    def _enter_mode(self):
        """Put (or keep) the Roomba in the configured mode. Safe to call repeatedly."""
        if self._oi_mode == 'full':
            self._roomba.full_mode()
        else:
            self._roomba.safe_mode()

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _command_is_stale(self):
        if self._last_cmd_time is None:
            return True
        age_s = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        return age_s > self._cmd_vel_timeout_s

    def shutdown(self):
        """Stop the wheels, reset the Roomba, and release the serial port."""
        self.get_logger().info('Stopping wheels and releasing the Roomba')
        try:
            self._roomba.close()
        except Exception as exc:
            self.get_logger().error(f'Could not close the Roomba cleanly: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RoombaBridge()
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
