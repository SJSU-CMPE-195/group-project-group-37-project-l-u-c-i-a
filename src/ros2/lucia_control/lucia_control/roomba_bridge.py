"""
roomba_bridge.py

ROS2 node that owns the Roomba's serial port (nothing else may open it).

- Listens on /cmd_vel and turns it into left/right wheel speeds
- Keeps the Roomba in Safe mode by re-sending the mode command on a timer, so it
  recovers on its own after a wheel-drop / cliff stop (no restart needed)
- Stops the wheels if /cmd_vel goes quiet, since the Roomba would otherwise keep
  driving at the last speed forever
- Publishes /odom and the odom->base_link transform, estimated from the speeds
  it COMMANDS the wheels to run at, not from a sensor reading - see
  odometry.py and docs/changelogs/2026-10-09.md for why. This will drift more
  than real encoder feedback, but it's what's available until the Roomba's
  sensor replies reach the Pi again.
- publish_tf:=false leaves the odom->base_link transform to robot_localization
  (lucia_bringup's mapping.launch.py), which combines /odom's forward speed
  with the IMU's heading. /odom itself is still published either way.

- Service ~/reconnect (/roomba_bridge/reconnect, std_srvs/Trigger): for when the
  Roomba was turned off and on, or its battery died and it was put back on. It
  reopens the serial port, wakes the Roomba, sends Start + the mode, and beeps.
  The heartbeat alone can't do that: it only re-sends Safe (131), which a
  freshly powered-on Roomba ignores until it gets Start (128). Nothing comes back
  from the Roomba over serial (TODO.md, Phase 1 Step B), so the beep is the only
  confirmation. The web page's "Reconnect Roomba" button calls it; mapping keeps
  running, so the map isn't lost.
      ros2 service call /roomba_bridge/reconnect std_srvs/srv/Trigger
- A serial error (adapter unplugged) is logged instead of crashing the node;
  plug it back in and reconnect.

Battery is not read yet.

Usage:
    ros2 run lucia_control roomba_bridge --ros-args -p port:=/dev/ttyUSB0
"""

import math

import rclpy
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from nav_msgs.msg import Odometry as OdometryMsg
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from serial import SerialException
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from lucia_control.motion import twist_to_wheel_speeds
from lucia_control.odometry import Odometry
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
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('publish_tf', True)
        # Rough guesses for how far off the commanded speeds are, as variances in
        # /odom's twist - robot_localization needs nonzero values to weigh /odom
        # against the IMU. Not measured; revisit with real encoder data.
        self.declare_parameter('linear_velocity_variance', 0.0025)    # (m/s)^2, ~5 cm/s
        self.declare_parameter('angular_velocity_variance', 0.04)     # (rad/s)^2, ~11 deg/s

        port = self.get_parameter('port').value
        self._oi_mode = self.get_parameter('oi_mode').value
        self._wheel_base_mm = self.get_parameter('wheel_base_mm').value
        self._max_speed_mm_s = self.get_parameter('max_speed_mm_s').value
        self._cmd_vel_timeout_s = self.get_parameter('cmd_vel_timeout_s').value
        drive_period_s = self.get_parameter('drive_period_s').value
        heartbeat_period_s = self.get_parameter('heartbeat_period_s').value
        self._odom_frame_id = self.get_parameter('odom_frame_id').value
        self._base_frame_id = self.get_parameter('base_frame_id').value
        self._publish_tf = self.get_parameter('publish_tf').value
        self._linear_velocity_variance = self.get_parameter('linear_velocity_variance').value
        self._angular_velocity_variance = self.get_parameter('angular_velocity_variance').value

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
        self._odom = Odometry(self._wheel_base_mm)
        self._last_tick_time = None

        self._port = port
        self._serial_error = False
        self.get_logger().info(f'Opening Roomba on {port} (takes a few seconds)')
        self._roomba = RoombaOI(port)
        self._roomba.start()
        self._enter_mode()
        if self._oi_mode == 'full':
            self.get_logger().warning(
                'Full mode: the Roomba will NOT stop by itself when lifted. Bench testing only.')

        self._odom_pub = self.create_publisher(OdometryMsg, 'odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(drive_period_s, self._drive_tick)
        self.create_timer(heartbeat_period_s, self._enter_mode)
        self.create_service(Trigger, '~/reconnect', self._on_reconnect)

        self.get_logger().info(
            f'Ready: mode={self._oi_mode}, wheel_base={self._wheel_base_mm} mm, '
            f'max_speed={self._max_speed_mm_s} mm/s, cmd_vel_timeout={self._cmd_vel_timeout_s} s, '
            f'odom is open-loop (commanded speed, not sensor-measured) - see odometry.py, '
            f'odom->base_link transform: {"published here" if self._publish_tf else "left to robot_localization"}')

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
        if self._roomba is None:
            left, right = 0, 0
        self._write(lambda roomba: roomba.drive_direct(left, right))
        # Estimated from the speeds just sent above, not a sensor reading - see odometry.py.
        self._update_and_publish_odom(left, right)

    def _update_and_publish_odom(self, commanded_left_mm_s, commanded_right_mm_s):
        now = self.get_clock().now()
        dt_s = 0.0 if self._last_tick_time is None else (now - self._last_tick_time).nanoseconds / 1e9
        self._last_tick_time = now
        self._odom.update(commanded_left_mm_s, commanded_right_mm_s, dt_s)

        x_mm, y_mm, heading_rad = self._odom.pose()
        quat = _yaw_to_quaternion(heading_rad)
        stamp = now.to_msg()

        msg = OdometryMsg()
        msg.header.stamp = stamp
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.pose.pose.position.x = x_mm / 1000.0
        msg.pose.pose.position.y = y_mm / 1000.0
        msg.pose.pose.orientation = quat
        # Pose covariance not filled in: nothing fuses /odom's pose, and its true
        # uncertainty isn't known.
        # Twist (in base_link): what robot_localization fuses. Sideways speed is
        # always 0 for a two-wheeled robot, so that one is near-certain.
        msg.twist.twist.linear.x = (commanded_left_mm_s + commanded_right_mm_s) / 2.0 / 1000.0
        msg.twist.twist.angular.z = (
            (commanded_right_mm_s - commanded_left_mm_s) / self._wheel_base_mm)
        msg.twist.covariance[0] = self._linear_velocity_variance
        msg.twist.covariance[7] = 1e-4
        msg.twist.covariance[35] = self._angular_velocity_variance
        self._odom_pub.publish(msg)

        if not self._publish_tf:
            return
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self._odom_frame_id
        tf.child_frame_id = self._base_frame_id
        tf.transform.translation.x = x_mm / 1000.0
        tf.transform.translation.y = y_mm / 1000.0
        tf.transform.rotation = quat
        self._tf_broadcaster.sendTransform(tf)

    def _enter_mode(self):
        """Put (or keep) the Roomba in the configured mode. Safe to call repeatedly."""
        if self._oi_mode == 'full':
            self._write(lambda roomba: roomba.full_mode())
        else:
            self._write(lambda roomba: roomba.safe_mode())

    def _on_reconnect(self, request, response):
        """Rebuild the connection to the Roomba (see the module docstring). Takes about 5 s."""
        self.get_logger().info('Reconnecting to the Roomba: reopen port, wake, Start, mode, beep')
        self._left = self._right = 0
        self._last_cmd_time = None          # wheels stay stopped until a new command arrives
        old, self._roomba = self._roomba, None
        if old is not None:
            try:
                old.stop()
                old.ser.close()             # not old.close(): that also resets the Roomba and waits 3 s
            except (SerialException, OSError):
                pass
        try:
            roomba = RoombaOI(self._port)
            roomba.wake()                   # only does something if the cable has the BRC wire
            roomba.start()
            if self._oi_mode == 'full':
                roomba.full_mode()
            else:
                roomba.safe_mode()
            roomba.beep()
        except (SerialException, OSError) as exc:
            response.success = False
            response.message = f'Could not open the Roomba on {self._port}: {exc}'
            self.get_logger().error(response.message)
            return response
        self._roomba = roomba
        self._serial_error = False
        response.success = True
        response.message = (f'Sent wake, Start and {self._oi_mode.capitalize()} mode. The Roomba should beep; '
                            'if it doesn\'t, check it\'s on (press CLEAN once to wake it) and try again.')
        self.get_logger().info('Reconnect commands sent (listen for the beep)')
        return response

    def _write(self, action):
        """
        Send one command to the Roomba. On a serial error (adapter unplugged), log
        it once and carry on rather than crash; reconnecting fixes it.
        """
        if self._roomba is None:
            return
        try:
            action(self._roomba)
        except (SerialException, OSError) as exc:
            if not self._serial_error:
                self.get_logger().error(
                    f'Roomba serial error: {exc}. Plug it back in, then Reconnect Roomba on the page.')
            self._serial_error = True
            return
        if self._serial_error:
            self.get_logger().info('Roomba serial port working again')
            self._serial_error = False

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
        if self._roomba is None:
            return
        try:
            self._roomba.close()
        except Exception as exc:
            self.get_logger().error(f'Could not close the Roomba cleanly: {exc}')


def _yaw_to_quaternion(yaw_rad):
    """A rotation around the vertical (z) axis only, as a Quaternion message."""
    q = Quaternion()
    q.z = math.sin(yaw_rad / 2.0)
    q.w = math.cos(yaw_rad / 2.0)
    return q


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
