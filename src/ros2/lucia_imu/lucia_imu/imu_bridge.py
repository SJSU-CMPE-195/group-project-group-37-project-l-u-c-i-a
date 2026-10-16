"""
imu_bridge.py

ROS2 node that owns the IMU Pico's USB serial port (nothing else may open it).

- Reads the Pico's line stream (src/imu/stream_main.py, parsed by imu_line.py)
- Publishes each reading as sensor_msgs/Imu on /imu/data: orientation (game
  rotation vector: gyro + accelerometer, no magnetometer) and angular velocity.
  Linear acceleration isn't sent by the Pico, so it's marked as not provided
  (covariance[0] = -1).
- Warns if no valid data arrives for no_data_warn_s

Covariances are rough starting values (about 3 deg orientation, about 1 deg/s
rate), not measured. Tune them once the IMU is fused with /odom.

Usage:
    ros2 run lucia_imu imu_bridge
    ros2 run lucia_imu imu_bridge --ros-args -p port:=/dev/ttyACM0
To see the heading once every 2 s while checking it by hand:
    ros2 run lucia_imu imu_bridge --ros-args --log-level imu_bridge:=debug
"""

import math

import rclpy
import serial
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu

from lucia_imu.imu_line import parse_line, yaw_rad

MAX_BUFFER_BYTES = 4096


class ImuBridge(Node):
    def __init__(self):
        super().__init__('imu_bridge')

        self.declare_parameter('port', '/dev/imu')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('poll_period_s', 0.005)
        self.declare_parameter('no_data_warn_s', 1.0)
        self.declare_parameter('orientation_variance', 0.0025)
        self.declare_parameter('angular_velocity_variance', 0.0004)

        port = self.get_parameter('port').value
        self._frame_id = self.get_parameter('frame_id').value
        poll_period_s = self.get_parameter('poll_period_s').value
        self._no_data_warn_s = self.get_parameter('no_data_warn_s').value
        self._orientation_variance = self.get_parameter('orientation_variance').value
        self._angular_velocity_variance = self.get_parameter('angular_velocity_variance').value

        if min(poll_period_s, self._no_data_warn_s) <= 0:
            raise ValueError('poll_period_s and no_data_warn_s must be greater than 0')

        self._buffer = b''
        self._last_reading = None
        self._last_data_time = None
        self._warned_no_data = False
        self._good_count = 0
        self._bad_count = 0

        self.get_logger().info(f'Opening IMU Pico on {port}')
        # USB serial to the Pico: the baud rate is ignored, data moves at USB speed.
        self._serial = serial.Serial(port, 115200, timeout=0)
        self._start_time = self.get_clock().now()

        self._pub = self.create_publisher(Imu, 'imu/data', 10)
        self.create_timer(poll_period_s, self._poll)
        self.create_timer(self._no_data_warn_s, self._check_data_flowing)
        self.create_timer(2.0, self._log_status)

        self.get_logger().info(f'Ready: publishing /imu/data in frame {self._frame_id}')

    # ------------------------------------------------------------------
    # Timers
    # ------------------------------------------------------------------

    def _poll(self):
        """Read whatever has arrived and publish every complete line."""
        try:
            waiting = self._serial.in_waiting
            if not waiting:
                return
            self._buffer += self._serial.read(waiting)
        except serial.SerialException as e:
            self.get_logger().error(f'Serial read failed: {e}', throttle_duration_sec=5.0)
            return

        *lines, self._buffer = self._buffer.split(b'\n')
        if len(self._buffer) > MAX_BUFFER_BYTES:
            self._buffer = b''  # no newline in a long time: not our stream

        for raw in lines:
            if not raw.startswith(b'I,'):
                text = raw.decode('utf-8', errors='replace').strip()
                if text:
                    self.get_logger().info(f'Pico: {text}')
                continue
            reading = parse_line(raw.decode('utf-8', errors='replace'))
            if reading is None:
                self._bad_count += 1
                continue
            self._publish(reading)

    def _check_data_flowing(self):
        now = self.get_clock().now()
        since = self._last_data_time or self._start_time
        quiet_s = (now - since).nanoseconds / 1e9
        if quiet_s > self._no_data_warn_s:
            if not self._warned_no_data:
                self.get_logger().warning(
                    f'No IMU data for {quiet_s:.1f} s - is the Pico running stream_main.py '
                    f'as main.py, and is nothing else (e.g. VS Code MicroPico) holding the port?')
            self._warned_no_data = True
        elif self._warned_no_data:
            self.get_logger().info('IMU data flowing again')
            self._warned_no_data = False

    def _log_status(self):
        if self._last_reading is None:
            return
        self.get_logger().debug(
            f'yaw {math.degrees(yaw_rad(self._last_reading)):+.1f} deg, '
            f'yaw rate {math.degrees(self._last_reading.gz):+.1f} deg/s, '
            f'{self._good_count / 2.0:.0f} readings/s, {self._bad_count} bad lines so far')
        self._good_count = 0

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _publish(self, reading):
        now = self.get_clock().now()
        self._last_reading = reading
        self._last_data_time = now
        self._good_count += 1

        msg = Imu()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self._frame_id
        msg.orientation.w = reading.qw
        msg.orientation.x = reading.qx
        msg.orientation.y = reading.qy
        msg.orientation.z = reading.qz
        msg.angular_velocity.x = reading.gx
        msg.angular_velocity.y = reading.gy
        msg.angular_velocity.z = reading.gz
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = self._orientation_variance
            msg.angular_velocity_covariance[i] = self._angular_velocity_variance
        msg.linear_acceleration_covariance[0] = -1.0  # not provided
        self._pub.publish(msg)

    def shutdown(self):
        if self._serial.is_open:
            self._serial.close()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ImuBridge()
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
