"""ROS 2 node for the robot's sensor Pico: publishes the VL53L5CX ToF frames
and the BNO08X IMU (imu/data, for the EKF).

The robot has one Pico 2 for both, so this is the only program that opens it.
It replaced lucia_imu's imu_bridge in mapping.launch.py on 2026-11-10. See the
package README for topics and parameters.
"""
import array
import math
import sys
import threading
import time

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image, Imu, Range
from std_msgs.msg import Bool

from lucia_tof import imu, protocol
from lucia_tof.near_filter import NearFilter

# A sequence jump bigger than this is taken as a Pico restart, not drops.
MAX_SEQ_GAP = 1000
FIELD_OF_VIEW = math.radians(45.0)


class SensorState:

    def __init__(self, name, frame_id):
        self.name = name
        self.frame_id = frame_id
        self.last_seq = None
        self.last_frame = None  # time.monotonic() of the last frame
        self.near = True
        self.frames = 0
        self.drops = 0
        self.diag_frames = 0
        self.diag_time = time.monotonic()


class ImuState:

    def __init__(self):
        self.last = None        # time.monotonic() of the last good reading
        self.reading = None     # the last good imu.Reading
        self.source = None      # 'packets' (src/pico2/main.py) or 'lines' (src/imu/stream_main.py)
        self.count = 0
        self.bad = 0
        self.diag_count = 0
        self.diag_time = time.monotonic()
        self.log_count = 0
        self.warned = False


class TofNode(Node):

    def __init__(self):
        super().__init__('tof_node')
        self.port = self.declare_parameter('port', '/dev/ttyACM0').value
        self.baud = self.declare_parameter('baud', 115200).value
        names = self.declare_parameter(
            'sensor_names', ['left', 'middle', 'right']).value
        prefix = self.declare_parameter('frame_id_prefix', 'tof_').value
        self.flip_h = self.declare_parameter('flip_h', False).value
        self.flip_v = self.declare_parameter('flip_v', False).value
        self.max_range = self.declare_parameter('max_range', 4.0).value
        self.stale_timeout = self.declare_parameter('stale_timeout', 0.5).value
        # Near / clear per sensor, with the glitch guards in near_filter.py
        self.near_mm = self.declare_parameter('near_mm', 300).value
        self.near_frames = self.declare_parameter('near_frames', 3).value
        self.min_range_mm = self.declare_parameter('min_range_mm', 20).value
        near_rate = self.declare_parameter('near_rate', 10.0).value
        # The IMU on the same Pico, published like lucia_imu's imu_bridge did:
        # same topic, frame and variances (rough values, not measured).
        self.publish_imu = self.declare_parameter('publish_imu', True).value
        self.imu_frame_id = self.declare_parameter('imu_frame_id', 'imu_link').value
        self.orientation_variance = self.declare_parameter('orientation_variance', 0.0025).value
        self.angular_velocity_variance = self.declare_parameter(
            'angular_velocity_variance', 0.0004).value
        self.linear_acceleration_variance = self.declare_parameter(
            'linear_acceleration_variance', 0.01).value
        self.imu_warn_s = self.declare_parameter('imu_warn_s', 1.0).value
        # Upside-down board: with src/pico2/main.py, gravity in the first
        # reading decides; stream_main.py sends no acceleration, so this says.
        self.imu_upside_down = self.declare_parameter('imu_upside_down', False).value
        self.imu_flip = None    # decided at the first reading, before anything is published
        self.imu_undecided = 0

        self.sensors = [SensorState(n, prefix + n + '_link') for n in names]
        self.near_filters = [NearFilter(self.near_mm, self.near_frames, self.min_range_mm)
                             for _ in names]
        self.image_pubs = [
            self.create_publisher(Image, 'tof/{}/image'.format(n), qos_profile_sensor_data)
            for n in names]
        self.range_pubs = [
            self.create_publisher(Range, 'tof/{}/nearest'.format(n), qos_profile_sensor_data)
            for n in names]
        self.near_pub = self.create_publisher(Bool, 'tof/near', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', 10) if self.publish_imu else None
        self.imu_state = ImuState()
        self.start_time = time.monotonic()

        # Zone index in the published image -> zone index in the packet.
        self.zone_map = []
        for r in range(protocol.SIDE):
            for c in range(protocol.SIDE):
                sr = protocol.SIDE - 1 - r if self.flip_v else r
                sc = protocol.SIDE - 1 - c if self.flip_h else c
                self.zone_map.append(sr * protocol.SIDE + sc)

        self.parser = protocol.Parser(len(names))
        self.clock_sync = protocol.ClockSync()
        self.connected = False
        self.lock = threading.Lock()

        self.create_timer(1.0 / near_rate, self.publish_near)
        self.create_timer(1.0, self.publish_diagnostics)
        if self.publish_imu:
            self.create_timer(self.imu_warn_s, self.check_imu)
            self.create_timer(2.0, self.log_imu)
            self.get_logger().info('publishing the IMU on imu/data in frame ' + self.imu_frame_id)
        threading.Thread(target=self.serial_loop, daemon=True).start()

    # --- serial ------------------------------------------------------------

    def serial_loop(self):
        while rclpy.ok():
            try:
                ser = serial.Serial(self.port, self.baud, timeout=0.1)
            except serial.SerialException as e:
                self.get_logger().warn(
                    'cannot open {}: {}'.format(self.port, e), throttle_duration_sec=10.0)
                time.sleep(1.0)
                continue

            self.get_logger().info('connected to ' + self.port)
            # The Pico may have restarted while the port was gone, so its
            # ticks and sequence numbers can start over.
            self.parser = protocol.Parser(len(self.sensors))
            self.clock_sync.reset()
            with self.lock:
                self.connected = True
                for s in self.sensors:
                    s.last_seq = None
                for f in self.near_filters:
                    f.reset()
            try:
                while rclpy.ok():
                    data = ser.read(ser.in_waiting or 1)
                    recv_ns = self.get_clock().now().nanoseconds
                    for item in self.parser.feed(data):
                        try:
                            self.handle_item(item, recv_ns)
                        except Exception as e:  # noqa: BLE001 - one bad item mustn't stop this thread
                            self.get_logger().error(
                                'skipped a reading: {!r}'.format(e), throttle_duration_sec=5.0)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error('lost {}: {}'.format(self.port, e))
            finally:
                ser.close()
                with self.lock:
                    self.connected = False
                self.publish_near()

    def handle_item(self, item, recv_ns):
        if isinstance(item, protocol.Packet):
            self.handle_packet(item, recv_ns)
        elif isinstance(item, protocol.ImuPacket):
            self.handle_imu(imu.from_packet(item), 'packets', recv_ns)
        elif imu.is_line(item):
            self.handle_imu(imu.from_line(item), 'lines', recv_ns)
        else:
            self.get_logger().info('pico: ' + item)

    def handle_packet(self, pkt, recv_ns):
        s = self.sensors[pkt.sensor]
        nearest_mm, near = self.near_filters[pkt.sensor].update(pkt.distances)
        stamp = Time(nanoseconds=self.clock_sync.stamp(pkt.ticks_ms, recv_ns)).to_msg()

        with self.lock:
            if s.last_seq is not None:
                gap = (pkt.seq - s.last_seq - 1) & 0xFFFF
                if gap <= MAX_SEQ_GAP:
                    s.drops += gap
            s.last_seq = pkt.seq
            s.last_frame = time.monotonic()
            s.near = near
            s.frames += 1

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = s.frame_id
        img.height = img.width = protocol.SIDE
        img.encoding = '16UC1'
        img.is_bigendian = 0
        img.step = protocol.SIDE * 2
        # 0 = no reading (ROS depth image convention), also for readings too close to be real
        mm = array.array('H', (d if d >= self.min_range_mm else 0
                               for d in (pkt.distances[i] for i in self.zone_map)))
        if sys.byteorder != 'little':
            mm.byteswap()
        img.data = mm.tobytes()
        self.image_pubs[pkt.sensor].publish(img)

        rng = Range()
        rng.header = img.header
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = FIELD_OF_VIEW
        rng.min_range = self.min_range_mm / 1000.0
        rng.max_range = float(self.max_range)
        if nearest_mm == protocol.INVALID:
            rng.range = math.inf
        else:
            rng.range = max(nearest_mm / 1000.0, rng.min_range)   # covered: as close as it measures
        self.range_pubs[pkt.sensor].publish(rng)

        self.publish_near()

    # --- IMU ---------------------------------------------------------------

    def handle_imu(self, reading, source, recv_ns):
        if reading is not None:
            if self.imu_flip is None:
                self.decide_flip(reading)
                if self.imu_flip is None:
                    return          # can't tell yet which way up the board is
            if self.imu_flip:
                reading = imu.turned_over(reading)
        with self.lock:
            st = self.imu_state
            st.source = source
            if reading is None:
                st.bad += 1
                return
            st.last = time.monotonic()
            st.reading = reading
            st.count += 1
        if self.imu_pub is None:
            return

        # Stamped when it arrived, as imu_bridge did. (Packets also carry the
        # Pico's own sample time, if that's ever needed.)
        msg = Imu()
        msg.header.stamp = Time(nanoseconds=recv_ns).to_msg()
        msg.header.frame_id = self.imu_frame_id
        q = msg.orientation
        q.w, q.x, q.y, q.z = reading.orientation
        g = msg.angular_velocity
        g.x, g.y, g.z = reading.gyro
        for i in (0, 4, 8):
            msg.orientation_covariance[i] = self.orientation_variance
            msg.angular_velocity_covariance[i] = self.angular_velocity_variance
        if reading.accel is None:
            msg.linear_acceleration_covariance[0] = -1.0  # not sent (stream_main.py)
        else:
            a = msg.linear_acceleration
            a.x, a.y, a.z = reading.accel
            for i in (0, 4, 8):
                msg.linear_acceleration_covariance[i] = self.linear_acceleration_variance
        self.imu_pub.publish(msg)

    def decide_flip(self, raw):
        """Which way up the IMU board is, once, from the first reading that can tell.

        A board mounted upside down (chip side down) reports turning left as
        turning right, so its readings are turned over before publishing.
        Readings with acceleration (src/pico2/main.py) show it from gravity;
        stream_main.py's have none, so imu_upside_down decides for those.
        """
        if raw.accel is None:
            self.imu_flip = self.imu_upside_down
            self.get_logger().info(
                'IMU readings without acceleration (stream_main.py): taking the board as {} '
                '(imu_upside_down)'.format('upside down' if self.imu_flip else 'the right way up'))
            return
        seen = imu.upside_down(raw)
        if seen is None:            # no acceleration report yet, or not lying flat
            self.imu_undecided += 1
            if self.imu_undecided >= 100:   # about a second: go by the setting
                self.imu_flip = self.imu_upside_down
                self.get_logger().warning(
                    'Gravity on the IMU\'s z axis is {:+.1f} m/s^2, too small to tell which way '
                    'up it is; going by imu_upside_down: {}'.format(
                        raw.accel[2], str(self.imu_upside_down).lower()))
            return
        self.imu_flip = seen
        way = 'upside down: turning its readings over' if seen else 'the right way up'
        self.get_logger().info('IMU board is {} (gravity {:+.1f} m/s^2 on its z axis)'.format(
            way, raw.accel[2]))
        if seen != self.imu_upside_down:
            self.get_logger().warning(
                'imu_upside_down is {} but gravity says the board is {}; going by gravity. '
                'Set imu_upside_down: {} in lucia_tof/config/tof.yaml (it decides for '
                'stream_main.py, which sends no acceleration)'.format(
                    str(self.imu_upside_down).lower(), 'upside down' if seen else 'the right way up',
                    str(seen).lower()))

    def check_imu(self):
        """Warn once when the IMU goes quiet, and say when it's back."""
        now = time.monotonic()
        with self.lock:
            st = self.imu_state
            quiet_s = now - (st.last if st.last is not None else self.start_time)
            if quiet_s > self.imu_warn_s and not st.warned:
                st.warned = True
                news = 'quiet'
            elif quiet_s <= self.imu_warn_s and st.warned:
                st.warned = False
                news = 'back'
            else:
                return
        if news == 'quiet':
            self.get_logger().warning(
                'No IMU data for {:.1f} s - is the Pico on {} running src/pico2/main.py (or '
                'src/imu/stream_main.py) with the IMU wired, and is nothing else holding the '
                'port (e.g. VS Code MicroPico)?'.format(quiet_s, self.port))
        else:
            self.get_logger().info('IMU data flowing again')

    def log_imu(self):
        """Heading and rate every 2 s, at debug level (--log-level tof_node:=debug)."""
        with self.lock:
            st = self.imu_state
            reading, source, bad = st.reading, st.source, st.bad
            count, st.log_count = st.count - st.log_count, st.count
        if reading is None:
            return
        self.get_logger().debug(
            'IMU: yaw {:+.1f} deg, yaw rate {:+.1f} deg/s, {:.0f} readings/s ({}), '
            '{} bad so far'.format(math.degrees(imu.yaw_rad(reading)),
                                   math.degrees(reading.gyro[2]), count / 2.0, source, bad))

    # --- near flag and diagnostics -----------------------------------------

    def is_stale(self, s, now):
        return s.last_frame is None or now - s.last_frame > self.stale_timeout

    def publish_near(self):
        now = time.monotonic()
        with self.lock:
            near = not self.connected or any(
                s.near or self.is_stale(s, now) for s in self.sensors)
        self.near_pub.publish(Bool(data=near))

    def publish_diagnostics(self):
        now = time.monotonic()
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        with self.lock:
            for s in self.sensors:
                fps = (s.frames - s.diag_frames) / (now - s.diag_time)
                s.diag_frames, s.diag_time = s.frames, now
                st = DiagnosticStatus()
                st.name = 'tof: ' + s.name
                st.hardware_id = self.port
                if not self.connected:
                    st.level, st.message = DiagnosticStatus.ERROR, 'not connected'
                elif self.is_stale(s, now):
                    st.level, st.message = DiagnosticStatus.ERROR, 'no recent frames'
                else:
                    st.level, st.message = DiagnosticStatus.OK, 'ok'
                age = 'never' if s.last_frame is None else '{:.2f}'.format(now - s.last_frame)
                st.values = [
                    KeyValue(key='frame rate (Hz)', value='{:.1f}'.format(fps)),
                    KeyValue(key='frames', value=str(s.frames)),
                    KeyValue(key='dropped frames', value=str(s.drops)),
                    KeyValue(key='seconds since last frame', value=age),
                    KeyValue(key='crc errors (all sensors)', value=str(self.parser.crc_errors)),
                ]
                msg.status.append(st)

            ims = self.imu_state
            rate = (ims.count - ims.diag_count) / (now - ims.diag_time)
            ims.diag_count, ims.diag_time = ims.count, now
            d = DiagnosticStatus()
            d.name = 'imu (sensor pico)'
            d.hardware_id = self.port
            if not self.connected:
                d.level, d.message = DiagnosticStatus.ERROR, 'not connected'
            elif ims.last is None or now - ims.last > self.imu_warn_s:
                d.level, d.message = DiagnosticStatus.ERROR, 'no recent readings'
            else:
                d.level, d.message = DiagnosticStatus.OK, 'ok'
            age = 'never' if ims.last is None else '{:.2f}'.format(now - ims.last)
            sent_by = {'packets': 'src/pico2/main.py', 'lines': 'src/imu/stream_main.py'}
            d.values = [
                KeyValue(key='rate (Hz)', value='{:.1f}'.format(rate)),
                KeyValue(key='readings', value=str(ims.count)),
                KeyValue(key='bad readings', value=str(ims.bad)),
                KeyValue(key='seconds since last reading', value=age),
                KeyValue(key='sent by', value=sent_by.get(ims.source, 'nothing yet')),
                KeyValue(key='published on imu/data', value=str(self.publish_imu)),
                KeyValue(key='board upside down (readings turned over)',
                         value={None: 'not known yet', True: 'yes', False: 'no'}[self.imu_flip]),
            ]
            msg.status.append(d)
        self.diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TofNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
