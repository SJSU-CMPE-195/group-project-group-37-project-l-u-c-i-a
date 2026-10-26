"""ROS 2 node that publishes the Pico's VL53L5CX ToF frames.

See the package README for topics and parameters.
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
from sensor_msgs.msg import Image, Range
from std_msgs.msg import Bool

from lucia_tof import protocol

# A sequence jump bigger than this is taken as a Pico restart, not drops.
MAX_SEQ_GAP = 1000
FIELD_OF_VIEW = math.radians(45.0)
MIN_RANGE = 0.02  # m


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
        near_rate = self.declare_parameter('near_rate', 10.0).value

        self.sensors = [SensorState(n, prefix + n + '_link') for n in names]
        self.image_pubs = [
            self.create_publisher(Image, 'tof/{}/image'.format(n), qos_profile_sensor_data)
            for n in names]
        self.range_pubs = [
            self.create_publisher(Range, 'tof/{}/nearest'.format(n), qos_profile_sensor_data)
            for n in names]
        self.near_pub = self.create_publisher(Bool, 'tof/near', 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, '/diagnostics', 10)

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
            try:
                while rclpy.ok():
                    data = ser.read(ser.in_waiting or 1)
                    recv_ns = self.get_clock().now().nanoseconds
                    for item in self.parser.feed(data):
                        if isinstance(item, str):
                            self.get_logger().info('pico: ' + item)
                        else:
                            self.handle_packet(item, recv_ns)
            except (serial.SerialException, OSError) as e:
                self.get_logger().error('lost {}: {}'.format(self.port, e))
            finally:
                ser.close()
                with self.lock:
                    self.connected = False
                self.publish_near()

    def handle_packet(self, pkt, recv_ns):
        s = self.sensors[pkt.sensor]
        stamp = Time(nanoseconds=self.clock_sync.stamp(pkt.ticks_ms, recv_ns)).to_msg()

        with self.lock:
            if s.last_seq is not None:
                gap = (pkt.seq - s.last_seq - 1) & 0xFFFF
                if gap <= MAX_SEQ_GAP:
                    s.drops += gap
            s.last_seq = pkt.seq
            s.last_frame = time.monotonic()
            s.near = pkt.near
            s.frames += 1

        img = Image()
        img.header.stamp = stamp
        img.header.frame_id = s.frame_id
        img.height = img.width = protocol.SIDE
        img.encoding = '16UC1'
        img.is_bigendian = 0
        img.step = protocol.SIDE * 2
        mm = array.array('H', (max(pkt.distances[i], 0) for i in self.zone_map))
        if sys.byteorder != 'little':
            mm.byteswap()
        img.data = mm.tobytes()
        self.image_pubs[pkt.sensor].publish(img)

        rng = Range()
        rng.header = img.header
        rng.radiation_type = Range.INFRARED
        rng.field_of_view = FIELD_OF_VIEW
        rng.min_range = MIN_RANGE
        rng.max_range = float(self.max_range)
        if pkt.nearest_mm == protocol.INVALID:
            rng.range = math.inf
        else:
            rng.range = pkt.nearest_mm / 1000.0
        self.range_pubs[pkt.sensor].publish(rng)

        self.publish_near()

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
