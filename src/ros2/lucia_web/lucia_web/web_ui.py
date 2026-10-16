"""
web_ui.py

ROS2 node that serves the LUCIA control page: open http://<pi>:8080
(http://10.42.0.1:8080 on the Pi's own hotspot).

From the page you can:
- see whether the lidar, IMU and wheels (/scan, /imu/data, /odom) are alive
- start/stop mapping (runs `ros2 launch lucia_bringup mapping.launch.py`)
- watch the map build live, with the robot's position on it
- drive with an on-screen pad or the arrow keys (testing tool, like teleop)
- save the map and download the PNG

Driving is hold-to-move: the page sends a command every 100 ms while a button
or key is held. If commands stop arriving for drive_timeout_s (button
released, tab closed, WiFi dropped), this node sends one stop, and
roomba_bridge's own /cmd_vel timeout stops the wheels as a second layer.

Plain http.server: no extra Python packages needed in the image.

Usage:
    ros2 run lucia_web web_ui
    ros2 run lucia_web web_ui --ros-args -p http_port:=8080
"""

import base64
import json
import os
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from tf2_ros import Buffer, TransformException, TransformListener

from lucia_web.logic import (drive_to_twist, encode_grid, topic_status,
                             valid_map_name, yaw_from_quaternion)
from lucia_web.mapping_process import MappingProcess

MAP_FILE_EXTENSIONS = ('.png', '.yaml')


class WebUi(Node):
    def __init__(self):
        super().__init__('web_ui')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('maps_dir', '/maps')
        self.declare_parameter('max_linear_mps', 0.15)
        self.declare_parameter('max_angular_rad_s', 0.6)
        self.declare_parameter('drive_timeout_s', 0.3)

        self._http_port = self.get_parameter('http_port').value
        self.maps_dir = self.get_parameter('maps_dir').value
        self._max_linear = self.get_parameter('max_linear_mps').value
        self._max_angular = self.get_parameter('max_angular_rad_s').value
        self._drive_timeout_s = self.get_parameter('drive_timeout_s').value

        self.static_dir = os.path.join(get_package_share_directory('lucia_web'), 'static')
        self.mapping = MappingProcess()

        self._lock = threading.Lock()
        self._last_seen = {'lidar': None, 'imu': None, 'wheels': None}
        self._map = None            # (version, dict for the page)
        self._map_version = 0
        self._pose = None           # (x, y, yaw) in the map frame
        self._drive = (0.0, 0.0)
        self._drive_time = None
        self._driving = False

        self.create_subscription(LaserScan, 'scan', lambda _: self._seen('lidar'),
                                 qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', lambda _: self._seen('imu'), 10)
        self.create_subscription(Odometry, 'odom', lambda _: self._seen('wheels'), 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(0.1, self._drive_tick)
        self.create_timer(0.2, self._update_pose)

        handler = type('Handler', (_Handler,), {'ui': self})
        self._server = ThreadingHTTPServer(('0.0.0.0', self._http_port), handler)
        self._server.daemon_threads = True
        threading.Thread(target=self._server.serve_forever, daemon=True).start()

        self.get_logger().info(
            f'Web UI on port {self._http_port} - http://10.42.0.1:{self._http_port} on the Pi hotspot')

    # ------------------------------------------------------------------
    # ROS callbacks and timers
    # ------------------------------------------------------------------

    def _seen(self, name):
        with self._lock:
            self._last_seen[name] = time.monotonic()

    def _on_map(self, msg):
        info = msg.info
        page_map = {
            'width': info.width,
            'height': info.height,
            'resolution': info.resolution,
            'origin_x': info.origin.position.x,
            'origin_y': info.origin.position.y,
            'data': base64.b64encode(encode_grid(msg.data)).decode('ascii'),
        }
        with self._lock:
            self._map_version += 1
            page_map['version'] = self._map_version
            self._map = page_map

    def _update_pose(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', Time())
        except TransformException:
            with self._lock:
                self._pose = None
            return
        tr, q = t.transform.translation, t.transform.rotation
        with self._lock:
            self._pose = (tr.x, tr.y, yaw_from_quaternion(q.x, q.y, q.z, q.w))

    def _drive_tick(self):
        with self._lock:
            fresh = (self._drive_time is not None
                     and time.monotonic() - self._drive_time < self._drive_timeout_s)
            linear, angular = self._drive if fresh else (0.0, 0.0)
            send = fresh or self._driving    # one stop after commands go quiet
            self._driving = fresh
        if send:
            msg = Twist()
            msg.linear.x = linear
            msg.angular.z = angular
            self._cmd_pub.publish(msg)

    # ------------------------------------------------------------------
    # Called from HTTP threads
    # ------------------------------------------------------------------

    def set_drive(self, linear_frac, angular_frac):
        twist = drive_to_twist(linear_frac, angular_frac, self._max_linear, self._max_angular)
        with self._lock:
            self._drive = twist
            self._drive_time = time.monotonic()

    def status(self):
        now = time.monotonic()
        with self._lock:
            sensors = {name: topic_status(None if t is None else now - t)
                       for name, t in self._last_seen.items()}
            pose = self._pose
            map_version = self._map['version'] if self._map else None
        return {
            'mapping': self.mapping.running,
            'use_imu': self.mapping.use_imu,
            'sensors': sensors,
            'pose': None if pose is None else {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            'map_version': map_version,
            'log': self.mapping.log_tail(),
        }

    def map_if_newer(self, version):
        with self._lock:
            if self._map is None or self._map['version'] == version:
                return None
            return self._map

    def save_map(self, name):
        if not valid_map_name(name):
            return False, 'Use 1-40 letters, digits, - or _'
        if not self.mapping.running:
            return False, 'Start mapping first - the map only exists while it runs'
        path = os.path.join(self.maps_dir, name)
        if os.path.exists(path + '.png'):
            return False, f'"{name}" already exists - pick another name'
        try:
            result = subprocess.run(
                ['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', path, '--fmt', 'png'],
                capture_output=True, text=True, timeout=30)
        except subprocess.TimeoutExpired:
            return False, 'map_saver_cli timed out (is slam_toolbox publishing /map?)'
        if result.returncode != 0 or not os.path.exists(path + '.png'):
            tail = (result.stdout + result.stderr).strip().splitlines()[-3:]
            return False, 'Save failed: ' + ' | '.join(tail)
        return True, f'Saved {name}.png'

    def list_maps(self):
        try:
            names = [f for f in os.listdir(self.maps_dir) if f.endswith('.png')]
        except FileNotFoundError:
            return []
        names.sort(key=lambda f: os.path.getmtime(os.path.join(self.maps_dir, f)), reverse=True)
        return names

    def shutdown(self):
        self._server.shutdown()
        if self.mapping.running:
            self.get_logger().info('Stopping mapping')
            self.mapping.stop()


class _Handler(BaseHTTPRequestHandler):
    ui = None   # set by WebUi

    def log_message(self, *args):
        pass    # the page polls every second; don't flood the log

    # GET ---------------------------------------------------------------

    def do_GET(self):
        path, _, query = self.path.partition('?')
        if path in ('/', '/index.html'):
            self._send_file(os.path.join(self.ui.static_dir, 'index.html'), 'text/html')
        elif path == '/api/status':
            self._send_json(self.ui.status())
        elif path == '/api/map':
            version = _query_int(query, 'version')
            page_map = self.ui.map_if_newer(version)
            if page_map is None:
                self.send_response(204)
                self.end_headers()
            else:
                self._send_json(page_map)
        elif path == '/api/maps':
            self._send_json({'maps': self.ui.list_maps()})
        elif path.startswith('/maps/'):
            name = path[len('/maps/'):]
            base, ext = os.path.splitext(name)
            if not (valid_map_name(base) and ext in MAP_FILE_EXTENSIONS):
                self._send_json({'error': 'bad file name'}, 400)
                return
            content_type = 'image/png' if ext == '.png' else 'text/plain'
            self._send_file(os.path.join(self.ui.maps_dir, name), content_type)
        else:
            self._send_json({'error': 'not found'}, 404)

    # POST --------------------------------------------------------------

    def do_POST(self):
        body = self._read_json()
        if body is None:
            self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
            return
        if self.path == '/api/drive':
            self.ui.set_drive(body.get('linear', 0), body.get('angular', 0))
            self._send_json({'ok': True})
        elif self.path == '/api/mapping/start':
            started = self.ui.mapping.start(use_imu=bool(body.get('use_imu', True)))
            self._send_json({'ok': started, 'message': 'Mapping started' if started else 'Already running'})
        elif self.path == '/api/mapping/stop':
            stopped = self.ui.mapping.stop()
            self._send_json({'ok': stopped, 'message': 'Mapping stopped' if stopped else 'Not running'})
        elif self.path == '/api/save':
            ok, message = self.ui.save_map(body.get('name', ''))
            self._send_json({'ok': ok, 'message': message})
        else:
            self._send_json({'error': 'not found'}, 404)

    # Helpers -----------------------------------------------------------

    def _read_json(self):
        try:
            length = int(self.headers.get('Content-Length', 0))
            return json.loads(self.rfile.read(length) or b'{}')
        except (ValueError, json.JSONDecodeError):
            return None

    def _send_json(self, obj, code=200):
        data = json.dumps(obj).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', 'application/json')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _send_file(self, path, content_type):
        try:
            with open(path, 'rb') as f:
                data = f.read()
        except FileNotFoundError:
            self._send_json({'error': 'not found'}, 404)
            return
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(data)))
        self.end_headers()
        self.wfile.write(data)


def _query_int(query, key):
    for part in query.split('&'):
        k, _, v = part.partition('=')
        if k == key:
            try:
                return int(v)
            except ValueError:
                return None
    return None


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebUi()
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
