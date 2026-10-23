"""
web_ui.py

ROS2 node that serves the LUCIA control page: open http://<pi>:8080
(http://10.42.0.1:8080 on the Pi's own hotspot).

From the page you can:
- see whether the lidar, IMU and wheels (/scan, /imu/data, /odom) are alive
- start/stop mapping (runs `ros2 launch lucia_bringup mapping.launch.py`), with a
  settings menu first (speed, acceleration, the tight-space slowdown, robot
  size, Explore, the stuck check: settings.py). They're saved next to the maps
  (settings.json) and handed to the nodes as a parameters file; this node uses
  three itself (speed limit for the slider, robot size, carrying on after
  driving by hand), which override its own parameters below
- watch the map build live, with the robot's position on it
- watch the lidar live as a radar around the robot (forward is up)
- set the driving speed (the drive pad, and Nav2 through its speed_limit topic)
- explore on its own (frontier exploration), return home, or click the map to
  send the robot somewhere: these go to lucia_nav's mission node on
  /mission/command, and its /mission/state and Nav2's /plan are drawn on the map
- drive with an on-screen pad or WASD / arrow keys (testing tool, like teleop).
  Driving by hand while the robot drives itself pauses the mission (lucia_nav's
  'pause'); resume_after_s (3 s) after the last key press it carries on by
  itself ('resume'): Explore keeps going from where it was driven to, HOME and
  go-to head for the same spot. STOP cancels that.
- see live metrics: heading, speed, turn rate (IMU), distance driven, area
  mapped, sensor message rates, the Pi's CPU temperature / load / memory
- read every node's log (/rosout) and the launch output in a drawer under the map
- reconnect to the Roomba after it was turned off and on (roomba_bridge's
  ~/reconnect service), without stopping mapping
- honk the horn and play tunes on the Roomba's speaker (roomba_bridge's ~/song
  topic): built-in ones, saved ones (tunes.json next to the saved maps), typed
  ones, and melodies taken from MIDI files (tunes.py)
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
import collections
import json
import os
import subprocess
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav2_msgs.msg import SpeedLimit
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rcl_interfaces.msg import Log
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from rclpy.time import Time
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from lucia_web.logic import (AutoResume, RateMeter, Trail, clamp_speed, cpu_percent,
                             cpu_times, drive_to_twist, encode_grid,
                             log_level_name, mapped_area_m2, mission_text,
                             parse_loadavg, parse_meminfo, parse_temp,
                             radar_nearest, scan_to_radar, thin_path,
                             topic_status, valid_map_name, yaw_from_quaternion)
from lucia_web import settings as page_settings
from lucia_web.mapping_process import MappingProcess
from lucia_web.tunes import (BUILTIN, guess_melody_part, load_library,
                             melody, parse_midi, parse_tune_text, save_library,
                             tune_seconds, tune_text, valid_tune_name)

MAP_FILE_EXTENSIONS = ('.png', '.yaml')
MISSION_FRESH_S = 3.0       # mission node publishes its state twice a second
PLAN_FRESH_S = 5.0
CMD_FRESH_S = 1.0           # roomba_bridge stops the wheels after 0.5 s without a command
LOG_KEEP = 2000             # /rosout entries kept for the page
LOG_SEND_MAX = 500          # most entries sent in one /api/logs reply
LOG_MSG_MAX = 500           # characters kept per log message
RECONNECT_TIMEOUT_S = 20.0  # roomba_bridge's reconnect takes about 5 s
MIDI_MAX_BYTES = 2_000_000


class WebUi(Node):
    def __init__(self):
        super().__init__('web_ui')

        self.declare_parameter('http_port', 8080)
        self.declare_parameter('maps_dir', '/maps')
        self.declare_parameter('speed_mps', 0.30)           # starting top speed (the slider); safety_gate slows it near things
        self.declare_parameter('min_speed_mps', 0.05)
        self.declare_parameter('max_speed_mps', 0.30)       # roomba_bridge's own cap is 0.30
        self.declare_parameter('turn_rate_ratio', 4.0)      # rad/s of turning per m/s of speed (0.15 -> 0.6)
        self.declare_parameter('drive_timeout_s', 0.3)
        self.declare_parameter('resume_after_s', 3.0)       # driving by hand: carry on this long after; 0 = never
        self.declare_parameter('radar_slices', 180)         # 2 degrees each
        self.declare_parameter('robot_radius_m', 0.23)      # for drawing; 18 in across (nav2.yaml has the same)

        self._http_port = self.get_parameter('http_port').value
        self.maps_dir = self.get_parameter('maps_dir').value
        self._min_speed = self.get_parameter('min_speed_mps').value
        self._max_speed = self.get_parameter('max_speed_mps').value
        self._turn_rate_ratio = self.get_parameter('turn_rate_ratio').value
        self._drive_timeout_s = self.get_parameter('drive_timeout_s').value
        self._resume = AutoResume(self.get_parameter('resume_after_s').value)
        self._radar_slices = self.get_parameter('radar_slices').value
        self._robot_radius = self.get_parameter('robot_radius_m').value

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
        self._speed = clamp_speed(self.get_parameter('speed_mps').value,
                                  self._min_speed, self._max_speed)
        self._radar = None          # latest radar view for the page, see _on_scan
        self._radar_seq = 0
        self._laser_yaw = {}        # scan frame_id -> its yaw on the robot, from TF
        self._mission = None        # (time, state dict) from /mission/state
        self._plan = None           # (time, [(x, y), ...]) from Nav2's /plan
        self._rates = {name: RateMeter() for name in self._last_seen}
        self._gyro = None           # (time, rad/s) turn rate measured by the IMU
        self._cmd = None            # (time, m/s, rad/s) last command to the wheels (/cmd_vel_safe)
        self._trail = Trail()
        self._area_m2 = None
        self._pi = {}               # Pi health: temp_c, load1, cpu_pct, mem_pct, cpus
        self._cpu_prev = None
        self._logs = collections.deque(maxlen=LOG_KEEP)
        self._log_seq = 0

        self.create_subscription(LaserScan, 'scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(Imu, 'imu/data', self._on_imu, 10)
        self.create_subscription(Odometry, 'odom', lambda _: self._seen('wheels'), 10)
        map_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, 'map', self._on_map, map_qos)
        self.create_subscription(String, 'mission/state', self._on_mission, 10)
        self.create_subscription(Path, 'plan', self._on_plan, 1)
        self.create_subscription(Twist, 'cmd_vel_safe', self._on_cmd_safe, 10)
        rosout_qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.RELIABLE,
                                durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Log, 'rosout', self._on_log, rosout_qos)

        self._cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self._mission_pub = self.create_publisher(String, 'mission/command', 10)
        self._speed_limit_pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self._reconnect_client = self.create_client(Trigger, 'roomba_bridge/reconnect')
        self._song_pub = self.create_publisher(String, 'roomba_bridge/song', 10)
        self._tunes_path = os.path.join(self.maps_dir, 'tunes.json')
        self._tunes_lock = threading.Lock()
        self._settings_path = os.path.join(self.maps_dir, 'settings.json')
        self._settings = page_settings.load(self._settings_path)
        self._apply_settings(self._settings)
        self._reconnecting = threading.Lock()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.create_timer(0.1, self._drive_tick)
        self.create_timer(0.2, self._resume_tick)
        self.create_timer(0.2, self._update_pose)
        self.create_timer(1.0, self._publish_speed_limit)
        self.create_timer(2.0, self._read_pi_health)

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
        now = time.monotonic()
        with self._lock:
            self._last_seen[name] = now
            self._rates[name].tick(now)

    def _on_imu(self, msg):
        self._seen('imu')
        with self._lock:
            self._gyro = (time.monotonic(), msg.angular_velocity.z)

    def _on_cmd_safe(self, msg):
        with self._lock:
            self._cmd = (time.monotonic(), msg.linear.x, msg.angular.z)

    def _on_log(self, msg):
        entry = {
            't': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            'level': log_level_name(msg.level),
            'node': msg.name,
            'msg': msg.msg[:LOG_MSG_MAX],
        }
        with self._lock:
            self._log_seq += 1
            entry['seq'] = self._log_seq
            self._logs.append(entry)

    def _read_pi_health(self):
        def read(path):
            try:
                with open(path) as f:
                    return f.read()
            except OSError:
                return None
        cpu_now = cpu_times(read('/proc/stat'))
        pi = {
            'temp_c': parse_temp(read('/sys/class/thermal/thermal_zone0/temp')),
            'load1': parse_loadavg(read('/proc/loadavg')),
            'cpu_pct': cpu_percent(self._cpu_prev, cpu_now),
            'mem_pct': parse_meminfo(read('/proc/meminfo')),
            'cpus': os.cpu_count(),
        }
        self._cpu_prev = cpu_now
        with self._lock:
            self._pi = pi

    def _robot_pose_at(self, stamp):
        """Robot pose in the map at a message's time, else the latest; None if there's no map yet."""
        for when in (Time.from_msg(stamp), Time()):
            try:
                t = self._tf_buffer.lookup_transform('map', 'base_link', when)
            except TransformException:
                continue
            tr, q = t.transform.translation, t.transform.rotation
            return {'x': tr.x, 'y': tr.y, 'yaw': yaw_from_quaternion(q.x, q.y, q.z, q.w)}
        return None

    def _on_scan(self, msg):
        self._seen('lidar')
        frame = msg.header.frame_id
        if frame not in self._laser_yaw:
            # The lidar's mounting is a static transform, so look it up once.
            try:
                t = self._tf_buffer.lookup_transform('base_link', frame, Time())
            except TransformException:
                return      # not received yet; the next scan tries again
            q = t.transform.rotation
            self._laser_yaw[frame] = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        slices = scan_to_radar(msg.ranges, msg.angle_min, msg.angle_increment,
                               msg.range_min, msg.range_max,
                               self._laser_yaw[frame], self._radar_slices)
        # The pose the scan was taken from, so the page can draw it on the map too.
        radar = {'slices_cm': slices, 'nearest': radar_nearest(slices),
                 'pose': self._robot_pose_at(msg.header.stamp)}
        with self._lock:
            self._radar_seq += 1
            radar['seq'] = self._radar_seq
            self._radar = radar

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
        area = mapped_area_m2(msg.data, info.resolution)
        with self._lock:
            self._area_m2 = area
            self._map_version += 1
            page_map['version'] = self._map_version
            self._map = page_map

    def _on_mission(self, msg):
        try:
            state = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(state, dict):
            return
        with self._lock:
            self._mission = (time.monotonic(), state)

    def _on_plan(self, msg):
        points = thin_path((p.pose.position.x, p.pose.position.y) for p in msg.poses)
        with self._lock:
            self._plan = (time.monotonic(), points)

    def _publish_speed_limit(self):
        # Nav2's controller only listens while it runs and keeps no history,
        # so this is sent every second rather than only when the slider moves.
        msg = SpeedLimit()
        msg.percentage = False
        with self._lock:
            msg.speed_limit = float(self._speed)
        self._speed_limit_pub.publish(msg)

    def _mission_state(self):
        """Latest mission state if the mission node is alive, else None. Call with the lock held."""
        if self._mission is None or time.monotonic() - self._mission[0] > MISSION_FRESH_S:
            return None
        return self._mission[1]

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
            if self.mapping.running:
                self._trail.add(tr.x, tr.y, time.monotonic())

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
        with self._lock:
            twist = drive_to_twist(linear_frac, angular_frac,
                                   self._speed, self._speed * self._turn_rate_ratio)
            mission = self._mission_state()
            mode = mission.get('mode') if mission else None
            now = time.monotonic()
            what = self._resume.drive(mode, twist != (0.0, 0.0), now)
            if what == 'ignore':
                return True         # a key let go: don't interrupt the mission with stops
            if what == 'pause':
                # Driving by hand takes over: pause the mission, then drive. Mark it
                # paused here so the next presses go through before its next update.
                self._mission = (self._mission[0],
                                 dict(mission, mode='paused', goal=None, paused_from=mode,
                                      message='Paused: driving by hand'))
            self._drive = twist
            self._drive_time = now
        if what == 'pause':
            self._mission_pub.publish(String(data='pause'))
            self.get_logger().info('Driving by hand: paused the mission'
                                   + ('' if self._resume.after_s <= 0 else
                                      f'; it carries on {self._resume.after_s:g} s after the last key press'))
        return True

    def _resume_tick(self):
        """Driving by hand is over (nothing for resume_after_s): let the mission carry on."""
        with self._lock:
            mission = self._mission_state()
            due = self._resume.due(mission.get('mode') if mission else None, time.monotonic())
        if due:
            self._mission_pub.publish(String(data='resume'))
            self.get_logger().info('No driving by hand for a while: resuming the mission')

    def reconnect_roomba(self):
        """Ask roomba_bridge to rebuild its Roomba connection. Called from an HTTP thread; waits for the answer."""
        if not self.mapping.running:
            return False, 'Start mapping first - the Roomba driver runs with it'
        if not self._reconnecting.acquire(blocking=False):
            return False, 'Already reconnecting'
        try:
            # Don't let it drive off the moment the Roomba wakes up.
            with self._lock:
                self._drive_time = None
                self._resume.cancel()
                mission = self._mission_state()
            if mission and mission.get('mode') != 'idle':
                self._mission_pub.publish(String(data='stop'))
            if not self._reconnect_client.service_is_ready():
                return False, 'roomba_bridge is not running (see Logs)'
            done = threading.Event()
            future = self._reconnect_client.call_async(Trigger.Request())
            future.add_done_callback(lambda _: done.set())
            if not done.wait(RECONNECT_TIMEOUT_S):
                return False, 'roomba_bridge did not answer (see Logs)'
            result = future.result()
            if result is None:
                return False, 'Reconnect failed (see Logs)'
            return result.success, result.message
        finally:
            self._reconnecting.release()

    # ---- Tunes ---------------------------------------------------------

    def tunes_list(self):
        def entry(name, notes):
            return {'name': name, 'text': tune_text(notes), 'notes': len(notes),
                    'seconds': round(tune_seconds(notes), 1)}
        with self._tunes_lock:
            custom = load_library(self._tunes_path)
        return {'builtin': [entry(n, t) for n, t in BUILTIN.items()],
                'custom': [entry(n, t) for n, t in sorted(custom.items(), key=lambda kv: kv[0].lower())]}

    def _tune_from(self, body):
        """(name, tune) from {'name': ...} (built-in or saved) or {'text': ...}. ValueError if neither works."""
        if not isinstance(body, dict):
            raise ValueError('bad request')
        if body.get('text'):
            return body.get('name') or 'Tune', parse_tune_text(body['text'])
        name = body.get('name')
        if name in BUILTIN:
            return name, BUILTIN[name]
        with self._tunes_lock:
            custom = load_library(self._tunes_path)
        if name in custom:
            return name, custom[name]
        raise ValueError(f'No tune called "{name}"')

    def check_tune(self, body):
        try:
            _, tune = self._tune_from(body)
        except ValueError as exc:
            return {'ok': False, 'message': str(exc)}
        return {'ok': True, 'text': tune_text(tune), 'tune': tune, 'notes': len(tune),
                'seconds': round(tune_seconds(tune), 1)}

    def play_tune(self, body):
        try:
            name, tune = self._tune_from(body)
        except ValueError as exc:
            return False, str(exc)
        if not self.mapping.running:
            return False, 'Start mapping first - the Roomba driver runs with it'
        self._song_pub.publish(String(data=json.dumps({'notes': tune})))
        return True, f'Playing {name} ({tune_seconds(tune):.1f} s)'

    def save_tune(self, body):
        name = body.get('name', '').strip() if isinstance(body.get('name'), str) else ''
        if not valid_tune_name(name):
            return False, 'Name it with 1-40 letters, digits, spaces or ( ) \' ! & . , + -'
        if name in BUILTIN:
            return False, f'"{name}" is a built-in tune - pick another name'
        try:
            tune = parse_tune_text(body.get('text'))
        except ValueError as exc:
            return False, str(exc)
        with self._tunes_lock:
            library = load_library(self._tunes_path)
            existed = name in library
            library[name] = tune
            try:
                save_library(self._tunes_path, library)
            except OSError as exc:
                return False, f'Could not save: {exc}'
        return True, f'{"Updated" if existed else "Saved"} "{name}"'

    def delete_tune(self, body):
        name = body.get('name')
        with self._tunes_lock:
            library = load_library(self._tunes_path)
            if name not in library:
                return False, 'Only saved tunes can be deleted' if name in BUILTIN else f'No saved tune "{name}"'
            del library[name]
            try:
                save_library(self._tunes_path, library)
            except OSError as exc:
                return False, f'Could not delete: {exc}'
        return True, f'Deleted "{name}"'

    def midi_tune(self, data, part, start_s, transpose):
        """A MIDI file -> its parts, and one part's melody as a tune (text format)."""
        try:
            parts = parse_midi(data)
            index = part if part is not None and 0 <= part < len(parts) else guess_melody_part(parts)
            tune, info = melody(parts[index]['notes'], start_s or 0.0, transpose or 0)
        except ValueError as exc:
            return {'ok': False, 'message': str(exc)}
        return {
            'ok': True,
            'parts': [{'index': k, 'name': p['name'], 'notes': len(p['notes']),
                       'seconds': round(p['notes'][-1][1], 1)} for k, p in enumerate(parts)],
            'part': index, 'text': tune_text(tune), 'notes': len(tune),
            'seconds': round(tune_seconds(tune), 1), 'info': info,
        }

    def settings_info(self):
        """The settings menu: every setting, with its saved value."""
        return {'settings': [dict(s, value=self._settings[s['key']]) for s in page_settings.describe()]}

    def start_mapping(self, body):
        """
        Start mapping with the settings from the page's menu (body['settings']),
        or the saved ones. Returns (ok, message).
        """
        if self.mapping.running:
            return False, 'Already running'
        values = body.get('settings') if isinstance(body.get('settings'), dict) else dict(self._settings)
        if 'use_imu' in body and 'settings' not in body:
            values['use_imu'] = bool(body['use_imu'])
        settings, errors = page_settings.clean(values)
        if errors:
            return False, 'Settings: ' + '; '.join(errors)
        params_path = os.path.join(tempfile.gettempdir(), 'lucia_settings.yaml')
        try:
            with open(params_path, 'w') as f:
                f.write(page_settings.params_yaml(settings))
        except OSError as exc:
            return False, f'Could not write the settings for the launch: {exc}'
        try:
            page_settings.save(self._settings_path, settings)
        except OSError as exc:      # still start; they just won't be there next time
            self.get_logger().warning(f'Could not save the settings to {self._settings_path}: {exc}')
        self._settings = settings
        self._apply_settings(settings)
        with self._lock:
            self._trail.reset()
        if not self.mapping.start(use_imu=settings['use_imu'], settings_file=params_path):
            return False, 'Already running'
        return True, 'Mapping started'

    def _apply_settings(self, settings):
        """The settings this node uses itself: speed limit (the slider), robot size, carrying on."""
        with self._lock:
            old_max = self._max_speed
            self._max_speed = settings['top_speed']
            # A slider at the top stays at the top; otherwise it only comes down if it must
            at_top = self._speed is not None and self._speed >= old_max - 1e-6
            self._speed = self._max_speed if at_top else min(self._speed, self._max_speed)
            self._robot_radius = settings['robot_radius']
            self._resume.after_s = settings['resume_after']

    def logs_after(self, seq):
        with self._lock:
            entries = [e for e in self._logs if seq is None or e['seq'] > seq]
            last = self._log_seq
        return {'entries': entries[-LOG_SEND_MAX:], 'last': last}

    def trail_since(self, session, start):
        with self._lock:
            points = self._trail.points
            current = self._trail.session
            if session != current or start is None or start > len(points):
                start = 0
            new = [(round(x, 3), round(y, 3)) for x, y in points[start:]]
        return {'session': current, 'from': start, 'points': new}

    def send_mission(self, body):
        text = mission_text(body)
        if text is None:
            return False, 'Unknown request'
        with self._lock:
            self._resume.cancel()       # an order from the page replaces carrying on by itself
        if text == 'stop':
            # Stop the wheels right here too, in case the mission node isn't running.
            with self._lock:
                self._drive_time = None
            self._cmd_pub.publish(Twist())
        elif not self.mapping.running:
            return False, 'Start mapping first - navigation runs with it'
        self._mission_pub.publish(String(data=text))
        return True, {'explore': 'Exploring', 'home': 'Going home',
                      'stop': 'Stopped'}.get(text, 'Going there')

    def set_speed(self, mps):
        speed = clamp_speed(mps, self._min_speed, self._max_speed)
        if speed is None:
            return False, 'Speed must be a number'
        with self._lock:
            self._speed = speed
        return True, f'Speed {speed * 100:.0f} cm/s'

    def radar_if_newer(self, seq):
        with self._lock:
            if self._radar is None or self._radar['seq'] == seq:
                return None
            return self._radar

    def status(self):
        now = time.monotonic()
        with self._lock:
            sensors = {name: topic_status(None if t is None else now - t)
                       for name, t in self._last_seen.items()}
            pose = self._pose
            map_version = self._map['version'] if self._map else None
            speed = self._speed
            mission = self._mission_state()
            plan = None
            if (mission and mission.get('mode') != 'idle' and self._plan is not None
                    and now - self._plan[0] < PLAN_FRESH_S):
                plan = self._plan[1]
            cmd = None
            if self._cmd is not None:
                fresh = now - self._cmd[0] < CMD_FRESH_S
                cmd = {'linear': self._cmd[1] if fresh else 0.0, 'angular': self._cmd[2] if fresh else 0.0}
            telemetry = {
                'cmd': cmd,
                'gyro_z': self._gyro[1] if self._gyro and now - self._gyro[0] < CMD_FRESH_S else None,
                'ground_speed': self._trail.ground_speed(),
                'distance_m': self._trail.distance_m,
                'trip_s': self.mapping.uptime_s,
                'area_m2': self._area_m2,
                'rates': {name: round(m.hz(now), 1) for name, m in self._rates.items()},
                'pi': self._pi,
            }
            trail = {'session': self._trail.session, 'len': len(self._trail.points)}
            resume_in_s = self._resume.seconds_left(now) if mission and mission.get('mode') == 'paused' else None
        return {
            'mapping': self.mapping.running,
            'use_imu': self.mapping.use_imu,
            'sensors': sensors,
            'pose': None if pose is None else {'x': pose[0], 'y': pose[1], 'yaw': pose[2]},
            'map_version': map_version,
            'speed': {'mps': speed, 'min': self._min_speed, 'max': self._max_speed},
            'mission': mission,
            'resume_in_s': resume_in_s,
            'plan': plan,
            'telemetry': telemetry,
            'trail': trail,
            'robot_radius_m': self._robot_radius,
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
        elif path == '/api/scan':
            radar = self.ui.radar_if_newer(_query_int(query, 'seq'))
            if radar is None:
                self.send_response(204)
                self.end_headers()
            else:
                self._send_json(radar)
        elif path == '/api/logs':
            self._send_json(self.ui.logs_after(_query_int(query, 'after')))
        elif path == '/api/launchlog':
            self._send_json({'lines': self.ui.mapping.log_tail(200)})
        elif path == '/api/trail':
            self._send_json(self.ui.trail_since(_query_str(query, 'session'), _query_int(query, 'from')))
        elif path == '/api/tunes':
            self._send_json(self.ui.tunes_list())
        elif path == '/api/maps':
            self._send_json({'maps': self.ui.list_maps()})
        elif path == '/api/settings':
            self._send_json(self.ui.settings_info())
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
        path, _, query = self.path.partition('?')
        if path == '/api/tunes/midi':
            length = int(self.headers.get('Content-Length') or 0)
            if not 0 < length <= MIDI_MAX_BYTES:
                self._send_json({'ok': False, 'message': 'MIDI file missing or over 2 MB'}, 400)
                return
            self._send_json(self.ui.midi_tune(self.rfile.read(length), _query_int(query, 'part'),
                                              _query_float(query, 'start'), _query_int(query, 'transpose')))
            return
        body = self._read_json()
        if body is None:
            self._send_json({'ok': False, 'message': 'bad JSON'}, 400)
            return
        if self.path == '/api/drive':
            ok = self.ui.set_drive(body.get('linear', 0), body.get('angular', 0))
            self._send_json({'ok': ok})
        elif self.path == '/api/mission':
            ok, message = self.ui.send_mission(body)
            self._send_json({'ok': ok, 'message': message})
        elif self.path == '/api/speed':
            ok, message = self.ui.set_speed(body.get('mps'))
            self._send_json({'ok': ok, 'message': message})
        elif self.path == '/api/tunes/check':
            self._send_json(self.ui.check_tune(body))
        elif self.path == '/api/tunes/play':
            ok, message = self.ui.play_tune(body)
            self._send_json({'ok': ok, 'message': message})
        elif self.path == '/api/tunes/save':
            ok, message = self.ui.save_tune(body)
            self._send_json({'ok': ok, 'message': message})
        elif self.path == '/api/tunes/delete':
            ok, message = self.ui.delete_tune(body)
            self._send_json({'ok': ok, 'message': message})
        elif self.path == '/api/roomba/reconnect':
            ok, message = self.ui.reconnect_roomba()
            self._send_json({'ok': ok, 'message': message})
        elif self.path == '/api/mapping/start':
            ok, message = self.ui.start_mapping(body)
            self._send_json({'ok': ok, 'message': message})
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


def _query_str(query, key):
    for part in query.split('&'):
        k, _, v = part.partition('=')
        if k == key:
            return v
    return None


def _query_float(query, key):
    v = _query_str(query, key)
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if f == f and abs(f) < 1e6 else None     # no NaN / inf


def _query_int(query, key):
    v = _query_str(query, key)
    try:
        return int(v)
    except (TypeError, ValueError):
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
