# lucia_tof

Reads the robot's sensor Pico, one Pico 2 with the three VL53L5CX
time-of-flight sensors and the BNO08X IMU, and publishes both as standard ROS 2
messages. The Pico runs `src/pico2/main.py` and streams over USB serial; this
node reads it and publishes. No custom message types, so nothing beyond the
standard message packages is needed to subscribe.

`tof_node` is the only program that opens the sensor Pico (one program per
device). `lucia_bringup`'s `mapping.launch.py` starts it whenever the IMU
(`use_imu`) or the ToF safety stop (`use_tof`) is on; it replaced
`lucia_imu`'s `imu_bridge` there on 2026-11-10. It also understands Ling's
IMU-only `src/imu/stream_main.py`, so the robot keeps its IMU heading whichever
of the two programs is loaded on the Pico.

## Run

```bash
colcon build --packages-select lucia_tof
source install/setup.bash
ros2 launch lucia_tof tof.launch.py
```

The launch file uses `config/tof.yaml`, which expects the udev symlink
`/dev/imu` (the Pi's rule matches this Pico by its serial number; see
`udev/99-lucia-tof.rules`), or override the port:
`ros2 run lucia_tof tof_node --ros-args -p port:=/dev/ttyACM0`.

Only one program can have the port open, so close any serial terminal or
`pc_heatmap.py` connected to the Pico first, and stop mapping on the web page
(it runs this node). The node reconnects by itself if
the Pico is unplugged or restarts.

## Topics

`<name>` is `left`, `middle` or `right`: the robot's own left, middle and right
(checked on the robot 2026-11-10 with a hand in front of each). Each sensor
publishes about 60 Hz, independently; the sensors are not synchronized.

| Topic | Type | QoS | Contents |
|---|---|---|---|
| `tof/<name>/image` | `sensor_msgs/Image` | sensor data (best effort) | 4x4 distance grid |
| `tof/<name>/nearest` | `sensor_msgs/Range` | sensor data (best effort) | closest valid zone |
| `tof/near` | `std_msgs/Bool` | reliable, depth 10 | true = something close, or no data |
| `imu/data` | `sensor_msgs/Imu` | reliable, depth 10 | IMU orientation, turn rate and acceleration (unless `publish_imu` is false) |
| `tof/<name>/obstacles` | `sensor_msgs/PointCloud2` | sensor data | obstacles for Nav2's map, in `tof_<name>_link` |
| `tof/<name>/free` | `sensor_msgs/PointCloud2` | sensor data | free space up to these points, for Nav2's map |
| `/tf_static` | | | `base_link` → `tof_<name>_link`: where each sensor sits (`zones.py`) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | reliable | health, 1 Hz |

Subscribe to the first two with `rclpy.qos.qos_profile_sensor_data`; a
default (reliable) subscriber receives nothing from a best-effort publisher.

### `tof/<name>/image`

- 4x4, `encoding: 16UC1`, little-endian, `step: 8`.
- Each pixel is distance in **millimeters**; **0 means no valid reading** for
  that zone (out of range, or the sensor rejected the measurement). This is the
  usual ROS depth image convention.
- Row major, row 0 at the top. Each zone covers about 5.6 degrees of the
  sensor's 45 x 45 degree field of view.
- `header.frame_id` is `tof_<name>_link`. Nothing publishes TF for these
  frames yet.
- `header.stamp` is when the Pico read the frame, converted to ROS time.

```python
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

def on_image(msg):
    mm = np.frombuffer(msg.data, dtype='<u2').reshape(msg.height, msg.width)
    valid = mm > 0
    closest = mm[valid].min() if valid.any() else None

node.create_subscription(Image, 'tof/middle/image', on_image, qos_profile_sensor_data)
```

`cv_bridge.imgmsg_to_cv2(msg)` works too, and `rqt_image_view` shows the
images live.

### `tof/<name>/nearest`

`sensor_msgs/Range` with `range` in meters for the closest valid zone in that
sensor's frame, `+inf` when no zone is valid (nothing detected), `min_range`
0.02, `max_range` 4.0, `field_of_view` 0.785 rad. Same stamp and frame as the
image it came from.

### `tof/near`

True when any of these holds:

- some sensor sees something **in the robot's path**: within its width
  (`half_width_m`, robot radius + 2 cm each side), no further than
  `stop_distance_m` (0.15 m, the page's "ToF stop distance") past the front of
  the Roomba, above the floor and below `z_max_m`. `lucia_tof/zones.py` turns
  each zone's reading into a point on the robot, from where the sensors sit
  (25.4 cm up, level; the middle one straight ahead 7.6 cm behind the front, the
  side ones at 72.5° on the bracket). So the side sensors, which look almost
  sideways, don't stop it for a wall it's driving along. Decided with two
  guards against glitches (`lucia_tof/near_filter.py`): readings closer than the sensor can measure
  (`min_range_mm`, 20) are dropped, unless half or more of the zones read that
  close at once (something pressed against it); and a sensor's answer only
  changes after `near_frames` (3) frames in a row agree, about 50 ms. On the
  robot the middle sensor sent "0 mm" with nothing in front of it (2026-11-10),
  which used to count as near and would have stopped the robot for nothing.
  The Pico's own near flag isn't used;
- some sensor has sent no frame for `stale_timeout` (0.5 s), including at
  startup before the first frame;
- the Pico is not connected.

So false always means "all sensors are reporting and nothing is close". It is
sent on every frame and at least 10 Hz, so a subscriber can also treat silence
as a fault. Meant as a simple stop signal; for anything smarter, use the
images.

### `imu/data`

Same as `lucia_imu`'s `imu_bridge` published, so the EKF (`lucia_bringup`
`config/ekf.yaml`) is unchanged: frame `imu_link`, the game rotation vector as
`orientation` (gyro + accelerometer, no magnetometer, so the heading drifts
slowly instead of jumping near the Roomba's motors), the gyro in rad/s as
`angular_velocity`, and rough variances (`orientation_variance`,
`angular_velocity_variance`). `linear_acceleration` (m/s², gravity included)
is filled in from `src/pico2/main.py`; Ling's `stream_main.py` doesn't send it,
so then `linear_acceleration_covariance[0]` is -1 (not provided). Stamped when
the reading arrives.

**Upside-down board.** A board mounted chip side down reports turning left as
turning right, so its readings are turned over (180° about its x axis) before
publishing. With `main.py`, the node tells from gravity in the first reading
(about +9.8 m/s² on the board's z axis the right way up, −9.8 upside down) and
logs which; nothing is published before that. `stream_main.py` sends no
acceleration, so then `imu_upside_down` decides. The robot's IMU is upside down
(2026-11-10), so `config/tof.yaml` sets it to true.

About 100 readings a second from `main.py`, 50 from `stream_main.py`. It warns
in the log if no reading arrives for `imu_warn_s` (1 s). To see the heading
while checking the IMU by hand, run the node alone with
`--ros-args --log-level tof_node:=debug`: yaw, yaw rate and readings per
second every 2 s.

### `/diagnostics`

One status per sensor named `tof: <name>`: ERROR when disconnected or stale,
otherwise OK. Values: frame rate, frame count, dropped frames (from the Pico's
sequence numbers), seconds since the last frame, CRC errors. One more,
`imu (sensor pico)`: ERROR when disconnected or without a reading for
`imu_warn_s`; values: rate, readings, bad readings, seconds since the last,
and which Pico program sent them. View with `rqt_robot_monitor` or
`ros2 topic echo /diagnostics`.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyACM0` (`/dev/imu` in the launch config) | Pico serial port |
| `baud` | 115200 | ignored over USB serial |
| `sensor_names` | `[left, middle, right]` | must match the Pico's `SENSORS` order |
| `frame_id_prefix` | `tof_` | frame IDs are `<prefix><name>_link` |
| `flip_h`, `flip_v` | false | mirror each image left/right or top/bottom |
| `max_range` | 4.0 | m, reported in `Range.max_range` |
| `stale_timeout` | 0.5 | s without a frame before a sensor counts as stale |
| `near_rate` | 10.0 | Hz, minimum `tof/near` publish rate |
| `stop_distance_m` | 0.15 | something in the path this far past the front of the Roomba = near (the page sets it) |
| `half_width_m` | 0.25 | the path's half width (the page sets it from the robot radius + 2 cm) |
| `front_m` | 0.17 | front of the Roomba from its middle |
| `z_min_m`, `z_max_m` | 0.02, 0.45 | lower = floor, higher = it passes under |
| `radial` | false | readings straight out from the face (corner zones read no longer than the middle ones facing a wall, 2026-11-10) |
| `map_obstacles` | true | publish `tof/<name>/obstacles` and `free` |
| `mark_max_mm`, `free_max_mm` | 1000, 1500 | obstacles up to 1 m; nothing in range = free up to 1.5 m |
| `floor_margin_mm` | 100 | bottom row: this much shorter than the floor = something on it |
| `floor_mm_<name>` | measured | each sensor's bottom-row floor distances (4 zones, mm) |
| `base_frame` | `base_link` | parent frame of the `tof_<name>_link` transforms |
| `near_mm` | 300 | only for a sensor `zones.py` doesn't know: closer than this = near |
| `near_frames` | 3 | frames in a row before a sensor's near / clear answer changes |
| `min_range_mm` | 20 | readings closer than this are glitches (unless half the zones agree: covered) |
| `publish_imu` | true | publish `imu/data` (`mapping.launch.py` sets it from `use_imu`) |
| `imu_frame_id` | `imu_link` | frame of `imu/data` |
| `orientation_variance` | 0.0025 | rad², about 3 degrees; rough, not measured |
| `angular_velocity_variance` | 0.0004 | (rad/s)², about 1 degree/s |
| `linear_acceleration_variance` | 0.01 | (m/s²)²; the EKF doesn't use acceleration |
| `imu_warn_s` | 1.0 | s without an IMU reading before it warns |
| `imu_upside_down` | false (true in `config/tof.yaml`) | the board is chip side down: turn its readings over. Gravity decides when the readings include it (`main.py`), and a mismatch with this is logged |

## Checking the sensors: `tof_check`

```bash
ros2 run lucia_tof tof_check            # 10 s; --seconds 20 for longer
```

Stop mapping first (it refuses while `tof_node` runs: both would read the same
port). Twice a second it prints each sensor's nearest distance and NEAR, decided
the way `tof_node` does (in the path within `--stop`, default 0.15 m); then frames per second (about 60 each), frames with a
glitch, each sensor's 4x4 grid in mm (-1 = no reading, row 0 at the top), the
IMU's rate and anything the Pico printed. For the hand test, hold a hand about
15 cm in front of one sensor at a time: only that one should drop and say NEAR.

## On Nav2's map: `tof/<name>/obstacles` and `tof/<name>/free`

Each frame, every zone becomes one point in the sensor's frame
(`lucia_tof/obstacles.py`): an obstacle (within `mark_max_mm`, 1 m, not the
floor, not higher than `z_max_m`), or free space up to there (the floor,
anything further, or nothing in range: free to `free_max_mm`, 1.5 m). The floor
shows only in each sensor's bottom row, at its own distance (`floor_mm_<name>`,
measured on open floor with `tof_check`: left about 70 cm, middle 63, right 55);
a bottom-row reading `floor_margin_mm` (10 cm) shorter is something on it.
Re-measure them if the bracket moves.

Nav2's `tof_layer` (`lucia_nav/config/nav2.yaml`, both costmaps) marks the
obstacles and clears along the lines to the free points, but only from 0.7 m
out. Its own layer, so the lidar's clearing, which passes over low things,
can't erase them. Close to the robot a low object drops out of view, and its
mark stays until the robot has moved away. The page's "Low obstacles on the
map" turns the layer on (off by default until it's been tried on the robot).
The node publishes where each sensor sits (`base_link` → `tof_<name>_link`).

## Low obstacles: what the sensors can't see

Mounted 25.4 cm up and level, their view only reaches 22.5° down, so a low object
drops out of view as the robot gets close (distances from the front of the
Roomba): 5 cm tall at about 42 cm, 10 cm at about 30 cm, 15 cm at about 17 cm;
20 cm and taller stay in view. The lidar (30 cm up) only sees things taller than
that. So the stop can't catch things lower than about 15 cm at a 15 cm stop
distance. The bracket can't be tilted (2026-11-10), so what they see goes on
Nav2's map (above), which remembers a low object seen further out after it
drops out of view.

## Known gaps

Checked on the robot 2026-11-10 with `tof_check` (these were open before):
- **Image orientation:** row 0 is the top (the floor shows in the bottom row)
  and column 0 the sensor's left (a book on the robot's left half showed in the
  middle sensor's left two columns), so `flip_h` / `flip_v` stay false.
- **Distance:** straight out from the face (`radial: false`): facing a flat
  wall, the corner zones read no longer than the middle ones. Many zones read
  nothing in that test, so it's not certain; within 1 m the difference is a
  few cm.
- **TF:** `tof_node` publishes `base_link` → `tof_<name>_link` from `zones.py`.

Still open: the floor distances were measured on carpet; a different floor, or
the robot rocking when it brakes, may move them (watch for ghost obstacles
just ahead with "Low obstacles on the map" on).

## Serial protocol

The byte format is documented at the top of `src/pico2/main.py`. Protocol
version 2 also carries IMU packets. `lucia_tof/protocol.py` parses it and
`lucia_tof/imu.py` turns IMU packets, and Ling's `stream_main.py` text lines
(`I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>`), into readings. Neither has a ROS
dependency; their tests run with `python -m pytest test` from this directory.
