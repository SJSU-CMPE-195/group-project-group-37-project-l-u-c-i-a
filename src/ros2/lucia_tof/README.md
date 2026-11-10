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

`<name>` is `left`, `middle` or `right` (left to right as seen standing in
front of the robot). Each sensor publishes about 60 Hz, independently; the
sensors are not synchronized.

| Topic | Type | QoS | Contents |
|---|---|---|---|
| `tof/<name>/image` | `sensor_msgs/Image` | sensor data (best effort) | 4x4 distance grid |
| `tof/<name>/nearest` | `sensor_msgs/Range` | sensor data (best effort) | closest valid zone |
| `tof/near` | `std_msgs/Bool` | reliable, depth 10 | true = something close, or no data |
| `imu/data` | `sensor_msgs/Imu` | reliable, depth 10 | IMU orientation, turn rate and acceleration (unless `publish_imu` is false) |
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

- some sensor's latest frame has a valid zone closer than the Pico's
  `NEAR_MM` (300 mm by default, set in `src/pico2/main.py`);
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
| `publish_imu` | true | publish `imu/data` (`mapping.launch.py` sets it from `use_imu`) |
| `imu_frame_id` | `imu_link` | frame of `imu/data` |
| `orientation_variance` | 0.0025 | rad², about 3 degrees; rough, not measured |
| `angular_velocity_variance` | 0.0004 | (rad/s)², about 1 degree/s |
| `linear_acceleration_variance` | 0.01 | (m/s²)²; the EKF doesn't use acceleration |
| `imu_warn_s` | 1.0 | s without an IMU reading before it warns |

## Known gaps

- **Image orientation is not verified yet.** Wave a hand on one side of a
  sensor in `rqt_image_view` and set `flip_h` / `flip_v` so the image matches
  what the sensor sees.
- **Radial vs. perpendicular distance is not verified.** It matters only if
  the images are turned into 3D points: aim a sensor at a flat wall; if the
  corner zones read longer than the center, distances are radial.
- **No TF.** The `tof_*_link` frames need static transforms from `base_link`
  once the mounting is final.

## Serial protocol

The byte format is documented at the top of `src/pico2/main.py`. Protocol
version 2 also carries IMU packets. `lucia_tof/protocol.py` parses it and
`lucia_tof/imu.py` turns IMU packets, and Ling's `stream_main.py` text lines
(`I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>`), into readings. Neither has a ROS
dependency; their tests run with `python -m pytest test` from this directory.
