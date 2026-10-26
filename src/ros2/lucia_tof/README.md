# lucia_tof

Publishes the three VL53L5CX time-of-flight sensors on the Pico 2 as standard
ROS 2 messages. The Pico runs `src/tof_pico/heatmap/pico_stream.py` and streams
frames over USB serial; this node reads them and publishes. No custom message
types, so nothing beyond the standard message packages is needed to subscribe.

## Run

```bash
colcon build --packages-select lucia_tof
source install/setup.bash
ros2 launch lucia_tof tof.launch.py
```

The launch file uses `config/tof.yaml`, which expects the udev symlink
`/dev/lucia_tof`. Install the rule once from `udev/99-lucia-tof.rules`, or
override the port: `ros2 run lucia_tof tof_node --ros-args -p port:=/dev/ttyACM0`.

Only one program can have the port open, so close any serial terminal or
`pc_heatmap.py` connected to the Pico first. The node reconnects by itself if
the Pico is unplugged or restarts.

## Topics

`<name>` is `left`, `middle` or `right` (left to right as seen standing in
front of the robot). Each sensor publishes about 15 Hz, independently; the
sensors are not synchronized.

| Topic | Type | QoS | Contents |
|---|---|---|---|
| `tof/<name>/image` | `sensor_msgs/Image` | sensor data (best effort) | 8x8 distance grid |
| `tof/<name>/nearest` | `sensor_msgs/Range` | sensor data (best effort) | closest valid zone |
| `tof/near` | `std_msgs/Bool` | reliable, depth 10 | true = something close, or no data |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | reliable | health, 1 Hz |

Subscribe to the first two with `rclpy.qos.qos_profile_sensor_data`; a
default (reliable) subscriber receives nothing from a best-effort publisher.

### `tof/<name>/image`

- 8x8, `encoding: 16UC1`, little-endian, `step: 16`.
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
  `NEAR_MM` (300 mm by default, set in `pico_stream.py`);
- some sensor has sent no frame for `stale_timeout` (0.5 s), including at
  startup before the first frame;
- the Pico is not connected.

So false always means "all sensors are reporting and nothing is close". It is
sent on every frame and at least 10 Hz, so a subscriber can also treat silence
as a fault. Meant as a simple stop signal; for anything smarter, use the
images.

### `/diagnostics`

One status per sensor named `tof: <name>`: ERROR when disconnected or stale,
otherwise OK. Values: frame rate, frame count, dropped frames (from the Pico's
sequence numbers), seconds since the last frame, CRC errors. View with
`rqt_robot_monitor` or `ros2 topic echo /diagnostics`.

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `port` | `/dev/ttyACM0` (`/dev/lucia_tof` in the launch config) | Pico serial port |
| `baud` | 115200 | ignored over USB serial |
| `sensor_names` | `[left, middle, right]` | must match the Pico's `SENSORS` order |
| `frame_id_prefix` | `tof_` | frame IDs are `<prefix><name>_link` |
| `flip_h`, `flip_v` | false | mirror each image left/right or top/bottom |
| `max_range` | 4.0 | m, reported in `Range.max_range` |
| `stale_timeout` | 0.5 | s without a frame before a sensor counts as stale |
| `near_rate` | 10.0 | Hz, minimum `tof/near` publish rate |

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

The byte format is documented at the top of `pico_stream.py`.
`lucia_tof/protocol.py` parses it and has no ROS dependency; its tests run
with `python -m pytest test` from this directory.
