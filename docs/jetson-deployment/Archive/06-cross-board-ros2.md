# Phase 6 — Cross-board ROS2 Verification

Confirm the Pi's ROS2 Docker container can see the Jetson's ZED topics
over the direct Ethernet link.

---

## Prerequisites

- Ethernet link up and both boards pinging each other (Phase 5)
- ZED node running on Jetson (Phase 4)
- `lucia/ros2:latest` Docker image on the Pi

---

## Step 1 — Start ZED node on Jetson

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

Leave this running in one terminal.

---

## Step 2 — Launch ROS2 container on Pi

```bash
docker run -it --rm \
  --network=host \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  --device=/dev/rplidar \
  --device=/dev/roomba \
  -v $(pwd)/src/ros2:/ros2_ws/src \
  lucia/ros2:latest bash
```

---

## Step 3 — Check topic visibility from Pi

Inside the container:

```bash
ros2 topic list | grep zed
```

**Actual output:**
```
# paste here
```

Expected topics visible:
- [ ] `/zed/zed_node/obj_det/objects`
- [ ] `/zed/zed_node/depth/depth_registered`
- [ ] `/zed/zed_node/odom`

---

## Step 4 — Echo a ZED message from Pi

```bash
ros2 topic echo /zed/zed_node/obj_det/objects --once
```

**Actual output:**
```
# paste here
```

> If this fails with a message type error, the Pi container is missing `zed_msgs`.
> See the notes section below for the fix.

---

## Step 5 — Check topic latency

```bash
ros2 topic hz /zed/zed_node/depth/depth_registered
```

**Actual output:**
```
# paste here
```

Expected: ~15 Hz for depth at default ZED settings.

---

## Notes

### If `zed_msgs` is missing on the Pi

The Pi runs ROS2 Jazzy; the Jetson runs Humble. The ZED wrapper uses custom
`zed_msgs` message types. If the Pi can't deserialize them, build `zed_msgs`
inside the Pi container:

```bash
# Inside the Pi container
cd /ros2_ws/src
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
cd /ros2_ws
colcon build --packages-select zed_msgs
source install/setup.bash
```

Then retry the echo.

<!-- Add any other notes, errors encountered, or workarounds here -->
