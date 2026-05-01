# Phase 4 — ZED ROS2 Wrapper

Build the Stereolabs ZED ROS2 wrapper and confirm the camera publishes
the topics that Nav2 on the Pi will consume.

---

## Prerequisites

- ZED SDK 4.x installed and self-test passing (Phase 2)
- ROS2 Humble installed and sourced (Phase 3)

---

## Step 1 — Create workspace and clone

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
```

**Commit / tag cloned:** <!-- paste git log --oneline -1 output -->

---

## Step 2 — Install dependencies

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

**Result:** <!-- Success / missing deps -->

---

## Step 3 — Build

This takes 15–30 minutes on the Jetson.

```bash
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
```

**Result:** <!-- Success / build errors -->

**Packages built:**
```
# paste colcon build summary
```

---

## Step 4 — Source the workspace

```bash
source ~/ros2_ws/install/setup.bash
```

Add to `~/.bashrc`:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Step 5 — Enable object detection

Object detection is **disabled by default**. Find and edit the ZED 2i param file:

```bash
find ~/ros2_ws -name "zed2i.yaml" 2>/dev/null
```

Open the file and set:

```yaml
object_detection:
  od_enabled: true
```

**Param file path:** <!-- fill in -->

---

## Step 6 — Launch the ZED node

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

**Result:** <!-- Launched successfully / errors -->

---

## Step 7 — Verify topics

In a second terminal:

```bash
ros2 topic list | grep zed
```

**Actual output:**
```
# paste here
```

Confirm these are present:
- [ ] `/zed/zed_node/obj_det/objects`
- [ ] `/zed/zed_node/depth/depth_registered`
- [ ] `/zed/zed_node/odom`
- [ ] `/zed/zed_node/point_cloud/cloud_registered`

---

## Step 8 — Confirm data is flowing

```bash
ros2 topic hz /zed/zed_node/depth/depth_registered
ros2 topic hz /zed/zed_node/obj_det/objects
```

**Actual output:**
```
# paste here
```

---

## Notes

<!-- Build errors, param file location, object detection model download time, any quirks -->
