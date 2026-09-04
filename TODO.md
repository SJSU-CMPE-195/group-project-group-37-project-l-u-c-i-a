# Project L.U.C.I.A — TODO

> Working checklist. Check items off as they're completed.
> Hardware blocked items are marked **[BLOCKED]** until the dependency is resolved.

---

## Raspberry Pi

### Hardware / OS
- [ ] Set up udev rules (`/etc/udev/rules.d/99-lucia.rules`)
  - Plug in Roomba and RPLidar one at a time, find serial numbers via `udevadm info`
  - Create symlinks `/dev/roomba` and `/dev/rplidar`
  - Verify with `ls -la /dev/roomba /dev/rplidar`
- [ ] Set static IP `192.168.1.1` on `eth0` (for direct Jetson link)

### Docker / ROS2 Environment
- [ ] Write a `Dockerfile` so the `lucia/ros2:latest` image is reproducible
  - Currently only exists as a committed container on the Pi (image ID `e7eddf700148`)
  - Base: `ros:jazzy-ros-base` arm64
  - Should install: `ros-jazzy-rplidar-ros`, `ros-jazzy-slam-toolbox`, `ros-jazzy-nav2-bringup`, `python3-colcon-common-extensions`
- [ ] Build `zed_msgs` inside the Pi container
  - Needed so Nav2 on the Pi can deserialize ZED object detection messages from the Jetson

### ROS2 Nodes — `lucia_control`
- [ ] Implement `src/ros2/lucia_control/lucia_control/roomba_bridge.py`
  - Subscribe to `/cmd_vel` (`geometry_msgs/Twist`)
  - Translate `linear.x` + `angular.z` → `RoombaOI.drive_direct(left, right)`
  - Read bump and cliff sensors; enforce hard stop regardless of Nav2 commands if triggered
  - Publish sensor state on `/roomba/bumpers` and `/roomba/cliffs`

### ROS2 Config — `lucia_lidar`
- [ ] Create `src/ros2/lucia_lidar/config/rplidar.yaml` — serial port, baud rate, frame ID
- [ ] Create `src/ros2/lucia_lidar/config/slam_toolbox.yaml` — map update rate, scan topic, mode

### ROS2 Launch — `lucia_bringup`
- [ ] Write `src/ros2/lucia_bringup/launch/pi_stack.launch.py`
  - Launches: `rplidar_ros`, `slam_toolbox`, `nav2`, `roomba_bridge`
- [ ] Test full stack launch inside Docker container

### Testing
- [ ] Run container with RPLidar passed through, confirm `/scan` topic publishes
- [ ] Confirm `slam_toolbox` builds a map from scan data
- [ ] Drive the Roomba manually via `ros2 topic pub /cmd_vel` and confirm `roomba_bridge` responds

---

## Jetson Nano

> **Credentials:** `lucia` / `lucia-143-tomato`
> **SSH (USB):** `ssh lucia@192.168.55.1`
> **SSH (Ethernet, when on same network):** `ssh lucia@192.168.0.21`
> **Hostname:** `lucia-jetson`
> **JetPack:** 4.6.1 / L4T R32.7.1 / Ubuntu 18.04 / CUDA 10.2

### Hardware ✅
- [x] Confirmed Jetson Nano 4GB (eMMC, B01 revision, 40-pin GPIO)
- [x] Reflashed with L4T R32.7.1 via recovery mode (FC_REC pin + flash.sh)
- [x] First boot setup complete — hostname `lucia-jetson`, user `lucia`, MAXN power mode
- [x] System updated (`apt upgrade`), Docker 20.10.21 confirmed pre-installed
- [x] `lucia` added to `docker` group
- [ ] Confirm ZED 2i is in the **blue USB 3.0 port** (not USB 2.0)
- [ ] Confirm dedicated 5V/4A power supply for sustained operation (currently USB-powered from laptop)

### ZED SDK ✅
- [x] Installed ZED SDK 3.8.2 for JetPack 4.6 / CUDA 10.2
- [x] Installed CUDA toolkit 10.2 and added to PATH (`nvcc --version` confirmed)
- [x] AI models downloaded and optimized during install
- [x] SDK libraries confirmed at `/usr/local/zed/lib/`
- [ ] Fix Python API install (numpy pip build failed — non-critical for now)
- [ ] Run ZED self-test once a display is available: `/usr/local/zed/tools/ZED_Diagnostic`

### ROS2 on Jetson
- [ ] Install Docker (already installed — skip)
- [ ] Pull `dustynv/ros:humble-ros-base-l4t-r32.7.1` — NVIDIA community image with
  ROS2 Humble built for JetPack 4.6 arm64 (native Humble apt packages don't support Ubuntu 18.04)
- [ ] Add `lucia` to docker group (done ✅)
- [ ] Test: `docker run --rm dustynv/ros:humble-ros-base-l4t-r32.7.1 ros2 topic list`

### ZED ROS2 Publisher (`lucia_vision`) ✅
- [x] Created `src/ros2/lucia_vision/` ROS2 C++ package
- [x] Custom node built successfully inside dustynv Docker container
- [x] `/zed/rgb/image/compressed` publishing at ~14 Hz
- [x] `/zed/odom` publishing at ~13 Hz
- [x] `/zed/objects` publishing at ~13 Hz

### Network
- [ ] Set static IP `192.168.1.2` on Jetson `eth0` (for direct Pi link)
- [ ] Connect Ethernet cable directly to Pi
- [ ] `ping 192.168.1.1` from Jetson — confirm link is up

---

## Integration

- [ ] From Pi Docker container: `ros2 topic list | grep zed` — confirm Jetson topics visible
- [ ] Echo ZED object detections from the Pi: `ros2 topic echo /zed/zed_node/obj_det/objects --once`
- [ ] Configure Nav2 costmap on the Pi to ingest `/zed/zed_node/obj_det/objects` as dynamic obstacle layer
- [ ] Calibrate TF transform between RPLidar frame and ZED 2i physical mount position
- [ ] Set up `rviz2` on a laptop (same `ROS_DOMAIN_ID=42`) for live map + object visualization
- [ ] End-to-end test: rover navigates to a goal autonomously and avoids a person walking in front of it

---

## Infrastructure

- [ ] Update `README.md` with actual project description, setup steps, and tech stack table
- [ ] Update `docs/setup/ros2-jetson.md` to reflect actual JetPack 4.6 / Docker approach
- [ ] Update `docs/setup/ros2-pi.md` with Dockerfile instructions once written

---

## Open Questions

- Should `roomba_bridge` enforce a hard stop on bump/cliff regardless of Nav2 commands? **(Recommended: yes)**
- Fuse ZED visual odometry into `slam_toolbox`, or rely on LiDAR odometry alone?
- If Jetson goes offline mid-navigation, should Nav2 degrade gracefully (LiDAR-only) or stop and wait?
- ROS2 distro mismatch: Pi runs Jazzy, Jetson will run Humble via Docker — confirm `zed_msgs` deserializes correctly on Pi side before building Nav2 integration on top
