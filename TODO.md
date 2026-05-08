# Project L.U.C.I.A — TODO

> Working checklist. Check items off as they're completed.
> Hardware blocked items are marked **[BLOCKED]** until the dependency is resolved.

---

## Raspberry Pi

### Hardware / OS
- [x] Set up udev rule for RPLidar — `/dev/rplidar` symlink live (`serial=="0001"`)
- [ ] Set up udev rule for Roomba — plug in Roomba alone, run `udevadm info -a -n /dev/ttyUSB0 | grep serial` and add rule to `/etc/udev/rules.d/99-lucia.rules`
- [ ] Set static IP `192.168.1.1` on `eth0` (for direct Jetson link)

### Web Control Panel ✅
- [x] `obstacle_avoid.py` — reactive LiDAR avoidance demo (standalone, no ROS)
- [x] `slam_avoid.py` — SLAM + avoidance, saves occupancy grid PNG + path overlay on exit
- [x] `slam_avoid_server.py` — FastAPI web server with live radar, SLAM map, manual/auto toggle, bump indicators
- [x] Bumper data integrated — triggers backup + turn recovery in AUTO mode
- [x] `lucia.service` systemd unit — web server starts automatically on boot
- [x] `install-service.sh` — one-shot installer for the systemd service
- [ ] Test full auto mode end-to-end with Roomba + LiDAR both connected

### Docker / ROS2 Environment
- [ ] Write a `Dockerfile` so the `lucia/ros2:latest` image is reproducible
  - Base: `ros:jazzy-ros-base` arm64
  - Should install: `ros-jazzy-rplidar-ros`, `ros-jazzy-slam-toolbox`, `ros-jazzy-nav2-bringup`, `python3-colcon-common-extensions`
- [ ] Build `zed_msgs` inside the Pi container

### ROS2 Nodes — `lucia_control`
- [ ] Implement `src/ros2/lucia_control/lucia_control/roomba_bridge.py`
  - Subscribe to `/cmd_vel` (`geometry_msgs/Twist`)
  - Translate `linear.x` + `angular.z` → `RoombaOI.drive_direct(left, right)`
  - Read bump and cliff sensors; publish on `/roomba/bumpers` and `/roomba/cliffs`

### ROS2 Config — `lucia_lidar`
- [ ] Create `src/ros2/lucia_lidar/config/rplidar.yaml`
- [ ] Create `src/ros2/lucia_lidar/config/slam_toolbox.yaml`

### ROS2 Launch — `lucia_bringup`
- [ ] Write `src/ros2/lucia_bringup/launch/pi_stack.launch.py`
- [ ] Test full stack launch inside Docker container

---

## Jetson Nano

> **Credentials:** `lucia` / `lucia-143-tomato`
> **SSH (USB):** `ssh lucia@192.168.55.1`
> **SSH (Ethernet):** `ssh lucia@192.168.0.21`
> **JetPack:** 4.6.1 / L4T R32.7.1 / Ubuntu 18.04 / CUDA 10.2

### Hardware ✅
- [x] Confirmed Jetson Nano 4GB (eMMC, B01 revision, 40-pin GPIO)
- [x] Reflashed with L4T R32.7.1 via recovery mode (FC_REC pin + flash.sh)
- [x] First boot complete — hostname `lucia-jetson`, user `lucia`, MAXN power mode
- [x] System updated, Docker 20.10.21 pre-installed, `lucia` in docker group
- [ ] Confirm ZED 2i is in the **blue USB 3.0 port**
- [ ] Confirm dedicated 5V/4A barrel jack power supply

### ZED SDK ✅
- [x] ZED SDK 3.8.2 installed for JetPack 4.6 / CUDA 10.2
- [x] CUDA 10.2 confirmed (`nvcc --version`)
- [x] SDK libraries at `/usr/local/zed/lib/`
- [ ] Fix Python API (numpy pip build failed — non-critical)
- [ ] Run ZED diagnostic: `/usr/local/zed/tools/ZED_Diagnostic`

### ZED ROS2 Publisher (`lucia_vision`) ✅
- [x] Custom C++ ROS2 node built inside dustynv Docker container
- [x] `/zed/rgb/image/compressed` at ~14 Hz
- [x] `/zed/odom` at ~13 Hz
- [x] `/zed/objects` at ~13 Hz

### Network
- [ ] Set static IP `192.168.1.2` on Jetson `eth0`
- [ ] Connect Ethernet directly to Pi, confirm `ping 192.168.1.1`

---

## Integration

- [ ] Confirm Pi can see Jetson ROS2 topics (`ros2 topic list | grep zed`)
- [ ] Configure Nav2 costmap to ingest `/zed/zed_node/obj_det/objects` as dynamic obstacle layer
- [ ] Calibrate TF between RPLidar frame and ZED 2i mount position
- [ ] End-to-end: rover navigates to goal autonomously, avoids person walking in front

---

## Open Questions

- Fuse ZED visual odometry into `slam_toolbox`, or rely on LiDAR odometry alone?
- If Jetson goes offline mid-navigation, degrade gracefully (LiDAR-only) or stop and wait?
- ROS2 distro mismatch: Pi runs Jazzy, Jetson runs Humble — confirm `zed_msgs` deserializes correctly on Pi side
