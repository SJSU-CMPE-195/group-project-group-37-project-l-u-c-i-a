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
  - Needed so Nav2 on the Pi can deserialize ZED object detection messages from the Jetson (Humble → Jazzy cross-distro)

### ROS2 Nodes — `lucia_control`
- [ ] Implement `src/ros2/lucia_control/lucia_control/roomba_bridge.py`
  - Subscribe to `/cmd_vel` (`geometry_msgs/Twist`)
  - Translate `linear.x` + `angular.z` → `RoombaOI.drive_direct(left, right)`
  - Read bump and cliff sensors; enforce hard stop regardless of Nav2 commands if triggered
  - Publish sensor state on `/roomba/bumpers` and `/roomba/cliffs` (`std_msgs/Bool` or custom)

### ROS2 Config — `lucia_lidar`
- [ ] Create `src/ros2/lucia_lidar/config/rplidar.yaml` — serial port, baud rate, frame ID
- [ ] Create `src/ros2/lucia_lidar/config/slam_toolbox.yaml` — map update rate, scan topic, mode (mapping vs localization)

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

### Hardware
- [ ] Confirm J48 jumper is shorted (barrel jack power enabled)
- [ ] Confirm 5V/4A power supply connected to barrel jack
- [ ] Confirm ZED 2i is in the **blue USB 3.0 port** (not USB 2.0)
- [ ] Check JetPack version: `cat /etc/nv_tegra_release` (need R35.x for ZED SDK 4.x)

### ZED SDK
- [ ] Download ZED SDK 4.x installer for JetPack 5.x from Stereolabs
- [ ] Copy to Jetson and run installer (`chmod +x` → `./ZED_SDK_*.run`)
  - Accept AI modules (object detection neural nets) when prompted
- [ ] Run self-test: `/usr/local/zed/tools/ZED_Diagnostic`
- [ ] Verify live feed: `/usr/local/zed/tools/ZED_Explorer`

### ROS2 Humble
- [ ] Add ROS2 Humble apt repo and install `ros-humble-ros-base`
- [ ] Install `python3-colcon-common-extensions` and `python3-rosdep`
- [ ] `sudo rosdep init && rosdep update`
- [ ] Add to `~/.bashrc`:
  - `source /opt/ros/humble/setup.bash`
  - `export ROS_DOMAIN_ID=42`
  - `export RMW_IMPLEMENTATION=rmw_fastrtps_cpp`

### ZED ROS2 Wrapper
- [ ] Clone `zed-ros2-wrapper` into `~/ros2_ws/src`
- [ ] `rosdep install --from-paths src --ignore-src -r -y`
- [ ] `colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release`
- [ ] Test launch: `ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i`
- [ ] Confirm these topics publish:
  - `/zed/zed_node/obj_det/objects`
  - `/zed/zed_node/depth/depth_registered`
  - `/zed/zed_node/odom`
- [ ] Enable object detection in the ZED param YAML (`od_enabled: true`), relaunch, confirm `obj_det/objects` flows

### Network
- [ ] Set static IP `192.168.1.2` on Jetson `eth0`
- [ ] Connect Ethernet cable to Pi
- [ ] `ping 192.168.1.1` from Jetson — confirm link is up

---

## Integration

- [ ] From Pi Docker container: `ros2 topic list | grep zed` — confirm Jetson topics are visible
- [ ] Echo ZED object detections from the Pi: `ros2 topic echo /zed/zed_node/obj_det/objects --once`
- [ ] Configure Nav2 costmap on the Pi to ingest `/zed/zed_node/obj_det/objects` as dynamic obstacle layer
- [ ] Calibrate TF transform between RPLidar frame and ZED 2i physical mount position
- [ ] Set up `rviz2` on a laptop (same `ROS_DOMAIN_ID=42`) for live map + object visualization
- [ ] End-to-end test: rover navigates to a goal autonomously and avoids a person walking in front of it

---

## Infrastructure

- [ ] Update `README.md` with actual project description, setup steps, and tech stack table
- [ ] Update `docs/setup/ros2-pi.md` with Dockerfile instructions once written
- [ ] Add Jetson setup to `docs/setup/ros2-jetson.md` once steps are validated on real hardware

---

## Open Questions

- Should `roomba_bridge` enforce a hard stop on bump/cliff regardless of Nav2 commands? **(Recommended: yes)**
- Fuse ZED visual odometry into `slam_toolbox`, or rely on LiDAR odometry alone?
- If Jetson goes offline mid-navigation, should Nav2 degrade gracefully (LiDAR-only) or stop and wait?
- Cross-distro `zed_msgs` compatibility (Humble on Jetson → Jazzy on Pi) — confirm messages deserialize correctly before building Nav2 integration on top
