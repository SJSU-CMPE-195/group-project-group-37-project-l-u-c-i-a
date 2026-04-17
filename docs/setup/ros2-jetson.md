# ROS2 Setup — Jetson Nano

## Overview

The Jetson Nano runs ROS2 Jazzy natively via the official Ubuntu Noble apt packages.
JetPack 5.x provides Ubuntu 22.04 — however, ZED SDK 4.x requires JetPack 5.x which
ships Ubuntu 22.04 (Jammy), meaning ROS2 Humble is the correct distro for the Jetson.

> Note: confirm JetPack version before proceeding. Run `cat /etc/nv_tegra_release`.

---

## Prerequisites

- Jetson Nano flashed with JetPack 5.x
- J48 jumper shorted (barrel jack power enabled)
- 5V/4A power supply connected to barrel jack
- ZED 2i connected to USB 3.0 (blue port)
- Ethernet cable connected to Raspberry Pi

---

## Step 1 — Flash JetPack 5.x

Download NVIDIA SDK Manager on a host Ubuntu machine and flash the Jetson Nano with
JetPack 5.x. This provides Ubuntu 22.04 + CUDA + cuDNN.

---

## Step 2 — Install ZED SDK

Download the ZED SDK 4.x installer for JetPack 5.x from the Stereolabs website and run:

```bash
chmod +x ZED_SDK_*.run
./ZED_SDK_*.run
```

Run the self-test to confirm the ZED 2i is detected:

```bash
/usr/local/zed/tools/ZED_Explorer
```

---

## Step 3 — Install ROS2 Humble

```bash
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

---

## Step 4 — Install ZED ROS2 wrapper

```bash
cd ~/ros2_ws/src
git clone --recurse-submodules https://github.com/stereolabs/zed-ros2-wrapper.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Step 5 — Configure network

Set static IP on the Ethernet interface:

```bash
sudo nmcli con mod "Wired connection 1" \
  ipv4.addresses 192.168.1.2/24 \
  ipv4.method manual
sudo nmcli con up "Wired connection 1"
```

Add to `~/.bashrc`:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

---

## Step 6 — Test ZED topics

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

In a second terminal:

```bash
ros2 topic list
# Should show /zed/zed_node/obj_det/objects, /zed/zed_node/depth/depth_registered, etc.
```

---

## Step 7 — Confirm Pi can see Jetson topics

From the Raspberry Pi container:

```bash
ros2 topic list
# Should show all Jetson topics alongside Pi topics
```

If topics are not visible, check that both boards have the same `ROS_DOMAIN_ID` and
are reachable over Ethernet (`ping 192.168.1.1` from Jetson, `ping 192.168.1.2` from Pi).
