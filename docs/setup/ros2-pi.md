# ROS2 Setup — Raspberry Pi

## Overview

The Raspberry Pi runs Debian 13 (Trixie), which is not officially supported by any
ROS2 distro. ROS2 Jazzy targets Ubuntu 24.04 (Noble) — its apt packages depend on
Ubuntu-specific library names (`libpython3.12t64`, `libtinyxml2-10`) that do not
exist in Debian Trixie and cannot be installed directly.

**Solution: run ROS2 Jazzy inside a Docker container.**

Docker is given `--network=host` so ROS2 DDS discovery works across the network,
and USB devices are passed through via `--device` flags.

---

## Prerequisites

- Raspberry Pi running Debian 13 (Trixie), 64-bit (aarch64)
- SSH access to the Pi
- Internet connection on the Pi

---

## Step 1 — Install Docker

```bash
curl -fsSL https://get.docker.com | sh
```

Add your user to the docker group so `sudo` is not needed:

```bash
sudo usermod -aG docker lucia
```

Log out and back in for the group change to take effect:

```bash
exit
# reconnect via SSH
ssh lucia@192.168.0.140
```

Confirm Docker is working:

```bash
docker --version
groups  # should include 'docker'
```

---

## Step 2 — Pull the ROS2 Jazzy image

```bash
docker pull ros:jazzy-ros-base
```

This is ~700MB on arm64 and will take a few minutes.

Confirm the image is available:

```bash
docker images
```

---

## Step 3 — Set up udev rules for stable device names

*(Do this before running the container so device names are stable.)*

Find the USB vendor/product IDs of each device:

```bash
# Plug in each device one at a time and run:
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|{serial}'
```

Create the udev rules file:

```bash
sudo nano /etc/udev/rules.d/99-lucia.rules
```

Add the following (replace `<serial_number>` with the values found above):

```
# Roomba 650
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<roomba_serial>", SYMLINK+="roomba"

# RPLidar A2
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<rplidar_serial>", SYMLINK+="rplidar"
```

Reload udev:

```bash
sudo udevadm control --reload
sudo udevadm trigger
```

Confirm the symlinks exist:

```bash
ls -la /dev/roomba /dev/rplidar
```

---

## Step 4 — Run ROS2 Jazzy container

```bash
docker run -it --rm \
  --network=host \
  --device=/dev/rplidar \
  --device=/dev/roomba \
  -v $(pwd)/src/ros2:/ros2_ws/src \
  --name lucia_ros2 \
  ros:jazzy-ros-base \
  bash
```

Flag breakdown:
| Flag | Purpose |
|------|---------|
| `--network=host` | ROS2 DDS discovery works across the network |
| `--device=/dev/rplidar` | Passes RPLidar through to container |
| `--device=/dev/roomba` | Passes Roomba serial through to container |
| `-v $(pwd)/src/ros2:/ros2_ws/src` | Mounts our ROS2 packages into the container |
| `--name lucia_ros2` | Names the container for easy reference |

---

## Step 5 — Install ROS2 packages inside the container

Once inside the container:

```bash
apt update && apt install -y \
  ros-jazzy-rplidar-ros \
  ros-jazzy-slam-toolbox \
  ros-jazzy-nav2-bringup \
  python3-colcon-common-extensions
```

---

## Step 6 — Build the LUCIA workspace

```bash
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Step 7 — Test the RPLidar

Source ROS2 and launch the RPLidar node:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rplidar_ros rplidar_a2m8_launch.py \
  serial_port:=/dev/rplidar \
  serial_baudrate:=115200
```

In a second terminal (attach to the running container):

```bash
docker exec -it lucia_ros2 bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /scan
```

You should see a stream of `sensor_msgs/LaserScan` messages. If you do, the RPLidar
is working correctly.

---

## Notes

- The `--rm` flag means the container is deleted when you exit. For persistent use,
  remove `--rm` and use `docker start lucia_ros2` to restart it.
- The repo must be pulled/updated on the Pi before mounting via `-v` so the container
  sees the latest `src/ros2/` packages.
- `ROS_DOMAIN_ID=42` should be set inside the container (add to `~/.bashrc` inside
  or pass as `-e ROS_DOMAIN_ID=42` in the `docker run` command).
