# Phase 3 — ROS2 Humble Installation

Install ROS2 Humble on the Jetson. Humble targets Ubuntu 22.04 (Jammy),
which is what JetPack 5.x ships.

---

## Step 1 — Add the ROS2 apt repository

```bash
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
```

**Result:** <!-- Success / Errors -->

---

## Step 2 — Install ROS2 Humble base + tools

```bash
sudo apt install -y \
  ros-humble-ros-base \
  python3-colcon-common-extensions \
  python3-rosdep
```

**Result:** <!-- Success / Errors -->

---

## Step 3 — Initialize rosdep

```bash
sudo rosdep init
rosdep update
```

**Actual output:**
```
# paste here
```

---

## Step 4 — Configure ~/.bashrc

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export RMW_IMPLEMENTATION=rmw_fastrtps_cpp" >> ~/.bashrc
source ~/.bashrc
```

---

## Step 5 — Smoke test

```bash
ros2 topic list
```

Expected output:
```
/parameter_events
/rosout
```

**Actual output:**
```
# paste here
```

**Result:** <!-- ROS2 working / not working -->

---

## Notes

<!-- Any install issues, version conflicts, workarounds -->
