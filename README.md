# Project L.U.C.I.A.

SJSU CMPE 195A/B senior design, team 37. **Advisor:** Professor Wencen Wu

## Team

| Name | GitHub | Email |
|------|--------|-------|
| Christian Michael Villanueva | [@Chris4lyfie](https://github.com/Chris4lyfie) | christianmichael.villanueva@sjsu.edu |
| Ling Tang | [@ling-tang0922](https://github.com/ling-tang0922) | ling.tang@sjsu.edu |
| Ethanael Grimares | [@Memekushi](https://github.com/Memekushi) | ethanael.grimares@sjsu.edu |
| Noel Temores | [@noel-tem](https://github.com/noel-tem) | noel.temores@sjsu.edu |


## Project Description

An indoor robot that maps a room on its own. Built on a Roomba 650: turn it on, and it
explores the room by itself, builds a map with its lidar, and drives back to where it
started. A Raspberry Pi 5 runs everything and serves a control page over the robot's
own WiFi.

## Prototype Scope (October 2026)

- **Explores on its own.** Frontier exploration: it drives to the edges of what it has
  mapped, one after another, until nothing reachable is left, then returns HOME. Works on
  the robot as of 2026-10-17.
- **Builds a 2D map** with the lidar (slam_toolbox), with heading from an IMU (BNO08X) fused
  in, which removed the drift of the first maps.
- **Web control page** at http://10.42.0.1:8080: start mapping, Explore / Return HOME / STOP,
  a live radar of the lidar, live metrics (heading, speed, distance, area mapped, Pi
  health), every node's log, Reconnect Roomba (after it was turned off and on, without
  losing the map), a horn and tunes on the Roomba's speaker (including melodies imported
  from MIDI files), drive by keyboard for testing, save and download maps.
- **Starts by itself:** power on and the Pi brings up its hotspot and the page; no screen,
  keyboard or other network needed.
- **In progress:** a time-of-flight safety stop for things below the lidar (Noel), object
  recognition with a ZED 2i camera on a Jetson Nano (Ethan). Not working yet: battery and
  wheel encoder readings (the Roomba's serial replies don't reach the Pi).

![First maps: wheels only vs. wheels + IMU](media/maps/2026-10-16_room1_vs_room2.png)

## System Architecture


## Prerequisites


## Installation 


## Running the Prototype

1. Turn on the Roomba and the Pi. Give it about a minute: the Pi brings up its WiFi
   hotspot and starts the control page by itself.
2. Join the WiFi **`lucia-control`** (password `lucia-143-tomato`).
3. Open **http://10.42.0.1:8080**.
4. Press **Start mapping**. Wait for the Lidar, IMU, Wheels and Navigation lights to turn green.
5. Press **Explore**. Stay near it: it steers around what the lidar sees, not things lower
   than the lidar. **STOP** (top right, or Space / Esc) stops it. Driving with the keys
   takes over; it carries on exploring 3 s after you let go. If it gets stuck, it backs
   up and tries elsewhere; stuck 3 times in a row, it stops and plays a tune for help.
6. When it's back HOME, type a name and press **Save**; download the PNG from the list.

If the Roomba was turned off and on (or its battery died), turn it on and press
**Reconnect Roomba**; it beeps when it's listening again. **Horn** (or the H key) honks.
To turn everything off, `sudo poweroff` over SSH, or the Pi's power button (twice on the
desktop OS; not tried yet), rather than pulling the plug.

The full how-to (what each light means, the logs, updating the code, switching the Pi to
another WiFi for internet, debugging): **[`docs/setup/ros2-pi.md`](docs/setup/ros2-pi.md)**.

## Network

| | |
|---|---|
| Hotspot (at every boot) | SSID `lucia-control`, password `lucia-143-tomato` |
| Pi's address on the hotspot | `10.42.0.1` (page on port 8080, SSH `ssh lucia@10.42.0.1`) |
| Laptops / phones on the hotspot | get `10.42.0.10`–`10.42.0.100` |
| Internet | Not on the hotspot. To update the code, switch the Pi to a WiFi with internet: see "Network modes" in [`ros2-pi.md`](docs/setup/ros2-pi.md). Rebooting brings the hotspot back. |

First-time hotspot setup on a fresh Pi: [`docs/Raspberry Pi Initial Setup.md`](docs/Raspberry%20Pi%20Initial%20Setup.md).

## Testing


## Demo


## Technical Stack

**Hardware:** Roomba 650 base · Raspberry Pi 5 (256 GB microSD, 5 A UPS) · RPLidar A2M8 ·
BNO08X IMU on a Raspberry Pi Pico 2 · VL53L5CX time-of-flight sensors on a Pico 2 (in
progress) · Jetson Nano + ZED 2i stereo camera (in progress).

**Software:** ROS 2 Jazzy in Docker on the Pi (the Pi runs Debian 13, which ROS 2 doesn't
support directly). The container starts at boot (Docker Compose) and serves the control page.

| Package (`src/ros2/`) | What it does |
|---|---|
| `lucia_control` | `roomba_bridge`: drives the wheels over serial (Safe mode), position estimate, reconnect, tunes |
| `lucia_lidar` | RPLidar driver + slam_toolbox (the map) |
| `lucia_imu` | `imu_bridge`: reads the IMU Pico, publishes heading and turn rate |
| `lucia_bringup` | `mapping.launch.py` (starts everything), IMU + wheels fusion (EKF), `safety_gate` |
| `lucia_nav` | Nav2 set up for the Roomba, and `mission`: Explore, Return HOME, go to a spot |
| `lucia_web` | `web_ui`: the control page (also: logs, metrics, tunes and MIDI import) |
| `lucia_vision` | Jetson / ZED camera code (doesn't build on the Pi) |

Code for the Picos: `src/imu/` (IMU) and `src/tof_pico/` (ToF sensors).
Algorithms: SLAM (slam_toolbox), frontier exploration (our own, `lucia_nav/frontier.py`),
path planning and following (Nav2), sensor fusion (EKF, robot_localization), YOLO object
detection (Jetson, in progress). Languages: Python, MicroPython, C++.

## Docs

| | |
|---|---|
| [`docs/setup/ros2-pi.md`](docs/setup/ros2-pi.md) | Running, updating and debugging the robot |
| [`TODO.md`](TODO.md) | Live task list |
| [`docs/changelogs/`](docs/changelogs/) | What changed when, session by session |
| [`docs/setup/udev-rules.md`](docs/setup/udev-rules.md) | Fixed device names (`/dev/roomba`, `/dev/rplidar`, `/dev/imu`) |
| [`docs/BNO08X Pico2 Setup.md`](docs/BNO08X%20Pico2%20Setup.md) | The IMU Pico |
| [`docs/setup/ros2-jetson.md`](docs/setup/ros2-jetson.md), [`docs/jetson_nano_zed_setup.md`](docs/jetson_nano_zed_setup.md) | Jetson and camera |
| [`docs/Roomba Serial Scripts/`](docs/Roomba%20Serial%20Scripts/Roomba%20Serial%20Scripts.md) | Last semester's Python scripts for the Roomba (`src/scripts/`) |

## Last semester (195A): proof of concept

The first goal was to find out whether a Roomba could be taken over and driven from a
computer, or whether the team would need a different base. It could: Python scripts in
`src/scripts/` drive the Roomba 650 over its serial port (iRobot Open Interface), read its
sensors and play songs; their reference is in
[`docs/Roomba Serial Scripts/`](docs/Roomba%20Serial%20Scripts/Roomba%20Serial%20Scripts.md).
The lidar and camera were new to the team and were first tried out then.

### 3/22 Demo
![Demo 03/22/26](media/0322_demo.gif)

### Robot (May 8)
![Robot 05/08/26](media/0508_robot.png)

## CAD drawings

Latest version of each printed part; earlier versions are in
[`media/CAD_Drawings/`](media/CAD_Drawings/).

| | |
|---|---|
| Pi drawer V4 | ![Pi Drawer V4](media/CAD_Drawings/PiDrawer_V4_Drawing.png) |
| Jetson drawer V3 | ![Jetson Drawer V3](media/CAD_Drawings/JetsonDrawer_V3_Drawing.png) |
| Pi + Jetson assembly V2 | ![Pi and Jetson Assembly V2](media/CAD_Drawings/PiJetson_V2_Assembly_Drawing.png) |
| Arm V1 | ![Arm V1](media/CAD_Drawings/Arm_V1_Drawing.png) |
| Arm drawer V2 | ![Arm Drawer V2](media/CAD_Drawings/Arm_Drawer_V2_Drawing.png) |
| Lidar mount V1 | ![Lidar Mount V1](media/CAD_Drawings/LIDAR_Mount_V1_Drawing.png) |

## What's next

- Time-of-flight safety stop, so it can run without someone next to it
- Fix the Roomba's serial reply line: battery level and wheel encoders (more precise driving)
- Object recognition on the Jetson (ZED 2i + YOLO), sent to the Pi
- Later ideas: target tracking (follow a chosen object at a distance), 3D mapping with the camera
