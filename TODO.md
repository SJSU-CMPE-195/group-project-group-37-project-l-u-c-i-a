# Project L.U.C.I.A — TODO

> Rebuilding the robot software as small pieces (ROS2), one piece at a time.
> Order: **Control → Lidar → Bringup.** Check items off as they're done.
> Working branch: `195B-Christian`

---

## Right now

Phase 1, Step A (wheels move) is written and tested on a laptop, but **has not been tried on the real robot yet.**
Next: set up the Pi (bottom of this file), then try it.

---

## Ground rules (already decided)

- **One program per device.** Only the control program talks to the Roomba, and only the lidar program talks to the lidar. Nothing else opens those USB ports.
- **Safe mode.** The Roomba stops on its own if it's lifted or drives off a ledge. The control program turns it back on automatically, so no restart is needed after a pickup.
- **No bump sensor** (it was removed). "Stop, something's too close" uses the lidar and lives in Bringup, not Control.
- **Not now:** the web control panel.

---

## Phase 1 — Control (makes the wheels move)

Folder: `src/ros2/lucia_control/`

**Step A — Wheels move** (needs no wheel sensors)
- [x] Copy the Roomba driver (`roomba_oi.py`) into the package
- [x] Wheel-speed math (`motion.py`) + tests
- [x] Control program (`roomba_bridge.py`): enters Safe mode, listens for drive commands, keeps Safe mode alive, stops the wheels if commands go quiet
- [ ] Try it on the Pi, on the floor with clear space. Drive it over SSH with the keyboard tool (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`) in a second terminal.
- [ ] Pick the robot up and put it back down. It should carry on with no restart.

**Step B — Position tracking and battery**
- [ ] Run `sensor_monitor.py`: do the wheel sensor numbers change when the wheels turn? (Earlier, the Roomba's cable couldn't send readings back.)
- [ ] Measure wheel size and encoder counts per turn (drive a known distance, read the raw counts)
- [ ] Add position tracking (`/odom`) and battery to the control program

---

## Phase 2 — Lidar (builds the map)

Folder: `src/ros2/lucia_lidar/`
- [ ] Lidar settings file (port, speed, frame name)
- [ ] Map-building settings file (`slam_toolbox`)
- [ ] Lidar scans show up on `/scan`
- [ ] A map builds while you drive the robot around a room

Works best after Phase 1, Step B.

---

## Phase 3 — Bringup (starts everything)

Folder: `src/ros2/lucia_bringup/`
- [ ] One launch file that starts control + lidar + mapping together
- [ ] "Stop if something's too close" layer: watches the lidar and cancels drive commands

---

## Setup on the Pi (needed along the way)

- [ ] udev rules, so the Roomba and lidar always show up as `/dev/roomba` and `/dev/rplidar`
  - Until then, run control with `port:=/dev/ttyUSB0`
- [x] Write a Dockerfile so the container can be rebuilt (`docker/Dockerfile`)
  - Includes the serial library control needs, the lidar and mapping packages, and the keyboard driving tool (`teleop_twist_keyboard`)
- [ ] Build it on the Pi: `docker build -t lucia/ros2:latest docker/` (not built yet, no Docker on the laptop to test it). Needs internet, so do it on the ethernet network, not the robot's own WiFi.
- [ ] `update.sh` is hardcoded to pull the `testing` branch, not `195B-Christian`. Fix before using it for this work.
- [ ] Update `docs/setup/ros2-pi.md` to use the new image once it builds

---

## Check on the robot (only you can do these)

- [x] Wheel-drop switches are still there
- [ ] Is the cliff sensor still there?

---

## Later (not started)

- Autonomous navigation (Nav2), once the map and position tracking work
- IMU (motion sensor) blended with wheel data, once that hardware is ready
- ToF distance sensors as an emergency stop, once that hardware arrives
- Camera and Jetson: start from scratch when the time comes
