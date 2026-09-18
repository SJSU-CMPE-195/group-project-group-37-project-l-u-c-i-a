# Project L.U.C.I.A — TODO

> Rebuilding the robot software as small pieces (ROS2), one piece at a time.
> Order: **Control → Lidar → Bringup.** Check items off as they're done.
> Working branch: `195B-Christian`

---

## Right now

Phase 1, Step A (wheels move) **works on the real robot**: it drives from the keyboard tool over SSH.
The pickup test passes too: it stops when lifted and drives again when set down, with no restart.
Step A is done. Next: Step B (position tracking and battery).

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
- [x] Try it on the Pi, on the floor with clear space. Drive it over SSH with the keyboard tool (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`) in a second terminal. Hold the keys down, since a single tap is too brief to see.
- [x] Pick the robot up and put it back down. It should carry on with no restart.

**Step B — Position tracking and battery**
- [x] Checked 2026-09-18: **the Roomba's replies don't reach the Pi.** Battery voltage reads 0, encoders read 0. Driving still works, so the Pi-to-Roomba direction is fine.
- [ ] Fix the reply path (Roomba TX to the adapter's RX): check the wiring at both ends, test the adapter alone with a loopback, or try another adapter. Recheck with the battery voltage read.
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

- [x] The old web control panel no longer starts at boot. `lucia.service` ran `slam_avoid_server.py` on the same serial port and GPIO pins, which fought the ROS2 control program. Disabled on the Pi on 2026-09-18 with `sudo systemctl disable --now lucia`. To use the old panel again: `sudo systemctl start lucia`, and stop it before any ROS2 test. Tell the team.
- [x] The Pi's own WiFi (`lucia-control`) comes up after a reboot. Confirmed 2026-09-18. `pi-ap` was given priority 100 so it wins over saved networks like LiveLaughLove.
- [ ] udev rules, so the Roomba and lidar always show up as `/dev/roomba` and `/dev/rplidar`
  - They may already exist (the old service uses those names). Check: `ls -la /dev/roomba /dev/rplidar`
  - Until then, run control with `port:=/dev/ttyUSB0`
- [x] Write a Dockerfile so the container can be rebuilt (`docker/Dockerfile`)
  - Includes the serial library control needs, the lidar and mapping packages, and the keyboard driving tool (`teleop_twist_keyboard`)
- [x] Build it on the Pi: `docker build -t lucia/ros2:latest docker/` (built fine, about 8 minutes). Needs internet, so do it on a network with internet, not the robot's own WiFi.
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
