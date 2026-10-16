# Project L.U.C.I.A — TODO

> Rebuilding the robot software as small pieces (ROS2), one piece at a time.
> Order: **Control → Lidar → Bringup.** Check items off as they're done.
> Working branch: `195B-Christian`

---

## The goal

**The robot maps a room on its own.** Turn it on, it drives itself around (SWEEP mode),
builds the map as it goes, and you save the map at the end. Driving by keyboard is only
a test tool for checking one piece at a time; it is not how the robot is meant to run.

## Right now

Control (wheels, open-loop `/odom`) and the lidar (`/scan`) both work on the real robot.
Next: confirm a map builds (Phase 2), then the safety stop, then SWEEP mode.
Decided 2026-10-16: SWEEP mode first; frontier exploration is a later feature.

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
- [x] Added position tracking (`/odom` + the `odom`→`base_link` transform) to the control program (2026-10-09) — **open-loop**, estimated from commanded wheel speed, not a sensor reading, since the cable above isn't fixed yet. Will drift more than real encoder feedback. See `odometry.py` and `docs/changelogs/2026-10-09.md`.
- [x] Tried `/odom` on the real robot: position and heading both tracked driving + turning correctly.
- [ ] Once the cable is fixed: measure wheel size and encoder counts per turn (drive a known distance, read the raw counts), then switch `/odom` to use real encoder readings instead of commanded speed. `odometry.py`'s math doesn't change either way — only what feeds it does.
- [ ] Add battery to the control program (needs the cable fixed too)

---

## Phase 2 — Lidar (builds the map)

Folder: `src/ros2/lucia_lidar/`
- [x] Lidar settings file (`config/rplidar.yaml`) - port, speed, from how it actually ran last semester
- [x] Map-building settings file (`config/slam_toolbox.yaml`) - package defaults plus this lidar's real range limits
- [x] Tried on the real lidar (2026-10-09): the executable name is `rplidar_composition`, not `rplidar_node` (though the *node* it creates is still named `rplidar_node`, which is what the config file's settings apply to - that part was already right). Hit "Cannot start scan" at first - fixed by explicitly setting `scan_mode: Standard`, already in the config file; the ROS2 driver's own default scan mode isn't compatible with this lidar's firmware (Hardware Rev 5, Firmware 1.25). `/scan` now publishes at ~11.7 Hz.
- [x] Lidar scans show up on `/scan`
- [x] Launch file (`launch/lidar_and_map.launch.py`) starts the lidar + `slam_toolbox` together - written, syntax-checked, **not yet run** (no ROS2 on the laptop that wrote it)
  - Fixed 2026-10-16 before the first run: it now starts `slam_toolbox` through the package's own launch file (Jazzy's slam_toolbox needs a lifecycle activate step, so the executable-name guess is gone), and adds the missing `base_link`→`laser` transform, without which every scan is dropped
- [ ] Check which way the lidar faces: `ros2 run lucia_lidar scan_check` with a box in front of the robot (sets `laser_yaw`)
- [ ] A map builds while you drive the robot around a room - verify by saving it as a PNG (`map_saver_cli`), not a live viewer (see `docs/changelogs/2026-10-09.md`)

Works best after Phase 1, Step B.

---

## Phase 3 — Safety stop (before anything drives itself)

Folder: `src/ros2/lucia_bringup/`
- [ ] "Stop if something's too close" node: watches `/scan` and blocks drive commands
  that would hit something. Stands in for the removed bump sensor.
- [ ] Test it on its own, driving by keyboard toward a wall, before SWEEP mode uses it

---

## Phase 4 — SWEEP mode (the robot drives itself)

Folder: `src/ros2/lucia_bringup/`
- [ ] Port last semester's SWEEP logic (`src/scripts/slam_avoid_server.py`) to a ROS2
  node: drive straight; when something's ahead, spin toward the clearer side until
  the front is clear; repeat. Reads `/scan`, publishes drive commands. Needs no
  encoders, so it works with the current open-loop `/odom`.
- [ ] Drive commands go through the safety stop, never straight to the wheels
- [ ] Start/stop it without a keyboard session (e.g. a start/stop topic or service)

---

## Phase 5 — One command starts everything

Folder: `src/ros2/lucia_bringup/`
- [ ] One launch file: control + lidar + mapping + safety stop + SWEEP
- [ ] Docker Compose file so the container runs in the background and survives SSH drops
- [ ] Optionally start it all at boot

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

- Frontier exploration (feature, after SWEEP works): Nav2 + `m-explore-ros2` steers
  the robot toward unmapped areas instead of bouncing around. Needs real encoder
  odometry first (the cable fix in Phase 1, Step B).
- A way to actually view the map - live (RViz2, needs ROS2/Docker on a machine
  with a screen) or a revived web control panel. The web panel specifically was
  deferred on purpose, reaffirmed 2026-10-09 - treat reviving it as its own
  decision, not something to bundle into another task.
- IMU (motion sensor) blended with wheel data, once that hardware is ready
- ToF distance sensors as an emergency stop, once that hardware arrives
- Camera and Jetson: start from scratch when the time comes
