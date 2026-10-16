# Project L.U.C.I.A — TODO

> Rebuilding the robot software as small pieces (ROS2), one piece at a time.
> Order: **Control → Lidar → Map → Safety stop → SWEEP → one-command bringup.** Check items off as they're done.
> Work on `195B-Christian`, merge into `dev` and push both. The Pi pulls `dev`.

---

## The goal

**The robot maps a room on its own.** Turn it on, it drives itself around (SWEEP mode),
builds the map as it goes, and you save the map at the end. Driving by keyboard is only
a test tool for checking one piece at a time; it is not how the robot is meant to run.

## Right now

Control (wheels, open-loop `/odom`), the lidar (`/scan`), mapping and the IMU all work on the real robot.
2026-10-16: first room map by keyboard driving, then a much cleaner one with Ling's IMU fused in
(`ros2 launch lucia_bringup mapping.launch.py`).
Next: the safety stop (Phase 3) waits on the team's ToF sensors; then SWEEP mode.
Decided 2026-10-16: SWEEP mode first; frontier exploration is a later feature.

---

## Ground rules (already decided)

- **One program per device.** Only the control program talks to the Roomba, and only the lidar program talks to the lidar. Nothing else opens those USB ports.
- **Safe mode.** The Roomba stops on its own if it's lifted or drives off a ledge. The control program turns it back on automatically, so no restart is needed after a pickup.
- **No bump sensor** (it was removed). "Stop, something's too close" will use the ToF distance sensors (decided 2026-10-16) and lives in Bringup, not Control.
- **Web control page** (`lucia_web`): revived on request 2026-10-16. Runs from Docker Compose at boot; on the Pi's hotspot it's at http://10.42.0.1:8080.

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
- [x] Rechecked 2026-10-16, counting bytes instead of decoding them: **zero bytes** come back for battery voltage (packet 22) or either encoder (43/44), even while turning a wheel by hand. The old "reads 0" was really "no reply at all" (the driver turns an empty read into 0). The encoders themselves are untested. A beep command played in the same session, so the Roomba was awake and receiving: only the Roomba-to-Pi direction is broken
- [ ] Fix the reply path (Roomba TX to the adapter's RX, FTDI FT232R). Loopback test: unplug from the Roomba, jumper pins 3 and 4 on the cable's mini-DIN plug, send bytes, see if they echo. Echo → the fault is at the Roomba end (reseat); no echo → the adapter/cable (replace, e.g. a ready-made Create 2 serial cable)
- [x] Added position tracking (`/odom` + the `odom`→`base_link` transform) to the control program (2026-10-09) — **open-loop**, estimated from commanded wheel speed, not a sensor reading, since the cable above isn't fixed yet. Will drift more than real encoder feedback. See `odometry.py` and `docs/changelogs/2026-10-09.md`.
- [x] Tried `/odom` on the real robot: position and heading both tracked driving + turning correctly.
- [ ] Once the cable is fixed: measure wheel size and encoder counts per turn (drive a known distance, read the raw counts), then switch `/odom` to use real encoder readings instead of commanded speed. `odometry.py`'s math doesn't change either way — only what feeds it does.
- [ ] Add battery to the control program (needs the cable fixed too)
- [ ] Recover after the Roomba is power-cycled. The heartbeat only re-sends Safe (131), which a freshly powered-on Roomba ignores until it gets Start (128), so for now `roomba_bridge` has to be restarted. Fix carefully: sending Start while already in Safe mode drops the Roomba to Passive for a moment, so don't just send Start every second

---

## Phase 2 — Lidar (builds the map)

Folder: `src/ros2/lucia_lidar/`
- [x] Lidar settings file (`config/rplidar.yaml`) - port, speed, from how it actually ran last semester
- [x] Map-building settings file (`config/slam_toolbox.yaml`) - package defaults plus this lidar's real range limits
- [x] Tried on the real lidar (2026-10-09): the executable name is `rplidar_composition`, not `rplidar_node` (though the *node* it creates is still named `rplidar_node`, which is what the config file's settings apply to - that part was already right). Hit "Cannot start scan" at first - fixed by explicitly setting `scan_mode: Standard`, already in the config file; the ROS2 driver's own default scan mode isn't compatible with this lidar's firmware (Hardware Rev 5, Firmware 1.25). `/scan` now publishes at ~11.7 Hz.
- [x] Lidar scans show up on `/scan`
- [x] Launch file (`launch/lidar_and_map.launch.py`) starts the lidar + `slam_toolbox` together - written, syntax-checked, **not yet run** (no ROS2 on the laptop that wrote it)
  - Fixed 2026-10-16 before the first run: it now starts `slam_toolbox` through the package's own launch file (Jazzy's slam_toolbox needs a lifecycle activate step, so the executable-name guess is gone), and adds the missing `base_link`→`laser` transform, without which every scan is dropped
- [x] `scan_check` works on the Pi (2026-10-16): readings follow a hand moved around the lidar
- [x] Bench test, robot stationary (2026-10-16): launch file + `roomba_bridge` → slam_toolbox configured, activated, built and saved a map (`map_saver_cli`, 68×121 cells @ 5 cm). Only a small starburst is filled in: a stationary robot gives slam_toolbox a single scan, and it leaves cells unknown until a ray has crossed them at least twice. Driving fills it in
- [x] Settled which way the lidar faces (2026-10-16, `scan_check`): a hand in front of the robot read about -170°, a hand on its left about -92°. That's a plain 180° turn, not mirrored, so the default `laser_yaw` (pi) is right
- Known quirk: the lidar driver sometimes freezes after `Start` on the first run (motor spins, no `/scan`, Ctrl+C ignored). Ctrl+\ and rerun fixes it
- [x] A map builds while you drive the robot around a room (2026-10-16, `room1.png`): about a 2.6 × 2.9 m room with straight walls and clear corners, plus an opening on one side. One wall shows up doubled (about 20-30 cm apart), most likely heading drift from the open-loop `/odom`; it should improve once real encoders feed `/odom`

Works best after Phase 1, Step B.

**IMU: better heading for mapping** (Ling's BNO08X on a Pico 2, USB to the Pi)

The first map's doubled wall is heading drift from the open-loop `/odom`. The IMU fixes heading
without waiting on the Roomba cable.
- [x] Pico streaming code `src/imu/stream_main.py` (upload as `main.py`): game rotation vector + gyro, one `I,qw,qx,qy,qz,gx,gy,gz` line per reading at about 50 Hz. Ling to review against her setup
- [x] `lucia_imu` package: `imu_bridge` node reads the Pico and publishes `/imu/data` (parser has pure-Python tests). Not yet run on the Pi
- [x] Pico streams `I,` lines when plugged into the Pi (`/dev/ttyACM0`), about 50/s (2026-10-16). Loaded from the Pi with `mpremote` (installed via `pipx`): `mpremote connect /dev/ttyACM0 fs cp src/imu/stream_main.py :main.py`. That Pico previously held a ToF `main.py`; it's backed up on the Pi at `~/tof_main_backup.py`. The driver files `bno08x.py` / `i2c.py` live in the Pico's root (Ling's copies, import already fixed). Unused duplicates also got copied to the Pico's `lib/`; delete them next time the Pico is in VS Code (`mpremote` couldn't interrupt the running stream to do it)
- [x] udev rule `/dev/imu` added to `/etc/udev/rules.d/99-lucia.rules` (2026-10-16), matching the Pico's serial `fd92d5f577b9417b` (the ToF Pico has the same vendor/product ID). Confirm `ls -la /dev/imu` shows the link, and use `--device=/dev/imu` in the `docker run` command
- [x] `imu_bridge` on the Pi (2026-10-16): 50 readings/s, 0 bad lines. A quarter turn left by hand read +94.7° (positive = left, the ROS convention) and came back to about +1-2° when turned back
- [ ] Mount the IMU flat and fixed on the robot; note which way its axes point (needs a `base_link`→`imu_link` transform, like the lidar's)
- [x] Fusion written (not yet run): `lucia_bringup/config/ekf.yaml` takes forward speed from `/odom` and heading + turn rate from `/imu/data`; `roomba_bridge` gained `publish_tf` (off when the EKF runs) and now fills in `/odom`'s twist with rough variances. `ros2 launch lucia_bringup mapping.launch.py` starts roomba_bridge + imu_bridge + EKF + lidar + slam_toolbox (`use_imu:=false` = the first-map setup)
- [x] Image rebuilt on the Pi (about 6.5 min); plain `colcon build` skips `lucia_vision`. `mapping.launch.py` starts cleanly: `imu_bridge` on `/dev/imu`, `roomba_bridge` hands `odom`→`base_link` to the EKF, slam_toolbox registers the lidar, no warnings (2026-10-16)
- [x] Mapped the same room again with the IMU (`room2.png`, 2026-10-16): walls are single thin lines with square corners and much less speckle; the doubled wall from `room1.png` is gone. One run each with different starting headings, so not a controlled comparison, but the heading fix is clearly visible
- [ ] `imu_bridge`: skip the first, partial line after opening the port (it shows up as a harmless `Pico: 01477,...` log line)

---

## Phase 3 — Safety stop (before anything drives itself)

Folder: `src/ros2/lucia_bringup/`

Decided 2026-10-16: built on the **ToF distance sensors** the team is implementing
(`src/tof_pico/`, VL53L5CX on a Pico), not the lidar. Waits until those are ready.
- [ ] ToF Pico → Pi → ROS2 (same pattern as the IMU: Pico streams over USB, a bridge node publishes)
- [ ] "Stop if something's too close" node: blocks drive commands that would hit something.
  Stands in for the removed bump sensor.
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

Folder: `src/ros2/lucia_bringup/`, `src/ros2/lucia_web/`, `docker/compose.yml`
- [x] `mapping.launch.py`: control + IMU + EKF + lidar + slam_toolbox (2026-10-16). Safety stop and SWEEP get added once they exist
- [x] Web control page `lucia_web` (written 2026-10-16, not yet run on the Pi): status lights for lidar/IMU/wheels,
  Start/Stop mapping (runs `mapping.launch.py`, IMU on/off), live map with the robot's position, hold-to-drive pad
  and arrow keys (test tool; the page stops sending when released and `roomba_bridge` times out as a second layer),
  save map + download PNG. Plain `http.server`, no new packages. Parsing/validation and process control have
  pure-Python tests; the HTTP endpoints were smoke-tested with ROS stubbed out
- [x] `docker/compose.yml` (written, not yet run): container in the background, `restart: unless-stopped` so it starts
  at boot, runs `colcon build` then `web_ui`. Shares `/dev` with device-type rules, so an unplugged Pico or lidar at
  boot doesn't stop the container from starting
- [ ] On the Pi: `docker compose -f docker/compose.yml up -d`, open the page, map a room from it
- [ ] Reboot test: Pi comes up on its hotspot, laptop joins `lucia-control`, page loads at http://10.42.0.1:8080
- [ ] Later: a "Start SWEEP" button replaces the drive pad as the main control

---

## Setup on the Pi (needed along the way)

- [x] The old web control panel no longer starts at boot. `lucia.service` ran `slam_avoid_server.py` on the same serial port and GPIO pins, which fought the ROS2 control program. Disabled on the Pi on 2026-09-18 with `sudo systemctl disable --now lucia`. To use the old panel again: `sudo systemctl start lucia`, and stop it before any ROS2 test. Tell the team.
- [x] The Pi's own WiFi (`lucia-control`) comes up after a reboot. Confirmed 2026-09-18. `pi-ap` was given priority 100 so it wins over saved networks like LiveLaughLove.
- [x] udev rules, so the Roomba and lidar always show up as `/dev/roomba` and `/dev/rplidar`. Already existed: confirmed 2026-10-16 (`/dev/rplidar` → `ttyUSB0`, `/dev/roomba` → `ttyUSB1`), so no `port:=` override is needed
- [x] Write a Dockerfile so the container can be rebuilt (`docker/Dockerfile`)
  - Includes the serial library control needs, the lidar and mapping packages, and the keyboard driving tool (`teleop_twist_keyboard`)
- [x] Build it on the Pi: `docker build -t lucia/ros2:latest docker/` (built fine, about 8 minutes). Needs internet, so do it on a network with internet, not the robot's own WiFi.
- [x] `update.sh` was hardcoded to pull the `testing` branch. Fixed 2026-10-16 to pull `dev`, the team's new shared integration branch.
- [ ] Update `docs/setup/ros2-pi.md` to use the new image once it builds
- [x] `colcon build` on the Pi fails on `lucia_vision` (Jetson/ZED package, needs CUDA) and then skips the rest. The Dockerfile now writes a colcon defaults file (skip `lucia_vision`, symlink-install), so plain `colcon build` works once the image is rebuilt

---

## Check on the robot (only you can do these)

- [x] Wheel-drop switches are still there
- [ ] Is the cliff sensor still there?

---

## Later (not started)

- Frontier exploration (feature, after SWEEP works): Nav2 + `m-explore-ros2` steers
  the robot toward unmapped areas instead of bouncing around. Needs real encoder
  odometry first (the cable fix in Phase 1, Step B).
- Camera and Jetson: start from scratch when the time comes
