# Project L.U.C.I.A — TODO

> Rebuilding the robot software as small pieces (ROS2), one piece at a time.
> Order: **Control → Lidar → Map → Safety stop → driving itself (frontier exploration; SWEEP) → one-command bringup.** Check items off as they're done.
> Work on `195B-Christian`, merge into `dev` and push both. The Pi pulls `dev`.

---

## The goal

**The robot maps a room on its own.** Turn it on, it drives itself around (Explore:
frontier exploration), builds the map as it goes, comes back HOME, and you save the map at the end. Driving by keyboard is only
a test tool for checking one piece at a time; it is not how the robot is meant to run.

## Right now

Control (wheels, open-loop `/odom`), the lidar (`/scan`), mapping and the IMU all work on the real robot.
2026-10-16: first room map by keyboard driving, then a much cleaner one with Ling's IMU fused in,
then the web control page: power on, join `lucia-control`, open http://10.42.0.1:8080
(how-to: `docs/setup/ros2-pi.md`; maps in `media/maps/`).
Decided 2026-10-16: SWEEP mode first; frontier exploration is a later feature.
**Changed the same evening, at Christian's request:** frontier exploration, return HOME, a live
radar and a speed control on the web page, built on Nav2 (already in the Docker image). All
written and tested on the laptop (pure-Python tests + the page against a simulated robot).
**2026-10-17: Explore works on the robot** after fixing its routes (Phase 4). Then the web page v2
(big map, radar + metrics sidebar, on-screen logs, working keys; Phase 5), laptop-tested, not yet on
the Pi. The ToF safety gate is written too (`use_tof`, off until the sensors are mounted).

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
- [x] ToF Pico → Pi → ROS2: Noel's `lucia_tof` package (branch `tof_pico`, not merged into `dev` yet).
  Publishes an 8x8 distance grid and nearest reading per sensor, and `tof/near` (Bool: true = something
  within 300 mm, or a sensor silent). Before it goes on the Pi: its udev rule (`99-lucia-tof.rules`) matches
  any MicroPython Pico (`2e8a:0005`) without a serial number, so `/dev/lucia_tof` can grab the IMU Pico
  too. Add the ToF Pico's `ATTRS{serial}`, like the `/dev/imu` rule
- [x] "Stop if something's too close" node (2026-10-16, not yet run): `lucia_bringup` `safety_gate`.
  Everything that drives (page, Nav2, mission) publishes `/cmd_vel`; the gate passes it to `/cmd_vel_safe`,
  which `roomba_bridge` listens to in `mapping.launch.py`. With `use_tof:=true` it blocks forward motion
  while `tof/near` is true or silent; turning and backing up still pass so Nav2 can turn away. Default
  `use_tof:=false` = pass-through. Rules in `safety_logic.py` (tested)
- [ ] Merge `lucia_tof` (with the udev fix), mount the sensors, add its node to `mapping.launch.py` under `use_tof`
- [ ] Test it on its own, driving by keyboard toward a wall, before Explore relies on it

---

## Phase 4 — The robot drives itself

**Frontier exploration + return HOME** (Nav2), pulled forward 2026-10-16 at Christian's request.
Folder: `src/ros2/lucia_nav/`
- [x] Nav2 settings for the Roomba (`config/nav2.yaml`): Regulated Pure Pursuit controller, NavFn planner,
  spin/backup/wait recoveries, costmaps from `/map` + `/scan`. Default speed 0.15 m/s; the page's speed
  slider overrides it through Nav2's `speed_limit` topic. `launch/navigation.launch.py` starts only the four
  Nav2 servers used (nav2_bringup's launch also starts docking/collision-monitor servers that need their own
  settings). Included in `mapping.launch.py` (`use_nav:=false` to leave it out)
- [x] `mission` node: `/mission/command` (`explore`, `home`, `goto X Y`, `stop`) → Nav2 goals;
  `/mission/state` (JSON) for the page. Explore = find frontiers in `/map` (edges between seen-open and
  unseen), drive to the best one (near beats far, long beats short), skip ones that fail or time out,
  move on early if the lidar sees past the target on the way, go HOME when none are left.
  HOME = the map origin = where the robot was when mapping started. Our own frontier code instead of
  `m-explore-ros2` (not released for Jazzy, C++ build on the Pi). Logic in `frontier.py` /
  `mission_logic.py`, tested, including a full simulated two-room exploration
- [ ] First run on the Pi (after `git pull` + `docker compose -f docker/compose.yml restart`; no image rebuild
  expected since `ros-jazzy-nav2-bringup` is already installed):
  - [x] Executable names right, Nav2 starts (2026-10-16): Explore was enabled and Nav2 planned a path, which
    needs all four servers active. (To check by hand: `ros2 pkg executables <pkg>`, one package at a time)
  - [x] First Explore (2026-10-16): **bad routes.** The planner was allowed through unknown space
    (`allow_unknown: true`) and planned out of a gap in the wall and around the outside of the room, and the
    frontier picker chose by straight-line distance, so it picked a frontier with no mapped way there; the
    robot got stuck. Fixed, not yet re-run: `allow_unknown: false`, and frontiers now only count if a flood
    from the robot over mapped floor (0.2 m from walls) reaches them, ranked by that driving distance
  - [x] Explore again with the fix (2026-10-17): works ("working beautifully")
  - [ ] Radar: a hand in front of the robot shows at the top, a hand on its left shows on the left
  - [ ] Speed slider: the drive pad gets faster/slower
  - [ ] Click the map 0.5-1 m ahead in open floor: the robot plans (dashed line) and drives there
  - [ ] Return HOME from about 1 m away
  - [ ] Explore one room, staying next to it with STOP ready. Watch for: getting stuck without noticing
    (open-loop `/odom`), spinning in place too long, goals too close to walls
  - [x] Robot's real footprint (2026-10-17): 18 in (0.46 m) across with the arm and Pi drawer, so
    `robot_radius: 0.23` in `nav2.yaml` (inflation 0.45), mission `clearance_m` 0.25, web_ui `robot_radius_m`
    0.23. Not yet re-run: check Explore still fits through the room's doorway (needs > 0.5 m clear)
  - [ ] Measure where the lidar sits relative to the Roomba's center and set the `laser_tf` x/y in
    `lucia_lidar/launch/lidar_and_map.launch.py` (0, 0 today)
- [ ] Later: real encoder `/odom` (Phase 1, Step B) should make Nav2 noticeably more precise

**SWEEP mode** (last semester's bounce-around logic). Not started; may not be needed if Explore works
well. Team's call.
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
- [x] `mapping.launch.py`: control + IMU + EKF + lidar + slam_toolbox (2026-10-16). Same evening: + safety gate (`use_tof`) and Nav2 + mission (`use_nav`), not yet run
- [x] Web control page `lucia_web` (written 2026-10-16, not yet run on the Pi): status lights for lidar/IMU/wheels,
  Start/Stop mapping (runs `mapping.launch.py`, IMU on/off), live map with the robot's position, hold-to-drive pad
  and arrow keys (test tool; the page stops sending when released and `roomba_bridge` times out as a second layer),
  save map + download PNG. Plain `http.server`, no new packages. Parsing/validation and process control have
  pure-Python tests; the HTTP endpoints were smoke-tested with ROS stubbed out
- [x] `docker/compose.yml` (written, not yet run): container in the background, `restart: unless-stopped` so it starts
  at boot, runs `colcon build` then `web_ui`. Shares `/dev` with device-type rules, so an unplugged Pico or lidar at
  boot doesn't stop the container from starting
- [x] On the Pi (2026-10-16): `docker compose -f docker/compose.yml up -d`, page opened from the laptop, mapping started, driven and saved from the browser. Working
- [ ] Reboot test on the hotspot alone: Pi comes up on `lucia-control`, the container starts by itself, page loads at http://10.42.0.1:8080 (not separately confirmed yet)
- [x] Christian's changes to the web page (2026-10-16, tested against a simulated robot, not yet on the Pi):
  - Radar: live lidar around the robot, forward up, 1/2/4 m zoom, nearest-obstacle readout
  - Speed slider (5-30 cm/s): drive pad and Nav2
  - Autonomy card: Explore, Return HOME, STOP (also Space/Esc; floats on phones while driving itself),
    click-the-map go-to. The map shows frontiers, the current goal, HOME and Nav2's planned path.
    The drive pad is locked while the robot drives itself
  - Navigation light in the header
- [x] Web page v2 (2026-10-17, laptop-tested with scripted browser checks against the simulated robot, not yet on
  the Pi):
  - Layout: left column (about 1/5) with the radar at the top, live metrics under it, then the controls; the map
    fills the rest. Explore / HOME / STOP also in the header so they're always on screen
  - Metrics: heading + compass, wheel speed, ground speed, turn rate (IMU gyro), distance driven, time, distance
    from HOME, position, area mapped, frontiers, nearest obstacle, lidar/IMU/odom Hz, Pi CPU temp / CPU / memory
  - Map: wheel zoom, drag pan, Fit, Follow; robot trail and live lidar points drawn on it; robot at its real size
  - Logs drawer under the map: every node's `/rosout` log with level/node/text filters, launch output tab,
    Download as .txt
  - Keys fixed: they used to stop working once the speed slider or a checkbox had focus (and Space/Esc STOP did
    nothing then). Now they work anywhere except while typing in a text box, capitals included
  - Driving by hand during Explore/HOME stops the mission and takes over
- [ ] On the Pi: Pi-health metrics show numbers (container reads `/sys` thermal), logs drawer shows Nav2/mission
  lines, keys + takeover on the real robot

---

## Setup on the Pi (needed along the way)

- [x] The old web control panel no longer starts at boot. `lucia.service` ran `slam_avoid_server.py` on the same serial port and GPIO pins, which fought the ROS2 control program. Disabled on the Pi on 2026-09-18 with `sudo systemctl disable --now lucia`. To use the old panel again: `sudo systemctl start lucia`, and stop it before any ROS2 test. Tell the team.
- [x] The Pi's own WiFi (`lucia-control`) comes up after a reboot. Confirmed 2026-09-18. `pi-ap` was given priority 100 so it wins over saved networks like LiveLaughLove.
- [x] udev rules, so the Roomba and lidar always show up as `/dev/roomba` and `/dev/rplidar`. Already existed: confirmed 2026-10-16 (`/dev/rplidar` → `ttyUSB0`, `/dev/roomba` → `ttyUSB1`), so no `port:=` override is needed
- [x] Write a Dockerfile so the container can be rebuilt (`docker/Dockerfile`)
  - Includes the serial library control needs, the lidar and mapping packages, and the keyboard driving tool (`teleop_twist_keyboard`)
- [x] Build it on the Pi: `docker build -t lucia/ros2:latest docker/` (built fine, about 8 minutes). Needs internet, so do it on a network with internet, not the robot's own WiFi.
- [x] `update.sh` was hardcoded to pull the `testing` branch. Fixed 2026-10-16 to pull `dev`, the team's new shared integration branch.
- [x] `docs/setup/ros2-pi.md` rewritten (2026-10-16): running from the web page, what runs where, updating, network modes, udev rules, compose, IMU Pico, diagnostics, known issues
- [x] `colcon build` on the Pi fails on `lucia_vision` (Jetson/ZED package, needs CUDA) and then skips the rest. The Dockerfile now writes a colcon defaults file (skip `lucia_vision`, symlink-install), so plain `colcon build` works once the image is rebuilt

---

## Check on the robot (only you can do these)

- [x] Wheel-drop switches are still there
- [ ] Is the cliff sensor still there?

---

## Later (not started)

- (Frontier exploration moved up to Phase 4 on 2026-10-16. It runs on today's open-loop
  `/odom` + IMU heading; real encoders would make it more precise.)
- Camera and Jetson: start from scratch when the time comes
