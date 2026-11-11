# Project L.U.C.I.A — TODO

> Rebuilding the robot software as small pieces (ROS2), one piece at a time.
> Order: **Control → Lidar → Map → Safety stop → driving itself (frontier exploration; SWEEP) → one-command bringup.** Check items off as they're done.
> Work on `195B-Christian`, merge into `dev` and push both. The Pi pulls `dev`.

---

## The goal

**The robot maps a room on its own.** Turn it on, it drives itself around (Explore:
frontier exploration), builds the map as it goes, comes back HOME, and you save the map at the end. Driving by keyboard is only
a test tool for checking one piece at a time; it is not how the robot is meant to run.

**Next goal (2026-10-22): follow me.** Explore stays the main mode; if the Jetson's camera sees the follow
sign, the robot follows that person until they give the cancel sign, then goes back to Explore (Phase 6). First step: the
camera's view on the web page.

## Right now

**The robot maps a room by itself** (as of 2026-10-17): power on, join `lucia-control`, open
http://10.42.0.1:8080, Start mapping, **Explore**; it maps what it can reach and comes back HOME
(how-to: `docs/setup/ros2-pi.md`; maps in `media/maps/`). Working on the real robot: wheels
(open-loop `/odom`), lidar, IMU heading, slam_toolbox mapping, Nav2 + frontier exploration, the
web page (v2: big map, radar + metrics, logs drawer, keys).

History: 2026-10-16 first maps by keyboard, then IMU fused in, then the web page. SWEEP was
planned first; frontier exploration was moved ahead of it that evening at Christian's request.
2026-10-17: Explore fixed and working, page v2, README and network docs checked against the Pi.

On the robot 2026-10-23: gradual speed changes and carrying on after driving by hand work; tight
spaces slow it down (slowest speed then raised to 10 cm/s).

2026-11-10: Noel's code merged (`pico2`, `tof_pico`, `new-cable-rts-wake`; Ling's IMU address fix
came with it). **One Pico 2 for the IMU and the ToF sensors** (decided 2026-11-10): `lucia_tof`'s
`tof_node` reads it and publishes `/imu/data` too, so it replaced `imu_bridge` in `mapping.launch.py`;
it reads Noel's `src/pico2/main.py` and Ling's `stream_main.py` alike. "ToF safety stop" is a switch
in the page's settings (off). Reconnect Roomba wakes the Roomba with either cable.

**Not yet tried on the robot**: from 2026-10-17, clean container stop (`compose.yml`, needs `up -d`
once), Reconnect Roomba, tunes and MIDI import (the horn played), the 0.23 m footprint, the stuck
check; from 2026-10-23, the new page layout and the settings menu at Start mapping (Phase 5), and the
tight-space fixes (ramp keeps the curve, Nav2 look-ahead scales with speed; Phase 4); from
2026-11-10, `tof_node` as the IMU reader, the ToF safety stop switch, and the wake pulse for the
new cable. Waiting on hardware: the ToF safety stop itself (sensors not mounted).
Next: those first tries (see Phases 1, 2, 4, 5 and "Setup on the Pi"), then the ToF stop.

---

## Ground rules (already decided)

- **One program per device.** Only the control program talks to the Roomba, and only the lidar program talks to the lidar. Nothing else opens those USB ports.
- **The Roomba is only a drive base.** Its brushes and vacuum are removed (2026-10-17); it maps, it never cleans. No cleaning features.
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
- [x] Gradual speed changes, to spare the hardware on board (2026-10-17, not yet tried on the robot):
  `roomba_bridge` ramps toward each command instead of jumping (`motion.ramp_toward`, tested).
  0 to 15 cm/s in 0.5 s, 15 cm/s to a stop in 0.3 s (about 2 cm); 30 cm/s takes 1 s up, 0.5 s
  (about 8 cm) down. Every driver and every stop goes through it, STOP included
- [x] On the robot: the speed ramp works (Christian, 2026-10-23)
- [x] Top speed in the open, slower in tight spaces (2026-10-17, laptop-tested, not yet on the robot):
  the speed slider and Nav2 start at 0.30 m/s; `safety_gate` caps forward speed from the lidar (room
  around the front half, free run ahead): 0.30 in the open, ~0.21 nearing a doorway, ~0.13 in a 0.7 m
  doorway, 0.10 at the closest. Never stops; backing up and turning in place untouched; no lidar = 0.10
- [x] On the robot (2026-10-23): it slows down in tight spaces, but too much. The slowest speed
  (`safety_gate` `min_speed_mps`) went from 0.06 to 0.10 m/s the same day (doorway ~0.10 → ~0.13)
- [ ] On the robot: the 0.10 minimum feels right in tight spaces; the map is still clean at 30 cm/s

**Step B — Position tracking and battery**
- [x] Checked 2026-09-18: **the Roomba's replies don't reach the Pi.** Battery voltage reads 0, encoders read 0. Driving still works, so the Pi-to-Roomba direction is fine.
- [x] Rechecked 2026-10-16, counting bytes instead of decoding them: **zero bytes** come back for battery voltage (packet 22) or either encoder (43/44), even while turning a wheel by hand. The old "reads 0" was really "no reply at all" (the driver turns an empty read into 0). The encoders themselves are untested. A beep command played in the same session, so the Roomba was awake and receiving: only the Roomba-to-Pi direction is broken
- [x] **Fixed 2026-11-10 by Noel's new cable: the Roomba's replies reach the Pi** (16224 mV, 2522 of 2697 mAh
  in the reply check). Was: fix the reply path (Roomba TX to the adapter's RX, FTDI FT232R). Loopback test: unplug from the Roomba, jumper pins 3 and 4 on the cable's mini-DIN plug, send bytes, see if they echo. Echo → the fault is at the Roomba end (reseat); no echo → the adapter/cable (replace, e.g. a ready-made Create 2 serial cable)
- [ ] Noel's new cable (BG046PKOA, 2026-10-25): its BRC (wake) wire is on RTS; an RTS pulse woke the Roomba,
  DTR did nothing. `RoombaOI.wake()` pulses RTS and DTR together since 2026-11-10, so Reconnect Roomba works
  with either cable. On the robot by 2026-11-10 (FTDI FT232R, serial `BG046PKO`; "BG046PKOA" is how Windows
  shows it); its `/dev/roomba` udev line was added on the Pi that day. Replies come back with it (above).
  Still to try: Reconnect Roomba (should beep)
- [x] Added position tracking (`/odom` + the `odom`→`base_link` transform) to the control program (2026-10-09) — **open-loop**, estimated from commanded wheel speed, not a sensor reading, since the cable above isn't fixed yet. Will drift more than real encoder feedback. See `odometry.py` and `docs/changelogs/2026-10-09.md`.
- [x] Tried `/odom` on the real robot: position and heading both tracked driving + turning correctly.
- [ ] **Now possible (the cable works):** measure wheel size and encoder counts per turn (drive a known distance, read the raw counts), then switch `/odom` to use real encoder readings instead of commanded speed. `odometry.py`'s math doesn't change either way — only what feeds it does.
- [x] Battery (2026-11-10): `roomba_bridge` reads charging state, voltage, current, temperature, charge and capacity in
  one Query List request every 2 s (`battery.py`, 0.1 s read timeout so driving isn't held up) and publishes
  `/battery` (`sensor_msgs/BatteryState`); the page shows it in the header and Telemetry. Tested here; not yet on
  the robot
- [x] Recover after the Roomba is power-cycled (2026-10-17, not yet tried on the robot): a **Reconnect Roomba** button on
  the page calls `roomba_bridge`'s `~/reconnect` service, which reopens the port, wakes the Roomba (BRC pulse), sends
  Start + Safe and beeps; mapping keeps running. It's manual because nothing comes back from the Roomba, so
  `roomba_bridge` can't tell it was power-cycled (and sending Start every second would drop it to Passive each time).
  Also: a serial error (adapter unplugged) is now logged instead of crashing `roomba_bridge`
- [ ] Automatic recovery, now that the reply cable works (read the OI mode, packet 35, and re-send Start only when needed)

---

## Phase 2 — Lidar (builds the map)

Folder: `src/ros2/lucia_lidar/`
- [x] Lidar settings file (`config/rplidar.yaml`) - port, speed, from how it actually ran last semester
- [x] Map-building settings file (`config/slam_toolbox.yaml`) - package defaults plus this lidar's real range limits
- [x] Tried on the real lidar (2026-10-09): the executable name is `rplidar_composition`, not `rplidar_node` (though the *node* it creates is still named `rplidar_node`, which is what the config file's settings apply to - that part was already right). Hit "Cannot start scan" at first - fixed by explicitly setting `scan_mode: Standard`, already in the config file; the ROS2 driver's own default scan mode isn't compatible with this lidar's firmware (Hardware Rev 5, Firmware 1.25). `/scan` now publishes at ~11.7 Hz.
- [x] Lidar scans show up on `/scan`
- [x] Launch file (`launch/lidar_and_map.launch.py`) starts the lidar + `slam_toolbox` together; runs on the Pi (2026-10-16, see the bench test and the room maps below)
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
- [x] `lucia_imu` package: `imu_bridge` node reads the Pico and publishes `/imu/data` (parser has pure-Python tests). Runs on the Pi (2026-10-16)
- [x] Pico streams `I,` lines when plugged into the Pi (`/dev/ttyACM0`), about 50/s (2026-10-16). Loaded from the Pi with `mpremote` (installed via `pipx`): `mpremote connect /dev/ttyACM0 fs cp src/imu/stream_main.py :main.py`. That Pico previously held a ToF `main.py`; it's backed up on the Pi at `~/tof_main_backup.py`. The driver files `bno08x.py` / `i2c.py` live in the Pico's root (Ling's copies, import already fixed). Unused duplicates also got copied to the Pico's `lib/`; delete them next time the Pico is in VS Code (`mpremote` couldn't interrupt the running stream to do it)
- [x] udev rule `/dev/imu` added to `/etc/udev/rules.d/99-lucia.rules` (2026-10-16), matching the Pico's serial `fd92d5f577b9417b` (the ToF Pico has the same vendor/product ID). Confirm `ls -la /dev/imu` shows the link, and use `--device=/dev/imu` in the `docker run` command
- [x] `imu_bridge` on the Pi (2026-10-16): 50 readings/s, 0 bad lines. A quarter turn left by hand read +94.7° (positive = left, the ROS convention) and came back to about +1-2° when turned back
- [ ] Mount the IMU flat and fixed on the robot; note which way its axes point (needs a `base_link`→`imu_link` transform, like the lidar's)
- [x] The IMU on the robot's Pico (since 2026-11-10) is **upside down** and can't be turned over: gravity read
  −9.87 m/s² on its z axis. The EKF takes heading and turn rate only from the IMU, so the turn rate would have
  come out backwards. `tof_node` now turns the readings over (2026-11-10): it tells from gravity with Noel's
  `main.py`, from `imu_upside_down` (true in `lucia_tof/config/tof.yaml`) with `stream_main.py`. Checked with
  the captured reading and a simulated left turn; not yet on the robot
- [ ] On the robot: turn it left by hand during mapping; `angular_velocity.z` in `/imu/data` should be positive
- [x] Fusion (runs on the Pi since 2026-10-16, `room2.png`): `lucia_bringup/config/ekf.yaml` takes forward speed from `/odom` and heading + turn rate from `/imu/data`; `roomba_bridge` gained `publish_tf` (off when the EKF runs) and now fills in `/odom`'s twist with rough variances. `ros2 launch lucia_bringup mapping.launch.py` starts roomba_bridge + imu_bridge + EKF + lidar + slam_toolbox (`use_imu:=false` = the first-map setup)
- [x] Image rebuilt on the Pi (about 6.5 min); plain `colcon build` skips `lucia_vision`. `mapping.launch.py` starts cleanly: `imu_bridge` on `/dev/imu`, `roomba_bridge` hands `odom`→`base_link` to the EKF, slam_toolbox registers the lidar, no warnings (2026-10-16)
- [x] Mapped the same room again with the IMU (`room2.png`, 2026-10-16): walls are single thin lines with square corners and much less speckle; the doubled wall from `room1.png` is gone. One run each with different starting headings, so not a controlled comparison, but the heading fix is clearly visible
- [ ] Skip the first, partial line after opening the port (a harmless `pico: 01477,...` log line; `tof_node` does it too now)
- [x] **One Pico for the IMU and the ToF sensors** (decided 2026-11-10). Only one program may open it, so
  `lucia_tof`'s `tof_node` now publishes `/imu/data` (same frame, variances and topic as `imu_bridge`) and
  replaced `imu_bridge` in `mapping.launch.py`. It reads Noel's `src/pico2/main.py` packets (100 Hz, with
  acceleration) and Ling's `stream_main.py` lines alike, so nothing needs switching when the Pico's code
  changes. Noel's `main.py` now also finds the IMU at 0x4A (the robot's; it only tried 0x4B) and keeps
  streaming the IMU without the ToF sensors (it used to stop at a missing sensor). Tested here with a
  simulated Pico and stub ROS; not yet on the robot
- [ ] On the robot: Start mapping, IMU light green, `ros2 topic hz /imu/data` about 50/s (100/s once
  `src/pico2/main.py` is loaded)

---

## Phase 3 — Safety stop (before anything drives itself)

Folder: `src/ros2/lucia_bringup/`

Decided 2026-10-16: built on the **ToF distance sensors** the team is implementing
(`src/tof_pico/`, VL53L5CX on a Pico), not the lidar. Waits until those are ready.
- [x] ToF Pico → Pi → ROS2: Noel's `lucia_tof` package, merged 2026-11-10 (from `pico2`, which includes
  `tof_pico`). Three sensors at 4x4 and 60 Hz each: a distance grid and nearest reading per sensor, and
  `tof/near` (Bool: true = something within 300 mm, or a sensor silent). The ToF sensors share the IMU's
  Pico (`/dev/imu`, matched by serial), so its old any-Pico udev rule (`/dev/lucia_tof`) is gone
- [x] "Stop if something's too close" node (2026-10-16; runs in pass-through on the Pi, the blocking path not yet tried): `lucia_bringup` `safety_gate`.
  Everything that drives (page, Nav2, mission) publishes `/cmd_vel`; the gate passes it to `/cmd_vel_safe`,
  which `roomba_bridge` listens to in `mapping.launch.py`. With `use_tof:=true` it blocks forward motion
  while `tof/near` is true or silent; turning and backing up still pass so Nav2 can turn away. Default
  `use_tof:=false` = pass-through. Rules in `safety_logic.py` (tested)
- [x] `tof_node` in `mapping.launch.py` (2026-11-10): starts when `use_imu` or `use_tof` is on. "ToF safety
  stop" in the page's settings sets `use_tof` (off by default)
- [x] ToF sensors checked on the robot (2026-11-10, Noel's Pico with his `main.py`, read straight from `/dev/imu`
  with mapping stopped): all three at 60 frames/s, IMU 88/s, no damaged packets. Hand test: each sensor alone
  dropped to 16-26 cm and flagged NEAR, so left / middle / right are in the right order, and they're the robot's
  own left / middle / right (Christian; Noel's comment said "as seen standing in front", corrected). The cones
  overlap: a hand between two shows in both. The bottom rows of middle and right see the floor at 53-66 cm,
  beyond the 30 cm NEAR line
- [x] Glitch: the middle sensor sent "0 mm NEAR" twice in 10 s with nothing in front of it (2026-11-10). `tof_node`
  now decides near itself (`lucia_tof/near_filter.py`): drops readings under 20 mm unless half the zones agree
  (covered), and a sensor's answer changes only after 3 frames in a row (about 50 ms). Tested here; not yet on
  the robot. `ros2 run lucia_tof tof_check` replaces the pasted check script and counts glitches
- [x] Where the sensors sit (Christian, 2026-11-10, bracket drawing): 25.4 cm up, level; middle straight ahead,
  7.6 cm behind the Roomba's front; left / right on the bracket's side faces at 72.5° outward, 1.6 cm to the
  side and 1.2 cm behind the middle one (`lucia_tof/zones.py`). The bracket can't be tilted
- [x] The stop only counts what's **in the robot's path** (2026-11-10): within its width, no further than the
  page's new **"ToF stop distance"** (default 15 cm past the Roomba's front; it was 30 cm from the sensors, about
  22 cm past the front). Before, the side sensors (almost sideways) would have blocked forward driving along
  any wall. Tested with the robot's recorded grids and a simulated wall; not yet on the robot
- [ ] The stop itself (ramped like every stop: about 2 cm extra at 15 cm/s, 8 cm at 30 cm/s): "ToF safety
  stop" on, drive toward a wall by keyboard; also along a wall (it must not stop)
- [x] Checked on the robot (2026-11-10, `tof_check`): column 0 is the sensor's left, row 0 the top (no flips);
  distances straight out from the face; each sensor sees the floor only in its bottom row (left ~70 cm,
  middle ~63, right ~55: the bracket isn't perfectly level)
- [x] Low obstacles on Nav2's map (2026-11-10): `tof_node` publishes obstacle and free-space points per sensor
  (`obstacles.py`: the floor by each sensor's measured bottom-row distance, obstacles up to 1 m, nothing higher
  than the robot) and where the sensors sit (`/tf_static`). Nav2's `tof_layer` in both costmaps marks them and
  clears only from 0.7 m out, so a low object stays marked after it drops out of view. Page setting **"Low
  obstacles on the map"**, off by default. Tested here (geometry, robot's grids, stub ROS); not yet on the robot
- [ ] On the robot: Nav2 starts with `tof_layer` (Logs: no costmap errors); with it on, the robot still drives
  through open space (no ghost obstacles from the floor); a shoe or box lower than the lidar in its way gets
  planned around in Explore / goto
- [ ] Noel's known gaps (`lucia_tof/README.md`): image orientation (`flip_h` / `flip_v`) and radial vs.
  straight-ahead distance not checked; no `base_link`→`tof_*_link` transforms yet
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
- [ ] **Tight spaces stutter, then it figures it out** (Christian, 2026-10-23). Changed the same day, not yet
  tried: the speed ramp now changes forward speed and turning together (eased separately, a stop on a curve ran
  ~4 cm straight past it, toward the door frame), and Nav2 looks ahead 1.5 s of travel (0.25 m when slow in a
  tight space, was a fixed 0.4 m: less corner cutting, and its collision check stops flagging frames it would
  pass). If it still stutters: the Logs (Warnings and up) from one tight spot would show whether Nav2's recovery
  steps are running (clear the map, spin 90°, wait 5 s, back up 30 cm); if so, a faster recovery (no 5 s wait)
  is the next change
- [ ] Explore stutters a bit over uneven ground (seen 2026-10-17), smooth elsewhere. Not explained
  yet. Either the Roomba stops itself (Safe mode: a cliff sensor or wheel drop on a bump drops it to
  Passive until the 1 s heartbeat restarts it; the Pi can't see that) or Nav2's commands jump (lidar
  tilt, position corrections; the new ramp smooths those). Test: hold W by hand over the same patch
  at the same speed. Still stutters = the Roomba side

**Stuck check** (2026-10-17, laptop-tested, not yet tried on the robot). The wheels can't report, and
Nav2's progress checker watches the same open-loop odom, so a stuck robot went unnoticed.
- [x] `lucia_nav/motion_check.py`: twice a second, the mission node compares what the wheels were told
  (`/odom`) with what happened: IMU heading for turning, two lidar scans about 1 s apart for driving
  (which share of the commanded distance lines them up best). "Can't tell" in a bare hallway, never
  "stuck". 3 "not moving" in a row while driven = stuck, about 2 s after it stops at 15 cm/s.
  Simulated room/hallway tests, ~6 ms per check
- [x] Recovery in `mission_logic.py`: "uh-oh" tune, back up 15 cm if the lidar sees room behind,
  then Explore skips that frontier, HOME / go-to try again. 3 in a row without reaching a goal: stop,
  play a tune for help. Driving by hand it only says so. Page: **Really moving?** metric, red **!** on
  the map. `stuck_check:=false` on the mission node turns it off
- [x] On the robot: Explore ran into something low the lidar can't see and backed off on its own (Christian,
  reported 2026-11-10; height unknown). Still to watch: no false "stuck" in a normal Explore or a hallway
- [ ] Later: tell Nav2 where it got stuck (a small obstacle layer of stuck spots, which the lidar can't
  clear), so plans route around them; today only the frontier is skipped

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
- [x] Page layout v3 (2026-11-10, Christian's list): three columns: radar + metrics + controls | map + drawer |
  Jetson camera + ToF sensors + telemetry. ⤢ swaps the radar, camera or ToF card into the middle (remembered per
  browser). The radar shows the ToF sensors (fans, low obstacles orange, floor grey, the stop zone); the ToF card
  shows the three 4x4 grids; Telemetry: Roomba battery, graphs (battery 10 min, current, speed, Pi temperature),
  message rates, Pi. Header: ToF and Jetson lights, battery. Checked here in a browser against fake robot data
  (wide, laptop, dark, phone); not yet on the robot
- [x] `mapping.launch.py`: control + IMU + EKF + lidar + slam_toolbox (2026-10-16). Same evening: + safety gate (`use_tof`) and Nav2 + mission (`use_nav`); Nav2 runs on the robot since 2026-10-17, `use_tof:=true` not yet tried
- [x] Web control page `lucia_web` (2026-10-16, working on the Pi the same day, see below): status lights for lidar/IMU/wheels,
  Start/Stop mapping (runs `mapping.launch.py`, IMU on/off), live map with the robot's position, hold-to-drive pad
  and arrow keys (test tool; the page stops sending when released and `roomba_bridge` times out as a second layer),
  save map + download PNG. Plain `http.server`, no new packages. Parsing/validation and process control have
  pure-Python tests; the HTTP endpoints were smoke-tested with ROS stubbed out
- [x] `docker/compose.yml` (2026-10-16, running on the Pi since): container in the background, `restart: unless-stopped` so it starts
  at boot, runs `colcon build` then `web_ui`. Shares `/dev` with device-type rules, so an unplugged Pico or lidar at
  boot doesn't stop the container from starting
- [x] On the Pi (2026-10-16): `docker compose -f docker/compose.yml up -d`, page opened from the laptop, mapping started, driven and saved from the browser. Working
- [ ] Reboot test on the hotspot alone: Pi comes up on `lucia-control`, the container starts by itself, page loads at http://10.42.0.1:8080 (not separately confirmed yet)
- [x] Christian's changes to the web page (2026-10-16; used on the robot 2026-10-17):
  - Radar: live lidar around the robot, forward up, 1/2/4 m zoom, nearest-obstacle readout
  - Speed slider (5-30 cm/s): drive pad and Nav2
  - Autonomy card: Explore, Return HOME, STOP (also Space/Esc; floats on phones while driving itself),
    click-the-map go-to. The map shows frontiers, the current goal, HOME and Nav2's planned path.
    The drive pad was locked while the robot drives itself (replaced 2026-10-17: driving by hand takes over)
  - Navigation light in the header
- [x] Web page v2 (2026-10-17, scripted browser checks against the simulated robot; on the Pi since 2026-10-17, the
  checks below still open):
  - Layout: left column (about 1/5) with the radar at the top, live metrics under it, then the controls; the map
    fills the rest. Explore / HOME / STOP also in the header so they're always on screen
  - Metrics: heading + compass, wheel speed, ground speed, turn rate (IMU gyro), distance driven, time, distance
    from HOME, position, area mapped, frontiers, nearest obstacle, lidar/IMU/odom Hz, Pi CPU temp / CPU / memory
  - Map: wheel zoom, drag pan, Fit, Follow; robot trail and live lidar points drawn on it; robot at its real size
  - Logs drawer under the map: every node's `/rosout` log with level/node/text filters, launch output tab,
    Download as .txt
  - Keys fixed: they used to stop working once the speed slider or a checkbox had focus (and Space/Esc STOP did
    nothing then). Now they work anywhere except while typing in a text box, capitals included
  - Driving by hand during Explore/HOME stops the mission and takes over (changed later the same day:
    it pauses, and carries on 3 s after the last key press)
- [x] Driving by hand pauses Explore / HOME / go-to instead of stopping it (2026-10-17, browser-checked
  against the stub, not yet on the robot): mission `pause` / `resume`; `web_ui` resumes `resume_after_s`
  (3 s) after the last key press, with a countdown on the page; STOP cancels. Explore keeps its failed
  frontiers and stuck spots. Letting go of a key while paused stops the wheels at once
- [x] On the robot: take over during Explore, let go, it carries on after about 3 s (works, Christian, 2026-10-23)
- [x] Horn and tunes (2026-10-17; the horn played on the robot, tunes and MIDI import not yet): Horn button + H key; built-in tunes (Horn,
  Charge!, La Cucaracha, Mass Destruction); type/edit notes; preview in the browser; import a MIDI file's melody
  (pick the part, start time, octave); saved tunes in `~/maps/tunes.json` on the Pi. Played by `roomba_bridge`'s
  `~/song` topic: up to 64 notes as 4 song slots back to back. Persona / Pokémon tunes: import fan MIDIs (not
  committed to the repo)
- [ ] Later: tunes longer than 64 notes, by refilling the Roomba's song slots while it plays (untested whether
  the Roomba allows that; a short gap every 16 notes)
- [x] Trail drawn brighter (2026-10-23, Christian's request): violet (`--trail`), thicker, with a white edge so it
  shows on floor, unknown and walls alike (was thin, semi-transparent grey). Checked in a browser, both themes
- [x] **Page layout** (Christian's list, 2026-10-23; browser-checked against a stand-in robot, not yet on the Pi):
  the left column doesn't scroll (checked at 1366x768 and 1920x1080): the radar, pinned at the top and taking
  the height that's left, with the drive arrows on its edges; six key metrics (the full set moved to a Metrics tab
  in the drawer, now **Logs & metrics**); Mapping (Start / Stop, Reconnect, Top speed, status). **Horn (H) ▾** in the
  top bar opens the tunes: Horn, Charge!, saved ones, and MIDI import (the notes editor, La Cucaracha and Mass
  Destruction went). **Save map ▾** in the map toolbar, with the saved maps to download. On a phone the column goes
  under the map and scrolls, as before
- [x] **Settings menu** (2026-10-23, same checks): Start mapping… opens 20 settings in 6 groups (driving, robot,
  tight spaces, Explore, stuck check, driving by hand), each with help and its default; Reset to defaults; changed
  ones are marked. `lucia_web/settings.py` checks them (ranges, and that they agree), saves them in
  `~/maps/settings.json`, and writes a ROS 2 parameters file that `mapping.launch.py settings_file:=` hands to
  roomba_bridge, safety_gate, the mission node and Nav2 (costmap robot size), each value typed as its node declared
  it. `web_ui` applies the speed limit (slider), robot size (drawing) and resume delay itself. The tight-space
  distances are now `safety_gate` parameters. Not in it (yet): slam_toolbox's settings, the stuck check's own
  thresholds, keyboard turn rate
- [ ] On the Pi: Start mapping with the settings dialog; the nodes start (Logs → Launch output shows
  `settings_file:=`); changing the speed limit or the slowest tight-space speed shows on the robot
- [ ] On the Pi: Pi-health metrics show numbers (container reads `/sys` thermal), logs drawer shows Nav2/mission
  lines, keys + takeover on the real robot

---

## Phase 6 — Follow me (Jetson + ZED 2, hand signs)

**End goal** (Christian, 2026-10-22; on the agenda, not started): a person gives the robot a hand sign, the
Jetson sees it, and the robot follows that person; another sign stops it. The Nano side builds on Ethan's work
(`docs/jetson_nano_zed_setup.md`: ZED 2, SDK 3.8.2 and Python 3.6 on the Nano, object detection working
2026-10-16); the Pi side builds on the mission node.
**Explore stays the main thing** (Christian, 2026-10-23): the robot explores on its own; if the Jetson sees the
follow sign it switches to following that person, until the cancel sign, then goes back to Explore. Future work, after the camera view below.

**2026-10-23: the link works.** Ethan's `docs/jetson-deployment/follow_sender.py` (body tracking, arm signs,
UDP JSON) sent to `udp_listen.py` on the Pi over home WiFi (Jetson → Pi port 5005), ~10 Hz, stable
person IDs, sensible positions. Signs not yet seen firing (`"sign"` was always null in that run).

**Camera view on the page** (Christian, 2026-10-23): see what the Jetson camera and the model see, on the
same web page.
- [x] Jetson (Ethan, 2026-10-23): MJPEG stream on port 8080 at `/stream.mjpg` (or `/`): `zed_stream.py` plain,
  `follow_sender.py --stream` with the skeletons and the sign drawn on. Measured: 1280×720, about 4 frames/s,
  about 42 KB each (165 KB/s). Smaller frames (640×360) would cut that by about 4 if the WiFi struggles
- [x] Pi: `web_ui` relays it at `/api/camera.mjpg`, so the browser only talks to the Pi (2026-10-23). The
  Jetson's address is typed in the camera panel and saved on the Pi (`~/maps/camera.json`), not in git. The
  relay runs only while the panel is open; it drops a camera that's quiet for 5 s and the page retries.
  Checked with the real Jetson stream through `web_ui` on the laptop (picture, live / no-picture states,
  wrong address, recovery); not yet on the Pi
- [x] Page: **Camera** in the map toolbar opens the camera over the map's corner (bigger / smaller, address,
  close). Overlay on/off = which script runs on the Jetson
- [x] Page (2026-11-10): `web_ui` listens for the Jetson's UDP detections (port 5005, `logic.JetsonFeed`): a Jetson
  light in the header, and under the camera how many people, how far, and the last sign. The camera has its own
  card in the right column (⏸ pauses it, ⤢ makes it big)
- [ ] People as dots on the radar (needs which way the ZED's x / y / z point, and where the ZED sits on the robot)
- [x] The Jetson link (2026-11-10): Ethernet cable Pi ↔ Nano. Pi `192.168.1.1` (static), its `dnsmasq` gives the
  Nano's MAC (`48:b0:2d:5b:0e:c3`) `192.168.1.2` and nothing else; `follow_sender.service` sends to
  `--host 192.168.1.1`. Lease and ping work. Setup and checks: `docs/setup/ros2-pi.md`, "The Jetson link"
- [x] The camera stream over the link (2026-11-10): `curl` from the Pi gives 200, about 10 frames a second; on the
  page: Camera → ⚙ → `192.168.1.2:8080`. First the ZED wasn't detected (replugged, Jetson restarted), then the Nano
  switched itself off twice with the camera running: the adapter on hand is too weak. It stays on in **5 W mode**
  (`sudo nvpmodel -m 1` on the Jetson), with slower detection
- [ ] A 5 V 4 A adapter for the Nano on the bench (ask Ethan which one he used 2026-10-23), then `sudo nvpmodel -m 0`
  for full power. Is the stream smooth on the page over the hotspot? Even in 5 W mode it isn't enough
  (2026-11-11): the Nano restarted by itself, `dmesg` showed `soctherm: OC ALARM` (its over-current alarm) and the
  ZED failing to connect (`error -71`, "Cannot enable"), so the camera was gone. Stopgap: a powered USB 3 hub for
  the ZED
- [ ] Sign test: does a raised hand at 1.5-2 m stay in view and fire `follow`?

Flow: ZED 2 → Nano (finds people, reads the sign) → network → Pi node (`/person`, `/gesture`) → mission `follow`
→ `safety_gate` → wheels

Decide first:
- [ ] **Which signs.** The ZED's body tracking (same SDK, `HUMAN_BODY_*` models) gives each person a skeleton
  (shoulders, elbows, wrists, no fingers), a 3D position and an ID that stays with them, which is everything
  following needs. It can read arm signals, e.g. one hand above the head for 1 s = follow me, both hands up = stop.
  Finger signs (thumbs up, open palm, peace) need a second, hand-pose model on the Nano (e.g. NVIDIA's
  `trt_pose_hand`, made for the Nano, needs PyTorch): more work and more load. Suggested: arm signals first,
  finger signs later if wanted
- [ ] **How the Nano talks to the Pi.** The Nano's Ubuntu 18.04 (JetPack 4.6) can't run ROS2 directly.
  `lucia_vision` (`zed_publisher.cpp`, May 2026) runs ROS2 Humble in a dustynv container, but the Pi runs Jazzy,
  and two ROS2 versions talking to each other isn't officially supported. Suggested: Ethan's Python script sends a
  small JSON message over UDP about 10 times a second (person ID, position in meters, gesture), and a node on the Pi
  turns it into ROS2 topics. The Nano stays plain Python and the Pi side can be tested on the laptop
- [x] **The link.** Ethernet cable Nano ↔ Pi, the Pi hands the Nano its address (2026-11-10, see above)

Nano side (Ethan's area):
- [ ] Body tracking runs on the Nano fast enough (aim for 10 Hz or more). Check whether SDK 3.8.2 can run it
  together with the multi-class detector or only one model at a time
- [ ] Sign check: the sign has to be held about 1 s, so a passing wave doesn't count; reports which person (ID) gave it
- [ ] Sender: person ID, x/y/z, confidence, gesture, time → UDP to the Pi
- [ ] The Nano's script in the repo (today it's only written out in the setup doc)
- [ ] Power on the robot: 5 V 4 A for the Nano (it shuts off with the camera on a weaker supply), e.g. a buck
  converter from the Roomba battery rated for that
- [ ] Camera mount: height and tilt so a raised hand stays in view 1-2 m away (the robot is low, so tilt up).
  Measure its offset from the robot's center for the Pi side

Pi side:
- [ ] Receiver node (in `lucia_nav` or its own package; not in `lucia_vision`, which is the Jetson build the Pi
  skips): checks each message, converts camera position → robot frame (`base_link`), publishes `/person` and
  `/gesture`, drops anything older than about 0.3 s
- [ ] Mission `follow` mode: on the follow sign, lock onto that person's ID; turn to face them and stay about 1 m
  away, with the usual speed caps, through `safety_gate`. Stop sign, STOP on the page (Space/Esc), or the person
  out of sight for about 1 s → stop and wait. Driving by hand pauses it like Explore. Simplest first: steer straight
  at the person; if things in between get in the way, Nav2's follow-a-moving-point setup (keeps a distance, plans
  around obstacles) is the upgrade
- [ ] Page: follow state (waiting for a sign / following #ID / lost), the person shown on the radar and map
- [ ] Bonus: with mapping running, walking the robot around by following builds the map too
- [ ] Later, if wanted: the camera's video on the page (compressed stream from the Nano)

---

## Setup on the Pi (needed along the way)

- [x] The old web control panel no longer starts at boot. `lucia.service` ran `slam_avoid_server.py` on the same serial port and GPIO pins, which fought the ROS2 control program. Disabled on the Pi on 2026-09-18 with `sudo systemctl disable --now lucia`. To use the old panel again: `sudo systemctl start lucia`, and stop it before any ROS2 test. Tell the team.
- [x] The Pi's own WiFi (`lucia-control`) comes up after a reboot. Confirmed 2026-09-18. `pi-ap` was given priority 100 so it wins over saved networks like LiveLaughLove.
- [x] udev rules, so the Roomba and lidar always show up as `/dev/roomba` and `/dev/rplidar`. Already existed: confirmed 2026-10-16 (`/dev/rplidar` → `ttyUSB0`, `/dev/roomba` → `ttyUSB1`), so no `port:=` override is needed
- [x] Write a Dockerfile so the container can be rebuilt (`docker/Dockerfile`)
  - Includes the serial library control needs, the lidar and mapping packages, and the keyboard driving tool (`teleop_twist_keyboard`)
- [x] Build it on the Pi: `docker build -t lucia/ros2:latest docker/` (built fine, about 8 minutes). Needs internet, so do it on a network with internet, not the robot's own WiFi.
- [x] `update.sh` was hardcoded to pull the `testing` branch. Fixed 2026-10-16 to pull `dev`, the team's new shared integration branch.
  Removed 2026-10-17, with `docs/Updating the Repo on the Pi.md`: it joined a `lucia` WiFi that doesn't exist and ran
  `git clean -fd` (deletes saved maps). Updating is `git pull` + compose restart, see `docs/setup/ros2-pi.md`
- [x] Network settings checked on the Pi (2026-10-17): `pi-ap` autoconnect priority 100 (saved WiFi: 0), AP mode,
  10.42.0.1/24, channel 6, country US; `dnsmasq` serves 10.42.0.10-100; docker, dnsmasq, NetworkManager and
  avahi-daemon enabled at boot; container `restart: unless-stopped`. The setup doc was missing the priority step (added)
- [x] Container stops cleanly (written 2026-10-17, not yet tried): `ros2 run` was the container's main process and
  ignored SIGTERM, so every restart/shutdown waited 30 s and force-killed everything (the Roomba could keep driving).
  `compose.yml` now has `init: true` and starts `web_ui` directly. On the Pi: `docker compose -f docker/compose.yml up -d`
  once (restart keeps old settings), then `time docker compose -f docker/compose.yml restart` should take seconds
- [x] 2026-11-10 update on the Pi: pulled `dev` (daa0e30), container rebuilt with `lucia_tof`
  (`ros2 pkg executables lucia_tof` → `tof_node`). Added udev lines for the new Roomba cable (`BG046PKO`) and
  Noel's Pico (`72919cfe7798b116`, `/dev/imu`); the old lines stay. That Pico already runs Noel's `main.py`
  (ToF frames and IMU packets seen on the port)
- [ ] Pull the upside-down IMU fix (after daa0e30) on the Pi and restart the container
- [ ] Try `ssh lucia@lucia.local` from the laptop on a shared WiFi
- [ ] Try the Pi 5 power button for a clean shutdown (two presses on the desktop OS)
- [ ] Optional: let NetworkManager run the hotspot's address server (`ipv4.method shared`) instead of the separate
  `dnsmasq`; switching WiFi would then be one command each way, with no dnsmasq to stop and start
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
- Camera and Jetson: now Phase 6 (follow me with hand signs, 2026-10-22)
