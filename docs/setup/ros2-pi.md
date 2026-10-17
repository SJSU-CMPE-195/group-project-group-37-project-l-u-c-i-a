# ROS2 on the Raspberry Pi

How the robot's software runs on the Pi 5, how to use it day to day, and how to
update and debug it. Current as of 2026-10-17. For what changed when, see
`docs/changelogs/`. For the task list, see `TODO.md`.

---

## Running the robot (the normal way)

1. **Turn on the Roomba and the Pi.** At boot the Pi brings up its own WiFi
   hotspot and starts the software container, which serves the control page.
   Give it a minute or so.
2. **Connect your laptop or phone to the WiFi `lucia-control`** (password
   `lucia-143-tomato`).
3. **Open http://10.42.0.1:8080** in a browser.
4. Press **Start mapping** (left column). Within about 10 seconds the Lidar,
   IMU and Wheels lights should turn green and the map should appear; the
   Navigation light follows once Nav2 has started.
5. Press **Explore** (top right, or in the left column). The robot drives itself
   to the edges of what it has mapped (orange dots, "frontiers"), one after
   another, until nothing is left, then drives back **HOME** (the green H:
   where it was when you pressed Start mapping). Explore works on the robot
   (2026-10-17).
   - **HOME** sends it home at any time.
   - **STOP** (top right, or Space / Esc) stops it where it is. On a phone a
     STOP button also floats at the bottom while the robot drives itself.
   - **Speed** sets how fast it drives, for Explore, HOME and driving by hand.
   - Stay with it: it steers around what the lidar sees, not things lower than
     the lidar (until the ToF safety stop is on; see `use_tof` below).
6. Type a name and press **Save**. The map appears in the list underneath; click
   **download** to get the PNG. Saved maps live on the Pi in `~/maps/`.
7. Press **Stop mapping** when done.

**The page.** Left column, top to bottom: the **Radar** (the lidar right now,
forward up; red under 0.3 m, amber under 0.6 m), live **metrics**, the mapping
and autonomy controls, the drive pad, Save map. The rest is the **map**:
- Mouse wheel zooms, drag pans; **Fit** shows the whole map, **Follow** keeps
  the robot in the middle. `#follow` or `#zoom=3` at the end of the address sets
  the starting view.
- Drawn on it: the robot at its real size (18 in across), its trail (grey), the
  planned path (dashed), frontiers, HOME, the current goal, and the live lidar
  points.
- **Click to go**, then click the map, to send the robot to one spot.

Metrics: heading (0° = the way it faced at Start mapping, clockwise), speed the
wheels are told, ground speed (from the map), turn rate (IMU gyro), distance
driven, time mapping, distance from HOME, position, area mapped, frontiers,
nearest obstacle, lidar / IMU / odom message rates, Pi CPU temperature, CPU and
memory. Battery says n/a until the Roomba reply cable is fixed.

**Logs** (the bar under the map) show what's happening on the Pi: the newest
message plus warning / error counts. Open it for every ROS node's log (mission,
Nav2, slam_toolbox, roomba_bridge...), with level / node / text filters, a
**Launch output** tab (the raw `ros2 launch` text, where startup errors show
up), and **Download** to save what's shown as a text file.

**Horn and tunes** (on the Roomba's own speaker, while mapping runs): **Horn** in
the header or the **H** key honks. The *Horn and tunes* card plays the built-in
tunes (Horn, Charge!, La Cucaracha, Mass Destruction from Persona 3) or saved
ones, and **Preview here** plays them in the browser first. Notes can be typed
or edited (`B4:16 G#4:16 R:8 ...`: name and octave, C4 = middle C, or R for a
rest, then the length in 1/64 s), at most 64, one at a time. **Import from a
MIDI file** takes the melody of any MIDI file on your laptop or phone: pick the
part that has the tune, set where to start and the octave, preview, Save. Saved
tunes live on the Pi in `~/maps/tunes.json` (not in git). Keep the MIDI files
themselves out of the repo: fan MIDIs of game music are arrangements of
copyrighted songs.

**Driving by hand** (test tool): WASD or the arrow keys anywhere on the page
except while typing in a text box, or hold a pad button; it stops when you let
go. Only while mapping runs (the wheels driver is part of it). Driving by hand
during Explore or HOME stops the mission first.

### What the lights mean

| Light | Green when | If it stays red or yellow |
|---|---|---|
| Mapping | `mapping.launch.py` is running | Press Start mapping; open Logs → Launch output |
| Lidar | `/scan` arrived in the last second | See "Lidar freezes after Start" under Known issues |
| IMU | `/imu/data` arrived in the last second | Is the IMU Pico plugged in? Replug it; untick "Use IMU" to map without it |
| Wheels | `/odom` arrived in the last second | `roomba_bridge` isn't running: check Logs. (This light doesn't show whether the Roomba itself is on, since its replies don't reach the Pi; if the wheels don't move, see "Roomba turned off and on" under Known issues) |
| Navigation | The mission node is running and Nav2 is up (yellow: mapping runs but Nav2 isn't ready yet) | Open Logs, filter Warnings and up, look for `lifecycle_manager` or `controller_server` errors |
| Connection | The page can reach the Pi | Are you on `lucia-control`? Is the Pi on? |

Yellow means the topic has gone quiet for 1-5 seconds; red means longer, or never.

---

## What runs where

ROS2 Jazzy runs **inside a Docker container**. The Pi runs Debian 13 (Trixie),
which ROS2 doesn't support: Jazzy's apt packages target Ubuntu 24.04 and need
Ubuntu-only library versions. The official `ros:jazzy-ros-base` arm64 image,
plus our additions (`docker/Dockerfile`), sidesteps that.

| Package (`src/ros2/`) | Node | Owns | Publishes / does |
|---|---|---|---|
| `lucia_control` | `roomba_bridge` | `/dev/roomba` | `/cmd_vel_safe` → wheels (Safe mode, re-entered every second); `/odom` estimated from commanded wheel speeds; `~/reconnect` service (Reconnect Roomba); `~/song` topic (horn and tunes, up to 64 notes) |
| `lucia_lidar` | `rplidar_composition` (from `rplidar_ros`) | `/dev/rplidar` | `/scan` at about 10 Hz |
| `lucia_lidar` | `slam_toolbox` | | `/map`, `map`→`odom` |
| `lucia_imu` | `imu_bridge` | `/dev/imu` | `/imu/data` (heading + turn rate) at about 50 Hz |
| `lucia_bringup` | `ekf_filter_node` (`robot_localization`) | | Combines `/odom` forward speed with IMU heading → `odom`→`base_link` |
| `lucia_bringup` | `safety_gate` | | `/cmd_vel` → `/cmd_vel_safe` (what `roomba_bridge` listens to); with `use_tof:=true`, blocks forward motion while `tof/near` is true or silent |
| `lucia_nav` | Nav2: `controller_server`, `planner_server`, `behavior_server`, `bt_navigator`, `lifecycle_manager_navigation` | | Plans a path on `/map` and drives it (`/cmd_vel`, `/plan`); settings in `lucia_nav/config/nav2.yaml` |
| `lucia_nav` | `mission` | | Explore (frontiers) / return home / go to a spot: takes `/mission/command`, sends Nav2 goals, publishes `/mission/state` |
| `lucia_web` | `web_ui` | port 8080 | The control page; starts/stops `mapping.launch.py`, sends `/mission/command`, sets Nav2's `speed_limit`, reads every node's log (`/rosout`), keeps saved tunes in `~/maps/tunes.json` |
| `lucia_vision` | | | Jetson/ZED camera code. Doesn't build on the Pi (needs CUDA); skipped automatically |

Rule: **one program per device.** Nothing else opens a device's port while its
node runs. That includes VS Code's MicroPico console for the IMU Pico.

Launch files:
- `lucia_bringup/launch/mapping.launch.py`: everything (control, safety gate,
  IMU, EKF, lidar, slam_toolbox, Nav2 + mission). Options:
  `use_imu:=false` (wheels-only heading), `use_nav:=false` (mapping only, no
  Nav2), `use_tof:=true` (ToF safety stop on; needs Noel's `lucia_tof` node
  running, or forward driving stays blocked).
- `lucia_nav/launch/navigation.launch.py`: Nav2 + the mission node only.
- `lucia_lidar/launch/lidar_and_map.launch.py`: lidar + slam_toolbox only, plus
  the `base_link`→`laser` transform (`laser_yaw` default π: the lidar is mounted
  turned around, which `scan_check` confirmed).

Transform tree: `map` → `odom` (slam_toolbox) → `base_link` (EKF, or
`roomba_bridge` without the IMU) → `laser` and `imu_link` (static).

---

## Updating the code on the Pi

The hotspot has no internet, so updates need a network that does.

1. Put the Pi on a WiFi with internet (see "Network modes" below), or plug in Ethernet.
2. On the Pi:
   ```bash
   cd ~/repos/group-project-group-37-project-l-u-c-i-a
   git fetch origin && git checkout dev && git pull && git submodule update --init
   docker compose -f docker/compose.yml restart    # rebuilds the ROS2 code on start
   ```
3. If `docker/compose.yml` changed, use `up -d` instead of `restart`
   (`restart` keeps the container's old settings):
   ```bash
   docker compose -f docker/compose.yml up -d
   ```
4. If `docker/Dockerfile` changed, rebuild the image first (about 7 minutes):
   ```bash
   docker build -t lucia/ros2:latest docker/
   docker compose -f docker/compose.yml up -d --force-recreate
   ```

The Pi keeps only the `dev` and `main` branches. Work happens on personal
branches and is merged into `dev`; the Pi pulls `dev`.

(`update.sh` was removed on 2026-10-17. It joined a `lucia` WiFi that doesn't
exist, worked around NetworkManager to do it, and ran `git clean -fd`, which
deletes untracked files such as saved maps.)

---

## Network modes

| Mode | When | Pi address | Internet |
|---|---|---|---|
| Hotspot `lucia-control` (`pi-ap`) | Default at every boot | `10.42.0.1` | No |
| Shared WiFi | For updates | Varies: `ip -4 addr show wlan0` on the Pi | Yes |
| Ethernet | Bench work, or as a safe way in while switching WiFi | Varies: `ip -4 addr show eth0` | Depends |

The Pi has one WiFi radio, so it's either a hotspot or on a shared WiFi, not both.
At boot it always starts the hotspot: `pi-ap` has autoconnect priority 100 and
every saved WiFi has 0 (checked 2026-10-17). Saved WiFi on the Pi: Christian's
personal network and `LiveLaughLove`.

**Switch to a shared WiFi.** Ethernet SSH is the safest way in, but it works over
the hotspot too; the session just drops when the hotspot goes away:
```bash
nmcli connection show                        # saved networks
sudo systemctl stop dnsmasq                  # the hotspot's address server - stop it first
sudo nmcli connection up "<network name or UUID>" || sudo systemctl start dnsmasq
```
Then join your laptop to that same WiFi and `ssh lucia@lucia.local` (the Pi
announces its name on the network with avahi, so you don't need its IP; not yet
tried from the laptop). Otherwise: `ip -4 addr show wlan0` on the Pi, or your
router's device list.

If the WiFi doesn't come up (wrong password, out of range), NetworkManager falls
back to the hotspot by itself. The `|| sudo systemctl start dnsmasq` restarts the
address server for that case. If you ever see `lucia-control` but your laptop
gets no address, `dnsmasq` is still stopped: reboot the Pi, or give the laptop the
address `10.42.0.50/24` by hand, `ssh lucia@10.42.0.1`, and run
`sudo systemctl start dnsmasq`.

**Back to the hotspot:** reboot, or
`sudo nmcli connection up pi-ap && sudo systemctl start dnsmasq` (the session
drops; join `lucia-control`).

The page is on port 8080 whichever network the Pi is on, for example
`http://lucia.local:8080`.

SSH: `ssh lucia@10.42.0.1` on the hotspot, `ssh lucia@lucia.local` elsewhere.

**Turning it off without a screen:** press the Pi 5's power button, then press it
again (on the desktop version of the OS the first press opens a shutdown menu).
Don't hold it down: that cuts the power. Or `sudo poweroff` over SSH. Either way
the container gets told to stop, so mapping stops and the Roomba is released
cleanly before the Pi powers off. (The two-press behavior is from the Pi docs,
not yet tried on this robot.)

---

## Devices and udev rules

`/etc/udev/rules.d/99-lucia.rules` gives each device a fixed name:

```
# RPLidar (CP210x chip)
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="0001", SYMLINK+="rplidar"

# Roomba 650 (FTDI chip)
SUBSYSTEM=="tty", ATTRS{serial}=="BG03LJ9U", SYMLINK+="roomba"

# IMU Pico (BNO08X) - matched by serial: the ToF Pico has the same vendor/product ID
SUBSYSTEM=="tty", ATTRS{idVendor}=="2e8a", ATTRS{idProduct}=="0005", ATTRS{serial}=="fd92d5f577b9417b", SYMLINK+="imu"
```

Check: `ls -la /dev/roomba /dev/rplidar /dev/imu`. After editing the rules:
`sudo udevadm control --reload && sudo udevadm trigger`, then wait a second, or
replug the device.

Adding a device, such as the ToF Pico: get its IDs with
`udevadm info -q property -n /dev/ttyACM0 | grep -E 'ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT'`.

Everything goes through a **powered USB hub**. Without the official 5 A supply,
the Pi 5 limits its own USB ports to 600 mA in total (`usb_max_current_enable=0`).

---

## The container

`docker/compose.yml` runs it in the background and restarts it at boot
(`restart: unless-stopped`). On start, it runs `colcon build` and then `web_ui`.
Stopping it (`restart`, `down`, or a Pi shutdown) lets `web_ui` stop mapping
first, so the wheels stop and the Roomba is released; that takes a few seconds.
It shares all of `/dev` with access rules by device type, so it starts even if a
device is unplugged, and sees devices that are plugged in later.

```bash
cd ~/repos/group-project-group-37-project-l-u-c-i-a
docker compose -f docker/compose.yml up -d        # start (and at every boot from then on)
docker compose -f docker/compose.yml logs -f      # watch output (Ctrl+C stops watching only)
docker compose -f docker/compose.yml restart      # after a git pull
docker compose -f docker/compose.yml up -d        # after a change to compose.yml itself
docker compose -f docker/compose.yml down         # stop, and don't start at boot
docker exec -it lucia_ros2 bash                   # a shell inside, for ROS2 commands
```

Inside a `docker exec` shell, ROS2 and our workspace are already loaded. If a
command says `Package '...' not found`, run `source /ros2_ws/install/setup.bash`.

### Running things by hand (debugging)

Stop mapping on the page first: two copies of a node would fight over the same device.

```bash
docker exec -it lucia_ros2 bash
ros2 launch lucia_bringup mapping.launch.py                 # what Start mapping runs
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.5
ros2 run nav2_map_server map_saver_cli -f /maps/<name> --fmt png
```

Without compose (for example, to test a change in isolation), stop the compose
container first (`docker compose -f docker/compose.yml down`), then:
```bash
docker run -it --rm --network=host --device=/dev/rplidar --device=/dev/roomba --device=/dev/imu \
  -v $(pwd)/src/ros2:/ros2_ws/src -v ~/maps:/maps --name lucia_ros2 lucia/ros2:latest bash
colcon build && source install/setup.bash
```
Paste the `docker run` line on its own and wait for the `root@lucia:/ros2_ws#`
prompt before pasting anything else.

### The image

`docker/Dockerfile`, built on the Pi (needs internet, about 7 minutes):
```bash
docker build -t lucia/ros2:latest docker/
```
It adds `python3-serial`, `rplidar_ros`, `slam_toolbox`, `nav2_bringup` (includes
`map_saver_cli`), `teleop_twist_keyboard` and `robot_localization`. It also sets
`ROS_DOMAIN_ID=42`, and a colcon defaults file (skip `lucia_vision`,
`--symlink-install`) so plain `colcon build` works.

---

## The IMU Pico

Ling's BNO08X is wired to a Raspberry Pi Pico 2 (I²C address `0x4A`; wiring in
`docs/BNO08X Pico2 Setup.md`) and plugged into the Pi over USB. The Pico runs
`src/imu/stream_main.py` as its `main.py` and prints one line per reading, about
50 per second:

```
I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>
```
That's the game rotation vector (gyro + accelerometer, no compass, which the
Roomba's motors would throw off) and the gyro in rad/s. `imu_bridge` ignores every
line that doesn't start with `I,`.

**Quick check, without ROS2** (on the Pi, with nothing else using the port):
```bash
stty -F /dev/imu raw -echo
timeout 1 cat /dev/imu | head -3
```

**Load code onto the Pico from the Pi** (`mpremote` is installed via `pipx`):
```bash
M="mpremote connect /dev/imu"
$M exec "from machine import I2C, Pin; print(I2C(1, scl=Pin(3), sda=Pin(2), freq=100000).scan())"   # [74] = BNO08X found
$M fs ls :
$M fs cp src/imu/stream_main.py :main.py
$M reset
```
The driver files `bno08x.py` and `i2c.py` are already in the Pico's root.
`mpremote` can't interrupt the Pico while it's streaming. If it hangs, press
Ctrl+C and replug the Pico, or use VS Code's MicroPico from a laptop. This Pico
used to hold a ToF `main.py`, which is backed up on the Pi as `~/tof_main_backup.py`.

**Mounting:** flat, chip side up, fixed to the robot. Which way it points doesn't
matter: only heading and turn rate are used, and the EKF treats the starting
heading as 0.

---

## Diagnostics

| Tool | What it tells you |
|---|---|
| `ros2 run lucia_lidar scan_check` | Closest lidar reading's distance and angle, once a second. Hand in front of the robot ≈ ±180°, on its left ≈ -90° |
| `ros2 run lucia_imu imu_bridge --ros-args --log-level imu_bridge:=debug` | Heading, turn rate and readings per second every 2 s (stop mapping first: it owns `/dev/imu`) |
| `ros2 topic hz /scan` (or `/imu/data`, `/odom`) | Whether a topic is publishing, and how fast |
| `ros2 node list`, `ros2 topic list --no-daemon` | What's running and visible |
| `ros2 run tf2_ros tf2_echo odom laser` | Whether the transform chain is complete |

**Roomba reply check** (are encoder or battery readings coming back at all?).
Stop mapping first, then inside the container:
```bash
python3 - <<'EOF'
import serial, time
s = serial.Serial('/dev/roomba', 115200, timeout=1)
time.sleep(1); s.write(bytes([128])); time.sleep(0.2)
def ask(pkt):
    s.reset_input_buffer(); s.write(bytes([142, pkt])); return s.read(2)
r = ask(22); print("battery voltage:", len(r), "of 2 bytes", int.from_bytes(r, 'big') if len(r) == 2 else '')
EOF
```
As of 2026-10-16 this gets **0 bytes**: commands reach the Roomba, but nothing
comes back. See Known issues.

---

## Known issues

- **Roomba replies don't reach the Pi** (battery, encoders, sensors). Commands
  work; the Roomba's transmit line (mini-DIN pin 4) to the FTDI adapter is broken
  somewhere. Until it's fixed, `/odom` is estimated from commanded wheel speed,
  and the IMU provides heading. Next step: a loopback test. Unplug from the Roomba,
  jumper pins 3 and 4 on the cable's plug, send bytes and see if they come back.
  If they do, the fault is at the Roomba end; if not, replace the adapter or cable.
- **Lidar freezes after `Start`** (rare, seen once on a first run): the motor
  spins, but there's no `current scan mode` line and no `/scan`, and Ctrl+C is
  ignored. On the page: Stop mapping, then Start mapping. In a terminal: Ctrl+\ and rerun.
- **Roomba turned off and on, or its battery died:** turn it on and press
  **Reconnect Roomba** on the page, then listen for a short beep. Mapping keeps
  running, so the map isn't lost. (A freshly powered-on Roomba ignores the
  Safe-mode heartbeat until it gets Start; Reconnect reopens the serial port,
  wakes it, sends Start and Safe, and beeps.) No beep: press the Roomba's CLEAN
  button once to wake it and try again. From a terminal:
  `ros2 service call /roomba_bridge/reconnect std_srvs/srv/Trigger`.
- **First line from the IMU** shows up in the log as `Pico: 01477,...`. It's half
  a line caught mid-stream, and harmless.
- **`lucia_vision` build error:** shouldn't happen any more with the current
  image (it's skipped automatically). With an older image, use
  `colcon build --packages-skip lucia_vision`.

---

## First-time setup (a fresh Pi)

1. Install Docker and let `lucia` use it without `sudo`, then log out and back in:
   ```bash
   curl -fsSL https://get.docker.com | sh
   sudo usermod -aG docker lucia
   ```
2. Hotspot and DHCP (`pi-ap`, `dnsmasq`): see the "Raspberry Pi Initial Setup"
   section of the README.
3. Clone the repo to `~/repos/group-project-group-37-project-l-u-c-i-a`, check out
   `dev`, and run `git submodule update --init`.
4. udev rules: see "Devices and udev rules" above.
5. `mkdir -p ~/maps`, build the image, then `docker compose -f docker/compose.yml up -d`.
6. `pipx install mpremote` (for loading code onto the Picos).
7. Make sure nothing else starts on the robot's devices at boot. The old
   `lucia.service` (last semester's web panel) is disabled:
   `sudo systemctl disable --now lucia`.
