# Roomba SSH Scripts

Python scripts for controlling the Roomba 650 over an SSH session. All scripts in `src/scripts/ssh/` are designed to work without a display server, physical keyboard access, or `evdev` — they run entirely in the terminal over SSH.

---

## Overview

Connect to the Pi over its WiFi access point or ethernet, then run scripts remotely.

**SSH into the Pi:**
```bash
# Via the Pi's AP (lucia-control)
ssh lucia@10.42.0.1

# Via ethernet
ssh lucia@192.168.0.140
```

**Run scripts from the `src/scripts/` directory:**
```bash
cd ~/repos/group-project-group-37-project-l-u-c-i-a/src/scripts
PYTHONPATH=. python3 ssh/<script>.py
```

All scripts use the serial port at `/dev/ttyUSB0` by default.

---

## Script Reference

### `control_panel_ssh.py` — Interactive Control Panel (SSH)

The primary way to control the Roomba over SSH. Uses `curses` to draw a live terminal UI with real-time sensor data and keyboard drive controls — no display server or `evdev` required. Also displays live UPS data from the Geekworm X1202 (battery voltage, charge %, AC status) if the board is present.

**Dependencies:**
```bash
pip install smbus2   # required for X1202 UPS readings
# RPi.GPIO is pre-installed on Raspberry Pi OS
```

**Usage:**
```bash
PYTHONPATH=. python3 ssh/control_panel_ssh.py
PYTHONPATH=. python3 ssh/control_panel_ssh.py --port /dev/ttyUSB0 --speed 300
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Serial port |
| `--speed` | `300` | Starting drive speed in mm/s |

**Drive controls (hold key to move, release to stop):**

| Key(s) | Action |
|--------|--------|
| `W` / `↑` | Forward |
| `S` / `↓` | Backward |
| `A` / `←` | Spin left (CCW) |
| `D` / `→` | Spin right (CW) |

> Combined key presses (e.g. `W+A`) are not supported over SSH — only one key is detected at a time.

**Hotkeys:**

| Key | Action |
|-----|--------|
| `+` / `=` | Increase speed by 50 mm/s (max 500) |
| `-` | Decrease speed by 50 mm/s (min 50) |
| `1` | Play Mass Destruction |
| `2` | Play La Cucaracha |
| `T` | Run square drive demo |
| `R` | Reset Roomba |
| `X` | Power off Roomba |
| `Q` / `ESC` | Quit |

**Display panels:**

| Panel | Data |
|-------|------|
| Sensors | Bump, wheel drop, cliff sensors |
| Battery | Roomba battery voltage, current, temp, charge % |
| Encoders | Raw left/right wheel encoder counts |
| X1202 UPS | Pack voltage (V), state of charge (%), AC power status, charge status |
| Drive | Direction, wheel speeds, current speed setting |
| Controls | Key reference |

> The X1202 UPS panel shows `(unavailable)` if `smbus2` or `RPi.GPIO` are not installed, or if the board is not connected.

**Note on key-repeat and latency:** SSH terminals send repeated key events while a key is held. The script uses a grace window (`DRIVE_TIMEOUT`) — the Roomba keeps driving as long as a drive key was received within that window, then stops automatically.

For the best response, run this on your **local machine** (not the Pi) before connecting:
```bash
xset r rate 150 50
#             ^   ^
#     150 ms initial repeat delay
#         50 repeats/sec (event every 20 ms)
```
The script's `DRIVE_TIMEOUT` is set to 200 ms to match. If you skip the `xset` step, driving will feel choppy because the default ~250 ms initial delay exceeds the timeout.

---

### `sensor_monitor.py` — Live Sensor Dashboard

Continuously polls and displays all major Roomba sensors in a refreshing terminal dashboard. Works identically over SSH.

**Usage:**
```bash
PYTHONPATH=. python3 ssh/sensor_monitor.py
PYTHONPATH=. python3 ssh/sensor_monitor.py --port /dev/ttyUSB0 --interval 0.25
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Serial port |
| `--interval` | `0.5` | Poll interval in seconds |

**Displays:**

| Section | Data |
|---------|------|
| Bump Sensors | Left and right bump state |
| Wheel Drop | Left and right wheel drop state |
| Cliff Sensors | Left, front-left, front-right, right |
| Battery | Voltage (mV), current (mA), temperature (°C), charge (mAh / %) |
| Wheel Encoders | Raw left and right encoder counts (wraps at 65535) |

Press `Ctrl+C` to exit.

---

### `test_led.py` — LED Display Test

Verifies the serial connection by displaying "LUCI" on the Roomba's 7-segment LED display for 5 seconds.

**Usage:**
```bash
PYTHONPATH=. python3 ssh/test_led.py
PYTHONPATH=. python3 ssh/test_led.py --port /dev/ttyUSB0
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Serial port |

**Use this first** when connecting to confirm the serial link is working before running any drive scripts.

---

### `reset.py` — Soft Reset

Sends opcode 7 to reboot the Roomba's OI. The Roomba will return to passive mode after rebooting.

**Usage:**
```bash
PYTHONPATH=. python3 ssh/reset.py
PYTHONPATH=. python3 ssh/reset.py --port /dev/ttyUSB0
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Serial port |

> All scripts automatically reset the Roomba on exit via the `RoombaOI` context manager. Use this script only if you need a manual reset without running another script.

---

### `power_off.py` — Power Off

Powers down the Roomba using opcode 133. The Roomba will enter sleep mode and stop responding until the CLEAN button is pressed.

**Usage:**
```bash
PYTHONPATH=. python3 ssh/power_off.py
PYTHONPATH=. python3 ssh/power_off.py --port /dev/ttyUSB0
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `/dev/ttyUSB0` | Serial port |

---

## Troubleshooting

**`PermissionError: could not open port /dev/ttyUSB0`**
- Run `sudo chmod 666 /dev/ttyUSB0` for a one-time fix.
- Permanent fix: `sudo usermod -aG dialout $USER` then log out and back in.

**Terminal display is garbled in `control_panel_ssh.py`**
- Your terminal must support at least 80×24 characters. Resize the window and try again.
- Make sure your SSH client is passing `$TERM` correctly. Try: `TERM=xterm-256color ssh lucia@10.42.0.1`

**Driving feels unresponsive or choppy**
- Run `xset r rate 150 50` on your local machine before SSHing in. This lowers the initial key-repeat delay to 150 ms, which is under the script's 200 ms `DRIVE_TIMEOUT`.
- Only one key at a time is supported over SSH — diagonal movement is not available.

**Roomba does not move after connecting**
- The script calls `start()` and `full_mode()` automatically. If it still won't move, check battery level with `sensor_monitor.py`.
- Make sure the Roomba is not on its charging dock.

**`serial.SerialException: could not open port`**
- Wrong port. Check with `ls /dev/tty*` before and after plugging in the USB-serial adapter.
- Cable not plugged in, or the USB-serial adapter driver is not loaded.

**Two scripts cannot run at the same time**
- Only one process can hold the serial port at a time. Close any other running script before starting a new one.

**X1202 UPS panel shows `(unavailable)`**
- Install the dependency: `pip install smbus2`
- Make sure I2C is enabled on the Pi: `sudo raspi-config` → Interface Options → I2C → Enable.
- Verify the board is detected: `i2cdetect -y 1` should show `0x36` on the grid.
