# Roomba Serial Scripts

Python scripts for communicating with the Roomba 650 over serial using the iRobot Open Interface (OI).

---

## Overview

All scripts live in `src/scripts/` and communicate with the Roomba via USB-to-serial adapter at 115200 baud. They are built on a shared wrapper class (`roomba_oi.py`) so serial connection setup and opcode encoding is handled in one place.

**Dependencies:**
```
pip install pyserial pynput
```

**Port values:**
- Windows: `COM5` (check Device Manager if unsure)
- Linux / WSL with usbipd: `/dev/ttyUSB0`

---

## Script Reference

### `roomba_oi.py` — OI Wrapper Library

Not run directly. Imported by all other scripts.

Wraps the iRobot Open Interface serial protocol into a Python class. Handles serial connection setup, byte encoding, mode switching, motion commands, and sensor reads.

**Key methods:**

| Method | Description |
|--------|-------------|
| `start()` | Enter OI passive mode. Always call first. |
| `full_mode()` | Enter full control mode. Safety stops disabled. |
| `safe_mode()` | Enter safe mode. Safety stops active. |
| `drive(velocity, radius)` | Drive with a single velocity and turning radius. |
| `drive_direct(left, right)` | Control each wheel independently. |
| `stop()` | Stop all wheel movement. |
| `seek_dock()` | Command Roomba to return to charging dock. |
| `display_text(text)` | Display up to 4 ASCII chars on the 7-segment display. |
| `read_bumps()` | Returns bump and wheel-drop sensor states. |
| `read_cliffs()` | Returns all four cliff sensor states. |
| `read_battery()` | Returns voltage, current, temp, charge %, etc. |
| `read_encoders()` | Returns raw left/right wheel encoder counts. |

**Usage pattern:**
```python
from roomba_oi import RoombaOI

with RoombaOI('COM5') as roomba:
    roomba.start()
    roomba.full_mode()
    roomba.drive(200, 32768)  # forward at 200 mm/s
```

The `with` block automatically stops the robot and closes the serial port on exit.

---

### `test_led.py` — LED Display Test

Verifies the serial connection by displaying "LUCI" on the Roomba's 7-segment LED display for 5 seconds.

**Usage:**
```
python test_led.py
python test_led.py --port COM3
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `COM5` | Serial port |

**Use this first** when connecting to a new machine to confirm the serial link is working before running any drive scripts.

---

### `drive_keyboard.py` — Real-Time Keyboard Control

Hold keys to drive the Roomba in real time. Releasing all keys stops the robot. Supports combined key inputs for smooth arced movement.

**Requirements:** `pip install pynput`

**Usage:**
```
python drive_keyboard.py --port COM5
python drive_keyboard.py --port COM5 --speed 300
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `COM5` | Serial port |
| `--speed` | `200` | Base wheel speed in mm/s (50–500) |

**Controls:**

| Key(s) | Action |
|--------|--------|
| `W` | Forward |
| `S` | Backward |
| `A` | Spin left (CCW) |
| `D` | Spin right (CW) |
| `W + A` | Arc forward-left |
| `W + D` | Arc forward-right |
| `S + A` | Arc backward-left |
| `S + D` | Arc backward-right |
| `Q` or `ESC` | Quit |

**How it works:**

Runs a 20 Hz loop that reads which keys are currently held, computes independent left/right wheel velocities, and sends `drive_direct` commands only when the state changes. Arc movements are achieved by running one wheel slower than the other (e.g. W+A runs the left wheel at half speed).

---

### `drive_demos.py` — Automated Drive Patterns

Runs predefined autonomous movement patterns. Useful for validating drive commands and estimating odometry accuracy.

**Usage:**
```
python drive_demos.py --port COM5 --demo square
python drive_demos.py --port COM5 --demo figure_eight
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `COM5` | Serial port |
| `--demo` | `square` | Pattern to run: `square` or `figure_eight` |

**Patterns:**

| Demo | Description |
|------|-------------|
| `square` | Four 600 mm legs with 90° left turns |
| `figure_eight` | Two 600 mm diameter circles in opposite directions |

**Note:** Movement accuracy is timing-based and approximate. Surface friction, battery level, and wheel slip all affect the result. Use `sensor_monitor.py` to observe encoder counts during a run.

---

### `sensor_monitor.py` — Live Sensor Dashboard

Continuously polls and displays all major Roomba sensors in a refreshing terminal dashboard.

**Usage:**
```
python sensor_monitor.py --port COM5
python sensor_monitor.py --port COM5 --interval 0.25
```

**Arguments:**

| Argument | Default | Description |
|----------|---------|-------------|
| `--port` | `COM5` | Serial port |
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

## OI Mode Reference

The Roomba 650 OI has three active modes. All scripts use **Full Mode**.

| Mode | Opcode | Behavior |
|------|--------|----------|
| Passive | 128 | Read sensors only. No drive commands. |
| Safe | 131 | Drive commands work. Auto-stops on cliff/wheel-drop. |
| Full | 132 | Full control. No automatic safety stops. |

---

## Serial Command Structure

Drive commands encode velocity and radius as signed 16-bit big-endian integers.

**Example — drive forward at 200 mm/s:**
```
[137] [0, 200] [127, 255]
  ^      ^         ^
opcode  +200     32767 (straight)
```

**Example — spin left in place at 150 mm/s:**
```
[137] [0, 150] [0, 1]
  ^      ^       ^
opcode  +150   radius=1 (CCW spin)
```

`drive_direct` (opcode 145) sends right wheel velocity then left wheel velocity, each as a signed 16-bit big-endian integer.

---

## Troubleshooting

**`serial.SerialException: could not open port`**
- Wrong port name. Check Device Manager (Windows) or `ls /dev/tty*` (Linux).
- Cable not plugged in, or driver not installed for the USB-serial adapter.

**Roomba does not move after connecting**
- Must call `start()` then `full_mode()` before any drive commands.
- Check battery level with `sensor_monitor.py`.

**Commands seem delayed or dropped**
- Reduce polling frequency in `sensor_monitor.py` if running alongside another script — two scripts cannot share one serial port simultaneously.

**WSL: port not found**
- Use `usbipd` to forward the USB device into WSL, or run scripts natively on Windows.
