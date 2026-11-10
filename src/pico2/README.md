# Pico 2 sensor board

The Pico 2 reads three VL53L5CX ToF sensors and a BNO08X IMU and streams both
over USB serial to the Pi 5. The packet format is documented at the top of
`main.py`; `src/ros2/lucia_tof/lucia_tof/protocol.py` parses it, and
`lucia_tof`'s `tof_node` publishes the ToF frames and the IMU (`/imu/data`).
This is the robot's only Pico (decided 2026-11-10); on the Pi it's `/dev/imu`.

## Wiring

| Device             | Pico 2                               |
|--------------------|--------------------------------------|
| ToF SDA / SCL      | GP0 / GP1 (I2C0), all three shared   |
| ToF left LPn       | GP14                                 |
| ToF middle LPn     | GP15                                 |
| ToF right LPn      | not connected (always on)            |
| IMU SDA / SCL      | GP2 / GP3 (I2C1)                     |
| IMU INT            | GP16 (required)                      |
| IMU RST            | not connected                        |
| IMU ADD            | GND = 0x4A (the robot's IMU), or not connected = 0x4B; either is found |
| IMU PS0, PS1       | GND (I2C mode)                       |

Pins and addresses are set in the configuration block of `main.py`.

## Files on the Pico

```text
main.py                 <- src/pico2/main.py
lib/vl53l5cx/           <- from the vl53l5cx submodule (see src/tof_pico/README.md)
lib/imu/bno08x.py       <- src/imu/bno08x.py
lib/imu/i2c.py          <- src/imu/i2c.py
```

With [mpremote](https://docs.micropython.org/en/latest/reference/mpremote.html),
from the repo root:

```sh
mpremote fs mkdir :lib/imu
mpremote fs cp src/imu/bno08x.py :lib/imu/bno08x.py + fs cp src/imu/i2c.py :lib/imu/i2c.py
mpremote fs cp src/pico2/main.py :main.py + reset
```

If the IMU is missing or fails to start, `main.py` logs why and keeps
streaming the ToF sensors. The other way round too: without the ToF sensors
(not wired yet, or one not answering) it logs why and streams the IMU alone.
The Pi then treats the missing ToF sensors as "something close", so the ToF
safety stop, if it's on, blocks forward driving.

### Loading it onto the robot's Pico from the Pi

Stop mapping on the web page first (its `tof_node` holds the port). From the
repo on the Pi, with the `vl53l5cx` submodule checked out
(`git submodule update --init`):

```sh
M="mpremote connect /dev/imu"
$M fs mkdir :lib; $M fs mkdir :lib/imu; $M fs mkdir :lib/vl53l5cx   # "exists" errors are fine
$M fs cp src/imu/bno08x.py :lib/imu/bno08x.py + fs cp src/imu/i2c.py :lib/imu/i2c.py
$M fs cp vl53l5cx/vl53l5cx/__init__.py :lib/vl53l5cx/__init__.py + fs cp vl53l5cx/vl53l5cx/_config_file.py :lib/vl53l5cx/_config_file.py
$M fs cp vl53l5cx/vl53l5cx/mp.py :lib/vl53l5cx/mp.py + fs cp vl53l5cx/vl53l5cx/vl_fw_config.bin :lib/vl53l5cx/vl_fw_config.bin
$M fs cp src/pico2/main.py :main.py + reset
```

The `bno08x.py` / `i2c.py` already in the Pico's root are Ling's copies for
`stream_main.py` (with the import changed to `from bno08x import BNO08X`); they
can stay. `main.py` uses the unchanged copies in `lib/imu/`. To go back to the
IMU-only program: `$M fs cp src/imu/stream_main.py :main.py + reset`; the Pi
reads either.

If `mpremote` hangs because the Pico is busy streaming, press Ctrl+C, replug
the Pico and try again right away.

## Demo on a PC

```sh
pip install pyserial numpy matplotlib
python src/pico2/pc_demo.py              # or --port COM8
```

One window with the three ToF heat maps, live accelerometer and yaw / pitch /
roll plots, and a 3D box that turns with the IMU (blue = top, orange =
front). Press `r` to take the current orientation as level. `--save demo.png`
records 5 s and saves a screenshot instead of opening the window.

`python src/pico2/pc_heatmap.py` shows just the heat maps, with options for
smoothing, interpolation and per-cell values.

Close any other program holding the serial port first (MicroPico, a REPL, or
the other viewer).
