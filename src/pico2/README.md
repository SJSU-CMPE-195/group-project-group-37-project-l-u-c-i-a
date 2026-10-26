# Pico 2 sensor board

The Pico 2 reads three VL53L5CX ToF sensors and a BNO08X IMU and streams both
over USB serial to the Pi 5. The packet format is documented at the top of
`main.py`; `src/ros2/lucia_tof/lucia_tof/protocol.py` parses it.

## Wiring

| Device             | Pico 2                               |
|--------------------|--------------------------------------|
| ToF SDA / SCL      | GP0 / GP1 (I2C0), all three shared   |
| ToF left LPn       | GP14                                 |
| ToF middle LPn     | GP15                                 |
| ToF right LPn      | not connected (always on)            |
| IMU SDA / SCL      | GP2 / GP3 (I2C1)                     |
| IMU INT            | GP16 (required)                      |
| IMU RST, ADD       | not connected (ADD floats high: 0x4B)|
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
streaming the ToF sensors.

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
