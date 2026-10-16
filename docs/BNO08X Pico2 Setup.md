# GY-BNO08X with Raspberry Pi Pico 2 and MicroPython

## Analysis

The Pico 2 communicates with the GY-BNO08X over I²C. The sensor is detected at address `0x4B, and the working software configuration uses the sensor's software reset through I²C:

```python
reset_pin=None
```

The sensor's hardware reset pin is optional due to the reset in the I2C.

## 1. Software requirements

The Pico must be running **MicroPython**.

The original CircuitPython code used modules such as:

```python
import board
import busio
from adafruit_bno08x.i2c import BNO08X_I2C
```

That code does not run on MicroPython because `board` and `busio` are CircuitPython modules.

Use this MicroPython driver instead:

<https://github.com/bradcar/bno08x_i2c_spi_MicroPython>

Copy these driver files to the Pico:

```text
bno08x.py
i2c.py
```

The files should be uploaded together and should come from the same driver version.

The correct imports are:

```python
from bno08x import *
from i2c import BNO08X_I2C
```

Inside `i2c.py`, the base-class import should be:

```python
from bno08x import BNO08X
```

It should not use:

```python
from imu import ...
from lib.bno08x import ...
```

## 2. Pico 2 wiring

Use 3.3 V power only.

| GY-BNO08X pin | Pico 2 connection | Purpose |
|---|---|---|
| VCC | Physical pin 36, 3V3(OUT) | Sensor power |
| GND | Physical pin 38, GND | Ground |
| ADD | Physical pin 3, GND | Ground |
| SCL | Physical pin 5, GPIO3 | I²C clock |
| SDA | Physical pin 4, GPIO2 | I²C data |
| INT | Physical pin 21, GPIO16 | Sensor interrupt |
| RST | Optional | Hardware reset |
| PS0 | GND | I²C interface mode |
| PS1 | GND | I²C interface mode |
| CS | Leave disconnected for I²C | Not used |

In the MicroPython code, the I²C pins are:

```python
scl=Pin(3)
sda=Pin(2)
```

### Important power warning

Do not connect sensor VCC to Pico physical pin 37. Physical pin 37 is `3V3_EN`, not 3.3 V output. Connecting VCC there caused the Pico to disconnect.

Use physical pin 36 for `3V3(OUT)`.

## 3. I²C address

Run this scan:

```python
from machine import I2C, Pin

i2c = I2C(
    1,
    scl=Pin(3),
    sda=Pin(2),
    freq=100000
)

print(i2c.scan())
```

The scan produced:

```text
[75]
```

Decimal 75 is hexadecimal `0x4B`, so the driver must use:

```python
address=0x4B
```

## 4. Final working program

Save this as `main.py`:

```python
from machine import I2C, Pin

print("Starting... ")

i2c = I2C(
    1,
    scl=Pin(3),
    sda=Pin(2),
    freq=100000
)

print("I2C is created")
print("I2C devices:", i2c.scan())

int_pin = Pin(16, Pin.IN, Pin.PULL_UP) # int pin
# reset can be used through the i2c
print("Pins assigned")

from bno08x import *
from i2c import BNO08X_I2C

print("Libraries and drivers imported")

bno = BNO08X_I2C(
    i2c,
    address=0x4B,
    int_pin=int_pin,
    reset_pin=None
)

print("BNO08X initialized")

bno.acceleration.enable(10)

print("Acceleration enabled")

while True:
    bno.update_sensors()

    if bno.acceleration.updated:
        x, y, z = bno.acceleration
        print("X:", x, "Y:", y, "Z:", z)
```

## 5. Uploading with VS Code and MicroPico

Recommended project layout:

```text
project/
├── main.py
└── lib/
    ├── bno08x.py
    └── i2c.py
```

In VS Code:

1. Make sure the MicroPico status bar says **Board Connected**.
2. Upload `bno08x.py` and `i2c.py` to the Pico.
3. Upload `main.py`.
4. Press `Ctrl+D` in the MicroPython terminal to perform a soft reboot.
5. Run the current file.

If imports behave unexpectedly, avoid duplicate copies of the driver in both the Pico root and `/lib`. Old copies can cause the Pico to execute a different file than the one currently open in VS Code.

## 6. Troubleshooting Logs

### CircuitPython/MicroPython mismatch

Error:

```text
ImportError: no module named 'board'
```

Cause: CircuitPython code was being run on MicroPython.

Fix: Replace `board`, `busio`, and `adafruit_bno08x` with the MicroPython driver and `machine.I2C`.

### Missing driver

Error:

```text
ImportError: no module named 'bno08x'
```

Cause: `bno08x.py` had not been uploaded to the Pico.

Fix: Upload `bno08x.py` and `i2c.py`.

### Incorrect import path

Error:

```text
ImportError: no module named 'imu'
```

Cause: An older or mismatched `i2c.py` was being used, or the Pico was still running an old uploaded copy.

Fix: Use matching current driver files and ensure `i2c.py` imports:

```python
from bno08x import BNO08X
```

Upload the file again and press `Ctrl+D`.

### Sensor detected

Output:

```text
I2C devices: [75]
```

Meaning: The sensor is responding at address `0x4B`. This confirmed that the SDA, SCL, power, and I²C interface mode were sufficiently correct for detection.

### Hardware reset errors

Errors encountered:

```text
OSError: [Errno 5] EIO
RuntimeError: Hard reset not acknowledged, check BNO08x wiring
```

Cause: The driver could detect the sensor, but the hardware reset sequence was not acknowledged.

The hardware reset wire can be moved to another unused GPIO if needed. For example:

```python
reset_pin = Pin(27, Pin.OUT, value=1)
```

The sensor RST wire must be moved to GPIO27 at the same time. Do not reuse GPIO2, GPIO3, or GPIO16 because they are used for SDA, SCL, and INT.

### Soft reset and reset-cause mismatch

With `reset_pin=None`, the sensor accepted the software reset and returned product information, including firmware versions. The driver then reported:

```text
RuntimeError: Soft reset cause mismatch; check reset_pin wiring
```

This happened after successful I²C communication and product identification. It is a driver reset-cause validation issue, not an address-scan failure.

The current configuration leaves the reset pin out of the constructor:

```python
reset_pin=None
```

## 7. What successful output should look like

The beginning should look similar to:

```text
Starting...
I2C is created
I2C devices: [75]
Pins assigned
Libraries and drivers imported
BNO08X initialized
Acceleration enabled
X: ...
Y: ...
Z: ...
```

The acceleration values are printed in meters per second squared. The Z-axis will usually show approximately gravitational acceleration when the sensor is stationary, depending on the sensor orientation.

## Current status

- Pico 2 is running MicroPython.
- The BNO08X is detected at `0x4B`.
- I²C uses GPIO3 for SCL and GPIO2 for SDA.
- INT uses GPIO16.
- The software-reset configuration uses `reset_pin=None`.
- Sensor VCC must be connected to Pico physical pin 36, not pin 37.
- The final application is ready to run once the driver files are uploaded and the soft-reset validation issue is resolved or bypassed.
