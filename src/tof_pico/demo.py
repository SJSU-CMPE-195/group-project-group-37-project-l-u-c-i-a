from time import sleep_ms

from machine import I2C, Pin
from vl53l5cx.mp import VL53L5CXMP
from vl53l5cx import (
    DATA_DISTANCE_MM,
    DATA_TARGET_STATUS,
    STATUS_VALID,
    RESOLUTION_4X4,
    RESOLUTION_8X8,
)

# --- Configuration ---------------------------------------------------------
# All sensors share I2C0 (GP0 = SDA, GP1 = SCL).
SDA_PIN, SCL_PIN = 0, 1
I2C_FREQ = 1_000_000

# Listed left to right as seen standing in front of the sensors.
# (name, LPn GPIO or None if not connected, unique I2C address)
# At most one sensor may have LPn not connected: it is always enabled, so it
# must be readdressed first while the others are held off.
SENSORS = [
    ("left", 14, 0x32),
    ("middle", 15, 0x31),
    ("right", None, 0x30),
]

DEFAULT_ADDR = 0x29

RESOLUTION = RESOLUTION_8X8
RANGING_FREQ = 10         # Hz (max 15 for 8x8, 60 for 4x4)
# ---------------------------------------------------------------------------

WIDTH = 8 if RESOLUTION == RESOLUTION_8X8 else 4


def set_i2c_address(tof, new_addr):
    # Port of ST ULD vl53l5cx_set_i2c_address(). The new address is kept
    # until the sensor loses power.
    tof._wr_byte(0x7FFF, 0x00)
    tof._wr_byte(0x0004, new_addr)
    tof.addr = new_addr
    tof._wr_byte(0x7FFF, 0x02)


def assign_address(i2c, name, target_addr):
    # Only the sensor being configured is enabled at this point. After a soft
    # reboot it may still hold its address from the last run, so check first.
    found = i2c.scan()
    if target_addr in found:
        return VL53L5CXMP(i2c, addr=target_addr)
    if DEFAULT_ADDR not in found:
        raise ValueError("{} sensor not detected (scan: {})".format(
            name, [hex(a) for a in found]))

    tof = VL53L5CXMP(i2c, addr=DEFAULT_ADDR)
    set_i2c_address(tof, target_addr)
    if target_addr not in i2c.scan():
        raise ValueError("{} sensor did not move to 0x{:02X}".format(name, target_addr))
    return tof


def setup_sensors(i2c):
    if sum(1 for _, lpn, _ in SENSORS if lpn is None) > 1:
        raise ValueError("only one sensor may have LPn not connected")

    # Hold every controllable sensor off so only the always-on one answers.
    lpns = {}
    for _, lpn_pin, _ in SENSORS:
        if lpn_pin is not None:
            lpns[lpn_pin] = Pin(lpn_pin, Pin.OUT, value=0)
    sleep_ms(100)

    # Always-on sensor first, then enable the rest one at a time.
    order = sorted(SENSORS, key=lambda s: s[1] is not None)
    by_name = {}
    for name, lpn_pin, addr in order:
        if lpn_pin is not None:
            lpns[lpn_pin].value(1)
            sleep_ms(100)
        by_name[name] = assign_address(i2c, name, addr)
        print("{} sensor at 0x{:02X}".format(name, addr))

    print("I2C scan:", [hex(a) for a in i2c.scan()])

    sensors = []
    for name, _, _ in SENSORS:
        tof = by_name[name]
        print("Uploading firmware to {} sensor...".format(name))
        tof.init()
        tof.resolution = RESOLUTION
        tof.ranging_freq = RANGING_FREQ
        sensors.append((name, tof))

    for _, tof in sensors:
        tof.start_ranging({DATA_DISTANCE_MM, DATA_TARGET_STATUS})

    return sensors


def format_row(results, row):
    cells = []
    for i in range(row * WIDTH, (row + 1) * WIDTH):
        if results.target_status[i] == STATUS_VALID:
            cells.append("{:4}".format(results.distance_mm[i]))
        else:
            cells.append("  xx")
    return " ".join(cells)


def print_frames(sensors, latest):
    # Grids printed side by side, left to right, matching the physical layout.
    # Cursor is moved home each frame so the output redraws in place.
    col_width = WIDTH * 5 - 1
    gap = "   |   "
    lines = [gap.join("{:^{}}".format(name, col_width) for name, _ in sensors)]
    for row in range(WIDTH):
        lines.append(gap.join(format_row(latest[name], row) for name, _ in sensors))
    print("\x1b[H" + "\x1b[K\n".join(lines) + "\x1b[K")


def main():
    i2c = I2C(0, sda=Pin(SDA_PIN), scl=Pin(SCL_PIN), freq=I2C_FREQ)
    sensors = setup_sensors(i2c)

    # Clear the screen and hide the cursor before the redraw loop.
    print("\x1b[2J\x1b[?25l", end="")
    latest = {}
    while True:
        for name, tof in sensors:
            if tof.check_data_ready():
                latest[name] = tof.get_ranging_data()

        # Print once every sensor has a new frame.
        if len(latest) == len(sensors):
            print_frames(sensors, latest)
            latest = {}
        sleep_ms(5)


main()
