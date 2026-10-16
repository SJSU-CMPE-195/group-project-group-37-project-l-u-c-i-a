# Runs on the Pico. Streams the BNO08X's heading and rotation rate to the
# Raspberry Pi over USB serial, for the Pi's ROS2 imu_bridge node
# (src/ros2/lucia_imu) to publish on /imu/data. Upload as main.py so it runs
# on boot, together with bno08x.py and i2c.py (see docs/BNO08X Pico2 Setup.md,
# including the i2c.py import fix: `from bno08x import BNO08X`).
#
# Same wiring and setup as main.py; only the reports and the output differ.
#
# Protocol: one text line per game-quaternion report, about 50 per second:
#   I,<qw>,<qx>,<qy>,<qz>,<gx>,<gy>,<gz>\n
#   qw..qz  game rotation vector (unit quaternion). "Game" = gyro + accelerometer
#           only, no magnetometer: the Roomba's motors and metal would throw a
#           compass off, so heading drifts slowly instead of jumping.
#   gx..gz  gyro, rad/s (latest reading when the quaternion arrives)
# Every other line (startup text, errors) starts with something other than "I,"
# and is ignored by the Pi. If this format changes, change
# src/ros2/lucia_imu/lucia_imu/imu_line.py to match.

from machine import I2C, Pin

from bno08x import *
from i2c import BNO08X_I2C

REPORT_HZ = 50

print("# starting")

i2c = I2C(
    1,
    scl=Pin(3),
    sda=Pin(2),
    freq=100000
)
print("# I2C devices:", i2c.scan())

int_pin = Pin(16, Pin.IN, Pin.PULL_UP)

bno = BNO08X_I2C(
    i2c,
    address=0x4A,       # matches main.py (Ling changed it from 0x4B on 2026-10-16)
    int_pin=int_pin,
    reset_pin=None
)

bno.game_quaternion.enable(REPORT_HZ)
bno.gyro.enable(REPORT_HZ)
print("# streaming")

gx = gy = gz = 0.0
while True:
    bno.update_sensors()

    if bno.gyro.updated:
        gx, gy, gz = bno.gyro

    if bno.game_quaternion.updated:
        qw, qx, qy, qz = bno.game_quaternion
        print("I,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f" % (qw, qx, qy, qz, gx, gy, gz))
