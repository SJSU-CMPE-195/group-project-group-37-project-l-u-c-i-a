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