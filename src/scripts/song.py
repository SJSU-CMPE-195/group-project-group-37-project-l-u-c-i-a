import serial
import time

ser = serial.Serial('COM5', 115200, timeout=1)  # change COM port
time.sleep(2)

# Start OI
ser.write(bytes([128]))
time.sleep(0.1)

# Safe mode
ser.write(bytes([132]))
time.sleep(0.1)

# La Cucaracha - standard short phrase
# Notes:
# C4=60 D4=62 E4=64 F4=65 G4=67 A4=69 Bb4=70
#
# Duration units are 1/64 sec
# 16 = quarter-ish, 32 = half-ish, 8 = short

song = [
    140, 0, 16,

    64, 16, 64, 16, 64, 16,   # E E E
    60, 12, 64, 12, 67, 24,   # C E G
    67, 16, 67, 16, 65, 16,   # G G F
    64, 16, 62, 16, 60, 24,   # E D C
    64, 16, 64, 16, 64, 16,   # E E E
    60, 12                    # C
]

ser.write(bytes(song))
time.sleep(0.1)

# Play song 0
ser.write(bytes([141, 0]))

time.sleep(6)
ser.close()