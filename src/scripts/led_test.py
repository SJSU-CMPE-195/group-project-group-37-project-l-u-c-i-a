import serial
import time

ser = serial.Serial('COM5', 115200, timeout=1) # adjust COM port as needed
time.sleep(2)

# Start open interface
ser.write(bytes([128])) # 128 start code
time.sleep(0.1)

# Safe mode
ser.write(bytes([132]))
time.sleep(0.1)

# Display "LUCI"
ser.write(bytes([164, 76, 85, 67, 73]))

time.sleep(5)

ser.close()