import serial
import time

ser = serial.Serial('COM5', 115200, timeout=1) # adjust COM port as needed
time.sleep(2)

# Start open interface
ser.write(bytes([128])) # 128 start code
time.sleep(0.1)

# Reset roomba
ser.write(bytes[7])