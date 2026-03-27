"""
power_off.py

Powers off the Roomba using the OI power command (opcode 133).
The Roomba will enter sleep mode and stop responding until powered back on.

Usage:
    python power_off.py
    python power_off.py --port /dev/ttyUSB0
"""

import argparse
import time

from roomba_oi import RoombaOI


def main():
    parser = argparse.ArgumentParser(description='Power off the Roomba')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    roomba = RoombaOI(args.port)
    roomba.start()
    roomba.stop()
    roomba._send(133)  # opcode 133 = power off
    print("Roomba powered off.")
    time.sleep(0.5)
    roomba.ser.close()  # close port directly — Roomba is off, skip reset


if __name__ == '__main__':
    main()
