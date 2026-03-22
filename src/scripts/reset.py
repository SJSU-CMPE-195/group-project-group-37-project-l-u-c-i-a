"""
reset.py

Sends a soft reset command to the Roomba (opcode 7).
The Roomba will reboot its OI and return to passive mode.

Usage:
    python reset.py
    python reset.py --port COM3
"""

import argparse
import time

from roomba_oi import RoombaOI


def main():
    parser = argparse.ArgumentParser(description='Soft reset the Roomba')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    roomba = RoombaOI(args.port)
    roomba.start()
    roomba._send(7)  # opcode 7 = soft reset
    print("Reset command sent. Roomba is rebooting.")
    time.sleep(3)    # allow time to reboot
    roomba.ser.close()  # close port directly — skip stop/passive_mode on a rebooting Roomba


if __name__ == '__main__':
    main()