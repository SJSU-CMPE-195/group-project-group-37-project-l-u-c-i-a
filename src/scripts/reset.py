"""
reset.py

Sends a soft reset command to the Roomba (opcode 7).
The Roomba will reboot its OI and return to passive mode.

Usage:
    python reset.py
    python reset.py --port COM3
"""

import argparse

from roomba_oi import RoombaOI


def main():
    parser = argparse.ArgumentParser(description='Soft reset the Roomba')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        print("Resetting Roomba...")
    print("Done.")


if __name__ == '__main__':
    main()