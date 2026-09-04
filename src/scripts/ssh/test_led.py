"""
test_led.py

Connects to the Roomba and displays "LUCI" on the 7-segment LED display
for 5 seconds. Used to verify the serial connection and OI interface.

Usage:
    python test_led.py
    python test_led.py --port COM3
"""

import argparse
import time

from roomba_oi import RoombaOI


def main():
    parser = argparse.ArgumentParser(description='Roomba LED display test')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        roomba.display_text('LUCI')
        print("Displaying 'LUCI' for 5 seconds...")
        time.sleep(5)
    print("Done.")


if __name__ == '__main__':
    main()
