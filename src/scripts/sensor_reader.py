"""
sensor_reader.py

Continuously polls and displays Roomba sensor data in the terminal.
Reads bump sensors, cliff sensors, battery state, and wheel encoders.

Usage:
    python sensor_reader.py --port COM5
    python sensor_reader.py --port /dev/ttyUSB0 --interval 0.25
"""

import argparse
import os
import time

from roomba_oi import RoombaOI


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def bool_str(val):
    return "YES" if val else "---"


def print_dashboard(bumps, cliffs, battery, encoders, iteration):
    clear_screen()
    print("=" * 45)
    print("   LUCIA — Roomba Sensor Dashboard")
    print("=" * 45)

    print("\n[BUMP SENSORS]")
    print(f"  Left:  {bool_str(bumps['bump_left']):<6}  Right: {bool_str(bumps['bump_right'])}")

    print("\n[WHEEL DROP]")
    print(f"  Left:  {bool_str(bumps['wheeldrop_left']):<6}  Right: {bool_str(bumps['wheeldrop_right'])}")

    print("\n[CLIFF SENSORS]")
    print(f"  Left:       {bool_str(cliffs['left'])}")
    print(f"  Front-Left: {bool_str(cliffs['front_left'])}")
    print(f"  Front-Right:{bool_str(cliffs['front_right'])}")
    print(f"  Right:      {bool_str(cliffs['right'])}")

    print("\n[BATTERY]")
    print(f"  Voltage:     {battery['voltage_mV']:>5} mV")
    print(f"  Current:     {battery['current_mA']:>5} mA")
    print(f"  Temperature: {battery['temperature_C']:>5} °C")
    print(f"  Charge:      {battery['charge_mAh']:>5} / {battery['capacity_mAh']} mAh  ({battery['charge_pct']}%)")

    print("\n[WHEEL ENCODERS]  (raw counts, wrap at 65535)")
    print(f"  Left:  {encoders['left']:>6}")
    print(f"  Right: {encoders['right']:>6}")

    print(f"\n  Poll #{iteration}   Press Ctrl+C to exit.")
    print("=" * 45)


def main():
    parser = argparse.ArgumentParser(description='Roomba sensor reader')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='Polling interval in seconds (default: 0.5)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.safe_mode()
        time.sleep(0.5)

        iteration = 0
        try:
            while True:
                iteration += 1
                bumps    = roomba.read_bumps()
                cliffs   = roomba.read_cliffs()
                battery  = roomba.read_battery()
                encoders = roomba.read_encoders()

                print_dashboard(bumps, cliffs, battery, encoders, iteration)
                time.sleep(args.interval)

        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == '__main__':
    main()
