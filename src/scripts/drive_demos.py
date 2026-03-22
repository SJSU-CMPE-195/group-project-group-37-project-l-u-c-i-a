"""
drive_demos.py

Automated drive patterns built on top of RoombaOI.
Provides timed straight movement, in-place turns, and demo sequences.

Note: timing-based movement is approximate. Accuracy depends on
battery level, surface friction, and wheel slip. Use encoder
odometry (see sensor_monitor.py) for more precise positioning.

Usage:
    python drive_demos.py --port COM5 --demo square
    python drive_demos.py --port COM5 --demo figure_eight
    python drive_demos.py --port /dev/ttyUSB0 --demo square
"""

import argparse
import math
import time

from roomba_oi import RoombaOI

# Physical constants for the Roomba 650
WHEEL_BASE_MM = 235  # distance between left and right wheel contact points


def forward(roomba, speed_mm_s, distance_mm):
    """Drive straight forward a given distance."""
    duration = distance_mm / abs(speed_mm_s)
    roomba.drive(abs(speed_mm_s), 32768)
    time.sleep(duration)
    roomba.stop()


def backward(roomba, speed_mm_s, distance_mm):
    """Drive straight backward a given distance."""
    duration = distance_mm / abs(speed_mm_s)
    roomba.drive(-abs(speed_mm_s), 32768)
    time.sleep(duration)
    roomba.stop()


def turn_left(roomba, speed_mm_s, degrees):
    """Spin counter-clockwise in place by a given number of degrees."""
    arc = math.radians(degrees) * (WHEEL_BASE_MM / 2)
    duration = arc / abs(speed_mm_s)
    roomba.drive(abs(speed_mm_s), 1)  # radius=1 → spin CCW
    time.sleep(duration)
    roomba.stop()


def turn_right(roomba, speed_mm_s, degrees):
    """Spin clockwise in place by a given number of degrees."""
    arc = math.radians(degrees) * (WHEEL_BASE_MM / 2)
    duration = arc / abs(speed_mm_s)
    roomba.drive(abs(speed_mm_s), -1)  # radius=-1 → spin CW
    time.sleep(duration)
    roomba.stop()


# ------------------------------------------------------------------
# Demo patterns
# ------------------------------------------------------------------

def demo_square(roomba, side_mm=600, speed_mm_s=200):
    """Drive a square: four forward legs with 90-degree left turns."""
    print("Demo: square")
    for i in range(4):
        print(f"  Side {i + 1}")
        forward(roomba, speed_mm_s, side_mm)
        time.sleep(0.3)
        turn_left(roomba, 150, 90)
        time.sleep(0.3)
    print("Done.")


def demo_figure_eight(roomba, speed_mm_s=200):
    """
    Drive a rough figure-eight using arced turns.
    Each circle is ~600 mm in diameter (radius=300).
    """
    print("Demo: figure eight")
    radius = 300
    circumference = 2 * math.pi * radius
    duration = circumference / speed_mm_s

    print("  Left loop")
    roomba.drive(speed_mm_s, radius)
    time.sleep(duration)
    roomba.stop()
    time.sleep(0.3)

    print("  Right loop")
    roomba.drive(speed_mm_s, -radius)
    time.sleep(duration)
    roomba.stop()
    print("Done.")


# ------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='Roomba automated drive demos')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    parser.add_argument('--demo', choices=['square', 'figure_eight'],
                        default='square',
                        help='Which demo to run (default: square)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)

        if args.demo == 'square':
            demo_square(roomba)
        elif args.demo == 'figure_eight':
            demo_figure_eight(roomba)


if __name__ == '__main__':
    main()
