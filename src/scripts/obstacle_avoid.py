"""
obstacle_avoid.py

Obstacle avoidance tech demo using the RPLiDAR and Roomba 650.
The Roomba drives forward autonomously and spins away from anything
detected within --safe-dist mm in the forward arc.

Runs in safe mode so cliff and wheel-drop sensors remain active.

Install:
    pip install rplidar pyserial

Usage:
    python3 obstacle_avoid.py
    python3 obstacle_avoid.py --roomba-port /dev/ttyUSB0 --lidar-port /dev/ttyUSB1
    python3 obstacle_avoid.py --safe-dist 800 --speed 200

Note:
    Angle 0° is wherever the LiDAR's front marker points.
    Adjust --fov or physically orient the LiDAR so 0° faces the
    same direction as the Roomba's front bumper.
"""

import argparse
import os
import threading
import time

from lidar_reader import LidarReader
from roomba_oi import RoombaOI

FORWARD = 'FORWARD'
BLOCKED = 'BLOCKED'
TURNING = 'TURNING'

SPIN_SPEED = 150   # mm/s, wheels opposite directions
SPIN_TIME  = 0.8   # seconds per turn burst (~55° at 150 mm/s)


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def sector_min(scan, lo, hi, min_quality):
    readings = [
        dist for q, ang, dist in scan
        if q >= min_quality and dist > 0 and lo <= ang < hi
    ]
    return min(readings) if readings else None


def get_front(scan, fov, min_quality):
    """Nearest return in the forward arc (straddles 0°)."""
    readings = []
    for q, ang, dist in scan:
        if q < min_quality or dist == 0:
            continue
        if ang >= (360 - fov) or ang < fov:
            readings.append(dist)
    return min(readings) if readings else None


def get_sides(scan, min_quality):
    """Nearest return in left (240–300°) and right (60–120°) sectors."""
    left  = sector_min(scan, 240, 300, min_quality)
    right = sector_min(scan,  60, 120, min_quality)
    return left, right


def lidar_worker(lidar, shared, lock, stop_event):
    try:
        for scan in lidar.iter_scans():
            if stop_event.is_set():
                break
            with lock:
                shared['scan'] = scan
    except Exception:
        pass


def print_status(state, front_mm, left_mm, right_mm, scan_n, safe_dist):
    clear_screen()
    print("=" * 45)
    print("   LUCIA — Obstacle Avoidance Demo")
    print("=" * 45)

    def fmt(v):
        return f"{v:.0f} mm" if v is not None else "  ---"

    print(f"\n  State:    {state}")
    print(f"  Front:    {fmt(front_mm)}  (threshold: {safe_dist} mm)")
    print(f"  Left:     {fmt(left_mm)}")
    print(f"  Right:    {fmt(right_mm)}")
    print(f"\n  Scans:    {scan_n}")
    print(f"\n  Ctrl+C to stop.")
    print("=" * 45)


def main():
    parser = argparse.ArgumentParser(description='Roomba obstacle avoidance demo')
    parser.add_argument('--roomba-port',  default='/dev/ttyUSB0',
                        help='Roomba serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--lidar-port',   default='/dev/ttyUSB1',
                        help='LiDAR serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--speed',        type=int, default=200,
                        help='Forward speed in mm/s (default: 200)')
    parser.add_argument('--safe-dist',    type=int, default=600,
                        help='Stop threshold in mm (default: 600)')
    parser.add_argument('--fov',          type=int, default=30,
                        help='Forward detection arc ±degrees (default: 30)')
    parser.add_argument('--min-quality',  type=int, default=5,
                        help='Minimum LiDAR point quality (default: 5)')
    args = parser.parse_args()

    shared     = {'scan': []}
    lock       = threading.Lock()
    stop_event = threading.Event()

    print(f"Connecting to LiDAR on  {args.lidar_port}...")
    print(f"Connecting to Roomba on {args.roomba_port}...")

    with LidarReader(args.lidar_port) as lidar, RoombaOI(args.roomba_port) as roomba:
        roomba.start()
        roomba.safe_mode()   # keeps cliff + wheel-drop safety stops active
        time.sleep(0.5)

        lt = threading.Thread(
            target=lidar_worker,
            args=(lidar, shared, lock, stop_event),
            daemon=True,
        )
        lt.start()

        print("Waiting for first LiDAR scan...")
        while True:
            with lock:
                if shared['scan']:
                    break
            time.sleep(0.1)

        state      = FORWARD
        scan_n     = 0
        turn_start = 0.0
        turn_dir   = 1   # +1 = left, -1 = right

        try:
            while True:
                with lock:
                    scan = list(shared['scan'])

                front_mm = left_mm = right_mm = None
                if scan:
                    scan_n  += 1
                    front_mm = get_front(scan, args.fov, args.min_quality)
                    left_mm, right_mm = get_sides(scan, args.min_quality)

                # ---- state machine ----

                if state == FORWARD:
                    if front_mm is not None and front_mm <= args.safe_dist:
                        roomba.stop()
                        state = BLOCKED
                    else:
                        roomba.drive_direct(args.speed, args.speed)

                elif state == BLOCKED:
                    left_clear  = left_mm  if left_mm  is not None else 9999
                    right_clear = right_mm if right_mm is not None else 9999
                    # spin toward the side with more clearance
                    if left_clear >= right_clear:
                        roomba.drive_direct(-SPIN_SPEED, SPIN_SPEED)   # spin left
                        turn_dir = 1
                    else:
                        roomba.drive_direct(SPIN_SPEED, -SPIN_SPEED)   # spin right
                        turn_dir = -1
                    state      = TURNING
                    turn_start = time.time()

                elif state == TURNING:
                    if time.time() - turn_start >= SPIN_TIME:
                        roomba.stop()
                        if front_mm is None or front_mm > args.safe_dist:
                            state = FORWARD
                        else:
                            state = BLOCKED   # still blocked — turn again

                print_status(state, front_mm, left_mm, right_mm, scan_n, args.safe_dist)
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            stop_event.set()
            roomba.stop()


if __name__ == '__main__':
    main()
