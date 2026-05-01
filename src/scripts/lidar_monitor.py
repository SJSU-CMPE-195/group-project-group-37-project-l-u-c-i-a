"""
lidar_monitor.py

Live terminal dashboard for the RPLiDAR sensor.
Displays device info, health, and per-scan statistics.
Mirrors the style of sensor_monitor.py.

Install:
    pip install rplidar-robotics

Usage:
    python3 lidar_monitor.py --port /dev/ttyUSB1
    python3 lidar_monitor.py --port /dev/ttyUSB1 --min-quality 10
"""

import argparse
import os

from lidar_reader import LidarReader

COMPASS = [
    ('N',   337.5, 360.0),
    ('N',     0.0,  22.5),
    ('NE',   22.5,  67.5),
    ('E',    67.5, 112.5),
    ('SE',  112.5, 157.5),
    ('S',   157.5, 202.5),
    ('SW',  202.5, 247.5),
    ('W',   247.5, 292.5),
    ('NW',  292.5, 337.5),
]


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def nearest_in_sector(scan, lo, hi, min_quality):
    readings = [
        dist for q, ang, dist in scan
        if q >= min_quality and dist > 0 and lo <= ang < hi
    ]
    return min(readings) if readings else None


def compass_nearest(scan, min_quality):
    result = {}
    for entry in COMPASS:
        direction = entry[0]
        lo, hi = entry[1], entry[2]
        dist = nearest_in_sector(scan, lo, hi, min_quality)
        if direction in result:
            if dist is not None:
                prev = result[direction]
                result[direction] = dist if prev is None else min(prev, dist)
        else:
            result[direction] = dist
    return result


def dist_bar(dist_mm, max_mm=5000, width=20):
    if dist_mm is None:
        return '.' * width + '  ---'
    filled = int(min(dist_mm, max_mm) / max_mm * width)
    bar = '#' * filled + '-' * (width - filled)
    return f'{bar}  {dist_mm:>5.0f} mm'


def print_dashboard(info, health, scan, scan_num, min_quality):
    clear_screen()
    print("=" * 50)
    print("   LUCIA — LiDAR Monitor")
    print("=" * 50)

    print(f"\n[DEVICE]")
    print(f"  Model:    {info.get('model', '?')}")
    print(f"  Firmware: {info.get('firmware', '?')}")
    print(f"  Serial:   {info.get('serialnumber', '?')}")

    status, err = health
    print(f"\n[HEALTH]  {status}  (error code: {err})")

    if scan:
        valid = [(q, a, d) for q, a, d in scan if q >= min_quality and d > 0]
        distances = [d for _, _, d in valid]
        print(f"\n[SCAN #{scan_num}]")
        print(f"  Points total:  {len(scan):>5}")
        print(f"  Points valid:  {len(valid):>5}  (quality >= {min_quality})")
        if distances:
            print(f"  Nearest:  {min(distances):>7.0f} mm")
            print(f"  Farthest: {max(distances):>7.0f} mm")
            print(f"  Average:  {sum(distances)/len(distances):>7.0f} mm")

        print(f"\n[NEAREST PER DIRECTION]  (|####| = 5 m scale)")
        sectors = compass_nearest(scan, min_quality)
        for direction in ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']:
            bar = dist_bar(sectors.get(direction))
            print(f"  {direction:<2}  |{bar}|")
    else:
        print("\n  Waiting for scan data...")

    print(f"\n  Press Ctrl+C to exit.")
    print("=" * 50)


def main():
    parser = argparse.ArgumentParser(description='RPLiDAR live monitor')
    parser.add_argument('--port', default='/dev/ttyUSB1',
                        help='Serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--min-quality', type=int, default=5,
                        help='Minimum point quality to display (default: 5)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with LidarReader(args.port) as lidar:
        info   = lidar.get_info()
        health = lidar.get_health()
        scan_num = 0

        try:
            for scan in lidar.iter_scans():
                scan_num += 1
                print_dashboard(info, health, scan, scan_num, args.min_quality)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == '__main__':
    main()
