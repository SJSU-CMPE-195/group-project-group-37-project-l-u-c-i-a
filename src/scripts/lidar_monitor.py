"""
lidar_monitor.py

Live terminal dashboard for the YDLIDAR G4 sensor.
Displays device info, health, and per-scan statistics.
Mirrors the style of sensor_monitor.py.

Install:
    pip install ydlidar

Usage:
    python3 lidar_monitor.py --port /dev/ttyUSB0
    python3 lidar_monitor.py --port /dev/ttyUSB0 --min-range 80
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


def nearest_in_sector(scan, lo, hi, min_range_mm):
    readings = [
        dist for _, ang, dist in scan
        if dist >= min_range_mm and lo <= ang < hi
    ]
    return min(readings) if readings else None


def compass_nearest(scan, min_range_mm):
    result = {}
    for entry in COMPASS:
        direction = entry[0]
        lo, hi = entry[1], entry[2]
        dist = nearest_in_sector(scan, lo, hi, min_range_mm)
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


def print_dashboard(info, health, scan, scan_num, min_range_mm):
    clear_screen()
    print("=" * 50)
    print("   LUCIA — LiDAR Monitor  (YDLIDAR G4)")
    print("=" * 50)

    print(f"\n[DEVICE]")
    print(f"  Port:        {info.get('port', '?')}")
    print(f"  Baudrate:    {info.get('baudrate', '?')}")
    print(f"  Scan freq:   {info.get('scan_frequency', '?')} Hz")
    print(f"  Sample rate: {info.get('sample_rate', '?')} kSps")
    print(f"  Range:       {info.get('min_range_m', '?')} – {info.get('max_range_m', '?')} m")

    status, err = health
    print(f"\n[HEALTH]  {status}  (error code: {err})")

    if scan:
        distances = [d for _, _, d in scan]
        print(f"\n[SCAN #{scan_num}]")
        print(f"  Points valid:  {len(scan):>5}")
        if distances:
            print(f"  Nearest:  {min(distances):>7.0f} mm")
            print(f"  Farthest: {max(distances):>7.0f} mm")
            print(f"  Average:  {sum(distances)/len(distances):>7.0f} mm")

        print(f"\n[NEAREST PER DIRECTION]  (|####| = 5 m scale)")
        sectors = compass_nearest(scan, min_range_mm)
        for direction in ['N', 'NE', 'E', 'SE', 'S', 'SW', 'W', 'NW']:
            bar = dist_bar(sectors.get(direction))
            print(f"  {direction:<2}  |{bar}|")
    else:
        print("\n  Waiting for scan data...")

    print(f"\n  Press Ctrl+C to exit.")
    print("=" * 50)


def main():
    parser = argparse.ArgumentParser(description='YDLIDAR G4 live monitor')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--min-range', type=int, default=80,
                        help='Minimum valid range in mm (default: 80)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with LidarReader(args.port) as lidar:
        info   = lidar.get_info()
        health = lidar.get_health()
        scan_num = 0

        try:
            for scan in lidar.iter_scans():
                scan_num += 1
                print_dashboard(info, health, scan, scan_num, args.min_range)
        except KeyboardInterrupt:
            print("\nStopped.")


if __name__ == '__main__':
    main()
