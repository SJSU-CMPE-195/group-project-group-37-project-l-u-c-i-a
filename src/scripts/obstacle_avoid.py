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
import csv
import os
import threading
import time

from lidar_reader import LidarReader
from roomba_oi import RoombaOI

try:
    import smbus2
    import RPi.GPIO as GPIO
    _HAS_UPS = True
except ImportError:
    _HAS_UPS = False

MAX17040_ADDR = 0x36
GPIO_POWER    = 6    # HIGH = AC OK, LOW = AC fail
GPIO_CHARGE   = 16   # LOW  = charging, HIGH = not charging

FORWARD = 'FORWARD'
BLOCKED = 'BLOCKED'
TURNING = 'TURNING'

SPIN_SPEED = 150   # mm/s, wheels opposite directions
SPIN_TIME  = 0.8   # seconds per turn burst (~55° at 150 mm/s)


def read_ups():
    """Read UPS voltage and state-of-charge from the MAX17040 over I2C."""
    if not _HAS_UPS:
        return {}
    result = {}
    try:
        bus   = smbus2.SMBus(1)
        vcell = bus.read_i2c_block_data(MAX17040_ADDR, 0x02, 2)
        soc   = bus.read_i2c_block_data(MAX17040_ADDR, 0x04, 2)
        bus.close()
        result['voltage'] = ((vcell[0] << 8 | vcell[1]) >> 4) * 1.25 / 1000
        result['soc']     = soc[0] + soc[1] / 256.0
    except Exception:
        pass
    try:
        result['power_ok'] = bool(GPIO.input(GPIO_POWER))
        result['charging'] = not bool(GPIO.input(GPIO_CHARGE))
    except Exception:
        pass
    return result


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


def print_status(state, front_mm, left_mm, right_mm, scan_n, safe_dist, ups, ups_warn, ups_stop):
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

    soc = ups.get('soc')
    if soc is not None:
        flag = ' *** LOW BATTERY ***' if soc < ups_warn else ''
        print(f"\n  UPS:      {soc:.1f}%  {ups.get('voltage', 0):.3f} V{flag}")
        if ups.get('charging') is not None:
            chg = 'Charging' if ups['charging'] else 'On battery'
            pwr = 'AC OK' if ups.get('power_ok') else 'AC FAIL'
            print(f"            {pwr}  {chg}")
    elif _HAS_UPS:
        print(f"\n  UPS:      (no data)")

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
    parser.add_argument('--log',          default=None,
                        help='Path to write CSV sensor log (e.g. run.csv)')
    parser.add_argument('--ups-warn',     type=int, default=20,
                        help='UPS %% below which to show a warning (default: 20)')
    parser.add_argument('--ups-stop',     type=int, default=10,
                        help='UPS %% below which to stop the robot (default: 10)')
    args = parser.parse_args()

    if _HAS_UPS:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_POWER,  GPIO.IN)
        GPIO.setup(GPIO_CHARGE, GPIO.IN)

    shared     = {'scan': []}
    lock       = threading.Lock()
    stop_event = threading.Event()

    log_file   = None
    log_writer = None
    if args.log:
        log_file   = open(args.log, 'w', newline='')
        log_writer = csv.writer(log_file)
        log_writer.writerow(['timestamp', 'scan_n', 'state', 'front_mm', 'left_mm', 'right_mm', 'ups_soc', 'ups_voltage'])

    print(f"Connecting to LiDAR on  {args.lidar_port}...")
    print(f"Connecting to Roomba on {args.roomba_port}...")

    with LidarReader(args.lidar_port) as lidar, RoombaOI(args.roomba_port) as roomba:
        print("Waking Roomba...")
        roomba.wake()
        roomba.start()

        battery = roomba.read_battery()
        if battery['voltage_mV'] < 10000:
            print(f"ERROR: Roomba appears to be off or not responding "
                  f"(voltage: {battery['voltage_mV']} mV). "
                  f"Power it on and try again.")
            raise SystemExit(1)

        print(f"Roomba online — battery {battery['charge_pct']}% "
              f"({battery['voltage_mV']} mV)")

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

                ups = read_ups()
                ups_soc = ups.get('soc')
                if ups_soc is not None and ups_soc < args.ups_stop:
                    roomba.stop()
                    print(f"\nUPS battery critical ({ups_soc:.1f}%) — stopping.")
                    break

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

                print_status(state, front_mm, left_mm, right_mm, scan_n, args.safe_dist,
                             ups, args.ups_warn, args.ups_stop)
                if log_writer:
                    log_writer.writerow([
                        f'{time.time():.3f}', scan_n, state,
                        f'{front_mm:.0f}' if front_mm is not None else '',
                        f'{left_mm:.0f}'  if left_mm  is not None else '',
                        f'{right_mm:.0f}' if right_mm is not None else '',
                        f'{ups_soc:.1f}'  if ups_soc  is not None else '',
                        f'{ups.get("voltage", ""):.3f}' if ups.get('voltage') is not None else '',
                    ])
                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            stop_event.set()
            roomba.stop()
            if log_file:
                log_file.close()
                print(f"Log saved to {args.log}")
            if _HAS_UPS:
                GPIO.cleanup()


if __name__ == '__main__':
    main()
