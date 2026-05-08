"""
slam_avoid.py

SLAM + obstacle avoidance — extends obstacle_avoid.py.
Runs the same reactive avoidance behaviour (FORWARD / BLOCKED / TURNING)
and adds real-time 2D occupancy-grid mapping using breezyslam with LiDAR
scans and dead-reckoning odometry from the Roomba's wheel encoders.

Saves a map PNG on Ctrl+C exit. obstacle_avoid.py is left untouched.

Install:
    pip install breezyslam rplidar-roboticia pyserial Pillow

Usage:
    python3 slam_avoid.py
    python3 slam_avoid.py --roomba-port /dev/ttyUSB0 --lidar-port /dev/ttyUSB1
    python3 slam_avoid.py --map-out run.png --map-size 10 --map-pixels 500

Note:
    Angle 0° is wherever the LiDAR's front marker points.
    Adjust --fov or physically orient the LiDAR so 0° faces the
    same direction as the Roomba's front bumper.
"""

import argparse
import csv
import math
import os
import threading
import time

from lidar_reader import LidarReader
from roomba_oi import RoombaOI

try:
    from breezyslam.algorithms import RMHC_SLAM
    from breezyslam.sensors import Laser
    _HAS_SLAM = True
except ImportError:
    _HAS_SLAM = False

try:
    from PIL import Image, ImageDraw
    _HAS_PIL = True
except ImportError:
    _HAS_PIL = False

try:
    import smbus2
    import RPi.GPIO as GPIO
    _HAS_UPS = True
except ImportError:
    _HAS_UPS = False

# Roomba 650 drivetrain — iRobot OI spec
WHEEL_DIAMETER_MM = 72.2
WHEEL_BASE_MM     = 235.0
TICKS_PER_REV     = 508.8
MM_PER_TICK       = (math.pi * WHEEL_DIAMETER_MM) / TICKS_PER_REV

MAX17040_ADDR = 0x36
GPIO_POWER    = 6    # HIGH = AC OK
GPIO_CHARGE   = 16   # LOW  = charging

FORWARD    = 'FORWARD'
BLOCKED    = 'BLOCKED'
TURNING    = 'TURNING'
SPIN_SPEED = 150   # mm/s
SPIN_TIME  = 0.8   # seconds per turn burst
SCAN_SLOTS = 360   # angular resolution fed to breezyslam


# -----------------------------------------------------------------------
# Odometry
# -----------------------------------------------------------------------

def _tick_delta(prev, curr):
    """Handle 16-bit unsigned encoder wraparound (0–65535)."""
    d = curr - prev
    if d >  32767: d -= 65536
    elif d < -32768: d += 65536
    return d


class Odometry:
    """Dead-reckoning 2-D pose from Roomba wheel encoder counts."""

    def __init__(self):
        self.x           = 0.0   # mm
        self.y           = 0.0   # mm
        self.heading_rad = 0.0
        self._prev_left  = None
        self._prev_right = None
        # Incremental values consumed by breezyslam each update
        self.d_xy_mm     = 0.0
        self.d_theta_deg = 0.0

    def update(self, left_count, right_count):
        if self._prev_left is None:
            self._prev_left, self._prev_right = left_count, right_count
            return

        dl = _tick_delta(self._prev_left,  left_count)  * MM_PER_TICK
        dr = _tick_delta(self._prev_right, right_count) * MM_PER_TICK
        self._prev_left, self._prev_right = left_count, right_count

        dist   = (dl + dr) / 2.0
        dtheta = (dr - dl) / WHEEL_BASE_MM

        self.heading_rad += dtheta
        self.x           += dist * math.cos(self.heading_rad)
        self.y           += dist * math.sin(self.heading_rad)
        self.d_xy_mm      = dist
        self.d_theta_deg  = math.degrees(dtheta)

    def heading_deg(self):
        return math.degrees(self.heading_rad) % 360


# -----------------------------------------------------------------------
# SLAM helpers
# -----------------------------------------------------------------------

def _path_out(map_out):
    """Derive the path-overlay filename from the base map filename."""
    base, ext = os.path.splitext(map_out)
    return f"{base}_path{ext or '.png'}"


def world_to_pixel(x_mm, y_mm, map_pixels, map_size_m):
    """Convert SLAM world coordinates (mm) to image pixel coordinates."""
    scale = map_pixels / (map_size_m * 1000)
    px = int(x_mm * scale)
    py = map_pixels - 1 - int(y_mm * scale)  # flip y: image origin is top-left
    return px, py


def scan_to_distances(scan, num_slots, min_quality):
    """Bin a (quality, angle_deg, dist_mm) list into a flat distance array."""
    distances = [0] * num_slots
    for q, ang, dist in scan:
        if q < min_quality or dist == 0:
            continue
        idx = int(ang) % num_slots
        if distances[idx] == 0 or dist < distances[idx]:
            distances[idx] = int(dist)
    return distances


def save_maps(slam, map_out, pixels, map_size_m, path_pixels):
    """
    Save two files:
      <map_out>            — raw occupancy grid (white=free, black=obstacle, gray=unknown)
      <map_out stem>_path  — same grid with robot path overlaid in colour
                             (green dot = start, cyan trail = path, red dot = final position)
    """
    mapbytes = bytearray(pixels * pixels)
    slam.getmap(mapbytes)

    if not _HAS_PIL:
        raw = map_out.rsplit('.', 1)[0] + '.bin'
        with open(raw, 'wb') as f:
            f.write(mapbytes)
        print(f"Pillow not installed — raw map bytes saved → {raw}")
        return

    img_raw = Image.frombytes('L', (pixels, pixels), bytes(mapbytes))
    img_raw.save(map_out)
    print(f"Map saved → {map_out}")

    if not path_pixels:
        return

    img_path = img_raw.convert('RGB')
    draw     = ImageDraw.Draw(img_path)
    r        = 2   # dot radius in pixels

    for i, (px, py) in enumerate(path_pixels):
        if i == 0:
            color = (0, 220, 0)      # green — start
        elif i == len(path_pixels) - 1:
            color = (255, 60, 60)    # red — final position
        else:
            color = (0, 200, 255)    # cyan — path trail

        draw.ellipse([px - r, py - r, px + r, py + r], fill=color)

    out = _path_out(map_out)
    img_path.save(out)
    print(f"Map with path saved → {out}")


# -----------------------------------------------------------------------
# Sensor helpers (mirrors obstacle_avoid.py)
# -----------------------------------------------------------------------

def get_front(scan, fov, min_quality):
    readings = [
        dist for q, ang, dist in scan
        if q >= min_quality and dist > 0
        and (ang >= (360 - fov) or ang < fov)
    ]
    return min(readings) if readings else None


def _sector_min(scan, lo, hi, min_quality):
    readings = [
        dist for q, ang, dist in scan
        if q >= min_quality and dist > 0 and lo <= ang < hi
    ]
    return min(readings) if readings else None


def get_sides(scan, min_quality):
    return (
        _sector_min(scan, 240, 300, min_quality),
        _sector_min(scan,  60, 120, min_quality),
    )


# -----------------------------------------------------------------------
# UPS (mirrors obstacle_avoid.py)
# -----------------------------------------------------------------------

def read_ups():
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


# -----------------------------------------------------------------------
# Display
# -----------------------------------------------------------------------

# Radar grid dimensions — chosen so characters form a near-circle
# (terminal chars are ~2× taller than wide, so cols = rows * 2 - 1)
_RADAR_ROWS   = 21
_RADAR_COLS   = 41
_RADAR_R_ROWS = _RADAR_ROWS // 2   # radius in row units
_RADAR_RANGE  = 3000               # mm shown from center to edge


def render_scan(scan, min_quality, safe_dist):
    """
    Return an ASCII polar radar string showing the current LiDAR scan.

    Legend:
      o  robot (center)
      ^  forward direction marker
      .  range rings at 1 m and 2 m
      *  obstacle outside safe_dist
      #  obstacle inside safe_dist (danger zone)
    """
    cr = _RADAR_ROWS // 2
    cc = _RADAR_COLS // 2

    grid = [[' '] * _RADAR_COLS for _ in range(_RADAR_ROWS)]

    # Range rings at 1 m, 2 m, and 3 m (outer edge)
    for ring_mm in (1000, 2000, 3000):
        frac = ring_mm / _RADAR_RANGE
        for deg in range(0, 360, 3):
            rad = math.radians(deg)
            r = int(round(cr - frac * _RADAR_R_ROWS * math.cos(rad)))
            c = int(round(cc + frac * _RADAR_R_ROWS * 2 * math.sin(rad)))
            if 0 <= r < _RADAR_ROWS and 0 <= c < _RADAR_COLS:
                if grid[r][c] == ' ':
                    grid[r][c] = '.'

    # Forward marker
    grid[0][cc] = '^'

    # Scan points
    for q, ang, dist in scan:
        if q < min_quality or dist == 0 or dist > _RADAR_RANGE:
            continue
        frac = dist / _RADAR_RANGE
        rad  = math.radians(ang)
        r = int(round(cr - frac * _RADAR_R_ROWS * math.cos(rad)))
        c = int(round(cc + frac * _RADAR_R_ROWS * 2 * math.sin(rad)))
        if 0 <= r < _RADAR_ROWS and 0 <= c < _RADAR_COLS:
            grid[r][c] = '#' if dist <= safe_dist else '*'

    # Robot at center
    grid[cr][cc] = 'o'

    lines = ['  ' + ''.join(row) for row in grid]
    lines.append(f"  {'<-- 1m -->':^{_RADAR_COLS}}  (# = danger  * = clear  ^ = fwd)")
    return '\n'.join(lines)


def clear_screen():
    os.system('cls' if os.name == 'nt' else 'clear')


def print_status(state, front_mm, left_mm, right_mm, scan_n, safe_dist,
                 odom, slam_active, ups, ups_warn, ups_stop,
                 scan=None, min_quality=5):
    clear_screen()
    print("=" * 50)
    print("   LUCIA — SLAM + Obstacle Avoidance")
    print("=" * 50)

    def fmt(v):
        return f"{v:.0f} mm" if v is not None else "  ---"

    print(f"\n  State:  {state}")
    print(f"  Front:  {fmt(front_mm)}  (threshold: {safe_dist} mm)")
    print(f"  Left:   {fmt(left_mm)}")
    print(f"  Right:  {fmt(right_mm)}")
    print(f"\n  Scans:  {scan_n}")
    print(f"  Pose:   x={odom.x:+.0f} mm  y={odom.y:+.0f} mm  hdg={odom.heading_deg():.1f}°")
    print(f"  SLAM:   {'active' if slam_active else 'inactive (pip install breezyslam)'}")

    soc = ups.get('soc')
    if soc is not None:
        flag = ' *** LOW BATTERY ***' if soc < ups_warn else ''
        print(f"\n  UPS:    {soc:.1f}%  {ups.get('voltage', 0):.3f} V{flag}")
        if ups.get('charging') is not None:
            chg = 'Charging' if ups['charging'] else 'On battery'
            pwr = 'AC OK' if ups.get('power_ok') else 'AC FAIL'
            print(f"          {pwr}  {chg}")
    elif _HAS_UPS:
        print(f"\n  UPS:    (no data)")

    if scan:
        print()
        print(render_scan(scan, min_quality, safe_dist))

    print(f"\n  Ctrl+C to stop and save map.")
    print("=" * 50)


# -----------------------------------------------------------------------
# LiDAR worker thread
# -----------------------------------------------------------------------

def lidar_worker(lidar, shared, lock, stop_event):
    try:
        for scan in lidar.iter_scans():
            if stop_event.is_set():
                break
            with lock:
                shared['scan'] = scan
    except Exception:
        pass


# -----------------------------------------------------------------------
# Main
# -----------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='SLAM + obstacle avoidance demo')
    parser.add_argument('--roomba-port', default='/dev/ttyUSB0',
                        help='Roomba serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--lidar-port',  default='/dev/ttyUSB1',
                        help='LiDAR serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--speed',       type=int,   default=200,
                        help='Forward speed in mm/s (default: 200)')
    parser.add_argument('--safe-dist',   type=int,   default=600,
                        help='Stop threshold in mm (default: 600)')
    parser.add_argument('--fov',         type=int,   default=30,
                        help='Forward detection arc ±degrees (default: 30)')
    parser.add_argument('--min-quality', type=int,   default=5,
                        help='Minimum LiDAR point quality (default: 5)')
    parser.add_argument('--map-out',     default='map.png',
                        help='Output file for SLAM map on exit (default: map.png)')
    parser.add_argument('--map-size',    type=float, default=10.0,
                        help='Map coverage in meters (default: 10)')
    parser.add_argument('--map-pixels',  type=int,   default=500,
                        help='Map resolution in pixels (default: 500)')
    parser.add_argument('--log',         default=None,
                        help='Path to write CSV sensor log')
    parser.add_argument('--ups-warn',    type=int,   default=20,
                        help='UPS %% below which to show a warning (default: 20)')
    parser.add_argument('--ups-stop',    type=int,   default=10,
                        help='UPS %% below which to stop the robot (default: 10)')
    args = parser.parse_args()

    if not _HAS_SLAM:
        print("WARNING: breezyslam not installed — SLAM disabled, avoidance only.")
        print("         pip install breezyslam")

    if _HAS_UPS:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_POWER,  GPIO.IN)
        GPIO.setup(GPIO_CHARGE, GPIO.IN)

    # --- SLAM init ---
    slam = None
    if _HAS_SLAM:
        laser = Laser(
            scan_size=SCAN_SLOTS,
            scan_rate_hz=10,
            detection_angle_degrees=360,
            distance_no_detection_mm=int(args.map_size * 1000),
            offset_mm=0,
        )
        slam = RMHC_SLAM(laser, args.map_pixels, args.map_size)

    odom        = Odometry()
    path_pixels = []   # (px, py) pixel coords of each SLAM pose sample
    shared      = {'scan': []}
    lock       = threading.Lock()
    stop_event = threading.Event()

    log_file = log_writer = None
    if args.log:
        log_file   = open(args.log, 'w', newline='')
        log_writer = csv.writer(log_file)
        log_writer.writerow([
            'timestamp', 'scan_n', 'state',
            'front_mm', 'left_mm', 'right_mm',
            'pose_x', 'pose_y', 'pose_hdg',
            'ups_soc', 'ups_voltage',
        ])

    print(f"Connecting to LiDAR on  {args.lidar_port}...")
    print(f"Connecting to Roomba on {args.roomba_port}...")

    with LidarReader(args.lidar_port) as lidar, RoombaOI(args.roomba_port) as roomba:
        print("Waking Roomba...")
        roomba.wake()
        roomba.start()

        battery = roomba.read_battery()
        if battery['voltage_mV'] < 10000:
            print(f"ERROR: Roomba not responding (voltage: {battery['voltage_mV']} mV).")
            raise SystemExit(1)

        print(f"Roomba online — battery {battery['charge_pct']}% ({battery['voltage_mV']} mV)")
        roomba.safe_mode()
        time.sleep(0.5)

        # Seed encoder baseline so first delta is zero
        enc = roomba.read_encoders()
        odom.update(enc['left'], enc['right'])

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

        state        = FORWARD
        scan_n       = 0
        turn_start   = 0.0
        prev_iter_t  = time.time()
        last_scan_id = None   # detect new scans from the lidar thread

        try:
            while True:
                t0      = time.time()
                loop_dt = t0 - prev_iter_t   # true elapsed since last slam.update()
                prev_iter_t = t0

                with lock:
                    raw_scan  = shared['scan']
                    scan_id   = id(raw_scan)
                    scan      = list(raw_scan)

                # --- Odometry ---
                try:
                    enc = roomba.read_encoders()
                    odom.update(enc['left'], enc['right'])
                except Exception:
                    pass  # keep last known delta; serial glitch won't crash the loop

                # --- SLAM update ---
                if slam is not None and scan:
                    distances = scan_to_distances(scan, SCAN_SLOTS, args.min_quality)
                    slam.update(distances, pose_change=(
                        odom.d_xy_mm, odom.d_theta_deg, loop_dt
                    ))
                    x_mm, y_mm, _ = slam.getpos()
                    px, py = world_to_pixel(x_mm, y_mm, args.map_pixels, args.map_size)
                    if 0 <= px < args.map_pixels and 0 <= py < args.map_pixels:
                        if not path_pixels or path_pixels[-1] != (px, py):
                            path_pixels.append((px, py))

                # --- Sensor extraction ---
                front_mm = left_mm = right_mm = None
                if scan:
                    if scan_id != last_scan_id:
                        scan_n      += 1
                        last_scan_id = scan_id
                    front_mm = get_front(scan, args.fov, args.min_quality)
                    left_mm, right_mm = get_sides(scan, args.min_quality)

                ups     = read_ups()
                ups_soc = ups.get('soc')
                if ups_soc is not None and ups_soc < args.ups_stop:
                    roomba.stop()
                    print(f"\nUPS critical ({ups_soc:.1f}%) — stopping.")
                    break

                # --- State machine (identical logic to obstacle_avoid.py) ---

                if state == FORWARD:
                    if front_mm is not None and front_mm <= args.safe_dist:
                        roomba.stop()
                        state = BLOCKED
                    else:
                        roomba.drive_direct(args.speed, args.speed)

                elif state == BLOCKED:
                    left_clear  = left_mm  if left_mm  is not None else 9999
                    right_clear = right_mm if right_mm is not None else 9999
                    if left_clear >= right_clear:
                        roomba.drive_direct(-SPIN_SPEED, SPIN_SPEED)
                    else:
                        roomba.drive_direct(SPIN_SPEED, -SPIN_SPEED)
                    state      = TURNING
                    turn_start = time.time()

                elif state == TURNING:
                    if time.time() - turn_start >= SPIN_TIME:
                        roomba.stop()
                        state = FORWARD if (front_mm is None or front_mm > args.safe_dist) else BLOCKED

                print_status(state, front_mm, left_mm, right_mm, scan_n, args.safe_dist,
                             odom, slam is not None, ups, args.ups_warn, args.ups_stop,
                             scan=scan, min_quality=args.min_quality)

                if log_writer:
                    log_writer.writerow([
                        f'{time.time():.3f}', scan_n, state,
                        f'{front_mm:.0f}' if front_mm is not None else '',
                        f'{left_mm:.0f}'  if left_mm  is not None else '',
                        f'{right_mm:.0f}' if right_mm is not None else '',
                        f'{odom.x:.1f}', f'{odom.y:.1f}', f'{odom.heading_deg():.1f}',
                        f'{ups_soc:.1f}'  if ups_soc             is not None else '',
                        f'{ups.get("voltage", ""):.3f}' if ups.get('voltage') is not None else '',
                    ])

                elapsed = time.time() - t0
                sleep_t = 0.05 - elapsed
                if sleep_t > 0:
                    time.sleep(sleep_t)

        except KeyboardInterrupt:
            print("\nStopped.")
        finally:
            stop_event.set()
            roomba.stop()
            if slam is not None:
                save_maps(slam, args.map_out, args.map_pixels, args.map_size, path_pixels)
            if log_file:
                log_file.close()
                print(f"Log saved → {args.log}")
            if _HAS_UPS:
                GPIO.cleanup()


if __name__ == '__main__':
    main()
