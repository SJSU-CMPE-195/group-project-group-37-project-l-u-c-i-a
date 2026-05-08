"""
slam_avoid_server.py

Web-based SLAM + obstacle avoidance control panel.
Same robot logic as slam_avoid.py — adds a FastAPI web server so you
can monitor and drive the Roomba from any browser on the same network.

Install:
    pip install fastapi "uvicorn[standard]" breezyslam rplidar-roboticia pyserial Pillow

Usage:
    python3 slam_avoid_server.py
    python3 slam_avoid_server.py --roomba-port /dev/roomba --lidar-port /dev/rplidar

Then open http://<pi-ip>:8000 in any browser on the same network.
"""

import argparse
import asyncio
import io
import json
import math
import os
import threading
import time
from contextlib import asynccontextmanager

try:
    import uvicorn
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import HTMLResponse, Response
except ImportError:
    raise SystemExit("Missing deps — run: pip install fastapi 'uvicorn[standard]'")

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
GPIO_POWER    = 6
GPIO_CHARGE   = 16

SCAN_SLOTS = 360
SPIN_SPEED = 150
SPIN_TIME  = 0.8

FORWARD = 'FORWARD'
BLOCKED = 'BLOCKED'
TURNING = 'TURNING'
BUMPED  = 'BUMPED'
IDLE    = 'IDLE'

BUMP_BACKUP_SPEED = -150   # mm/s — reverse after a bump
BUMP_BACKUP_TIME  = 0.5    # seconds to back up
BUMP_TURN_TIME    = 0.7    # seconds to spin away from the hit side


# -----------------------------------------------------------------------
# Odometry
# -----------------------------------------------------------------------

def _tick_delta(prev, curr):
    d = curr - prev
    if d >  32767: d -= 65536
    elif d < -32768: d += 65536
    return d


class Odometry:
    def __init__(self):
        self.x            = 0.0
        self.y            = 0.0
        self.heading_rad  = 0.0
        self._prev_left   = None
        self._prev_right  = None
        self.d_xy_mm      = 0.0
        self.d_theta_deg  = 0.0

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
# Sensor helpers
# -----------------------------------------------------------------------

def scan_to_distances(scan, num_slots, min_quality):
    distances = [0] * num_slots
    for q, ang, dist in scan:
        if q < min_quality or dist == 0:
            continue
        idx = int(ang) % num_slots
        if distances[idx] == 0 or dist < distances[idx]:
            distances[idx] = int(dist)
    return distances


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
# UPS
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
# Shared state
# -----------------------------------------------------------------------

class SharedState:
    def __init__(self):
        self._lock = threading.Lock()
        self._snap = {
            'status':      'connecting',   # connecting / waiting / running / error
            'mode':        'auto',         # auto / manual
            'drive_state': IDLE,
            'front_mm':    None,
            'left_mm':     None,
            'right_mm':    None,
            'scan_n':      0,
            'pose_x':      0.0,
            'pose_y':      0.0,
            'pose_hdg':    0.0,
            'slam_active': _HAS_SLAM,
            'battery_pct': None,
            'battery_mv':  None,
            'ups':         {},
            'scan':        [],
            'bump_left':   False,
            'bump_right':  False,
            'error':       '',
        }
        self._manual_l = 0
        self._manual_r = 0

        self.go_event   = threading.Event()
        self.stop_event = threading.Event()
        self.quit_event = threading.Event()

        self.slam        = None
        self.map_pixels  = 500
        self.map_size_m  = 10.0
        self.path_pixels = []

        # Shared LiDAR data — written by lidar_manager, read by robot_main and lidar_only_main
        self.lidar_shared = {'scan': []}
        self.lidar_lock   = threading.Lock()

    def snapshot(self):
        with self._lock:
            return dict(self._snap)

    def update(self, **kw):
        with self._lock:
            self._snap.update(kw)

    def get_mode(self):
        with self._lock:
            return self._snap['mode']

    def set_mode(self, mode):
        with self._lock:
            self._snap['mode'] = mode

    def set_manual_vel(self, left, right):
        with self._lock:
            self._manual_l = left
            self._manual_r = right

    def get_manual_vel(self):
        with self._lock:
            return self._manual_l, self._manual_r


# -----------------------------------------------------------------------
# LiDAR manager — single thread that owns the port
# -----------------------------------------------------------------------

def lidar_manager(args, state: SharedState):
    """Opens /dev/rplidar once and keeps it open. Reconnects on error.
    robot_main and lidar_only_main both read from state.lidar_shared."""
    while not state.quit_event.is_set():
        try:
            with LidarReader(args.lidar_port) as lidar:
                # Stop any in-progress scan and flush the serial buffer before
                # starting fresh — prevents "Descriptor length mismatch" when
                # the motor was already spinning from a previous session.
                lidar._lidar.stop()
                lidar._lidar.stop_motor()
                time.sleep(1)
                for scan in lidar.iter_scans():
                    if state.quit_event.is_set():
                        return
                    with state.lidar_lock:
                        state.lidar_shared['scan'] = scan
        except Exception as e:
            state.update(error=f'LiDAR: {e}')
            time.sleep(3)   # brief pause before reconnect attempt


# -----------------------------------------------------------------------
# Robot thread
# -----------------------------------------------------------------------

def robot_main(args, state: SharedState):
    if _HAS_UPS:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_POWER,  GPIO.IN)
        GPIO.setup(GPIO_CHARGE, GPIO.IN)

    if _HAS_SLAM:
        laser = Laser(
            scan_size=SCAN_SLOTS,
            scan_rate_hz=10,
            detection_angle_degrees=360,
            distance_no_detection_mm=int(args.map_size * 1000),
            offset_mm=0,
        )
        state.slam       = RMHC_SLAM(laser, args.map_pixels, args.map_size)
        state.map_pixels = args.map_pixels
        state.map_size_m = args.map_size

    odom = Odometry()

    try:
        with RoombaOI(args.roomba_port) as roomba:
            roomba.wake()
            roomba.start()

            batt = roomba.read_battery()
            if batt['voltage_mV'] < 10000:
                state.update(
                    status='error',
                    error=f"Roomba offline ({batt['voltage_mV']} mV)",
                )
                return

            state.update(
                status='waiting',
                battery_pct=batt['charge_pct'],
                battery_mv=batt['voltage_mV'],
            )

            # Wait for first LiDAR scan (5 s timeout)
            for _ in range(50):
                with state.lidar_lock:
                    if state.lidar_shared['scan']:
                        break
                time.sleep(0.1)
            else:
                state.update(status='error', error='LiDAR: no scan within 5 s')
                return

            last_scan_id = None

            # Outer loop — allows multiple Go / Stop cycles without restarting
            while not state.quit_event.is_set():
                state.go_event.wait()
                state.go_event.clear()
                if state.quit_event.is_set():
                    break

                roomba.safe_mode()
                time.sleep(0.3)

                enc = roomba.read_encoders()
                odom.update(enc['left'], enc['right'])

                auto_state  = FORWARD
                turn_start  = 0.0
                bump_start  = 0.0
                bump_dir    = 'none'   # 'left', 'right', 'both'
                prev_iter_t = time.time()

                # Inner loop — runs until Stop or quit
                while not state.stop_event.is_set() and not state.quit_event.is_set():
                    t0      = time.time()
                    loop_dt = max(t0 - prev_iter_t, 0.01)
                    prev_iter_t = t0

                    with state.lidar_lock:
                        raw  = state.lidar_shared['scan']
                        sid  = id(raw)
                        scan = list(raw)

                    try:
                        enc = roomba.read_encoders()
                        odom.update(enc['left'], enc['right'])
                    except Exception:
                        pass

                    bumps = {'bump_left': False, 'bump_right': False}
                    try:
                        bumps = roomba.read_bumps()
                    except Exception:
                        pass

                    # SLAM update
                    if state.slam is not None and scan:
                        dists = scan_to_distances(scan, SCAN_SLOTS, args.min_quality)
                        state.slam.update(dists, pose_change=(
                            odom.d_xy_mm, odom.d_theta_deg, loop_dt
                        ))
                        x_mm, y_mm, _ = state.slam.getpos()
                        scale = args.map_pixels / (args.map_size * 1000)
                        px = int(x_mm * scale)
                        py = args.map_pixels - 1 - int(y_mm * scale)
                        if 0 <= px < args.map_pixels and 0 <= py < args.map_pixels:
                            if not state.path_pixels or state.path_pixels[-1] != (px, py):
                                state.path_pixels.append((px, py))

                    # Sensor extraction
                    front_mm = left_mm = right_mm = None
                    scan_n_delta = 0
                    if scan:
                        if sid != last_scan_id:
                            scan_n_delta = 1
                            last_scan_id = sid
                        front_mm = get_front(scan, args.fov, args.min_quality)
                        left_mm, right_mm = get_sides(scan, args.min_quality)

                    ups     = read_ups()
                    ups_soc = ups.get('soc')
                    if ups_soc is not None and ups_soc < args.ups_stop:
                        roomba.stop()
                        state.update(status='waiting', drive_state=IDLE,
                                     error=f'UPS critical ({ups_soc:.1f}%)')
                        break

                    mode = state.get_mode()

                    if mode == 'auto':
                        # Bump is highest priority — interrupt any state except BUMPED
                        if auto_state != BUMPED and (bumps['bump_left'] or bumps['bump_right']):
                            roomba.stop()
                            auto_state = BUMPED
                            bump_start = time.time()
                            if bumps['bump_left'] and bumps['bump_right']:
                                bump_dir = 'both'
                            elif bumps['bump_left']:
                                bump_dir = 'left'
                            else:
                                bump_dir = 'right'

                        if auto_state == FORWARD:
                            if front_mm is not None and front_mm <= args.safe_dist:
                                roomba.stop()
                                auto_state = BLOCKED
                            else:
                                roomba.drive_direct(args.speed, args.speed)
                        elif auto_state == BLOCKED:
                            lc = left_mm  if left_mm  is not None else 9999
                            rc = right_mm if right_mm is not None else 9999
                            if lc >= rc:
                                roomba.drive_direct(-SPIN_SPEED, SPIN_SPEED)
                            else:
                                roomba.drive_direct(SPIN_SPEED, -SPIN_SPEED)
                            auto_state = TURNING
                            turn_start = time.time()
                        elif auto_state == TURNING:
                            if time.time() - turn_start >= SPIN_TIME:
                                roomba.stop()
                                if front_mm is None or front_mm > args.safe_dist:
                                    auto_state = FORWARD
                                else:
                                    auto_state = BLOCKED
                        elif auto_state == BUMPED:
                            elapsed = time.time() - bump_start
                            if elapsed < BUMP_BACKUP_TIME:
                                # Back straight up
                                roomba.drive_direct(BUMP_BACKUP_SPEED, BUMP_BACKUP_SPEED)
                            elif elapsed < BUMP_BACKUP_TIME + BUMP_TURN_TIME:
                                # Spin away from the hit side
                                if bump_dir == 'left':
                                    roomba.drive_direct(SPIN_SPEED, -SPIN_SPEED)   # spin right
                                else:
                                    roomba.drive_direct(-SPIN_SPEED, SPIN_SPEED)   # spin left
                            else:
                                roomba.stop()
                                auto_state = FORWARD

                        drive_label = auto_state
                    else:
                        ml, mr = state.get_manual_vel()
                        roomba.drive_direct(ml, mr)
                        auto_state  = FORWARD   # reset so auto resumes cleanly
                        drive_label = IDLE

                    compact_scan = [
                        [int(ang), int(dist)]
                        for q, ang, dist in scan
                        if q >= args.min_quality and dist > 0
                    ] if scan else []

                    with state._lock:
                        s = state._snap
                        s['status']      = 'running'
                        s['drive_state'] = drive_label
                        s['front_mm']    = front_mm
                        s['left_mm']     = left_mm
                        s['right_mm']    = right_mm
                        s['scan_n']     += scan_n_delta
                        s['pose_x']      = round(odom.x, 1)
                        s['pose_y']      = round(odom.y, 1)
                        s['pose_hdg']    = round(odom.heading_deg(), 1)
                        s['ups']         = ups
                        s['scan']        = compact_scan
                        s['bump_left']   = bumps['bump_left']
                        s['bump_right']  = bumps['bump_right']

                    elapsed = time.time() - t0
                    rem = 0.05 - elapsed
                    if rem > 0:
                        time.sleep(rem)

                # Inner loop exited — stop wheels and return to waiting
                roomba.stop()
                state.stop_event.clear()
                state.update(status='waiting', drive_state=IDLE)

    except Exception as e:
        state.update(status='error', error=f'Roomba: {e}')
    finally:
        if state.slam is not None and args.map_out:
            _save_maps(state.slam, args.map_out, args.map_pixels, state.path_pixels)
        if _HAS_UPS:
            GPIO.cleanup()


def lidar_only_main(args, state: SharedState):
    """Reads from the shared LiDAR data (no new serial connection) and streams it to the UI."""
    state.update(status='running', mode='lidar_only', error='')
    last_sid = None
    try:
        while not state.stop_event.is_set() and not state.quit_event.is_set():
            with state.lidar_lock:
                raw  = state.lidar_shared['scan']
                sid  = id(raw)
                scan = list(raw)

            if sid != last_sid and scan:
                last_sid = sid
                compact  = [
                    [int(ang), int(dist)]
                    for q, ang, dist in scan
                    if q >= args.min_quality and dist > 0
                ]
                front_mm = get_front(scan, args.fov, args.min_quality)
                left_mm, right_mm = get_sides(scan, args.min_quality)
                with state._lock:
                    s = state._snap
                    s['scan_n']   += 1
                    s['scan']      = compact
                    s['front_mm']  = front_mm
                    s['left_mm']   = left_mm
                    s['right_mm']  = right_mm

            time.sleep(0.05)
    finally:
        state.stop_event.clear()
        state.update(status='waiting', mode='auto', scan=[], drive_state=IDLE,
                     front_mm=None, left_mm=None, right_mm=None)


def run_check(args, state: SharedState) -> dict:
    """Non-blocking system check — inspects device files and installed packages."""
    import importlib

    def chk_mod(name):
        try:
            importlib.import_module(name)
            return True
        except ImportError:
            return False

    snap    = state.snapshot()
    running = snap['status'] in ('waiting', 'running')

    return {
        'devices': {
            'roomba':  {'ok': os.path.exists(args.roomba_port),  'path': args.roomba_port},
            'rplidar': {'ok': os.path.exists(args.lidar_port),   'path': args.lidar_port},
        },
        'packages': {
            'rplidar':    {'ok': chk_mod('rplidar')},
            'pyserial':   {'ok': chk_mod('serial')},
            'breezyslam': {'ok': chk_mod('breezyslam')},
            'Pillow':     {'ok': chk_mod('PIL')},
            'fastapi':    {'ok': chk_mod('fastapi')},
            'uvicorn':    {'ok': chk_mod('uvicorn')},
        },
        'runtime': {
            'roomba_connected': {
                'ok':     running,
                'detail': f"{snap['battery_pct']}%  {(snap['battery_mv'] or 0) / 1000:.2f} V"
                          if snap['battery_pct'] else 'not connected',
            },
            'slam_active': {'ok': _HAS_SLAM},
            'pillow':      {'ok': _HAS_PIL},
            'server':      {'ok': True, 'detail': 'running'},
        },
    }


def _save_maps(slam, map_out, pixels, path_pixels):
    if not _HAS_PIL:
        return
    try:
        mb  = bytearray(pixels * pixels)
        slam.getmap(mb)
        img = Image.frombytes('L', (pixels, pixels), bytes(mb))
        img.save(map_out)
        print(f"Map saved → {map_out}")
        if path_pixels:
            ip   = img.convert('RGB')
            draw = ImageDraw.Draw(ip)
            r, n = 2, len(path_pixels)
            for i, (px, py) in enumerate(path_pixels):
                c = (0, 220, 0) if i == 0 else (255, 60, 60) if i == n - 1 else (0, 200, 255)
                draw.ellipse([px - r, py - r, px + r, py + r], fill=c)
            base, ext = os.path.splitext(map_out)
            out = f"{base}_path{ext or '.png'}"
            ip.save(out)
            print(f"Map with path saved → {out}")
    except Exception as e:
        print(f"Map save error: {e}")


# -----------------------------------------------------------------------
# FastAPI
# -----------------------------------------------------------------------

_state: SharedState = None
_args               = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    yield
    if _state:
        _state.quit_event.set()
        _state.go_event.set()   # unblock if waiting on go


app = FastAPI(lifespan=lifespan)


@app.get("/", response_class=HTMLResponse)
async def index():
    safe_dist  = _args.safe_dist  if _args else 600
    auto_speed = _args.speed      if _args else 200
    return _HTML.replace('__SAFE_DIST__', str(safe_dist)) \
                .replace('__AUTO_SPEED__', str(auto_speed))


@app.get("/check")
async def system_check():
    return run_check(_args, _state)


@app.post("/reconnect")
async def reconnect():
    loop = asyncio.get_event_loop()
    await loop.run_in_executor(None, _reconnect)
    return {"ok": True}


@app.get("/map")
async def get_map():
    if _state is None or _state.slam is None or not _HAS_PIL:
        return Response(status_code=204)
    try:
        pixels = _state.map_pixels
        mb     = bytearray(pixels * pixels)
        _state.slam.getmap(mb)
        img = Image.frombytes('L', (pixels, pixels), bytes(mb))
        buf = io.BytesIO()
        img.save(buf, format='JPEG', quality=75)
        return Response(content=buf.getvalue(), media_type='image/jpeg')
    except Exception:
        return Response(status_code=500)


@app.websocket("/ws")
async def ws_handler(ws: WebSocket):
    await ws.accept()

    async def send_loop():
        while True:
            try:
                await ws.send_text(json.dumps(_state.snapshot()))
            except Exception:
                break
            await asyncio.sleep(0.05)

    send_task = asyncio.create_task(send_loop())

    try:
        while True:
            data = await ws.receive_text()
            _handle(json.loads(data))
    except WebSocketDisconnect:
        pass
    finally:
        send_task.cancel()


_lidar_only_thread: threading.Thread = None
_robot_thread:      threading.Thread = None


def _reconnect():
    """Kill the current robot thread and start a fresh one. Runs in a thread executor."""
    global _robot_thread
    # Signal the running thread to quit
    _state.quit_event.set()
    _state.go_event.set()      # unblock if it's waiting on Go
    _state.stop_event.set()    # unblock if it's in the inner loop
    if _robot_thread and _robot_thread.is_alive():
        _robot_thread.join(timeout=6)
    # Re-arm all events and reset status
    _state.quit_event.clear()
    _state.go_event.clear()
    _state.stop_event.clear()
    _state.update(
        status='connecting', error='',
        battery_pct=None, battery_mv=None,
        scan=[], front_mm=None, left_mm=None, right_mm=None,
        drive_state=IDLE,
    )
    _robot_thread = threading.Thread(target=robot_main, args=(_args, _state), daemon=True)
    _robot_thread.start()


def _handle(msg):
    global _lidar_only_thread
    cmd = msg.get('cmd')

    if cmd == 'go':
        # If lidar-only is running, stop it before starting full robot mode
        if _state.get_mode() == 'lidar_only' and _state._snap.get('status') == 'running':
            _state.stop_event.set()
            if _lidar_only_thread:
                _lidar_only_thread.join(timeout=2)
        _state.set_mode('auto')
        _state.update(error='')
        _state.go_event.set()

    elif cmd == 'stop':
        _state.stop_event.set()
        _state.set_manual_vel(0, 0)

    elif cmd == 'lidar_only':
        if _state._snap.get('status') == 'running':
            return   # already running something
        _lidar_only_thread = threading.Thread(
            target=lidar_only_main, args=(_args, _state), daemon=True
        )
        _lidar_only_thread.start()

    elif cmd == 'set_mode':
        mode = msg.get('mode', 'auto')
        _state.set_mode(mode)
        if mode == 'manual':
            _state.set_manual_vel(0, 0)

    elif cmd == 'drive':
        if _state.get_mode() == 'manual':
            _state.set_manual_vel(
                max(-500, min(500, int(msg.get('left',  0)))),
                max(-500, min(500, int(msg.get('right', 0)))),
            )


# -----------------------------------------------------------------------
# Embedded HTML/JS frontend
# -----------------------------------------------------------------------

_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>LUCIA Control Panel</title>
<style>
  *, *::before, *::after { box-sizing: border-box; margin: 0; padding: 0; }

  body {
    background: #0d0d1a;
    color: #d0d0e0;
    font-family: 'Courier New', monospace;
    font-size: 13px;
    height: 100vh;
    display: flex;
    flex-direction: column;
    overflow: hidden;
  }

  /* ---- header ---- */
  #hdr {
    background: #111128;
    border-bottom: 1px solid #222244;
    padding: 8px 16px;
    display: flex;
    align-items: center;
    gap: 12px;
    flex-shrink: 0;
  }
  #hdr h1 { color: #6ab0d4; font-size: 15px; letter-spacing: 2px; flex: 1; }
  #conn-dot {
    width: 9px; height: 9px; border-radius: 50%; background: #444;
    transition: background 0.3s;
  }
  #conn-dot.connecting { background: #e8a020; }
  #conn-dot.waiting    { background: #4a90d9; animation: pulse 1.5s infinite; }
  #conn-dot.running    { background: #27ae60; }
  #conn-dot.error      { background: #e74c3c; }
  @keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.4} }
  #conn-text { font-size: 12px; color: #888; min-width: 160px; }

  /* ---- controls bar ---- */
  #ctrl-bar {
    background: #0f0f22;
    border-bottom: 1px solid #222244;
    padding: 8px 16px;
    display: flex;
    align-items: center;
    justify-content: center;
    gap: 14px;
    flex-shrink: 0;
    flex-wrap: wrap;
  }

  button {
    padding: 6px 18px;
    border: none;
    border-radius: 3px;
    cursor: pointer;
    font-family: inherit;
    font-size: 13px;
    font-weight: bold;
    letter-spacing: 1px;
    transition: opacity 0.15s;
  }
  button:disabled { opacity: 0.35; cursor: default; }
  #btn-go   { background: #27ae60; color: #fff; }
  #btn-stop { background: #c0392b; color: #fff; }

  /* ---- mode toggle ---- */
  .mode-wrap { display: flex; align-items: center; gap: 8px; }
  .mode-lbl  { font-size: 12px; color: #555; transition: color 0.2s; }
  .mode-lbl.on { color: #6ab0d4; font-weight: bold; }

  .sw { position: relative; width: 44px; height: 22px; }
  .sw input { display: none; }
  .sw-track {
    position: absolute; inset: 0;
    background: #27ae60;
    border-radius: 22px;
    cursor: pointer;
    transition: background 0.25s;
  }
  .sw input:checked ~ .sw-track { background: #4a90d9; }
  .sw-thumb {
    position: absolute;
    width: 16px; height: 16px;
    top: 3px; left: 3px;
    background: #fff;
    border-radius: 50%;
    transition: transform 0.25s;
    pointer-events: none;
  }
  .sw input:checked ~ .sw-thumb { transform: translateX(22px); }

  #err-msg { font-size: 11px; color: #e74c3c; flex: 1; }

  #btn-lidar     { background: #1a4a6b; color: #6ab0d4; border: 1px solid #2a6090; }
  #btn-check     { background: #1e2a1e; color: #5a9a5a; border: 1px solid #2a4a2a; }
  #btn-reconnect { background: #2a1e10; color: #c8823a; border: 1px solid #5a3a10; }

  /* ---- system check panel ---- */
  #check-panel {
    display: none;
    background: #080812;
    border-bottom: 1px solid #222244;
    padding: 10px 16px;
    gap: 24px;
    flex-wrap: wrap;
  }
  #check-panel.show { display: flex; }
  .chk-group { display: flex; flex-direction: column; gap: 3px; }
  .chk-title { font-size: 10px; color: #555; letter-spacing: 1px; margin-bottom: 2px; }
  .chk-row   { display: flex; align-items: center; gap: 6px; font-size: 12px; }
  .chk-dot   { width: 7px; height: 7px; border-radius: 50%; flex-shrink: 0; }
  .chk-dot.ok  { background: #27ae60; }
  .chk-dot.bad { background: #e74c3c; }
  .chk-name  { color: #aaa; }
  .chk-detail { color: #555; font-size: 11px; }

  /* ---- main layout ---- */
  #body {
    display: flex;
    flex: 1;
    overflow: hidden;
    min-height: 0;
  }

  /* ---- left panel: radar + stats ---- */
  #left-panel {
    flex: 0 0 220px;
    border-right: 1px solid #222244;
    display: flex;
    flex-direction: column;
    align-items: center;
    padding: 10px 8px;
    gap: 8px;
    overflow-y: auto;
  }

  #radar {
    border: 1px solid #222244;
    border-radius: 50%;
    display: block;
    flex-shrink: 0;
  }

  /* ---- WASD indicator ---- */
  #wasd-wrap { display: none; flex-direction: column; align-items: center; gap: 3px; }
  #wasd-wrap.show { display: flex; }
  .krow { display: flex; gap: 3px; }
  .key {
    width: 28px; height: 28px;
    border: 1px solid #333;
    border-radius: 4px;
    background: #111128;
    display: flex; align-items: center; justify-content: center;
    font-size: 12px; color: #555;
    transition: background 0.08s, color 0.08s, border-color 0.08s;
    user-select: none;
  }
  .key.lit { background: #1a3a5c; border-color: #4a90d9; color: #6ab0d4; }

  #spd-wrap { display: none; align-items: center; gap: 6px; width: 100%; }
  #spd-wrap.show { display: flex; }
  #spd-slider { flex: 1; accent-color: #4a90d9; }
  #spd-val { color: #6ab0d4; min-width: 52px; font-size: 11px; }

  /* ---- manual control legend ---- */
  #ctrl-legend {
    display: none;
    flex-direction: column;
    gap: 2px;
    width: 100%;
    padding: 6px 8px;
    border: 1px solid #1e2240;
    border-radius: 4px;
    background: #0b0b18;
  }
  #ctrl-legend.show { display: flex; }
  .cl-row { display: flex; justify-content: space-between; align-items: center; }
  .cl-keys {
    font-size: 10px;
    color: #4a90d9;
    background: #111128;
    border: 1px solid #2a3060;
    border-radius: 3px;
    padding: 1px 4px;
  }
  .cl-desc { font-size: 10px; color: #555; }

  /* ---- bump indicators ---- */
  #bumpers { display: flex; gap: 6px; width: 100%; justify-content: center; }
  .bump-ind {
    flex: 1;
    padding: 4px 0;
    text-align: center;
    border-radius: 3px;
    border: 1px solid #2a2a3a;
    font-size: 11px;
    color: #444;
    background: #0d0d1a;
    transition: background 0.08s, color 0.08s, border-color 0.08s;
    letter-spacing: 1px;
  }
  .bump-ind.hit {
    background: #3a0a0a;
    border-color: #e74c3c;
    color: #e74c3c;
    font-weight: bold;
  }

  /* ---- stats (compact, inside left panel) ---- */
  #stats {
    width: 100%;
    flex-shrink: 0;
    display: flex;
    flex-direction: column;
    gap: 0;
  }

  .sr { display: flex; justify-content: space-between; padding: 2px 0; border-bottom: 1px solid #0f0f20; font-size: 11px; }
  .sl { color: #555; }
  .sv { color: #c0c0d8; font-weight: bold; }
  .sv.danger { color: #e74c3c; }
  .sv.warn   { color: #e8a020; }
  .sv.ok     { color: #27ae60; }

  /* ---- map panel (dominant right side) ---- */
  #map-panel {
    flex: 1;
    display: flex;
    flex-direction: column;
    padding: 10px 12px;
    overflow: hidden;
    min-width: 0;
    min-height: 0;
  }
  #map-hdr { color: #555; font-size: 11px; margin-bottom: 6px; flex-shrink: 0; }
  #map-none { color: #444; font-size: 12px; }
  #map-img {
    flex: 1;
    width: 100%;
    min-height: 0;
    border: 1px solid #222244;
    display: none;
    image-rendering: pixelated;
    object-fit: contain;
  }
</style>
</head>
<body>

<!-- header -->
<div id="hdr">
  <h1>L·U·C·I·A</h1>
  <div id="conn-dot" class="connecting"></div>
  <span id="conn-text">Connecting...</span>
</div>

<!-- controls bar -->
<div id="ctrl-bar">
  <button id="btn-go"    onclick="cmd('go')"        disabled>▶&nbsp;GO</button>
  <button id="btn-stop"  onclick="cmd('stop')"       disabled>■&nbsp;STOP</button>
  <button id="btn-lidar"     onclick="cmd('lidar_only')" disabled>◎&nbsp;LIDAR ONLY</button>
  <button id="btn-check"     onclick="doCheck()">⬡&nbsp;SYSTEM CHECK</button>
  <button id="btn-reconnect" onclick="doReconnect()">↺&nbsp;RECONNECT</button>

  <div class="mode-wrap">
    <span class="mode-lbl on" id="lbl-auto">AUTO</span>
    <label class="sw">
      <input type="checkbox" id="mode-sw" onchange="onModeToggle(this)">
      <div class="sw-track"></div>
      <div class="sw-thumb"></div>
    </label>
    <span class="mode-lbl" id="lbl-manual">MANUAL</span>
  </div>

  <span id="err-msg"></span>
</div>

<!-- system check panel (hidden until check runs) -->
<div id="check-panel">
  <div class="chk-group">
    <div class="chk-title">DEVICES</div>
    <div class="chk-row"><div class="chk-dot" id="ck-roomba-dev"></div><span class="chk-name">Roomba</span><span class="chk-detail" id="ck-roomba-dev-d"></span></div>
    <div class="chk-row"><div class="chk-dot" id="ck-lidar-dev"></div> <span class="chk-name">RPLidar</span><span class="chk-detail" id="ck-lidar-dev-d"></span></div>
  </div>
  <div class="chk-group">
    <div class="chk-title">PACKAGES</div>
    <div class="chk-row"><div class="chk-dot" id="ck-rplidar"></div>   <span class="chk-name">rplidar</span></div>
    <div class="chk-row"><div class="chk-dot" id="ck-pyserial"></div>  <span class="chk-name">pyserial</span></div>
    <div class="chk-row"><div class="chk-dot" id="ck-breezyslam"></div><span class="chk-name">breezyslam</span></div>
    <div class="chk-row"><div class="chk-dot" id="ck-pillow"></div>    <span class="chk-name">Pillow</span></div>
  </div>
  <div class="chk-group">
    <div class="chk-title">RUNTIME</div>
    <div class="chk-row"><div class="chk-dot" id="ck-roomba-conn"></div><span class="chk-name">Roomba connected</span><span class="chk-detail" id="ck-roomba-conn-d"></span></div>
    <div class="chk-row"><div class="chk-dot" id="ck-slam"></div>      <span class="chk-name">SLAM (breezyslam)</span></div>
    <div class="chk-row"><div class="chk-dot" id="ck-server"></div>    <span class="chk-name">Server</span></div>
  </div>
</div>

<!-- body -->
<div id="body">

  <!-- left: radar + stats -->
  <div id="left-panel">
    <canvas id="radar" width="200" height="200"></canvas>

    <div id="bumpers">
      <div class="bump-ind" id="bump-left">◀ LEFT</div>
      <div class="bump-ind" id="bump-right">RIGHT ▶</div>
    </div>

    <div id="wasd-wrap">
      <div class="krow"><div class="key" id="kw">W</div></div>
      <div class="krow">
        <div class="key" id="ka">A</div>
        <div class="key" id="ks">S</div>
        <div class="key" id="kd">D</div>
      </div>
      <div id="ctrl-legend">
        <div class="cl-row"><span class="cl-keys">W / ↑</span>    <span class="cl-desc">Forward</span></div>
        <div class="cl-row"><span class="cl-keys">S / ↓</span>    <span class="cl-desc">Backward</span></div>
        <div class="cl-row"><span class="cl-keys">A / ←</span>    <span class="cl-desc">Spin left</span></div>
        <div class="cl-row"><span class="cl-keys">D / →</span>    <span class="cl-desc">Spin right</span></div>
        <div class="cl-row"><span class="cl-keys">W+A / W+D</span><span class="cl-desc">Arc turn</span></div>
        <div class="cl-row"><span class="cl-keys">release</span>  <span class="cl-desc">Stop</span></div>
      </div>
    </div>

    <div id="spd-wrap">
      <span class="sl">Spd</span>
      <input type="range" id="spd-slider" min="50" max="400" value="__AUTO_SPEED__"
             oninput="onSpeed(this)">
      <span id="spd-val">__AUTO_SPEED__</span>
    </div>

    <div id="stats">
      <div class="sr"><span class="sl">Status</span>  <span class="sv" id="s-status">—</span></div>
      <div class="sr"><span class="sl">Mode</span>    <span class="sv" id="s-mode">—</span></div>
      <div class="sr"><span class="sl">State</span>   <span class="sv" id="s-drive">—</span></div>
      <div class="sr"><span class="sl">Scans</span>   <span class="sv" id="s-scans">—</span></div>
      <div class="sr"><span class="sl">Front</span>   <span class="sv" id="s-front">—</span></div>
      <div class="sr"><span class="sl">Left</span>    <span class="sv" id="s-left">—</span></div>
      <div class="sr"><span class="sl">Right</span>   <span class="sv" id="s-right">—</span></div>
      <div class="sr"><span class="sl">SLAM</span>    <span class="sv" id="s-slam">—</span></div>
      <div class="sr"><span class="sl">X</span>       <span class="sv" id="s-px">—</span></div>
      <div class="sr"><span class="sl">Y</span>       <span class="sv" id="s-py">—</span></div>
      <div class="sr"><span class="sl">Hdg</span>     <span class="sv" id="s-hdg">—</span></div>
      <div class="sr"><span class="sl">Batt</span>    <span class="sv" id="s-batt">—</span></div>
      <div class="sr"><span class="sl">UPS</span>     <span class="sv" id="s-ups">—</span></div>
    </div>
  </div>

  <!-- right: large map -->
  <div id="map-panel">
    <div id="map-hdr">SLAM map &nbsp;·&nbsp; refreshes every 3 s</div>
    <span id="map-none">No map yet — SLAM starts on GO</span>
    <img id="map-img" alt="SLAM map">
  </div>

</div><!-- /body -->

<script>
'use strict';

const SAFE_DIST  = __SAFE_DIST__;
const MAX_RANGE  = 3000;

// canvas
const cvs = document.getElementById('radar');
const ctx = cvs.getContext('2d');
const CX  = cvs.width  / 2;
const CY  = cvs.height / 2;
const CR  = CX - 6;   // usable radius in pixels

let ws          = null;
let manualSpeed = parseInt(document.getElementById('spd-slider').value);
let keysDown    = new Set();
let curMode     = 'auto';
let curStatus   = 'connecting';

// ---- WebSocket --------------------------------------------------------

function connect() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onmessage = e => render(JSON.parse(e.data));
  ws.onclose   = ()  => { setConn('connecting', 'Reconnecting...'); setTimeout(connect, 1500); };
  ws.onerror   = ()  => ws.close();
}

function cmd(name, extra = {}) {
  if (ws && ws.readyState === WebSocket.OPEN)
    ws.send(JSON.stringify({ cmd: name, ...extra }));
}

// ---- Render -----------------------------------------------------------

function setConn(cls, text) {
  const d = document.getElementById('conn-dot');
  d.className = cls;
  document.getElementById('conn-text').textContent = text;
}

function tv(id, val)  { document.getElementById(id).textContent = val ?? '—'; }
function el(id)       { return document.getElementById(id); }
function mm(v)        { return v != null ? `${Math.round(v)} mm` : '—'; }

function render(d) {
  curStatus = d.status;
  curMode   = d.mode;

  // connection dot
  const labels = {
    connecting: 'Connecting…',
    waiting:    'Waiting — press GO',
    running:    `Running  [ ${d.mode === 'lidar_only' ? 'LIDAR ONLY' : d.mode.toUpperCase()} ]`,
    error:      `Error: ${d.error}`,
  };
  setConn(d.status, labels[d.status] || d.status);

  // buttons
  const isRunning    = (d.status === 'running');
  const isConnecting = (d.status === 'connecting');
  el('btn-go').disabled        = isRunning || isConnecting;
  el('btn-stop').disabled      = !isRunning;
  el('btn-lidar').disabled     = isRunning || isConnecting;
  el('btn-reconnect').disabled = isRunning || isConnecting;

  // mode toggle
  el('mode-sw').checked     = (d.mode === 'manual');
  el('lbl-auto').className   = 'mode-lbl' + (d.mode === 'auto'   ? ' on' : '');
  el('lbl-manual').className = 'mode-lbl' + (d.mode === 'manual' ? ' on' : '');

  // manual controls
  const showManual = (d.mode === 'manual' && d.status === 'running');
  el('wasd-wrap').className    = showManual ? 'show' : '';
  el('ctrl-legend').className  = showManual ? 'show' : '';
  el('spd-wrap').className     = showManual ? 'show' : '';

  // error message
  el('err-msg').textContent = d.error || '';

  // stats
  tv('s-status', d.status.toUpperCase());
  tv('s-mode',   d.mode.toUpperCase());
  tv('s-drive',  d.drive_state);
  tv('s-scans',  d.scan_n);

  const fe = el('s-front');
  if (d.front_mm != null) {
    fe.textContent = `${Math.round(d.front_mm)} mm`;
    fe.className   = 'sv' + (d.front_mm <= SAFE_DIST ? ' danger' : d.front_mm <= SAFE_DIST * 1.5 ? ' warn' : '');
  } else {
    fe.textContent = '—'; fe.className = 'sv';
  }

  tv('s-left',  mm(d.left_mm));
  tv('s-right', mm(d.right_mm));
  tv('s-px',    d.pose_x  != null ? `${d.pose_x.toFixed(0)} mm` : '—');
  tv('s-py',    d.pose_y  != null ? `${d.pose_y.toFixed(0)} mm` : '—');
  tv('s-hdg',   d.pose_hdg != null ? `${d.pose_hdg.toFixed(1)}°` : '—');
  tv('s-slam',  d.slam_active ? 'ACTIVE' : 'inactive');

  if (d.battery_pct != null) {
    const pct = d.battery_pct;
    const be  = el('s-batt');
    be.textContent = `${pct}%  (${(d.battery_mv / 1000).toFixed(2)} V)`;
    be.className   = 'sv' + (pct < 15 ? ' danger' : pct < 30 ? ' warn' : ' ok');
  }

  const ups = d.ups || {};
  if (ups.soc != null) {
    const ue = el('s-ups');
    ue.textContent = `${ups.soc.toFixed(1)}%` + (ups.voltage ? `  ${ups.voltage.toFixed(3)} V` : '');
    ue.className   = 'sv' + (ups.soc < 15 ? ' danger' : ups.soc < 30 ? ' warn' : ' ok');
  }

  // bump indicators
  el('bump-left').className  = 'bump-ind' + (d.bump_left  ? ' hit' : '');
  el('bump-right').className = 'bump-ind' + (d.bump_right ? ' hit' : '');

  // radar
  if (d.scan && d.scan.length) drawRadar(d.scan, d.front_mm, d.pose_hdg || 0);
}

// ---- Radar canvas -----------------------------------------------------

function drawRadar(scan, frontMm, hdgDeg) {
  ctx.clearRect(0, 0, cvs.width, cvs.height);

  // background
  ctx.fillStyle = '#09090f';
  ctx.fillRect(0, 0, cvs.width, cvs.height);

  // range rings + labels
  const rings = [1000, 2000, 3000];
  rings.forEach(rmm => {
    const r = (rmm / MAX_RANGE) * CR;
    ctx.beginPath();
    ctx.arc(CX, CY, r, 0, 2 * Math.PI);
    ctx.strokeStyle = '#1c2030';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.fillStyle = '#2a3550';
    ctx.font = '9px monospace';
    ctx.fillText(`${rmm / 1000}m`, CX + r * 0.72, CY - r * 0.72);
  });

  // cross-hairs
  ctx.strokeStyle = '#161828';
  ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(CX, CY - CR); ctx.lineTo(CX, CY + CR); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(CX - CR, CY); ctx.lineTo(CX + CR, CY); ctx.stroke();

  // safe-dist ring (orange dashed)
  if (SAFE_DIST <= MAX_RANGE) {
    const sr = (SAFE_DIST / MAX_RANGE) * CR;
    ctx.beginPath();
    ctx.arc(CX, CY, sr, 0, 2 * Math.PI);
    ctx.strokeStyle = '#6b3300';
    ctx.setLineDash([4, 4]);
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // scan points
  for (const [ang, dist] of scan) {
    if (dist <= 0 || dist > MAX_RANGE) continue;
    const r   = (dist / MAX_RANGE) * CR;
    const rad = (ang - 90) * Math.PI / 180;  // 0° = up (forward)
    const x   = CX + r * Math.cos(rad);
    const y   = CY + r * Math.sin(rad);

    if (dist <= SAFE_DIST)            ctx.fillStyle = '#e74c3c';
    else if (dist <= SAFE_DIST * 1.5) ctx.fillStyle = '#e8a020';
    else                               ctx.fillStyle = '#2ecc71';

    ctx.beginPath();
    ctx.arc(x, y, 2, 0, 2 * Math.PI);
    ctx.fill();
  }

  // heading arrow
  const hRad = (hdgDeg - 90) * Math.PI / 180;
  const aLen = 22;
  ctx.strokeStyle = '#4a90d9';
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(CX, CY);
  ctx.lineTo(CX + aLen * Math.cos(hRad), CY + aLen * Math.sin(hRad));
  ctx.stroke();

  // robot dot
  ctx.fillStyle = '#6ab0d4';
  ctx.beginPath();
  ctx.arc(CX, CY, 5, 0, 2 * Math.PI);
  ctx.fill();

  // forward tick at top
  ctx.fillStyle = '#4a90d9';
  ctx.beginPath();
  ctx.moveTo(CX, CY - CR - 1);
  ctx.lineTo(CX - 4, CY - CR + 6);
  ctx.lineTo(CX + 4, CY - CR + 6);
  ctx.closePath();
  ctx.fill();
}

// ---- Mode toggle ------------------------------------------------------

function onModeToggle(el) {
  const mode = el.checked ? 'manual' : 'auto';
  cmd('set_mode', { mode });
  if (mode === 'manual') keysDown.clear();
}

// ---- Manual drive -----------------------------------------------------

function onSpeed(el) {
  manualSpeed = parseInt(el.value);
  document.getElementById('spd-val').textContent = `${manualSpeed} mm/s`;
  sendDrive();
}

const KEY_MAP = {
  ArrowUp:   'w', w: 'w', W: 'w',
  ArrowDown: 's', s: 's', S: 's',
  ArrowLeft: 'a', a: 'a', A: 'a',
  ArrowRight:'d', d: 'd', D: 'd',
};

document.addEventListener('keydown', e => {
  const k = KEY_MAP[e.key];
  if (!k) return;
  e.preventDefault();
  if (keysDown.has(k)) return;
  keysDown.add(k);
  el(`k${k}`)?.classList.add('lit');
  if (curMode === 'manual' && curStatus === 'running') sendDrive();
});

document.addEventListener('keyup', e => {
  const k = KEY_MAP[e.key];
  if (!k) return;
  keysDown.delete(k);
  el(`k${k}`)?.classList.remove('lit');
  if (curMode === 'manual' && curStatus === 'running') sendDrive();
});

function sendDrive() {
  let left = 0, right = 0;
  if (keysDown.has('w')) { left += manualSpeed; right += manualSpeed; }
  if (keysDown.has('s')) { left -= manualSpeed; right -= manualSpeed; }
  if (keysDown.has('a')) { left -= manualSpeed; right += manualSpeed; }
  if (keysDown.has('d')) { left += manualSpeed; right -= manualSpeed; }
  left  = Math.max(-500, Math.min(500, left));
  right = Math.max(-500, Math.min(500, right));
  cmd('drive', { left, right });
}

// ---- Map refresh ------------------------------------------------------

function refreshMap() {
  const img = el('map-img');
  const tmp = new Image();
  tmp.onload = () => {
    img.src = tmp.src;
    img.style.display = 'block';
    el('map-none').style.display = 'none';
  };
  tmp.src = `/map?t=${Date.now()}`;
}
setInterval(refreshMap, 3000);

// ---- Reconnect --------------------------------------------------------

async function doReconnect() {
  const btn = el('btn-reconnect');
  btn.textContent = '↺ RECONNECTING…';
  btn.disabled = true;
  try {
    await fetch('/reconnect', { method: 'POST' });
  } catch (e) {
    el('err-msg').textContent = 'Reconnect failed: ' + e;
    btn.disabled = false;
  } finally {
    btn.textContent = '↺ RECONNECT';
  }
}

// ---- System check -----------------------------------------------------

function dot(id, ok) {
  const d = el(id);
  if (d) d.className = 'chk-dot ' + (ok ? 'ok' : 'bad');
}

async function doCheck() {
  const btn = el('btn-check');
  btn.textContent = '⬡ CHECKING…';
  btn.disabled = true;

  try {
    const r = await fetch('/check');
    const d = await r.json();

    dot('ck-roomba-dev',  d.devices.roomba.ok);
    dot('ck-lidar-dev',   d.devices.rplidar.ok);
    el('ck-roomba-dev-d').textContent = d.devices.roomba.path;
    el('ck-lidar-dev-d').textContent  = d.devices.rplidar.path;

    dot('ck-rplidar',    d.packages.rplidar.ok);
    dot('ck-pyserial',   d.packages.pyserial.ok);
    dot('ck-breezyslam', d.packages.breezyslam.ok);
    dot('ck-pillow',     d.packages.Pillow.ok);

    dot('ck-roomba-conn', d.runtime.roomba_connected.ok);
    el('ck-roomba-conn-d').textContent = d.runtime.roomba_connected.detail;
    dot('ck-slam',   d.runtime.slam_active.ok);
    dot('ck-server', d.runtime.server.ok);

    el('check-panel').className = 'show';
  } catch (e) {
    el('err-msg').textContent = 'Check failed: ' + e;
  } finally {
    btn.textContent = '⬡ SYSTEM CHECK';
    btn.disabled = false;
  }
}

// ---- Boot -------------------------------------------------------------
connect();
</script>
</body>
</html>
"""


# -----------------------------------------------------------------------
# Args + entry point
# -----------------------------------------------------------------------

def main():
    global _state, _args

    parser = argparse.ArgumentParser(description='LUCIA web control panel')
    parser.add_argument('--roomba-port',  default='/dev/ttyUSB0',
                        help='Roomba serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--lidar-port',   default='/dev/ttyUSB1',
                        help='LiDAR serial port (default: /dev/ttyUSB1)')
    parser.add_argument('--host',         default='0.0.0.0',
                        help='Bind address (default: 0.0.0.0)')
    parser.add_argument('--port',         type=int, default=8000,
                        help='HTTP port (default: 8000)')
    parser.add_argument('--speed',        type=int, default=200,
                        help='Autonomous forward speed mm/s (default: 200)')
    parser.add_argument('--safe-dist',    type=int, default=600,
                        help='Obstacle stop threshold mm (default: 600)')
    parser.add_argument('--fov',          type=int, default=30,
                        help='Forward detection arc ±degrees (default: 30)')
    parser.add_argument('--min-quality',  type=int, default=5,
                        help='Minimum LiDAR point quality (default: 5)')
    parser.add_argument('--map-out',      default='map.png',
                        help='Map output path on shutdown (default: map.png)')
    parser.add_argument('--map-size',     type=float, default=10.0,
                        help='Map coverage in meters (default: 10)')
    parser.add_argument('--map-pixels',   type=int,   default=800,
                        help='Map resolution in pixels (default: 800)')
    parser.add_argument('--ups-warn',     type=int,   default=20,
                        help='UPS %% warning threshold (default: 20)')
    parser.add_argument('--ups-stop',     type=int,   default=10,
                        help='UPS %% stop threshold (default: 10)')
    _args = parser.parse_args()
    _state = SharedState()

    global _robot_thread
    threading.Thread(target=lidar_manager, args=(_args, _state), daemon=True).start()
    _robot_thread = threading.Thread(target=robot_main, args=(_args, _state), daemon=True)
    _robot_thread.start()

    print(f"\n  LUCIA Control Panel")
    print(f"  Open → http://localhost:{_args.port}  (or http://<pi-ip>:{_args.port})\n")
    uvicorn.run(app, host=_args.host, port=_args.port, log_level='warning')


if __name__ == '__main__':
    main()
