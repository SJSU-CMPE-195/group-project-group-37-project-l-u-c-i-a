"""
control_panel.py

All-in-one terminal control panel for the Roomba.
Live sensor dashboard + real-time drive control + hotkeys for all actions.

Requirements:
    pip install evdev  (Linux only)

Usage:
    python3 control_panel.py --port /dev/ttyUSB0 --device /dev/input/event3
    python3 control_panel.py --port /dev/ttyUSB0 --list-devices

Drive controls:
    W / ↑    — forward
    S / ↓    — backward
    A / ←    — spin left
    D / →    — spin right
    W+A / W+D — arc

Hotkeys:
    1        — play Mass Destruction
    2        — play La Cucaracha
    T        — drive square demo
    R        — reset Roomba
    X        — power off Roomba
    Q / ESC  — quit
"""

import argparse
import curses
import threading
import time

import evdev
from evdev import ecodes

from roomba_oi import RoombaOI
from song import (
    load_song, play_song, song_duration,
    MASS_DESTRUCTION, LA_CUCARACHA_1, LA_CUCARACHA_2
)
from drive_demos import forward, turn_left

# ------------------------------------------------------------------
# Key map
# ------------------------------------------------------------------
KEY_MAP = {
    ecodes.KEY_W:     'w',
    ecodes.KEY_A:     'a',
    ecodes.KEY_S:     's',
    ecodes.KEY_D:     'd',
    ecodes.KEY_Q:     'q',
    ecodes.KEY_ESC:   'esc',
    ecodes.KEY_UP:    'w',
    ecodes.KEY_DOWN:  's',
    ecodes.KEY_LEFT:  'a',
    ecodes.KEY_RIGHT: 'd',
    ecodes.KEY_1:     '1',
    ecodes.KEY_2:     '2',
    ecodes.KEY_T:     't',
    ecodes.KEY_R:     'r',
    ecodes.KEY_X:     'x',
}

SPEED = 300

# ------------------------------------------------------------------
# Device helpers
# ------------------------------------------------------------------

def list_keyboards():
    print("Available input devices:")
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
            caps = device.capabilities()
            if ecodes.EV_KEY in caps and ecodes.KEY_W in caps[ecodes.EV_KEY]:
                print(f"  {path}  —  {device.name}")
        except Exception:
            continue


def find_keyboard(device_path=None):
    if device_path:
        return evdev.InputDevice(device_path)
    for path in evdev.list_devices():
        try:
            device = evdev.InputDevice(path)
            caps = device.capabilities()
            if ecodes.EV_KEY in caps and ecodes.KEY_W in caps[ecodes.EV_KEY]:
                return device
        except Exception:
            continue
    return None

# ------------------------------------------------------------------
# Background threads
# ------------------------------------------------------------------

def key_reader(device, pressed, hotkeys, lock, stop_event):
    """Read evdev events and update pressed set + hotkey queue."""
    device.grab()
    try:
        for event in device.read_loop():
            if stop_event.is_set():
                break
            if event.type == ecodes.EV_KEY:
                key = KEY_MAP.get(event.code)
                if not key:
                    continue
                if event.value == 1:  # key down
                    with lock:
                        pressed.add(key)
                        if key not in ('w', 'a', 's', 'd'):
                            hotkeys.append(key)
                elif event.value == 0:  # key up
                    with lock:
                        pressed.discard(key)
    finally:
        device.ungrab()


def sensor_poller(roomba, sensors, lock, stop_event):
    """Poll Roomba sensors every 0.5s and update shared dict."""
    while not stop_event.is_set():
        try:
            bumps   = roomba.read_bumps()
            cliffs  = roomba.read_cliffs()
            battery = roomba.read_battery()
            encoders= roomba.read_encoders()
            with lock:
                sensors.update({
                    'bumps': bumps,
                    'cliffs': cliffs,
                    'battery': battery,
                    'encoders': encoders,
                })
        except Exception:
            pass
        time.sleep(0.5)

# ------------------------------------------------------------------
# Drive helpers
# ------------------------------------------------------------------

def compute_wheel_speeds(pressed, speed):
    w = 'w' in pressed
    s = 's' in pressed
    a = 'a' in pressed
    d = 'd' in pressed

    if not any([w, s, a, d]):
        return 0, 0
    if w and a:
        return speed // 2, speed
    if w and d:
        return speed, speed // 2
    if s and a:
        return -(speed // 2), -speed
    if s and d:
        return -speed, -(speed // 2)
    if w:
        return speed, speed
    if s:
        return -speed, -speed
    if a:
        return -speed, speed
    if d:
        return speed, -speed
    return 0, 0


def direction_label(left, right):
    if left > 0 and right > 0 and left == right:
        return "FORWARD"
    if left < 0 and right < 0 and left == right:
        return "BACKWARD"
    if left > 0 and right < 0:
        return "SPIN RIGHT"
    if left < 0 and right > 0:
        return "SPIN LEFT"
    if left != right and left > 0:
        return "ARC RIGHT"
    if left != right and right > 0:
        return "ARC LEFT"
    return "STOPPED"

# ------------------------------------------------------------------
# UI drawing
# ------------------------------------------------------------------

def draw(stdscr, sensors, s_lock, left, right, status_msg):
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    # Header
    title = " LUCIA — Roomba Control Panel "
    stdscr.attron(curses.A_REVERSE)
    stdscr.addstr(0, 0, title.center(w - 1))
    stdscr.attroff(curses.A_REVERSE)

    # --- Left column: sensors ---
    col = 2
    row = 2

    stdscr.addstr(row, col, "[ SENSORS ]", curses.A_BOLD)
    row += 1

    with s_lock:
        bumps   = sensors.get('bumps', {})
        cliffs  = sensors.get('cliffs', {})
        battery = sensors.get('battery', {})
        encoders= sensors.get('encoders', {})

    def yn(val):
        return "YES" if val else "---"

    stdscr.addstr(row,   col, f"Bump   L: {yn(bumps.get('bump_left'))}   R: {yn(bumps.get('bump_right'))}")
    stdscr.addstr(row+1, col, f"Drop   L: {yn(bumps.get('wheeldrop_left'))}   R: {yn(bumps.get('wheeldrop_right'))}")
    row += 3

    stdscr.addstr(row, col, "Cliff:")
    stdscr.addstr(row+1, col, f"  Left:{yn(cliffs.get('left'))}  FL:{yn(cliffs.get('front_left'))}")
    stdscr.addstr(row+2, col, f"  FR:{yn(cliffs.get('front_right'))}  Right:{yn(cliffs.get('right'))}")
    row += 4

    volt = battery.get('voltage_mV', 0)
    curr = battery.get('current_mA', 0)
    temp = battery.get('temperature_C', 0)
    pct  = battery.get('charge_pct', 0)
    stdscr.addstr(row,   col, "Battery:")
    stdscr.addstr(row+1, col, f"  {volt} mV  {curr} mA  {temp}°C")
    stdscr.addstr(row+2, col, f"  Charge: {pct}%")
    row += 4

    enc_l = encoders.get('left', 0)
    enc_r = encoders.get('right', 0)
    stdscr.addstr(row, col, f"Encoders  L:{enc_l:>6}  R:{enc_r:>6}")

    # --- Right column: drive + hotkeys ---
    col2 = w // 2
    row2 = 2

    stdscr.addstr(row2, col2, "[ DRIVE ]", curses.A_BOLD)
    row2 += 1
    stdscr.addstr(row2,   col2, f"  Direction: {direction_label(left, right):<12}")
    stdscr.addstr(row2+1, col2, f"  L: {left:>5} mm/s")
    stdscr.addstr(row2+2, col2, f"  R: {right:>5} mm/s")
    row2 += 4

    stdscr.addstr(row2, col2, "[ CONTROLS ]", curses.A_BOLD)
    row2 += 1
    controls = [
        ("W/A/S/D", "Drive"),
        ("1",       "Mass Destruction"),
        ("2",       "La Cucaracha"),
        ("T",       "Square demo"),
        ("R",       "Reset Roomba"),
        ("X",       "Power off"),
        ("Q/ESC",   "Quit"),
    ]
    for key, desc in controls:
        stdscr.addstr(row2, col2, f"  {key:<8} {desc}")
        row2 += 1

    # Status bar
    if status_msg:
        stdscr.addstr(h - 2, 2, f">> {status_msg}", curses.A_BOLD)

    # Footer
    stdscr.attron(curses.A_REVERSE)
    stdscr.addstr(h - 1, 0, " Q/ESC to quit ".center(w - 1))
    stdscr.attroff(curses.A_REVERSE)

    stdscr.refresh()

# ------------------------------------------------------------------
# Main
# ------------------------------------------------------------------

def run(stdscr, roomba, device, speed):
    curses.curs_set(0)

    pressed  = set()
    hotkeys  = []
    sensors  = {}
    k_lock   = threading.Lock()
    s_lock   = threading.Lock()
    stop_evt = threading.Event()

    # Start background threads
    kt = threading.Thread(target=key_reader,    args=(device, pressed, hotkeys, k_lock, stop_evt), daemon=True)
    st = threading.Thread(target=sensor_poller, args=(roomba, sensors, s_lock, stop_evt), daemon=True)
    kt.start()
    st.start()

    last_left, last_right = None, None
    status_msg = ""
    action_thread = None

    try:
        while True:
            with k_lock:
                active  = set(pressed)
                pending = list(hotkeys)
                hotkeys.clear()

            # Quit
            if 'q' in active or 'esc' in active:
                break

            # Handle hotkeys
            for key in pending:
                if key == 'r':
                    status_msg = "Resetting..."
                    draw(stdscr, sensors, s_lock, 0, 0, status_msg)
                    roomba.stop()
                    roomba.reset()
                    roomba.start()
                    roomba.full_mode()
                    status_msg = "Reset complete."

                elif key == 'x':
                    status_msg = "Powering off..."
                    draw(stdscr, sensors, s_lock, 0, 0, status_msg)
                    roomba.stop()
                    roomba._send(133)
                    time.sleep(0.5)
                    break

                elif key == '1' and (action_thread is None or not action_thread.is_alive()):
                    status_msg = "Playing: Mass Destruction"
                    def play_md():
                        load_song(roomba, 0, MASS_DESTRUCTION)
                        play_song(roomba, 0)
                        time.sleep(song_duration(MASS_DESTRUCTION))
                    action_thread = threading.Thread(target=play_md, daemon=True)
                    action_thread.start()

                elif key == '2' and (action_thread is None or not action_thread.is_alive()):
                    status_msg = "Playing: La Cucaracha"
                    def play_lc():
                        load_song(roomba, 0, LA_CUCARACHA_1)
                        load_song(roomba, 1, LA_CUCARACHA_2)
                        play_song(roomba, 0)
                        time.sleep(song_duration(LA_CUCARACHA_1) + 0.1)
                        play_song(roomba, 1)
                        time.sleep(song_duration(LA_CUCARACHA_2))
                    action_thread = threading.Thread(target=play_lc, daemon=True)
                    action_thread.start()

                elif key == 't' and (action_thread is None or not action_thread.is_alive()):
                    status_msg = "Running: Square demo"
                    def run_square():
                        for _ in range(4):
                            forward(roomba, speed, 600)
                            time.sleep(0.3)
                            turn_left(roomba, 150, 90)
                            time.sleep(0.3)
                    action_thread = threading.Thread(target=run_square, daemon=True)
                    action_thread.start()

            # Drive
            left, right = compute_wheel_speeds(active, speed)
            if (left, right) != (last_left, last_right):
                roomba.drive_direct(left, right)
                last_left, last_right = left, right
                if left == 0 and right == 0:
                    status_msg = ""

            draw(stdscr, sensors, s_lock, left, right, status_msg)
            time.sleep(0.05)

    finally:
        stop_evt.set()


def main():
    parser = argparse.ArgumentParser(description='Roomba terminal control panel')
    parser.add_argument('--list-devices', action='store_true',
                        help='List available keyboard input devices and exit')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial port (e.g. /dev/ttyUSB0)')
    parser.add_argument('--device', default=None,
                        help='Input device path (e.g. /dev/input/event3)')
    parser.add_argument('--speed', type=int, default=300,
                        help='Drive speed in mm/s (default: 300)')
    args = parser.parse_args()

    if args.list_devices:
        list_keyboards()
        return

    device = find_keyboard(args.device)
    if device is None:
        print("Error: no keyboard device found. Use --list-devices to see options.")
        return

    print(f"Using input device: {device.path} ({device.name})")
    print(f"Connecting on {args.port}...")

    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)
        curses.wrapper(run, roomba, device, args.speed)


if __name__ == '__main__':
    main()
