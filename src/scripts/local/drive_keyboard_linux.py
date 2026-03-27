"""
drive_keyboard_linux.py

Real-time keyboard control for the Roomba on Linux using evdev.
Reads directly from /dev/input for true key press and release events —
no jitter, no grace period hacks. Behaves identically to the Windows version.

Requirements:
    pip install evdev

Permissions:
    sudo usermod -aG input $USER   (then log out and back in)
    -- or run with sudo --

Usage:
    python3 drive_keyboard_linux.py --port /dev/ttyUSB0
    python3 drive_keyboard_linux.py --port /dev/ttyUSB0 --speed 300

Controls:
    W / ↑    — forward
    S / ↓    — backward
    A / ←    — spin left
    D / →    — spin right
    W + A    — arc forward-left
    W + D    — arc forward-right
    S + A    — arc backward-left
    S + D    — arc backward-right
    Q / ESC  — quit
"""

import argparse
import time
import threading

import evdev
from evdev import ecodes

from roomba_oi import RoombaOI

# Maps evdev key codes to our control keys
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
}


def list_keyboards():
    """Print all input devices that have keyboard keys."""
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
    """Return an input device. Uses device_path if given, otherwise auto-detects."""
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


def read_events(device, pressed, lock, stop_event):
    """
    Background thread — listens for key press and release events
    and maintains the pressed set accordingly.
    Ignores key repeat events (value=2).
    """
    device.grab()  # exclusive access — stops keys going to the terminal
    try:
        for event in device.read_loop():
            if stop_event.is_set():
                break
            if event.type == ecodes.EV_KEY and event.value in (0, 1):
                key = KEY_MAP.get(event.code)
                if key:
                    with lock:
                        if event.value == 1:    # key down
                            pressed.add(key)
                        else:                   # key up
                            pressed.discard(key)
    finally:
        device.ungrab()


def compute_wheel_speeds(pressed, speed):
    """
    Map the set of currently held keys to (left, right) wheel velocities.
    Combined keys produce arced movement.
    """
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


def print_status(left, right):
    if left > 0 and right > 0 and left == right:
        arrow = "▲  FORWARD"
    elif left < 0 and right < 0 and left == right:
        arrow = "▼  BACKWARD"
    elif left > 0 and right < 0:
        arrow = "↻  SPIN RIGHT"
    elif left < 0 and right > 0:
        arrow = "↺  SPIN LEFT"
    elif left != right and left > 0:
        arrow = "↗  ARC RIGHT"
    elif left != right and right > 0:
        arrow = "↖  ARC LEFT"
    else:
        arrow = "■  STOPPED"

    print(f"\r  {arrow:<20}  L:{left:>5} mm/s   R:{right:>5} mm/s    ", end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description='Roomba keyboard control (Linux)')
    parser.add_argument('--list-devices', action='store_true',
                        help='List available keyboard input devices and exit')
    parser.add_argument('--device', default=None,
                        help='Input device path (e.g. /dev/input/event4). Auto-detected if omitted.')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial port (e.g. /dev/ttyUSB0)')
    parser.add_argument('--speed', type=int, default=300,
                        help='Base wheel speed in mm/s (default: 300, max: 500)')
    args = parser.parse_args()

    speed = max(50, min(500, args.speed))

    if args.list_devices:
        list_keyboards()
        return

    # Find keyboard device
    device = find_keyboard(args.device)
    if device is None:
        print("Error: no keyboard input device found.")
        print("Try: sudo usermod -aG input $USER  (then log out and back in)")
        return

    print(f"Using input device: {device.path} ({device.name})")
    print(f"Connecting on {args.port}...")

    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)

        print("\nReady. Use W/A/S/D or arrow keys to drive, Q or ESC to quit.\n")

        pressed = set()
        lock = threading.Lock()
        stop_event = threading.Event()

        reader = threading.Thread(
            target=read_events,
            args=(device, pressed, lock, stop_event),
            daemon=True
        )
        reader.start()

        last_left, last_right = None, None

        try:
            while True:
                with lock:
                    active = set(pressed)

                if 'q' in active or 'esc' in active:
                    break

                left, right = compute_wheel_speeds(active, speed)

                if (left, right) != (last_left, last_right):
                    roomba.drive_direct(left, right)
                    print_status(left, right)
                    last_left, last_right = left, right

                time.sleep(0.05)  # 20 Hz drive loop

        except KeyboardInterrupt:
            pass

        finally:
            stop_event.set()
            roomba.stop()
            print("\n\nDisconnected.")


if __name__ == '__main__':
    main()
