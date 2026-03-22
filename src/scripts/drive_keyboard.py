"""
drive_keyboard.py

Real-time keyboard control for the Roomba using pynput.
Hold keys to drive — release to stop. Supports combined inputs
(e.g. W+A arcs forward-left) via independent wheel speed control.

Requirements:
    pip install pynput

Usage:
    python drive_keyboard.py --port COM5
    python drive_keyboard.py --port /dev/ttyUSB0 --speed 250

Controls:
    W        — forward
    S        — backward
    A        — spin left
    D        — spin right
    W + A    — arc forward-left
    W + D    — arc forward-right
    S + A    — arc backward-left
    S + D    — arc backward-right
    Q / ESC  — quit
"""

import argparse
import time

from pynput import keyboard
from roomba_oi import RoombaOI


# Keys currently held down
pressed = set()
running = True


def on_press(key):
    try:
        pressed.add(key.char.lower())
    except AttributeError:
        pressed.add(key)  # special keys like ESC


def on_release(key):
    global running
    try:
        pressed.discard(key.char.lower())
    except AttributeError:
        pressed.discard(key)

    if key == keyboard.Key.esc or (hasattr(key, 'char') and key.char and key.char.lower() == 'q'):
        running = False
        return False  # stop listener


def compute_wheel_speeds(speed):
    """
    Map the current set of held keys to (left, right) wheel velocities.
    Combined keys produce arced movement instead of jerky step turns.
    """
    w = 'w' in pressed
    s = 's' in pressed
    a = 'a' in pressed
    d = 'd' in pressed

    if not any([w, s, a, d]):
        return 0, 0

    # Forward + turn
    if w and a:
        return speed // 2, speed       # left wheel slower → arc left
    if w and d:
        return speed, speed // 2       # right wheel slower → arc right

    # Backward + turn
    if s and a:
        return -(speed // 2), -speed
    if s and d:
        return -speed, -(speed // 2)

    # Straight / spin in place
    if w:
        return speed, speed
    if s:
        return -speed, -speed
    if a:
        return -speed, speed           # spin CCW
    if d:
        return speed, -speed           # spin CW

    return 0, 0


def print_status(left, right):
    if left > 0 and right > 0:
        arrow = "▲  FORWARD"
    elif left < 0 and right < 0:
        arrow = "▼  BACKWARD"
    elif left > 0 and right <= 0:
        arrow = "↻  SPIN RIGHT"
    elif left <= 0 and right > 0:
        arrow = "↺  SPIN LEFT"
    elif left != right and left > 0:
        arrow = "↗  ARC RIGHT"
    elif left != right and right > 0:
        arrow = "↖  ARC LEFT"
    else:
        arrow = "■  STOPPED"

    print(f"\r  {arrow:<20}  L:{left:>5} mm/s   R:{right:>5} mm/s    ", end='', flush=True)


def main():
    parser = argparse.ArgumentParser(description='Roomba real-time keyboard control')
    parser.add_argument('--port', default='COM5',
                        help='Serial port (e.g. COM5 or /dev/ttyUSB0)')
    parser.add_argument('--speed', type=int, default=200,
                        help='Base wheel speed in mm/s (default: 200, max: 500)')
    args = parser.parse_args()

    speed = max(50, min(500, args.speed))

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)

        print("\nReady. Use W/A/S/D to drive, Q or ESC to quit.\n")

        listener = keyboard.Listener(on_press=on_press, on_release=on_release)
        listener.start()

        last_left, last_right = None, None

        try:
            while running and listener.is_alive():
                left, right = compute_wheel_speeds(speed)

                # Only send a command if the state changed — avoids flooding serial
                if (left, right) != (last_left, last_right):
                    roomba.drive_direct(left, right)
                    print_status(left, right)
                    last_left, last_right = left, right

                time.sleep(0.05)  # 20 Hz loop

        except KeyboardInterrupt:
            pass

        finally:
            listener.stop()
            roomba.stop()
            print("\n\nDisconnected.")


if __name__ == '__main__':
    main()
