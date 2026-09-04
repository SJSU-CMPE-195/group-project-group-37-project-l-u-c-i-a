"""
control_panel_ssh.py

SSH-compatible terminal control panel for the Roomba.
Reads input from the terminal via curses — works over any SSH session.
No physical keyboard or evdev required.

Usage (run from src/scripts/):
    PYTHONPATH=. python3 ssh/control_panel_ssh.py
    PYTHONPATH=. python3 ssh/control_panel_ssh.py --port /dev/ttyUSB0 --speed 300

Drive controls (hold key to move, release to stop):
    W / ↑    — forward
    S / ↓    — backward
    A / ←    — spin left
    D / →    — spin right
    W+A / W+D — not supported over SSH (only one key at a time)

Hotkeys (single press):
    +/=      — increase speed by 50 mm/s (max 500)
    -        — decrease speed by 50 mm/s (min 50)
    1        — play Mass Destruction
    2        — play La Cucaracha
    T        — drive square demo
    R        — reset Roomba
    X        — power off Roomba
    Q / ESC  — quit

Note:
    SSH terminals send key-repeat events while a key is held. This script
    uses a 200 ms grace window: the Roomba keeps driving as long as a drive
    key was received within the last 200 ms, then stops automatically.
    For best response, lower your terminal's key-repeat delay before connecting:
        xset r rate 150 50
"""

import sys
import os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import argparse
import curses
import threading
import time

try:
    import smbus2
    import RPi.GPIO as GPIO
    _HAS_UPS = True
except ImportError:
    _HAS_UPS = False

# X1202 UPS constants
MAX17040_ADDR = 0x36
GPIO_POWER    = 6    # HIGH = AC OK,      LOW = AC fail
GPIO_CHARGE   = 16   # LOW  = charging,   HIGH = not charging

def read_ups():
    """Read MAX17040 voltage/SOC over I2C and GPIO power/charge pins."""
    if not _HAS_UPS:
        return {}
    result = {}
    try:
        bus = smbus2.SMBus(1)
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


from roomba_oi import RoombaOI
from song import (
    load_song, play_song, song_duration,
    MASS_DESTRUCTION, LA_CUCARACHA_1, LA_CUCARACHA_2
)
from drive_demos import forward, turn_left

# How long (seconds) to keep driving after the last keypress before stopping.
# Must be > the client's key-repeat initial delay (set with: xset r rate 150 50).
# Default 0.3 s bridges the stock ~250 ms delay. Lower to 0.2 s if you have
# applied the recommended xset r rate setting on the client machine.
DRIVE_TIMEOUT = 0.2

# ------------------------------------------------------------------
# Drive helpers  (identical to local control_panel.py)
# ------------------------------------------------------------------

def compute_wheel_speeds(key, speed):
    if key == 'w':
        return speed, speed
    if key == 's':
        return -speed, -speed
    if key == 'a':
        return -speed, speed
    if key == 'd':
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
    return "STOPPED"

# ------------------------------------------------------------------
# UI drawing  (matches local control_panel.py layout)
# ------------------------------------------------------------------

def draw(stdscr, sensors, s_lock, left, right, speed, status_msg):
    stdscr.erase()
    h, w = stdscr.getmaxyx()

    title = " LUCIA — Roomba Control Panel (SSH) "
    stdscr.attron(curses.A_REVERSE)
    stdscr.addstr(0, 0, title.center(w - 1))
    stdscr.attroff(curses.A_REVERSE)

    col = 2
    row = 2

    stdscr.addstr(row, col, "[ SENSORS ]", curses.A_BOLD)
    row += 1

    with s_lock:
        bumps    = sensors.get('bumps', {})
        cliffs   = sensors.get('cliffs', {})
        battery  = sensors.get('battery', {})
        encoders = sensors.get('encoders', {})

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
    row += 2

    ups = sensors.get('ups', {})
    stdscr.addstr(row, col, "[ X1202 UPS ]", curses.A_BOLD)
    row += 1
    if ups:
        volt = ups.get('voltage')
        soc  = ups.get('soc')
        power_ok = ups.get('power_ok')
        charging = ups.get('charging')
        if volt is not None:
            stdscr.addstr(row, col, f"  {volt:.3f} V   {soc:.1f}%")
            row += 1
        if power_ok is not None:
            pwr_str = "AC OK  " if power_ok else "AC FAIL"
            chg_str = "Charging" if charging else "Not charging"
            stdscr.addstr(row, col, f"  {pwr_str}  {chg_str}")
    else:
        stdscr.addstr(row, col, "  (unavailable)")

    col2 = w // 2
    row2 = 2

    stdscr.addstr(row2, col2, "[ DRIVE ]", curses.A_BOLD)
    row2 += 1
    stdscr.addstr(row2,   col2, f"  Direction: {direction_label(left, right):<12}")
    stdscr.addstr(row2+1, col2, f"  L: {left:>5} mm/s")
    stdscr.addstr(row2+2, col2, f"  R: {right:>5} mm/s")
    stdscr.addstr(row2+3, col2, f"  Speed:  {speed:>3} mm/s")
    row2 += 5

    stdscr.addstr(row2, col2, "[ CONTROLS ]", curses.A_BOLD)
    row2 += 1
    controls = [
        ("W/A/S/D", "Drive (hold)"),
        ("+/-",     "Speed +/- 50 mm/s"),
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

    if status_msg:
        stdscr.addstr(h - 2, 2, f">> {status_msg}", curses.A_BOLD)

    stdscr.attron(curses.A_REVERSE)
    stdscr.addstr(h - 1, 0, " Q/ESC to quit ".center(w - 1))
    stdscr.attroff(curses.A_REVERSE)

    stdscr.refresh()

# ------------------------------------------------------------------
# Sensor poller thread
# ------------------------------------------------------------------

def sensor_poller(roomba, sensors, lock, stop_event):
    while not stop_event.is_set():
        try:
            bumps    = roomba.read_bumps()
            cliffs   = roomba.read_cliffs()
            battery  = roomba.read_battery()
            encoders = roomba.read_encoders()
            ups      = read_ups()
            with lock:
                sensors.update({
                    'bumps':    bumps,
                    'cliffs':   cliffs,
                    'battery':  battery,
                    'encoders': encoders,
                    'ups':      ups,
                })
        except Exception:
            pass
        time.sleep(0.5)

# ------------------------------------------------------------------
# Key tables
# ------------------------------------------------------------------

DRIVE_KEYS = {
    ord('w'): 'w', ord('W'): 'w', curses.KEY_UP:    'w',
    ord('s'): 's', ord('S'): 's', curses.KEY_DOWN:  's',
    ord('a'): 'a', ord('A'): 'a', curses.KEY_LEFT:  'a',
    ord('d'): 'd', ord('D'): 'd', curses.KEY_RIGHT: 'd',
}

HOTKEY_KEYS = {
    ord('1'): '1',
    ord('2'): '2',
    ord('t'): 't', ord('T'): 't',
    ord('r'): 'r', ord('R'): 'r',
    ord('x'): 'x', ord('X'): 'x',
    ord('q'): 'q', ord('Q'): 'q',
    ord('+'): '+', ord('='): '+',
    ord('-'): '-',
    27: 'esc',
}

# ------------------------------------------------------------------
# Main loop
# ------------------------------------------------------------------

def run(stdscr, roomba, speed):
    curses.curs_set(0)
    stdscr.keypad(True)
    stdscr.timeout(20)   # getch blocks for at most 20 ms

    if _HAS_UPS:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(GPIO_POWER,  GPIO.IN)
        GPIO.setup(GPIO_CHARGE, GPIO.IN)

    sensors  = {}
    s_lock   = threading.Lock()
    stop_evt = threading.Event()

    st = threading.Thread(target=sensor_poller,
                          args=(roomba, sensors, s_lock, stop_evt), daemon=True)
    st.start()

    current_key    = None   # active drive direction
    last_drive_t   = 0.0    # time of last drive keypress
    last_left      = None
    last_right     = None
    status_msg     = ""
    action_thread  = None

    try:
        while True:
            ch  = stdscr.getch()
            now = time.time()

            if ch in DRIVE_KEYS:
                current_key  = DRIVE_KEYS[ch]
                last_drive_t = now

            elif ch in HOTKEY_KEYS:
                current_key = None
                key = HOTKEY_KEYS[ch]

                if key in ('q', 'esc'):
                    break

                elif key == '+':
                    speed = min(speed + 50, 500)
                    status_msg = f"Speed: {speed} mm/s"

                elif key == '-':
                    speed = max(speed - 50, 50)
                    status_msg = f"Speed: {speed} mm/s"

                elif key == 'r':
                    status_msg = "Resetting..."
                    draw(stdscr, sensors, s_lock, 0, 0, speed, status_msg)
                    roomba.stop()
                    roomba.reset()
                    roomba.start()
                    roomba.full_mode()
                    status_msg = "Reset complete."

                elif key == 'x':
                    status_msg = "Powering off..."
                    draw(stdscr, sensors, s_lock, 0, 0, speed, status_msg)
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

            # Stop driving if key hasn't been seen within DRIVE_TIMEOUT
            if current_key is not None and (now - last_drive_t) > DRIVE_TIMEOUT:
                current_key = None

            left, right = compute_wheel_speeds(current_key, speed)
            if (left, right) != (last_left, last_right):
                roomba.drive_direct(left, right)
                last_left, last_right = left, right
                if left == 0 and right == 0:
                    status_msg = ""

            draw(stdscr, sensors, s_lock, left, right, speed, status_msg)

    finally:
        stop_evt.set()
        roomba.stop()
        if _HAS_UPS:
            GPIO.cleanup()


def main():
    parser = argparse.ArgumentParser(description='Roomba SSH control panel')
    parser.add_argument('--port', default='/dev/ttyUSB0',
                        help='Serial port (default: /dev/ttyUSB0)')
    parser.add_argument('--speed', type=int, default=300,
                        help='Drive speed in mm/s (default: 300)')
    args = parser.parse_args()

    print(f"Connecting on {args.port}...")
    with RoombaOI(args.port) as roomba:
        roomba.start()
        roomba.full_mode()
        time.sleep(0.5)
        curses.wrapper(run, roomba, args.speed)


if __name__ == '__main__':
    main()
