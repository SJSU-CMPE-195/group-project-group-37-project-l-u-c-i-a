"""Live 4x4 heat maps from the Pico's VL53L5CX sensors, plus IMU readout.

Reads packets streamed by main.py over USB serial and draws one 4x4 heat map
per sensor, left to right. Closer objects are darker. The measured frame rate
and dropped frame count of each sensor, and the IMU's rate and orientation,
are shown in the window title.

Requires: pip install pyserial numpy matplotlib

Usage:
    python pc_heatmap.py                  # auto-detect the Pico
    python pc_heatmap.py --port COM8
    python pc_heatmap.py --max-mm 1500 --values
    python pc_heatmap.py --smooth 0.5 --interp
"""
import argparse
import collections
import math
import sys
import threading
import time
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.animation import FuncAnimation

# Same parser as the Pi's ROS node; it has no ROS dependencies.
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2" / "lucia_tof"))
from lucia_tof import protocol  # noqa: E402

PICO_VID = 0x2E8A
SIDE = protocol.SIDE
# A sequence jump bigger than this is taken as a Pico restart, not drops.
MAX_SEQ_GAP = 1000


def find_pico_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == PICO_VID:
            return p.device
    return None


def count_drops(last, seq):
    """Packets missed between sequence numbers last and seq."""
    if last is None:
        return 0
    gap = (seq - last - 1) & 0xFFFF
    return gap if gap <= MAX_SEQ_GAP else 0


def yaw_pitch_roll(w, x, y, z):
    """Quaternion to yaw, pitch, roll in degrees (ZYX)."""
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    return math.degrees(yaw), math.degrees(pitch), math.degrees(roll)


class FrameReader(threading.Thread):
    """Parses packets from the Pico in the background and keeps the newest
    frame per sensor and the newest IMU sample. Text received outside packets
    (setup logs, tracebacks) is printed to the console."""

    def __init__(self, ser, n):
        super().__init__(daemon=True)
        self.ser = ser
        self.n = n
        self.parser = protocol.Parser(n)
        self.lock = threading.Lock()
        self.frames = [None] * n
        self.counts = [0] * n
        self.drops = [0] * n
        self.seqs = [None] * n
        self.imu = None
        self.imu_count = 0
        self.imu_drops = 0
        self.imu_seq = None
        # Every IMU sample, for plotting; take_imu_history() empties it.
        self.imu_history = collections.deque(maxlen=5000)

    def run(self):
        while True:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except (serial.SerialException, OSError, TypeError):
                # Port closed on exit, or the Pico was unplugged.
                return
            for item in self.parser.feed(data):
                if isinstance(item, str):
                    print("pico:", item)
                elif isinstance(item, protocol.ImuPacket):
                    with self.lock:
                        self.imu_drops += count_drops(self.imu_seq, item.seq)
                        self.imu_seq = item.seq
                        self.imu = item
                        self.imu_count += 1
                        self.imu_history.append(item)
                else:
                    grid = np.array(item.distances, dtype=float).reshape(SIDE, SIDE)
                    grid[grid < 0] = np.nan
                    idx = item.sensor
                    with self.lock:
                        self.drops[idx] += count_drops(self.seqs[idx], item.seq)
                        self.seqs[idx] = item.seq
                        self.frames[idx] = grid
                        self.counts[idx] += 1

    def latest(self):
        with self.lock:
            return list(self.frames), list(self.counts), list(self.drops)

    def latest_imu(self):
        with self.lock:
            return self.imu, self.imu_count, self.imu_drops

    def take_imu_history(self):
        """IMU samples received since the last call, oldest first."""
        with self.lock:
            samples = list(self.imu_history)
            self.imu_history.clear()
            return samples


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port (default: auto-detect Pico)")
    parser.add_argument("--names", default="left,middle,right",
                        help="comma-separated sensor names, left to right")
    parser.add_argument("--min-mm", type=int, default=0, help="color scale minimum")
    parser.add_argument("--max-mm", type=int, default=2000, help="color scale maximum")
    parser.add_argument("--values", action="store_true", help="print mm in each cell")
    parser.add_argument("--smooth", type=float, default=0.0,
                        help="temporal smoothing 0..0.95 (0 = off); also holds the "
                             "last good value when a zone briefly drops out")
    parser.add_argument("--interp", action="store_true",
                        help="draw smooth gradients between zones instead of blocks")
    parser.add_argument("--flip-h", action="store_true", help="mirror each grid left/right")
    parser.add_argument("--flip-v", action="store_true", help="mirror each grid top/bottom")
    args = parser.parse_args()

    port = args.port or find_pico_port()
    if not port:
        sys.exit("No Pico found. Pass --port (e.g. --port COM8).")

    names = [n.strip() for n in args.names.split(",")]
    n = len(names)

    # The port can only be open in one program: close the MicroPico / REPL
    # terminal first.
    ser = serial.Serial(port, 115200, timeout=0.1)
    print("Listening on", port)
    reader = FrameReader(ser, n)
    reader.start()

    cmap = plt.get_cmap("Oranges_r").copy()
    cmap.set_bad("#d0d0d0")  # invalid zones

    fig, axes = plt.subplots(1, n, figsize=(3.6 * n + 1, 4), squeeze=False,
                             layout="constrained")
    axes = axes[0]
    empty = np.full((SIDE, SIDE), np.nan)
    images, texts = [], []
    for ax, name in zip(axes, names):
        im = ax.imshow(empty, cmap=cmap, vmin=args.min_mm, vmax=args.max_mm,
                       interpolation="bicubic" if args.interp else "nearest")
        ax.set_title(name)
        ax.set_xticks([])
        ax.set_yticks([])
        images.append(im)
        cell_texts = []
        if args.values:
            for r in range(SIDE):
                for c in range(SIDE):
                    cell_texts.append(ax.text(c, r, "", ha="center", va="center",
                                              fontsize=7, color="#222222"))
        texts.append(cell_texts)

    cbar = fig.colorbar(images[0], ax=list(axes), shrink=0.9)
    cbar.set_label("distance (mm)  -  gray = no valid target")

    text_flip_mm = args.min_mm + 0.45 * (args.max_mm - args.min_mm)
    alpha = min(max(args.smooth, 0.0), 0.95)
    shown = [None] * n
    last_counts = [0] * n
    fps_counts = [0] * n
    imu_fps_count = [0]
    fps_time = [time.monotonic()]
    artists = images + [t for ts in texts for t in ts]

    def update(_):
        frames, counts, drops = reader.latest()
        for i in range(n):
            if frames[i] is None or counts[i] == last_counts[i]:
                continue
            last_counts[i] = counts[i]

            g = frames[i]
            if args.flip_h:
                g = g[:, ::-1]
            if args.flip_v:
                g = g[::-1, :]
            if alpha and shown[i] is not None:
                prev = shown[i]
                g = np.where(np.isnan(g), prev,
                             np.where(np.isnan(prev), g, alpha * prev + (1 - alpha) * g))
            shown[i] = g

            images[i].set_data(g)
            for t, v in zip(texts[i], g.flat):
                if np.isnan(v):
                    t.set_text("x")
                    t.set_color("#555555")
                else:
                    t.set_text(str(int(v)))
                    # Light text on the dark (near) end of the scale.
                    t.set_color("white" if v < text_flip_mm else "#222222")

        # Measured frames per second per sensor, in the window title.
        now = time.monotonic()
        if now - fps_time[0] >= 1.0:
            rates = [(c - p) / (now - fps_time[0]) for c, p in zip(counts, fps_counts)]
            fps_counts[:] = counts
            imu, imu_count, imu_drops = reader.latest_imu()
            imu_rate = (imu_count - imu_fps_count[0]) / (now - fps_time[0])
            imu_fps_count[0] = imu_count
            fps_time[0] = now
            title = "ToF heat map  |  " + "  ".join(
                "{} {:.1f} fps, {} dropped".format(name, r, d)
                for name, r, d in zip(names, rates, drops))
            if imu is not None:
                title += "  |  IMU {:.0f} Hz, {} dropped, yaw {:.1f} pitch {:.1f} roll {:.1f}".format(
                    imu_rate, imu_drops, *yaw_pitch_roll(*imu.orientation))
            fig.canvas.manager.set_window_title(title)
        return artists

    # blit=True redraws only the heat maps, not the whole figure.
    _anim = FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == "__main__":
    main()
