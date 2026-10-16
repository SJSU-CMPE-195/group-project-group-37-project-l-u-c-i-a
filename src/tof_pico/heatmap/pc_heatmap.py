"""Live 8x8 heat maps from the Pico's VL53L5CX sensors.

Reads frames streamed by pico_stream.py over USB serial and draws one 8x8
heat map per sensor, left to right. Closer objects are darker. The measured
frame rate of each sensor is shown in the window title.

Requires: pip install pyserial numpy matplotlib

Usage:
    python pc_heatmap.py                  # auto-detect the Pico
    python pc_heatmap.py --port COM8
    python pc_heatmap.py --max-mm 1500 --values
    python pc_heatmap.py --smooth 0.5 --interp
"""
import argparse
import sys
import threading
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
import serial.tools.list_ports
from matplotlib.animation import FuncAnimation

PICO_VID = 0x2E8A
ZONES = 64
SIDE = 8

SYNC = b"\xAA\x55"
PACKET_LEN = 2 + 1 + ZONES * 2 + 1  # sync, sensor index, data, checksum


def find_pico_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == PICO_VID:
            return p.device
    return None


class FrameReader(threading.Thread):
    """Parses packets from the Pico in the background and keeps the newest
    frame per sensor. Text received outside packets (setup logs, tracebacks)
    is printed to the console."""

    def __init__(self, ser, n):
        super().__init__(daemon=True)
        self.ser = ser
        self.n = n
        self.lock = threading.Lock()
        self.frames = [None] * n
        self.counts = [0] * n
        self.text = bytearray()

    def run(self):
        buf = bytearray()
        # After a bad packet, bytes up to the next good one are its remains,
        # not text.
        discarding = False
        while True:
            buf += self.ser.read(self.ser.in_waiting or 1)
            while True:
                start = buf.find(SYNC)
                if start < 0:
                    # Keep a trailing 0xAA in case it starts the next sync.
                    end = len(buf) - (1 if buf.endswith(SYNC[:1]) else 0)
                    if not discarding:
                        self._text(buf[:end])
                    del buf[:end]
                    break
                if not discarding:
                    self._text(buf[:start])
                del buf[:start]
                if len(buf) < PACKET_LEN:
                    break

                idx = buf[2]
                data = bytes(buf[3:3 + ZONES * 2])
                if idx >= self.n or (sum(data) & 0xFF) != buf[PACKET_LEN - 1]:
                    # Not a real packet; skip this sync and resync.
                    del buf[:1]
                    discarding = True
                    continue
                del buf[:PACKET_LEN]
                discarding = False
                self.text.clear()

                grid = np.frombuffer(data, dtype="<i2").astype(float).reshape(SIDE, SIDE)
                grid[grid < 0] = np.nan
                with self.lock:
                    self.frames[idx] = grid
                    self.counts[idx] += 1

    def _text(self, chunk):
        self.text += chunk
        while b"\n" in self.text:
            line, _, rest = self.text.partition(b"\n")
            self.text = bytearray(rest)
            line = line.decode("utf-8", "replace").strip()
            # Skip leftovers of corrupted binary packets.
            if line and line.isprintable():
                print("pico:", line)

    def latest(self):
        with self.lock:
            return list(self.frames), list(self.counts)


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
    fps_time = [time.monotonic()]
    artists = images + [t for ts in texts for t in ts]

    def update(_):
        frames, counts = reader.latest()
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
            fps_time[0] = now
            fig.canvas.manager.set_window_title(
                "ToF heat map  |  " + "  ".join(
                    "{} {:.1f} fps".format(name, r) for name, r in zip(names, rates)))
        return artists

    # blit=True redraws only the heat maps, not the whole figure.
    _anim = FuncAnimation(fig, update, interval=20, blit=True, cache_frame_data=False)
    plt.show()
    ser.close()


if __name__ == "__main__":
    main()
