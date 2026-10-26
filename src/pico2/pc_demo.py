"""Live demo of the Pico 2 sensor board: ToF heat maps and IMU plots.

Reads the stream from main.py over USB serial and shows, in one window:
  - one 4x4 heat map per ToF sensor, left to right (closer = darker)
  - accelerometer x, y, z (m/s^2, gravity included), last few seconds
  - yaw, pitch, roll (degrees), last few seconds
  - a 3D box that turns with the IMU

Keys: r = take the current orientation as level / facing forward (the IMU is
mounted upside down, so this makes the box start upright). Rates and drops
are shown at the top.

Requires: pip install pyserial numpy matplotlib

Usage:
    python pc_demo.py                     # auto-detect the Pico
    python pc_demo.py --port COM8 --max-mm 1500
    python pc_demo.py --save demo.png     # record 5 s, save a screenshot
"""
import argparse
import collections
import sys
import time

import matplotlib.pyplot as plt
import numpy as np
import serial
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from pc_heatmap import SIDE, FrameReader, find_pico_port, yaw_pitch_roll

AXIS_COLORS = ("#d62728", "#2ca02c", "#1f77b4")  # x / yaw, y / pitch, z / roll

# Box drawn for the IMU: long along x (forward), flat in z. Half sizes.
BOX = np.array([1.0, 0.6, 0.15])
BOX_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1)
                        for sz in (-1, 1)]) * BOX
# Corner indices per face (+z, -z, +x, -x, +y, -y). Top, bottom and front get
# their own colors so up and forward are easy to see.
BOX_FACES = [(1, 3, 7, 5), (0, 2, 6, 4), (4, 5, 7, 6), (0, 1, 3, 2),
             (2, 3, 7, 6), (0, 1, 5, 4)]
BOX_COLORS = ["#4292c6", "#636363", "#fd8d3c", "#c6dbef", "#c6dbef", "#c6dbef"]


def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz,
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw)


def quat_conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def quat_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--port", help="serial port (default: auto-detect Pico)")
    parser.add_argument("--names", default="left,middle,right",
                        help="comma-separated ToF sensor names, left to right")
    parser.add_argument("--min-mm", type=int, default=0, help="heat map color minimum")
    parser.add_argument("--max-mm", type=int, default=2000, help="heat map color maximum")
    parser.add_argument("--window", type=float, default=10.0,
                        help="seconds of IMU history to plot")
    parser.add_argument("--save", metavar="PNG",
                        help="record for 5 s, save a screenshot and exit")
    args = parser.parse_args()

    port = args.port or find_pico_port()
    if not port:
        sys.exit("No Pico found. Pass --port (e.g. --port COM8).")
    names = [n.strip() for n in args.names.split(",")]
    n = len(names)

    # The port can only be open in one program: close the MicroPico / REPL
    # terminal and pc_heatmap.py first.
    ser = serial.Serial(port, 115200, timeout=0.1)
    print("Listening on", port)
    reader = FrameReader(ser, n)
    reader.start()

    fig = plt.figure(figsize=(14, 8.5), layout="constrained")
    grid = fig.add_gridspec(2, max(n, 3), height_ratios=(1, 1.1))

    # --- ToF heat maps ---
    cmap = plt.get_cmap("Oranges_r").copy()
    cmap.set_bad("#d0d0d0")  # invalid zones
    images, cell_texts = [], []
    for i, name in enumerate(names):
        ax = fig.add_subplot(grid[0, i])
        im = ax.imshow(np.full((SIDE, SIDE), np.nan), cmap=cmap,
                       vmin=args.min_mm, vmax=args.max_mm)
        ax.set_title("ToF " + name)
        ax.set_xticks([])
        ax.set_yticks([])
        images.append(im)
        cell_texts.append([ax.text(c, r, "", ha="center", va="center", fontsize=9)
                           for r in range(SIDE) for c in range(SIDE)])
    fig.colorbar(images[-1], ax=fig.axes[:n], shrink=0.85,
                 label="distance (mm), gray = no target")
    text_flip_mm = args.min_mm + 0.45 * (args.max_mm - args.min_mm)

    # --- IMU plots ---
    ax_acc = fig.add_subplot(grid[1, 0])
    acc_lines = [ax_acc.plot([], [], color=c, label=l)[0]
                 for c, l in zip(AXIS_COLORS, "xyz")]
    ax_acc.set_ylim(-15, 15)
    ax_acc.set_xlabel("seconds")
    ax_acc.set_ylabel("m/s²")
    ax_acc.legend(loc="upper left", ncol=3)
    ax_acc.grid(alpha=0.3)

    ax_ypr = fig.add_subplot(grid[1, 1])
    ypr_lines = [ax_ypr.plot([], [], color=c, label=l)[0]
                 for c, l in zip(AXIS_COLORS, ("yaw", "pitch", "roll"))]
    ax_ypr.set_ylim(-185, 185)
    ax_ypr.set_yticks(range(-180, 181, 90))
    ax_ypr.set_xlabel("seconds")
    ax_ypr.set_ylabel("degrees")
    ax_ypr.legend(loc="upper left", ncol=3)
    ax_ypr.grid(alpha=0.3)

    ax_box = fig.add_subplot(grid[1, 2], projection="3d")
    ax_box.set_title("orientation (r = reset)")
    for setlim in (ax_box.set_xlim, ax_box.set_ylim, ax_box.set_zlim):
        setlim(-1.1, 1.1)
    ax_box.set_box_aspect((1, 1, 1))
    ax_box.set_xlabel("x")
    ax_box.set_ylabel("y")
    ax_box.set_zlabel("z")
    box = Poly3DCollection([BOX_CORNERS[list(f)] for f in BOX_FACES],
                           facecolors=BOX_COLORS, edgecolors="#333333", alpha=0.9)
    ax_box.add_collection3d(box)

    status = fig.suptitle("waiting for data...")

    # IMU history: time (s, from Pico ticks), accel xyz, yaw pitch roll.
    hist = collections.deque()
    ref = [None]  # orientation taken as level by 'r'
    latest_q = [None]
    first_ticks = [None]
    rate_state = {"t": time.monotonic(), "tof": [0] * n, "imu": 0}

    def on_key(event):
        if event.key == "r" and latest_q[0] is not None:
            ref[0] = latest_q[0]
    fig.canvas.mpl_connect("key_press_event", on_key)

    def update(_):
        # ToF
        frames, counts, drops = reader.latest()
        for i in range(n):
            g = frames[i]
            if g is None:
                continue
            images[i].set_data(g)
            for t, v in zip(cell_texts[i], g.flat):
                if np.isnan(v):
                    t.set_text("x")
                    t.set_color("#555555")
                else:
                    t.set_text(str(int(v)))
                    t.set_color("white" if v < text_flip_mm else "#222222")

        # IMU
        for s in reader.take_imu_history():
            if first_ticks[0] is None:
                first_ticks[0] = s.ticks_ms
            q = s.orientation
            if ref[0] is None:
                ref[0] = q
            rel = quat_mul(quat_conj(ref[0]), q)
            t = ((s.ticks_ms - first_ticks[0]) % (1 << 30)) / 1000.0
            hist.append((t, *s.accel, *yaw_pitch_roll(*rel)))
            latest_q[0] = q
        if hist:
            while hist[-1][0] - hist[0][0] > args.window:
                hist.popleft()
            data = np.array(hist)
            t = data[:, 0]
            for i, line in enumerate(acc_lines):
                line.set_data(t, data[:, 1 + i])
            for i, line in enumerate(ypr_lines):
                line.set_data(t, data[:, 4 + i])
            for ax in (ax_acc, ax_ypr):
                ax.set_xlim(t[-1] - args.window, t[-1])
            last = data[-1]
            ax_acc.set_title("accelerometer  x {:6.2f}  y {:6.2f}  z {:6.2f}  |a| {:5.2f}".format(
                last[1], last[2], last[3], float(np.linalg.norm(last[1:4]))))
            ax_ypr.set_title("orientation  yaw {:6.1f}  pitch {:6.1f}  roll {:6.1f}".format(
                last[4], last[5], last[6]))
            rot = quat_matrix(quat_mul(quat_conj(ref[0]), latest_q[0]))
            corners = BOX_CORNERS @ rot.T
            box.set_verts([corners[list(f)] for f in BOX_FACES])

        # Rates, once a second
        now = time.monotonic()
        dt = now - rate_state["t"]
        if dt >= 1.0:
            imu, imu_count, imu_drops = reader.latest_imu()
            tof_rates = [(c - p) / dt for c, p in zip(counts, rate_state["tof"])]
            imu_rate = (imu_count - rate_state["imu"]) / dt
            rate_state.update(t=now, tof=counts, imu=imu_count)
            status.set_text("ToF  " + "   ".join(
                "{} {:.0f} fps ({} dropped)".format(nm, r, d)
                for nm, r, d in zip(names, tof_rates, drops))
                + "        IMU {:.0f} Hz ({} dropped)".format(imu_rate, imu_drops))

    if args.save:
        end = time.monotonic() + 5
        while time.monotonic() < end:
            time.sleep(0.1)
            update(None)
        update(None)
        fig.savefig(args.save, dpi=90)
        print("saved", args.save)
    else:
        _anim = FuncAnimation(fig, update, interval=40, cache_frame_data=False)
        plt.show()
    ser.close()


if __name__ == "__main__":
    main()
