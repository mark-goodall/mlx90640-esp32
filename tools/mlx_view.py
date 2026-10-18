#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.9"
# dependencies = [
#     "pyserial>=3.5",
#     "numpy",
#     "matplotlib",
# ]
# ///
"""Live matplotlib view of MLX90640 frames streamed by the ESP32-C3.

Serial reading runs in a background thread; the plot always shows the most
recent frame. Hover over the image to read a pixel's temperature.

Packet format: see src/protocol.rs.
"""

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np
import serial

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mlx_reader import FRAME_LEN, HEIGHT, TYPE_TEXT, WIDTH, PacketReader, decode_frame  # noqa: E402


class FrameSource(threading.Thread):
    """Reads packets from the serial port and keeps the latest frame."""

    def __init__(self, ser):
        super().__init__(daemon=True)
        self.ser = ser
        self.lock = threading.Lock()
        self.latest = None
        self.count = 0
        self.stop = False

    def run(self):
        reader = PacketReader()
        while not self.stop:
            try:
                data = self.ser.read(self.ser.in_waiting or 1)
            except serial.SerialException as e:
                print(f"# serial error: {e}", file=sys.stderr)
                return
            for kind, payload in reader.feed(data):
                if kind == TYPE_TEXT:
                    print("#", payload.decode(errors="replace"), file=sys.stderr)
                elif len(payload) == FRAME_LEN:
                    f = decode_frame(payload)
                    with self.lock:
                        self.latest = f
                        self.count += 1

    def take(self):
        """Returns the newest frame not yet taken, or None."""
        with self.lock:
            f, self.latest = self.latest, None
            return f


def orient(img, args):
    if args.rotate:
        img = np.rot90(img, k=args.rotate // 90)
    if args.flip_h:
        img = img[:, ::-1]
    if args.flip_v:
        img = img[::-1, :]
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("--cmap", default="inferno", help="matplotlib colormap (default inferno)")
    ap.add_argument("--vmin", type=float, help="fixed lower colour limit (°C)")
    ap.add_argument("--vmax", type=float, help="fixed upper colour limit (°C)")
    ap.add_argument(
        "--interp", default="bicubic", help="image interpolation, e.g. nearest, bilinear, bicubic"
    )
    ap.add_argument("--flip-h", action="store_true", help="mirror left/right")
    ap.add_argument("--flip-v", action="store_true", help="mirror top/bottom")
    ap.add_argument("--rotate", type=int, default=0, choices=[0, 90, 180, 270])
    ap.add_argument("--no-markers", action="store_true", help="hide min/max markers")
    ap.add_argument("--reset", action="store_true", help="reset the ESP32-C3 on connect")
    ap.add_argument(
        "--snapshot", metavar="PNG", help="save the plot after --frames frames and exit"
    )
    ap.add_argument("-n", "--frames", type=int, default=10, help="frames before --snapshot")
    args = ap.parse_args()

    import matplotlib

    if args.snapshot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    if not args.snapshot and matplotlib.get_backend().lower() == "agg":
        sys.exit("no GUI display available (is DISPLAY set?); use --snapshot to save a PNG")

    ser = serial.Serial(args.port, timeout=0.1)
    if args.reset:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False
    source = FrameSource(ser)
    source.start()

    shape = (WIDTH, HEIGHT) if args.rotate in (90, 270) else (HEIGHT, WIDTH)
    fig, ax = plt.subplots(figsize=(8, 6.5) if shape[1] > shape[0] else (6, 8))
    fig.canvas.manager.set_window_title("MLX90640")
    im = ax.imshow(
        np.zeros(shape), cmap=args.cmap, interpolation=args.interp, vmin=0, vmax=1
    )
    ax.set_xticks([])
    ax.set_yticks([])
    cbar = fig.colorbar(im, ax=ax, fraction=0.046 * shape[0] / shape[1], pad=0.04)
    cbar.set_label("°C")
    (max_mark,) = ax.plot([], [], "+", color="white", ms=14, mew=2)
    (min_mark,) = ax.plot([], [], "x", color="cyan", ms=10, mew=2)
    max_text = ax.text(0, 0, "", color="white", fontsize=10, fontweight="bold")
    title = ax.set_title("waiting for frames…")
    fig.tight_layout()

    state = {"img": None, "lo": None, "hi": None, "t": [], "frames": 0}

    def fmt_coord(x, y):
        img = state["img"]
        c, r = int(round(x)), int(round(y))
        if img is not None and 0 <= r < img.shape[0] and 0 <= c < img.shape[1]:
            return f"pixel ({c}, {r})  {img[r, c]:.2f} °C"
        return ""

    ax.format_coord = fmt_coord

    def update(f):
        img = orient(np.array(f["temps"]).reshape(HEIGHT, WIDTH), args)
        state["img"] = img
        im.set_data(img)

        # Auto-range with smoothing so the colours don't flicker frame to frame.
        lo, hi = float(img.min()), float(img.max())
        if state["lo"] is None:
            state["lo"], state["hi"] = lo, hi
        else:
            state["lo"] += 0.3 * (lo - state["lo"])
            state["hi"] += 0.3 * (hi - state["hi"])
        vmin = args.vmin if args.vmin is not None else state["lo"]
        vmax = args.vmax if args.vmax is not None else max(state["hi"], vmin + 1.0)
        im.set_clim(vmin, vmax)

        if not args.no_markers:
            r, c = np.unravel_index(img.argmax(), img.shape)
            max_mark.set_data([c], [r])
            # Keep the label inside the image near the edges.
            left = c > img.shape[1] - 6
            max_text.set_position((c - 0.8 if left else c + 0.8, r + 1.5 if r < 2 else r - 0.8))
            max_text.set_horizontalalignment("right" if left else "left")
            max_text.set_text(f"{img[r, c]:.1f}°")
            r, c = np.unravel_index(img.argmin(), img.shape)
            min_mark.set_data([c], [r])

        now = time.monotonic()
        state["t"] = [t for t in state["t"] if now - t < 2.0] + [now]
        fps = (len(state["t"]) - 1) / (now - state["t"][0]) if len(state["t"]) > 1 else 0.0
        title.set_text(
            f"min {lo:.1f}  max {hi:.1f}  mean {img.mean():.1f} °C   "
            f"Ta {f['ambient']:.1f} °C   {fps:.1f} fps   frame {f['frame']}"
        )
        state["frames"] += 1

    try:
        if args.snapshot:
            while state["frames"] < args.frames:
                f = source.take()
                if f is None:
                    time.sleep(0.01)
                    continue
                update(f)
            fig.savefig(args.snapshot, dpi=100)
            print(f"saved {args.snapshot}", file=sys.stderr)
            return

        plt.show(block=False)
        while plt.fignum_exists(fig.number):
            f = source.take()
            if f is not None:
                update(f)
                fig.canvas.draw_idle()
            plt.pause(0.02)
    except KeyboardInterrupt:
        pass
    finally:
        source.stop = True
        source.join(timeout=1)
        ser.close()


if __name__ == "__main__":
    main()
