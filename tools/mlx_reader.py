#!/usr/bin/env -S uv run --script
# /// script
# requires-python = ">=3.8"
# dependencies = [
#     "pyserial>=3.5",
# ]
# ///
"""Read binary MLX90640 frames from the ESP32-C3 and print them as CSV.

Each frame is printed as 24 lines of 32 comma-separated temperatures (deg C),
followed by a blank line. Status messages from the device go to stderr.

Packet format: see src/protocol.rs.
"""

import argparse
import struct
import sys
import time

import serial

MAGIC = b"\xa5\x5aMX"
TYPE_FRAME = 0
TYPE_TEXT = 1
WIDTH, HEIGHT = 32, 24
PIXELS = WIDTH * HEIGHT
FRAME_HEADER = struct.Struct("<IIIIh")
FRAME_LEN = FRAME_HEADER.size + PIXELS * 2
MAX_PAYLOAD = 4096


def crc16(data, crc=0xFFFF):
    """CRC-16/CCITT-FALSE."""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc


class PacketReader:
    """Extracts (type, payload) packets from a byte stream, resyncing on
    garbage such as the ROM bootloader's boot messages."""

    def __init__(self):
        self.buf = bytearray()
        self.crc_errors = 0

    def feed(self, data):
        self.buf += data
        while True:
            start = self.buf.find(MAGIC)
            if start < 0:
                # Keep a possible partial magic at the end.
                del self.buf[: max(0, len(self.buf) - len(MAGIC) + 1)]
                return
            del self.buf[:start]
            if len(self.buf) < 7:
                return
            kind = self.buf[4]
            length = self.buf[5] | (self.buf[6] << 8)
            if kind not in (TYPE_FRAME, TYPE_TEXT) or length > MAX_PAYLOAD:
                del self.buf[:1]
                continue
            total = 7 + length + 2
            if len(self.buf) < total:
                return
            payload = bytes(self.buf[7 : 7 + length])
            crc = self.buf[7 + length] | (self.buf[8 + length] << 8)
            if crc16(payload, crc16(self.buf[4:7])) != crc:
                self.crc_errors += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            yield kind, payload


def decode_frame(payload):
    frame, time_ms, subpages, retries, ambient = FRAME_HEADER.unpack_from(payload)
    pixels = struct.unpack_from(f"<{PIXELS}h", payload, FRAME_HEADER.size)
    return {
        "frame": frame,
        "time_ms": time_ms,
        "subpages": subpages,
        "retries": retries,
        "ambient": ambient / 100.0,
        "temps": [p / 100.0 for p in pixels],
    }


def format_csv(temps, precision):
    fmt = f"{{:.{precision}f}}"
    rows = (
        ",".join(fmt.format(t) for t in temps[r * WIDTH : (r + 1) * WIDTH])
        for r in range(HEIGHT)
    )
    return "\n".join(rows) + "\n\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("port", nargs="?", default="/dev/ttyACM0")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-n", "--frames", type=int, default=0, help="stop after N frames (0 = forever)")
    ap.add_argument("-o", "--output", help="write CSV to this file instead of stdout")
    ap.add_argument("-p", "--precision", type=int, default=2, help="decimal places")
    ap.add_argument("--reset", action="store_true", help="reset the ESP32 on connect")
    ap.add_argument("--stats", action="store_true", help="print per-frame stats to stderr")
    args = ap.parse_args()

    out = open(args.output, "w") if args.output else sys.stdout
    ser = serial.Serial(args.port, args.baud, timeout=0.1)
    if args.reset:
        ser.dtr = False
        ser.rts = True
        time.sleep(0.1)
        ser.rts = False

    reader = PacketReader()
    count = 0
    last = None
    try:
        while not args.frames or count < args.frames:
            for kind, payload in reader.feed(ser.read(ser.in_waiting or 1)):
                if kind == TYPE_TEXT:
                    print("#", payload.decode(errors="replace"), file=sys.stderr)
                    continue
                if len(payload) != FRAME_LEN:
                    print(f"# unexpected frame length {len(payload)}", file=sys.stderr)
                    continue
                f = decode_frame(payload)
                out.write(format_csv(f["temps"], args.precision))
                out.flush()
                count += 1
                if args.stats:
                    t = f["temps"]
                    msg = (
                        f"# frame {f['frame']} Ta {f['ambient']:.2f} "
                        f"min {min(t):.2f} max {max(t):.2f} "
                        f"subpages {f['subpages']} retries {f['retries']}"
                    )
                    if last is not None and f["time_ms"] > last["time_ms"]:
                        dt = (f["time_ms"] - last["time_ms"]) / 1000.0
                        msg += f" {1 / dt:.2f} fps"
                    if reader.crc_errors:
                        msg += f" crc_errors {reader.crc_errors}"
                    print(msg, file=sys.stderr)
                last = f
                if args.frames and count >= args.frames:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if out is not sys.stdout:
            out.close()


if __name__ == "__main__":
    main()
