#!/usr/bin/env python3
import argparse
import math
import sys
import time

try:
    import serial
except ImportError:
    print("Missing dependency: pyserial")
    print("Install with: pip install pyserial")
    sys.exit(1)


SOF0 = 0x55
SOF1 = 0xAA
FRAME_LEN = 16
COUNTS_PER_REV = 4096.0


def checksum_ok(frame: bytes) -> bool:
    return (sum(frame[:-1]) & 0xFF) == frame[-1]


def parse_frame(frame: bytes):
    valid_mask = frame[2]
    values = []
    p = 3
    for _ in range(6):
        lo = frame[p]
        hi = frame[p + 1] & 0x0F
        raw12 = ((hi << 8) | lo) & 0x0FFF
        deg = raw12 * (360.0 / COUNTS_PER_REV)
        values.append((raw12, deg))
        p += 2
    return valid_mask, values


def read_one_frame(ser: serial.Serial):
    while True:
        b = ser.read(1)
        if not b:
            return None
        if b[0] != SOF0:
            continue

        b2 = ser.read(1)
        if not b2:
            return None
        if b2[0] != SOF1:
            continue

        rest = ser.read(FRAME_LEN - 2)
        if len(rest) != FRAME_LEN - 2:
            return None
        frame = bytes([SOF0, SOF1]) + rest
        if checksum_ok(frame):
            return frame


def fmt_valid(i: int, mask: int) -> str:
    return "OK" if (mask & (1 << i)) else "--"


def main():
    parser = argparse.ArgumentParser(
        description="USART2 encoder frame viewer (16-byte fixed frame)."
    )
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--timeout", type=float, default=0.2, help="Serial read timeout seconds")
    parser.add_argument("--raw", action="store_true", help="Print raw12 only")
    args = parser.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    print(f"Open {args.port} @ {args.baud}")
    print("Waiting frames...")

    frame_count = 0
    t0 = time.time()
    last_stat_t = t0

    try:
        while True:
            frame = read_one_frame(ser)
            if frame is None:
                continue

            frame_count += 1
            valid_mask, vals = parse_frame(frame)
            now = time.time()

            parts = []
            for i, (raw12, deg) in enumerate(vals):
                st = fmt_valid(i, valid_mask)
                if args.raw:
                    parts.append(f"E{i + 1}:{raw12:4d}({st})")
                else:
                    parts.append(f"E{i + 1}:{raw12:4d} {deg:7.2f}deg({st})")

            if (now - last_stat_t) >= 1.0:
                hz = frame_count / (now - t0)
                last_stat_t = now
                sys.stdout.write(f"[{hz:6.1f} Hz] ")
            else:
                sys.stdout.write(" " * 11)

            print(" | ".join(parts))

    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()


if __name__ == "__main__":
    main()
