"""扫舵机 ID：ID 未知时先跑这个（不依赖 ROS，直接开串口）。

用法：
  python3 tools/scan_ids.py --port /dev/ttyUSB0 --baud 115200 --range 1 12
"""
from __future__ import annotations

import argparse
import sys
import time

import serial


def build_pid(frame_id: int) -> bytes:
    return f"#{int(frame_id):03d}PID!".encode("ascii")


def scan(port: str, baud: int, first: int, last: int, wait: float = 0.15) -> list[int]:
    found: list[int] = []
    with serial.Serial(port, baud, timeout=wait) as link:
        for frame_id in range(first, last + 1):
            link.reset_input_buffer()
            link.write(build_pid(frame_id))
            time.sleep(wait)
            reply = link.read(64).decode("ascii", errors="ignore")
            if reply:
                print(f"ID {frame_id:3d} 应答: {reply.strip()!r}")
                found.append(frame_id)
            else:
                print(f"ID {frame_id:3d} 无应答")
    return found


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description="扫总线舵机 ID")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--range", nargs=2, type=int, default=[1, 12], metavar=("FIRST", "LAST"))
    args = parser.parse_args(argv)
    found = scan(args.port, args.baud, args.range[0], args.range[1])
    print(f"共找到 {len(found)} 个：{found}")
    sys.exit(0 if found else 1)


if __name__ == "__main__":
    main()
