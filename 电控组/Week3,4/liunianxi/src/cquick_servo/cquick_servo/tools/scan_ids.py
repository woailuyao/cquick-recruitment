"""扫舵机 ID：ID 未知时先跑这个（不依赖 ROS，直接开串口）。

用法：
  python3 tools/scan_ids.py --port /dev/ttyUSB0 --baud 115200 --range 1 12
"""
from __future__ import annotations

import argparse
import sys
import time

import serial


# 拼一帧读 ID：`#IDPID!`；ID 三位补零，和协议层 build_verb 的写法保持一致。
def build_pid(frame_id: int) -> bytes:
    return f"#{int(frame_id):03d}PID!".encode("ascii")


# 扫一个 ID 范围：逐个发 PID，等 wait 秒看有没有回包，有就记进结果列表。
def scan(port: str, baud: int, first: int, last: int, wait: float = 0.15) -> list[int]:
    found: list[int] = []
    # 独占打开串口（with 保证异常也会关）；扫描期间驱动必须停掉，否则两个进程抢同一根串口。
    with serial.Serial(port, baud, timeout=wait) as link:
        for frame_id in range(first, last + 1):
            # 每个 ID 之前清缓冲：防止上一个 ID 的迟到回包被算到这个 ID 头上。
            link.reset_input_buffer()
            link.write(build_pid(frame_id))
            time.sleep(wait)
            # 最多读 64 字节：正常回包只有 8 字节左右，读多了只会拖慢扫描。
            reply = link.read(64).decode("ascii", errors="ignore")
            if reply:
                print(f"ID {frame_id:3d} 应答: {reply.strip()!r}")
                found.append(frame_id)
            else:
                # 无应答也打印：真机上空通道就该是无应答，扫 24 路正好证明“只接了 10 个”。
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
    # 退出码约定：找到 ≥1 个返回 0，一个都没有返回 1，脚本里可直接 if 判断。
    sys.exit(0 if found else 1)


if __name__ == "__main__":
    main()
