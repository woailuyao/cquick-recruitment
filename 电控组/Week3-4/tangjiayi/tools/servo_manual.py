#!/usr/bin/env python3
"""手动驱动舵机：扫通道 / 指定通道来回摆 / 定点。

用法：
    python3 servo_manual.py                       # 扫通道 0~11，看舵机在哪一路动
    python3 servo_manual.py --ch 4                # 只在 4 号通道来回摆
    python3 servo_manual.py --ch 4 --pwm 1800     # 4 号通道定到 1800 不动
    python3 servo_manual.py --baud 9600           # 换波特率重试
    python3 servo_manual.py --scan 12 23          # 扫上半排 12~23

前提：舵机插在 24 路 3pin 排针上（不是白色 4pin 座），电池已接、电源灯亮。
"""
import argparse
import time

import serial


def build(ch, pwm, t_ms=700):
    return "#%03dP%dT%d!" % (ch, pwm, t_ms)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--ch", type=int, default=None)
    ap.add_argument("--pwm", type=int, default=None)
    ap.add_argument("--scan", type=int, nargs=2, metavar=("FROM", "TO"), default=None)
    a = ap.parse_args()

    s = serial.Serial(a.port, a.baud, timeout=0.2)
    print("已连上 %s @ %d" % (a.port, a.baud))

    def send(cmd, wait=1.0):
        s.write(cmd.encode())
        s.flush()
        print("  发 " + cmd, flush=True)
        time.sleep(wait)

    # 定点模式
    if a.ch is not None and a.pwm is not None:
        send(build(a.ch, a.pwm), 1.2)
        r = s.read(64)
        print("  回包:", r if r else b"(无)")
        s.close()
        return

    # 决定要试哪些通道
    if a.ch is not None:
        chans = [a.ch]
    elif a.scan:
        chans = list(range(a.scan[0], a.scan[1] + 1))
    else:
        chans = list(range(0, 12))

    print("要试的通道:", chans)
    print("先把这些通道全部回中 1500 ...")
    for ch in chans:
        s.write(build(ch, 1500, 600).encode())
    s.flush()
    time.sleep(1.5)

    for ch in chans:
        print("--- 通道 %d ---" % ch, flush=True)
        for pwm in (1500, 1750, 1250, 1500):
            send(build(ch, pwm), 1.0)

    print("\n扫完。哪一路动了，就用  --ch <那个号>  再跑一次确认。")
    s.close()


if __name__ == "__main__":
    main()
