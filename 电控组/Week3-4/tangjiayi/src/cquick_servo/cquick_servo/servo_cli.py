#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""servo_cli —— 任务 1 的桌面调试小工具（不依赖 ROS，插上 USB 就能用）。

任务 1 允许用串口助手手动发指令，但手敲 20 遍 `#001PID!` 太慢，
而且截图里的时间戳容易和指令对不上。这个工具就是把这些动作固定下来。

用法：
    # 扫 ID（把 1~6 都问一遍 PID，谁回话谁在线）
    python3 servo_cli.py --port /dev/ttyUSB0 scan

    # 单步验证链路（就是细则里那四步）
    python3 servo_cli.py --port /dev/ttyUSB0 read-id
    python3 servo_cli.py --port /dev/ttyUSB0 torque-on
    python3 servo_cli.py --port /dev/ttyUSB0 move 1500 1000
    python3 servo_cli.py --port /dev/ttyUSB0 read-pos

    # 标定：所有舵机一次性回 1500（装舵臂之前做）
    python3 servo_cli.py --port /dev/ttyUSB0 all-center --ids 1 2 3 4

    # 卸力（下电前必做）
    python3 servo_cli.py --port /dev/ttyUSB0 torque-off
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List

from cquick_servo import servo_protocol as proto
from cquick_servo.serial_link import SerialLink, SerialLinkError


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description='CQuick 总线舵机桌面调试工具')
    parser.add_argument('--port', default='/dev/ttyUSB0', help='串口设备名')
    parser.add_argument('--baud', type=int, default=115200, help='波特率，默认 115200')
    parser.add_argument('--ids', type=int, nargs='*', default=[1],
                        help='涉及的舵机 ID 列表，默认 1')
    sub = parser.add_subparsers(dest='command', required=True)

    sub.add_parser('scan', help='扫 ID：逐个发 PID 看谁在线')
    sub.add_parser('read-id', help='发 #001PID!，看回显')
    sub.add_parser('read-pos', help='发 #xxxPRAD!，读当前位置')
    sub.add_parser('torque-on', help='发 #xxxPULR!，恢复扭力')
    sub.add_parser('torque-off', help='发 #xxxPULK!，卸力')
    sub.add_parser('all-center', help='所有 ID 依次回中位 1500')

    move = sub.add_parser('move', help='转角度：move <pwm> [duration_ms]')
    move.add_argument('pwm', type=int)
    move.add_argument('duration_ms', type=int, nargs='?', default=1000)

    center = sub.add_parser('center', help='单个 ID 回中位')
    center.add_argument('duration_ms', type=int, nargs='?', default=800)
    return parser


def ask(link: SerialLink, frame: bytes, servo_id: int,
        timeout: float = 0.3) -> None:
    """发一帧并把回显打出来。"""
    reply = link.query(frame, servo_id, (), timeout)
    sent = frame.decode('ascii')
    if reply is None:
        print(f'  发 {sent:<16} -> 无回显（ID 不对 / 没接线 / 波特率不对）')
    else:
        print(f'  发 {sent:<16} -> 回 {reply.raw}（value={reply.value}）')


def main(argv: List[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    link = SerialLink(args.port, args.baud)
    try:
        link.open()
    except SerialLinkError as exc:
        print(f'[ERROR] {exc}')
        print('        查三件事：① 设备名（ls -l /dev/ttyUSB*）② 权限（dialout 组）'
              '③ 是不是只能充电的线')
        return 2

    print(f'[OK] 已打开 {args.port} @ {args.baud} 8-N-1')
    try:
        if args.command == 'scan':
            print('扫描 ID 1~254 太慢，默认只扫给定范围；用 --ids 1 2 3 4 指定')
            for servo_id in args.ids:
                ask(link, proto.build_read_id(servo_id), servo_id)

        elif args.command == 'read-id':
            for servo_id in args.ids:
                ask(link, proto.build_read_id(servo_id), servo_id)

        elif args.command == 'read-pos':
            for servo_id in args.ids:
                ask(link, proto.build_read_position(servo_id), servo_id)

        elif args.command == 'torque-on':
            for servo_id in args.ids:
                link.send(proto.build_torque_on(servo_id))
                print(f'  id {servo_id}: 已发 PULR（恢复扭力）')
                time.sleep(0.05)

        elif args.command == 'torque-off':
            for servo_id in args.ids:
                link.send(proto.build_torque_off(servo_id))
                print(f'  id {servo_id}: 已发 PULK（卸力）')
                time.sleep(0.05)

        elif args.command == 'move':
            for servo_id in args.ids:
                frame = proto.build_move_command(servo_id, args.pwm, args.duration_ms)
                link.send(frame)
                print(f'  id {servo_id}: 发 {frame.decode("ascii")}')
                time.sleep(0.05)
            time.sleep(args.duration_ms / 1000.0 + 0.1)
            for servo_id in args.ids:
                position = link.read_position(servo_id)
                print(f'  id {servo_id}: PRAD 读回 {position}（目标 {args.pwm}）')

        elif args.command == 'center':
            for servo_id in args.ids:
                link.send_move(servo_id, 1500, args.duration_ms)
                print(f'  id {servo_id}: 回中位 1500，用时 {args.duration_ms}ms')
                time.sleep(0.05)

        elif args.command == 'all-center':
            print(f'标定中：全部回中位 1500，IDs={args.ids}，每个间隔 250ms')
            for servo_id in args.ids:
                link.send(proto.build_torque_on(servo_id))
                link.send_move(servo_id, 1500, 800)
                time.sleep(0.25)
            time.sleep(1.0)
            for servo_id in args.ids:
                position = link.read_position(servo_id)
                verdict = 'OK' if position is not None and abs(position - 1500) <= 30 else 'CHECK'
                print(f'  id {servo_id}: PRAD {position}  [{verdict}]')
    except SerialLinkError as exc:
        print(f'[ERROR] {exc}')
        return 3
    finally:
        link.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
