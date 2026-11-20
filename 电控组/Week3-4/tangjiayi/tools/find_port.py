#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""串口自检：找出 TTL 到底是哪个设备名，并确认舵机在线。

用法：
    python3 find_port.py              # 扫所有候选口，报哪个能通
    python3 find_port.py --lsusb      # 连 USB 层一起看（需要 sudo dmesg 权限时更直观）

输出最后一行的 "可用端口: /dev/ttyXXX" 就是给 launch 用的值。
"""

import glob
import sys
import time

try:
    import serial
except ImportError:
    raise SystemExit('缺 python3-serial，先跑：sudo apt install -y python3-serial')

BAUDS = (115200, 9600, 250000, 1000000)


def candidates():
    """按优先级列出候选串口：优先 by-id 固定名，其次 ttyUSB/ttyACM。"""
    out = []
    for pat in ('/dev/serial/by-id/*', '/dev/ttyUSB*', '/dev/ttyACM*'):
        for p in sorted(glob.glob(pat)):
            if p not in out:
                out.append(p)
    return out


def probe(port, baud):
    """发版本查询，返回回包（bytes），失败返回 None。"""
    try:
        s = serial.Serial(port, baud, timeout=0.3)
    except Exception as e:
        return None, '打不开：%s' % e
    try:
        s.reset_input_buffer()
        s.write(b'#000PVER!')
        s.flush()
        time.sleep(0.3)
        r = s.read(128)
        if not r:
            # 再试一次，有些板子第一帧会丢
            s.reset_input_buffer()
            s.write(b'#000PVER!')
            s.flush()
            time.sleep(0.35)
            r = s.read(128)
        return r, None
    finally:
        s.close()


def main():
    print('=' * 60)
    print(' 串口自检')
    print('=' * 60)

    cands = candidates()
    if not cands:
        print('\n❌ 一个串口设备都没有。')
        print('   → TTL 没插 / USB 线是纯充电线 / 虚拟机没把 USB 设备接进来')
        print('   排查：再插一次 TTL，看右下角有没有弹出“是否连接到虚拟机”，选“连接到虚拟机”')
        return 1

    print('\n找到 %d 个候选设备：' % len(cands))
    for p in cands:
        print('   ', p)

    ok = None
    print('\n逐个敲门（每种波特率都试一次）...')
    for p in cands:
        for b in BAUDS:
            r, err = probe(p, b)
            if err:
                print('  %-40s baud=%-7d %s' % (p, b, err))
                break  # 打不开就不用换波特率了
            if r:
                print('  %-40s baud=%-7d ✅ 回包 %r' % (p, b, r))
                if ok is None:
                    ok = (p, b)
                break
            else:
                print('  %-40s baud=%-7d 无回包' % (p, b))

    print('\n' + '=' * 60)
    if ok:
        port, baud = ok
        print('可用端口: %s' % port)
        print('波特率  : %d' % baud)
        print('\n直接复制这一行启动驱动：')
        print('  ros2 launch cquick_servo servo.launch.py port:=%s' % port)
    else:
        print('❌ 设备存在，但所有波特率都没有回包。')
        print('   → 检查 TX/RX 是否交叉、GND 是否共地、板子是否上电')
        print('   → 再做一次回环：TTL 的 TX 和 RX 两个母头对插，重跑本脚本')
    print('=' * 60)
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
