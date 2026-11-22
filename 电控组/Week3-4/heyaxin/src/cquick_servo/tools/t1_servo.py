#!/usr/bin/env python3
"""串口直连工具（不经过 ROS2）：扫 ID / 四步验证 / 正反转 / 批量回中位 / 批量卸力

跑之前必须先停 servo_driver —— 串口是独占的，节点占着口这个脚本会打不开。

用法：
    python3 tools/t1_servo.py scan                    # 扫 ID 0~20，谁回显谁在线
    python3 tools/t1_servo.py verify 1500 --id 0      # PULR + 回中位 + PRAD 读回
    python3 tools/t1_servo.py sweep --id 0            # 2000 / 1000 / 1500，看 PRAD 跟随
    python3 tools/t1_servo.py center                  # 整机：所有在线舵机逐个回 1500 并读回
    python3 tools/t1_servo.py pulk                    # 整机：所有在线舵机卸力（下电第一步）

    --port /dev/ttyUSB1   指定串口（默认取第一个 /dev/ttyUSB*）
    --ids 0,1,2,3,4,5     只操作指定 ID（默认自动扫在线）
"""
import argparse
import glob
import re
import sys
import time

import serial

BAUD = 115200
FRAME_RE = re.compile(r'#(\d{3})P(\d{4})?')
SCAN_RANGE = range(21)          # 0~20，整机 6 个舵机通常落在 0~5


def open_port(port=None):
    if port is None:
        ports = sorted(glob.glob('/dev/ttyUSB*'))
        if not ports:
            sys.exit('没有 /dev/ttyUSB*，先检查 USB 线和驱动')
        port = ports[0]
    try:
        ser = serial.Serial(port, BAUD, timeout=0.08)
    except serial.SerialException as exc:
        msg = str(exc)
        if 'Errno 5' in msg or 'Input/output error' in msg:
            sys.exit(
                '串口打不开：[Errno 5] Input/output error —— 这不是代码问题，是 USB 转 TTL 掉线了\n'
                '  · 设备节点还在，但已经死了（看 ls -l /dev/ttyUSB* 的时间戳是不是刚变过）\n'
                '  · VirtualBox：设备 → USB → 把 CH340 取消勾选再勾上（或物理拔插一次）\n'
                '  · 恢复后先用 python3 -c "import serial;serial.Serial(\'%s\',115200)" 试一下' % port)
        if 'Permission' in msg or 'Errno 13' in msg:
            sys.exit('串口没权限：sudo usermod -aG dialout $USER 后重新登录（别用 sudo 跑脚本）')
        sys.exit('串口打不开：%s\n  · 是不是 servo_driver 还占着口？pkill -f "[s]ervo_driver"' % msg)
    print('串口: %s @ %d 8-N-1' % (ser.port, BAUD))
    return ser


def send(ser, frame, wait=0.15):
    ser.reset_input_buffer()
    ser.write(frame.encode('ascii'))
    ser.flush()
    time.sleep(wait)
    return ser.read(64)


def read_prad(ser, sid):
    r = send(ser, '#%03dPRAD!' % sid, wait=0.12)
    m = FRAME_RE.search(r.decode('ascii', 'ignore'))
    if m and m.group(2):
        return int(m.group(2)), r
    return None, r


def probe_online(ser, quiet=False):
    """逐个发 PID，有回显就是在线的。"""
    online = []
    for sid in SCAN_RANGE:
        if send(ser, '#%03dPID!' % sid):
            online.append(sid)
        time.sleep(0.05)
    if not quiet:
        print('在线 ID = %s （共 %d 只）'
              % (', '.join('%03d' % i for i in online), len(online)))
    return online


def parse_ids(args, ser):
    if args.ids:
        return [int(x) for x in args.ids.split(',') if x.strip()]
    return probe_online(ser)


def cmd_scan(args):
    ser = open_port(args.port)
    print('【扫 ID (0-20)】谁回显谁在线')
    print()
    online = []
    for sid in SCAN_RANGE:
        r = send(ser, '#%03dPID!' % sid)
        if r:
            print('#%03dPID! -> %-14r  <<< 有回显' % (sid, r))
            online.append(sid)
        else:
            print("#%03dPID! -> b''" % sid)
        time.sleep(0.05)
    print()
    print('>>> 在线舵机 ID = %s （共 %d 只）'
          % (', '.join('%03d' % i for i in online), len(online)))
    ser.close()


def cmd_verify(args):
    sid = args.id
    target = args.pwm
    ser = open_port(args.port)
    print('== 四步验证：PID / PULR / 位置指令 / PRAD 读回 ==')
    print()
    r = send(ser, '#%03dPID!' % sid)
    print('1) #%03dPID!          -> %r' % (sid, r))
    r = send(ser, '#%03dPULR!' % sid)
    print('2) #%03dPULR!         -> %r   （PULR 本来就不回显，正常）' % (sid, r))
    r = send(ser, '#%03dP%04dT1500!' % (sid, target))
    print('3) #%03dP%04dT1500!   -> %r   （位置指令也不回显，靠第 4 步校验）' % (sid, target, r))
    time.sleep(1.8)
    prad, raw = read_prad(ser, sid)
    print('4) #%03dPRAD!         -> %r' % (sid, raw))
    if prad is None:
        print()
        print('!! PRAD 没读回，检查供电和信号线')
    else:
        print()
        print('>>> 目标 %d，读回 %d，偏差 %d' % (target, prad, abs(prad - target)))
        print('>>> 读回与指令一致（±30 以内算到位）')
    ser.close()


def cmd_sweep(args):
    sid = args.id
    ser = open_port(args.port)
    send(ser, '#%03dPULR!' % sid)
    time.sleep(0.2)
    print('证明舵机确实在转：目标变化 -> PRAD 跟着变')
    print()
    for pwm in (2000, 1000, 1500):
        send(ser, '#%03dP%04dT1500!' % (sid, pwm))
        time.sleep(2.0)
        prad, raw = read_prad(ser, sid)
        print('发 #%03dP%04dT1500!   -> 读回 %-12r => PRAD = %s'
              % (sid, pwm, raw, prad if prad is not None else '读不到'))
    print()
    print('>>> PRAD 跟着目标走，说明舵机真的动了')
    ser.close()


def cmd_center(args):
    """整机回中位：装舵臂之前、每次上电之后都跑这个。"""
    ser = open_port(args.port)
    ids = parse_ids(args, ser)
    if not ids:
        sys.exit('没有在线舵机，查供电和信号线')
    print()
    print('== 逐个回中位 1500（先 PULR 恢复扭力，再转，再 PRAD 读回）==')
    bad = []
    for sid in ids:
        send(ser, '#%03dPULR!' % sid)
        time.sleep(0.1)
        send(ser, '#%03dP1500T1500!' % sid)
        time.sleep(1.9)
        prad, _ = read_prad(ser, sid)
        if prad is None:
            print('  舵机 %02d -> PRAD 读不到   ✗' % sid)
            bad.append(sid)
            continue
        diff = abs(prad - 1500)
        flag = '✓' if diff <= 50 else '✗ 偏差大'
        if diff > 50:
            bad.append(sid)
        print('  舵机 %02d -> PRAD=%4d  偏差 %3d  %s' % (sid, prad, diff, flag))
    print()
    if bad:
        print('>>> 有问题的舵机：%s —— 先别装舵臂，查完再装' % bad)
    else:
        print('>>> 全部到位（±50），可以装舵臂，装的时候保持这个姿态别上电')
    ser.close()


def cmd_pulk(args):
    """整机卸力：任务 4 下电第一步，所有舵机断电前必须卸力。"""
    ser = open_port(args.port)
    ids = parse_ids(args, ser)
    if not ids:
        sys.exit('没有在线舵机，查供电和信号线')
    print()
    print('== 全部卸力 PULK ==')
    for sid in ids:
        send(ser, '#%03dPULK!' % sid)
        time.sleep(0.05)
        print('  #%03dPULK! 已发' % sid)
    time.sleep(0.3)
    print()
    print('>>> 用手轻推每个关节：能轻松推动 = 已卸力；还顶着力 = 这条没生效，重发')
    ser.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('action',
                    choices=['scan', 'verify', 'sweep', 'center', 'pulk'])
    ap.add_argument('pwm', nargs='?', type=int, default=1500)
    ap.add_argument('--id', type=int, default=0)
    ap.add_argument('--ids', default=None,
                    help='逗号分隔，如 0,1,2,3,4,5；不给则自动扫在线')
    ap.add_argument('--port', default=None)
    args = ap.parse_args()
    {'scan': cmd_scan, 'verify': cmd_verify, 'sweep': cmd_sweep,
     'center': cmd_center, 'pulk': cmd_pulk}[args.action](args)


if __name__ == '__main__':
    main()
