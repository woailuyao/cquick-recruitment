#!/usr/bin/env python3
"""安全下电流程：归位 → 逐只卸力（PULK）→ 打印卸力后位置。

用法：
    source ~/ros2_ws/install/setup.bash
    python3 tools/safe_poweroff.py [/dev/ttyUSB0]

顺序不能颠倒：必须先让关节回到中位再把扭矩卸掉，
反过来（带着姿态直接 PULK）会让整机在自身重量下直接砸下来。

注意：
    - 运行前先停掉 servo_driver / ros2 launch，串口是独占的
    - 建议把整机架空（侧躺或吊起）后再归位，避免带自重拖慢甚至卡住
    - 脚本只负责到"卸力"这一步，拔电池和收拢姿态要人来做
"""
import glob
import sys
import time

from cquick_servo.serial_link import SerialLink
from cquick_servo import servo_protocol as p

# 可动关节：id=5 所在通道已过载隔离，id=6 与之镜像，一并跳过，
# 归位只对力学对称的 8 只下发；卸力仍然对 1~10 全部下发（卸力是安全操作）。
MOVE_IDS = [1, 2, 3, 4, 7, 8, 9, 10]
ALL_IDS = list(range(1, 11))

TARGET = 1500          # 中位
DURATION_MS = 2500     # 实测 700ms 走不到位，2500ms 稳定
TOLERANCE = 60         # 比 pos_tolerance(30) 放宽，这是人工核对不是控制器判定


def main() -> int:
    port = sys.argv[1] if len(sys.argv) > 1 else (sorted(glob.glob('/dev/ttyUSB*')) or [None])[0]
    if not port:
        print('没找到 /dev/ttyUSB*，先检查 USB 与电池供电')
        return 1

    t0 = time.time()

    def stamp() -> float:
        return round(time.time() - t0, 1)

    link = SerialLink(port, 115200)
    link.open()
    try:
        print('[%5.1fs] 串口 %s' % (stamp(), port))
        print('[%5.1fs] 卸力前位置: %s' % (stamp(), {i: link.read_position(i) for i in ALL_IDS}))

        # ① 恢复扭力后逐只回中位（一次只发一只，电流最小）
        for i in MOVE_IDS:
            link.send(p.build_torque_on(i))
            time.sleep(0.1)
        time.sleep(0.4)

        for i in MOVE_IDS:
            link.send_move(i, TARGET, DURATION_MS)
            print('[%5.1fs] id %2d -> %d' % (stamp(), i, TARGET))
            time.sleep(0.35)
        time.sleep(2.8)

        pos = {i: link.read_position(i) for i in ALL_IDS}
        print('[%5.1fs] 归位后: %s' % (stamp(), pos))
        arrived = [i for i in MOVE_IDS if pos.get(i) and abs(pos[i] - TARGET) <= TOLERANCE]
        print('        到位 %d/%d' % (len(arrived), len(MOVE_IDS)))

        # ② 全部 ID 卸力
        print('[%5.1fs] 开始卸力 PULK' % stamp())
        for i in ALL_IDS:
            link.send(p.build_torque_off(i))
            print('[%5.1fs] id %2d torque off' % (stamp(), i))
            time.sleep(0.12)
        time.sleep(0.6)

        # ③ 卸力后复查（位置应基本不变，但此时手推能自由转动）
        print('[%5.1fs] 卸力后: %s' % (stamp(), {i: link.read_position(i) for i in ALL_IDS}))
        print('[%5.1fs] 完成：全部卸力，现在手推各关节确认松脱，再拔电池' % stamp())
    finally:
        link.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
