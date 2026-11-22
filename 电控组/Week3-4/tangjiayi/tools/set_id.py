#!/usr/bin/env python3
"""众灵总线舵机改 ID / 巡检

用法：
  python3 set_id.py            只扫描，看总线上现在有哪些 ID
  python3 set_id.py 2          把 ID 000 的舵机改成 002
  python3 set_id.py -v 2       只验证 002 在不在线（改完断电重启后跑这个）

约定：
  - 改 ID 时总线上【只能挂一个 000 的舵机】，否则会全被改成同一个号
  - ID 写进舵机 EEPROM，必须拔电池 3 秒再上电才生效
  - 本脚本不碰 001 及以上的舵机，写到 001~004 是安全的
"""
import sys
import time

import serial

PORT = '/dev/ttyUSB0'
BAUD = 115200
SCAN_MAX = 8          # 扫 ID 0~8 足够，总线上最多 4 个


def open_port():
    return serial.Serial(PORT, BAUD, timeout=0.4)


def q(s, cmd, wait=0.4):
    s.reset_input_buffer()
    s.write(cmd.encode())
    s.flush()
    time.sleep(wait)
    return s.read(128)


def scan(s):
    """返回在线 ID 列表。0 号单独标注，因为众灵对 ID=0 回空值。"""
    online = []
    for i in range(SCAN_MAX + 1):
        r = q(s, '#%03dPID!' % i, 0.25)
        if r:
            online.append(i)
    return online


def main():
    args = [a for a in sys.argv[1:]]
    verify_only = '-v' in args
    args = [a for a in args if a != '-v']

    try:
        s = open_port()
    except Exception as e:
        print('打不开 %s：%s' % (PORT, e))
        print('先确认：TTL 插好、VMware 已"连接到虚拟机"、ros2 launch 已 Ctrl-C 停掉')
        return 1

    print('== 扫描总线上在线的 ID ==')
    online = scan(s)
    print('   在线: %s' % (online if online else '（一个都没有）'))

    if verify_only:
        if not args:
            s.close()
            return 0
        target = int(args[0])
        ver = q(s, '#%03dPVER!' % target)
        ok = bool(ver)
        print('\n%s %03d 号：%s' % ('[OK]' if ok else '[失败]', target,
                                   ver.decode(errors='replace') if ok else '无回包'))
        if not ok:
            print('   可能原因：还没断电重启 / ID 没写进去 / 这个舵机没接上')
        s.close()
        return 0 if ok else 2

    if not args:
        print('\n（只扫描模式。要改 ID 就加一个数字，例如：python3 set_id.py 2）')
        s.close()
        return 0

    new_id = int(args[0])
    if not 1 <= new_id <= 4:
        print('新 ID 只允许 1~4（和 servo_params.yaml 的 servo_ids 保持一致）')
        s.close()
        return 1

    if new_id in online:
        print('\n注意：%03d 号已经在线了。' % new_id)
        print('如果是上一次改成功、忘了断电重启，直接断电重启再跑 set_id.py -v %d 验证即可。' % new_id)
        if 0 not in online:
            print('总线上也没有 000，说明没有待改的舵机，退出。')
            s.close()
            return 0

    if 0 not in online:
        print('\n[失败] 总线上找不到 ID 000 的舵机。')
        print('   检查：① 是否只挂了这一个舵机 ② 它是不是已经被改过号了（跑 set_id.py 看在线列表）')
        s.close()
        return 2

    print('\n== 把 000 改成 %03d ==' % new_id)
    print('   发送: #000PID%03d!' % new_id)
    r = q(s, '#000PID%03d!' % new_id, 0.6)
    print('   回包: %s' % (r.decode(errors='replace') if r else '(空，正常)'))

    time.sleep(0.5)
    again = scan(s)
    print('   改完再扫一次: %s' % (again if again else '（一个都没有）'))

    s.close()
    print('\n下一步固定动作：')
    print('   ① 拔电池，等 3 秒，再插回（ID 掉电保存，必须重启才生效）')
    print('   ② 跑：python3 ~/set_id.py -v %d' % new_id)
    print('   ③ 看到 [OK] %03d 号 就说明这只改好了，换下一只' % new_id)
    return 0


if __name__ == '__main__':
    sys.exit(main())
