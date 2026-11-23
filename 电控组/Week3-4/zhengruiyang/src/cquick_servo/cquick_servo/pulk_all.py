#!/usr/bin/env python3
"""安全下电第一步：给所有舵机发 PULK 卸力。
用法：python3 pulk_all.py
注意：先 Ctrl+C 停掉驱动（launch），否则串口被占用打不开。
"""
import time
import serial

PORT = '/dev/cq_servo'           # 串口设备名（改成你实际的 /dev/cq_servo 或 /dev/ttyUSB0）
BAUD = 115200
SERVO_IDS = list(range(1, 11))   # 1~10 号舵机（按你实际数量改，比如只有 4 个就 [1,2,3,4]）

def main():
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.5)
    except Exception as e:
        print(f'打开串口失败 {PORT}：{e}')
        print('提示：如果驱动还开着，先 Ctrl+C 停掉它，再跑这个脚本')
        return

    print(f'开始卸力，共 {len(SERVO_IDS)} 个舵机…')
    for sid in SERVO_IDS:
        cmd = f'#{sid:03d}PULK!'
        ser.write(cmd.encode('ascii'))
        ser.flush()
        print(f'  已发送 {cmd}')
        time.sleep(0.05)          # 每条之间留 50ms，避免总线拥塞
    ser.close()
    print('全部舵机已卸力（PULK），现在可以断动力电了')

if __name__ == '__main__':
    main()
