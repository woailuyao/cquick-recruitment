"""扫描总线上的舵机 ID，输出在线清单（任务 3 的 ID 扫描用，不开 ROS）。

    ros2 run cquick_servo scan_ids
    ros2 run cquick_servo scan_ids --port /dev/ttyUSB0 --max-id 24
"""
import argparse
import time

import serial

from .servo_protocol import cmd_pid


def scan(port, baud, max_id, wait_s):
    """逐个 ID 发 PID，有回显就记一条。返回在线 ID 列表。"""
    online = []
    with serial.Serial(port, baud, timeout=0.2) as ser:
        for sid in range(1, max_id + 1):
            ser.reset_input_buffer()
            ser.write(cmd_pid(sid).encode())
            ser.flush()
            time.sleep(wait_s)
            buf = b""
            while True:
                d = ser.read(1)
                if not d:
                    break
                buf += d
                if b"!" in buf:
                    break
            if buf:
                online.append(sid)
                print(f"TX {cmd_pid(sid)}  <-  RX {buf!r}   [在线]")
            else:
                print(f"TX {cmd_pid(sid)}  <-  (无回显)    [离线]")
    return online


def main():
    """命令行入口。"""
    parser = argparse.ArgumentParser(description="总线舵机 ID 扫描")
    parser.add_argument("--port", default="/dev/cq_servo", help="串口设备名")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--max-id", type=int, default=24, help="扫描上限 ID")
    parser.add_argument("--wait", type=float, default=0.12, help="每个 ID 等待秒数")
    args = parser.parse_args()

    print(f"扫描 {args.port} @{args.baud}，ID 1~{args.max_id}")
    online = scan(args.port, args.baud, args.max_id, args.wait)
    print("-" * 40)
    print(f"在线舵机 {len(online)} 个：{online}")
    print("把这份清单对上 yaml 里的 servo_ids 参数")


if __name__ == "__main__":
    main()
