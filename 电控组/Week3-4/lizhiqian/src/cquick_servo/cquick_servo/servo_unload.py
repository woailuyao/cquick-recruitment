#!/usr/bin/env python3
import glob
import time
import rclpy
from rclpy.node import Node
import serial


class ServoUnload(Node):
    def __init__(self):
        super().__init__('servo_unload')
        self.declare_parameter('port', '')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('servo_ids', [1, 2, 3, 4, 5, 6, 7, 8, 9, 10])

        port = self.get_parameter('port').value
        if not port:
            ports = sorted(glob.glob('/dev/ttyUSB*'))
            if not ports:
                self.get_logger().error('没找到 /dev/ttyUSB*，检查 USB 线')
                return
            port = ports[0]

        baudrate = self.get_parameter('baudrate').value
        try:
            ser = serial.Serial(port, baudrate, timeout=1)
        except serial.SerialException as e:
            self.get_logger().error(f'串口打开失败: {e}')
            return

        self.get_logger().info(f'串口已打开: {port} @ {baudrate}')
        for sid in self.get_parameter('servo_ids').value:
            cmd = f'#{sid:03d}PULK!'
            try:
                ser.write(cmd.encode())
                ser.flush()
                time.sleep(0.2)
                self.get_logger().info(f'发送: {cmd}')
            except serial.SerialException as e:
                self.get_logger().error(f'串口写入失败: {e}')
                break
        ser.close()
        self.get_logger().info('全部卸力完成，可以断电')


def main(args=None):
    rclpy.init(args=args)
    ServoUnload()
    rclpy.shutdown()


if __name__ == '__main__':
    main()