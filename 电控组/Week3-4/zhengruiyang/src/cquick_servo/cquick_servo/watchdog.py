#!/usr/bin/env python3
"""看门狗节点：监控 /servo/state 心跳，超时则给所有舵机 PULK 卸力。"""
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header


class Watchdog(Node):
    def __init__(self):
        super().__init__('watchdog')
        self.declare_parameter('port', '/dev/cq_servo')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('servo_ids', [1, 2])
        self.declare_parameter('timeout_sec', 1.0)

        self._last_heartbeat = time.time()+3.0
        self._locked = False

        self.create_subscription(Header, '/servo/state', self._heartbeat_cb, 10)
        self.create_timer(0.5, self._check)

    def _heartbeat_cb(self, msg):
        self._last_heartbeat = time.time()
        if self._locked:
            self.get_logger().info('收到外部心跳，看门狗恢复')
            self._locked = False

    def _check(self):
        if self._locked:
            return   # 已锁，不乱动
        if time.time() - self._last_heartbeat > self.get_parameter('timeout_sec').value:
            self.get_logger().error('心跳超时！给所有舵机 PULK 卸力')
            self._pulk_all()
            self._locked = True

    def _pulk_all(self):
        port = self.get_parameter('port').value
        baud = self.get_parameter('baudrate').value
        try:
            ser = serial.Serial(port, baud, timeout=0.5)
            for sid in self.get_parameter('servo_ids').value:
                ser.write(f'#{sid:03d}PULK!'.encode())
                ser.flush()
            ser.close()
            self.get_logger().error('已给所有舵机 PULK 卸力')
        except Exception as e:
            self.get_logger().error(f'PULK 失败：{e}')


def main(args=None):
    rclpy.init(args=args)
    node = Watchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
