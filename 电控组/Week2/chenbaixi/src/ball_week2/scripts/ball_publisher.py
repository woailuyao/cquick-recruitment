#!/usr/bin/env python3
"""任务 1 - 发布者：每个周期把 3 个球依次发一遍"""

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from ball_week2.msg import BallPos

# 三个球的坐标，必须和细则给的一模一样，不许改
BALLS = [
    (1, 2.0, 3.0),
    (2, 5.5, 8.0),
    (3, 9.0, 4.0),
]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value

        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / freq, self.tick)

        self.add_on_set_parameters_callback(self.on_param)
        self.get_logger().info(f'发布者已启动，频率 {freq:.2f} Hz')

    def on_param(self, params):
        for p in params:
            if p.name == 'freq':
                self.destroy_timer(self.timer)
                self.timer = self.create_timer(1.0 / p.value, self.tick)
                self.get_logger().info(f'频率已改为 {p.value:.2f} Hz')
        return SetParametersResult(successful=True)

    def tick(self):
        for ball_id, x, y in BALLS:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布 → id={ball_id}  x={x:.2f}  y={y:.2f}')


def main():
    rclpy.init()
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
