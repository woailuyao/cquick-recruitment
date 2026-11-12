#!/usr/bin/env python3
"""任务1 发布者：2 Hz 定时器，每个周期把 3 个球的坐标各发一条。"""
import rclpy
from rclpy.node import Node

from ball_pos_pkg.msg import BallPos

# 考核细则 1.2 的固定坐标，一个都不能改
BALLS = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]


class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')

        # 频率做成参数：--ros-args -p freq:=5.0 就能改，不用动代码
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value

        self.pub = self.create_publisher(BallPos, 'ball_pos', 10)

        # create_timer 的参数是「周期（秒）」不是频率：2 Hz -> 0.5 秒
        self.timer = self.create_timer(1.0 / freq, self.publish_all)

        self.get_logger().info(f'发布者已启动：频率 {freq} Hz，话题 /ball_pos')

    def publish_all(self):
        """每个周期把 3 个球依次发一遍。"""
        for ball_id, x, y in BALLS:
            msg = BallPos()      # 每条都新建；复用同一个对象会把前一条覆盖掉
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.pub.publish(msg)
            self.get_logger().info(f'发布 球{msg.id}: ({msg.x:.1f}, {msg.y:.1f})')


def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
