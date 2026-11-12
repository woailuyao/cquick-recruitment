#!/usr/bin/env python3
# 任务1 · 订阅者
# 订阅话题 ball_pos，把收到的 id / x / y 打印出来

import rclpy
from rclpy.node import Node
from ball_pos.msg import BallPos


class BallSubscriber(Node):

    def __init__(self):
        super().__init__('ball_subscriber')

        # 话题名必须和发布者一字不差：ball_pos
        self.sub = self.create_subscription(
            BallPos,      # 消息类型
            'ball_pos',   # 话题名
            self.callback,
            10)           # 队列长度

        self.count = 0
        self.get_logger().info('订阅者已启动，等待 ball_pos 消息…')

    def callback(self, msg):
        """每收到一条消息就进这里一次"""
        self.count += 1
        self.get_logger().info(
            f'收到第{self.count}条 → 球{msg.id}: x={msg.x:.2f} y={msg.y:.2f}')


def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
