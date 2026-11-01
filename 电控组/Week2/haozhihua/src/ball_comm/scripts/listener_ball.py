#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_comm.msg import BallPos


class BallListener(Node):
    def __init__(self):
        super().__init__('ball_listener')
        # 创建订阅者：订阅 ball_pos 话题，收到就调 self.callback
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.callback, 10)
        self.seen = {}          # 用字典按 id 存，天然去重

    def callback(self, msg):
        self.seen[msg.id] = (msg.x, msg.y)
        self.get_logger().info(
            f'收到球 {msg.id}: ({msg.x:.2f}, {msg.y:.2f})  '
            f'已收 {len(self.seen)}/3')


def main():
    rclpy.init()
    node = BallListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()