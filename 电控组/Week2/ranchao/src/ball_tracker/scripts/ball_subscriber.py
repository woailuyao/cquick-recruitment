#!/usr/bin/env python3
"""任务1·订阅者：订阅 /ball_pos，收到一条打印一条"""
import rclpy
from rclpy.node import Node
from ball_tracker.msg import BallPos


class BallSubscriber(Node):

    def __init__(self):
        super().__init__('ball_subscriber')
        # 话题名必须和发布者一字不差，两个节点才能对上号
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.on_ball, 10)

    def on_ball(self, msg):
        """每收到一条 BallPos，这个函数自动执行一次"""
        self.get_logger().info(f'收到: 球{msg.id} 位置=({msg.x:.2f}, {msg.y:.2f})')


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
