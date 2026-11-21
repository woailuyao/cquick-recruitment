#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cquick_week2.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        # 订阅话题名必须和发布端完全一致，ball_pos 会解析成 /ball_pos
        self.sub = self.create_subscription(BallPos, 'ball_pos', self.cb, 10)
        self.count = 0
        self.get_logger().info('订阅者已启动，等待 ball_pos 消息...')

    def cb(self, msg):
        self.count += 1
        self.get_logger().info(
            f'收 第{self.count}条: id={msg.id} x={msg.x:.2f} y={msg.y:.2f}')

def main():
    rclpy.init()
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()