#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_package.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        # 创建订阅器，订阅ball_pos话题
        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10)
        self.subscription  # 防止变量被自动回收

    def listener_callback(self, msg):
        # 收到消息就自动执行这个回调
        self.get_logger().info(f'收到球信息：id={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    ball_subscriber = BallSubscriber()
    rclpy.spin(ball_subscriber)
    ball_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
