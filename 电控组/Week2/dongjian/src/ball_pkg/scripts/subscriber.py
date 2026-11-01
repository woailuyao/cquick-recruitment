#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        # 订阅同一个话题 ball_pos
        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10)
        self.subscription  # 防止被垃圾回收

    def listener_callback(self, msg):
        self.get_logger().info(f'收到球: ID={msg.id}, X={msg.x:.2f}, Y={msg.y:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()