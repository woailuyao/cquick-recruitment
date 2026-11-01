#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ballpos.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        # 订阅 'ball_pos' 话题
        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 打印收到的坐标
        self.get_logger().info(f'收到球: ID={msg.id}, X={msg.x}, Y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
   
