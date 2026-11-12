#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_task_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        # 创建订阅者，监听 'ball_pos' 话题
        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 收到消息后打印
        self.get_logger().info(f'收到消息 -> 球编号: {msg.id}, X坐标: {msg.x}, Y坐标: {msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
