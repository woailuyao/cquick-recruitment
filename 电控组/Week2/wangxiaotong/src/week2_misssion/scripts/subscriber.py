#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from week2_mission1.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')
        
        # 1. 创建订阅者 (消息类型, 话题名必须和发布者一样 'ball_pos', 回调函数, 队列长度)
        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # 回调里打印收到的数据
        self.get_logger().info(f'收到球 {msg.id}: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()