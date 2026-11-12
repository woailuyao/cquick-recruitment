#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('publish_frequency', 2.0) # 新增
        freq = self.get_parameter('publish_frequency').value # 新增
        # 1. 创建发布者，话题名为 'ball_pos'
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        # 2. 创建定时器，2Hz（每 0.5 秒发一次）
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        
        # 硬编码的三个球坐标（绝对不能改）
        self.balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0)
        ]

    def timer_callback(self):
        # 依次把 3 个球都发出去
        for ball_id, x, y in self.balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布: ID={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
