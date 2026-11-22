#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos 

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 创建发布者，话题名为 'ball_pos'，消息类型为 BallPos
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        
        # 2Hz 频率触发，即 0.5 秒一次
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        # 题目要求的三个球的固定坐标
        self.balls = [
            BallPos(id=1, x=2.0, y=3.0),
            BallPos(id=2, x=5.5, y=8.0),
            BallPos(id=3, x=9.0, y=4.0)
        ]
        self.index = 0

    def timer_callback(self):
        msg = self.balls[self.index]
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: id={msg.id}, x={msg.x}, y={msg.y}')
        # 循环切换 0, 1, 2 三个球的索引
        self.index = (self.index + 1) % 3 

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()