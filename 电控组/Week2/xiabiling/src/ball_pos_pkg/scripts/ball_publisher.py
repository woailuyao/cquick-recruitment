#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 创建发布者，话题名为 'ball_pos'，消息类型为 BallPos，队列长度为 10
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        # 定时器，5Hz 频率（0.2秒一次）
        self.timer = self.create_timer(0.5, self.timer_callback)
        
    def timer_callback(self):
        # 考核要求的三个固定坐标
        balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0)
        ]
        for ball_id, x, y in balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布球坐标: ID={msg.id}, x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
