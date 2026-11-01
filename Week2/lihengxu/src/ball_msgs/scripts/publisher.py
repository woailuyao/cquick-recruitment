#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_msgs.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        timer_period = 0.5  # 0.5秒触发一次，即 2 Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0
        
        # 题目要求的 3 个固定坐标
        self.balls = [
            {'id': 1, 'x': 2.0, 'y': 3.0},
            {'id': 2, 'x': 5.5, 'y': 8.0},
            {'id': 3, 'x': 9.0, 'y': 4.0}
        ]

    def timer_callback(self):
        msg = BallPos()
        current_ball = self.balls[self.count % 3] 
        
        msg.id = current_ball['id']
        msg.x = current_ball['x']
        msg.y = current_ball['y']
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: ID={msg.id}, X={msg.x}, Y={msg.y}')
        self.count += 1

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
