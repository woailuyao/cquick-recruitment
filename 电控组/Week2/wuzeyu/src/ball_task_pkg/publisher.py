#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_task_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.timer_callback) # 2Hz
        
        self.balls_data = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0)
        ]

    def timer_callback(self):
        for b in self.balls_data:
            msg = BallPos()
            msg.id = b[0]
            msg.x = b[1]
            msg.y = b[2]
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布中: 球编号 {msg.id}, 坐标 ({msg.x}, {msg.y})')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
