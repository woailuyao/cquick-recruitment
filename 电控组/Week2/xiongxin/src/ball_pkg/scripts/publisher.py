#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.declare_parameter('freq', 2.0)
        freq = self.get_parameter('freq').value
        
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(1.0 / freq, self.timer_callback)
        self.balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]

    def timer_callback(self):
        for b_id, b_x, b_y in self.balls:
            msg = BallPos()
            msg.id = b_id
            msg.x = b_x
            msg.y = b_y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布: ID={msg.id}, 坐标=({msg.x}, {msg.y})')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()