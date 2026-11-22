#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        
        self.declare_parameter('pub_frequency',2.0)
        freq = self.get_parameter('pub_frequency').value
        
        
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        for ball_id, x, y in self.balls:
            msg = BallPos()
            msg.id = ball_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布: 球{msg.id} 坐标 ({msg.x}, {msg.y})')

def main():
    rclpy.init()
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
