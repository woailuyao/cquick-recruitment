#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        # 直接写死 0.5 秒（2Hz），没有任何参数
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        # 1号球
        ball1 = BallPos()
        ball1.id = 1
        ball1.x = 2.0
        ball1.y = 3.0
        self.publisher_.publish(ball1)
        self.get_logger().info(f'发布球: ID={ball1.id}, X={ball1.x}, Y={ball1.y}')
        
        # 2号球
        ball2 = BallPos()
        ball2.id = 2
        ball2.x = 5.5
        ball2.y = 8.0
        self.publisher_.publish(ball2)
        self.get_logger().info(f'发布球: ID={ball2.id}, X={ball2.x}, Y={ball2.y}')
        
        # 3号球
        ball3 = BallPos()
        ball3.id = 3
        ball3.x = 9.0
        ball3.y = 4.0
        self.publisher_.publish(ball3)
        self.get_logger().info(f'发布球: ID={ball3.id}, X={ball3.x}, Y={ball3.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()