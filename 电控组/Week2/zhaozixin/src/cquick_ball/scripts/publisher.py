#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cquick_ball.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        self.timer = self.create_timer(0.2, self.timer_callback) # 0.5秒发一次 (2Hz)

    def timer_callback(self):
        # 必须使用考核要求的这3个固定坐标
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for b_id, x, y in balls:
            msg = BallPos()
            msg.id = b_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布: 球 {msg.id}, 坐标 ({msg.x}, {msg.y})')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()