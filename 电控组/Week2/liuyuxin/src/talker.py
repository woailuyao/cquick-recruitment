#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_interfaces.msg import BallPos

class BallPosPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, '/ball_pos', 10)
        timer_period = 0.2  # source ~/ros2_ws/install/setup.bashcd ~/ros2_ws
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        # 讲义固定坐标，不许改动
        msg1 = BallPos()
        msg1.id = 1
        msg1.x = 2.0
        msg1.y = 3.0
        self.publisher_.publish(msg1)
        self.get_logger().info(f'发布球1: id={msg1.id}, x={msg1.x}, y={msg1.y}')

        msg2 = BallPos()
        msg2.id = 2
        msg2.x = 5.5
        msg2.y = 8.0
        self.publisher_.publish(msg2)
        self.get_logger().info(f'发布球2: id={msg2.id}, x={msg2.x}, y={msg2.y}')

        msg3 = BallPos()
        msg3.id = 3
        msg3.x = 9.0
        msg3.y = 4.0
        self.publisher_.publish(msg3)
        self.get_logger().info(f'发布球3: id={msg3.id}, x={msg3.x}, y={msg3.y}')

def main(args=None):
    rclpy.init(args=args)
    publisher = BallPosPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()