#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ballpos.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        
        # 1. 声明一个参数，名字叫 'publish_rate'，默认值是 2.0 (即 2Hz)
        self.declare_parameter('publish_rate', 2.0)
        
        # 2. 从参数里读取频率，并算出时间间隔（1/频率）
        rate = self.get_parameter('publish_rate').value
        period = 1.0 / rate
        
        # 3. 用算出来的 period 创建定时器
        self.timer = self.create_timer(period, self.timer_callback)
        
        self.balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
    def timer_callback(self):
        for b_id, b_x, b_y in self.balls:
            msg = BallPos()
            msg.id = b_id
            msg.x = b_x
            msg.y = b_y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发送球: ID={msg.id}, X={msg.x}, Y={msg.y}')
def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
