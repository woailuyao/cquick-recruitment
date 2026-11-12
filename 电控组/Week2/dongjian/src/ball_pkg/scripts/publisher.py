#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pkg.msg import BallPos

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 创建发布者，话题名为 ball_pos
        self.publisher_ = self.create_publisher(BallPos, 'ball_pos', 10)
        # 定时器，0.5秒触发一次（即 2Hz）
        self.timer = self.create_timer(0.2, self.timer_callback)
        # 固定的 3 个球坐标
        self.balls = [
            (1, 2.0, 3.0),
            (2, 5.5, 8.0),
            (3, 9.0, 4.0)
        ]

    def timer_callback(self):
        # 每个周期把 3 个球都发一遍
        for b_id, x, y in self.balls:
            msg = BallPos()
            msg.id = b_id
            msg.x = x
            msg.y = y
            self.publisher_.publish(msg)
            self.get_logger().info(f'发布球 {msg.id}: x={msg.x}, y={msg.y}')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()