#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_pos_pkg.msg import BallPos  # 引入你的自定义消息

class BallPublisher(Node):
    def __init__(self):
        super().__init__('ball_publisher')
        # 创建发布者，话题名 /ball_pos，队列长度 10
        self.pub = self.create_publisher(BallPos, '/ball_pos', 10)
        # 2 Hz 定时器 = 0.5 秒执行一次
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        # 依次发送 3 个球的数据
        balls = [(1, 2.0, 3.0), (2, 5.5, 8.0), (3, 9.0, 4.0)]
        for bid, bx, by in balls:
            msg = BallPos()
            msg.id = bid
            msg.x = bx
            msg.y = by
            self.pub.publish(msg)
            # 打印发送日志
            self.get_logger().info(f'发了球 {msg.id}: ({msg.x}, {msg.y})')

def main(args=None):
    rclpy.init(args=args)
    node = BallPublisher()
    rclpy.spin(node) # 让程序挂起，一直运行
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
