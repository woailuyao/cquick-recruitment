#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_msg_pkg.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber_node')
        self.sub = self.create_subscription(
            BallPos,
            '/ball_pos',
            self.callback,
            10
        )
    def callback(self, msg):
        # 收到消息打印
        self.get_logger().info(f"收到球信息：id={msg.id}, x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = BallSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

