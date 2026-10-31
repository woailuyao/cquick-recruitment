#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallSubscriber(Node):
    def __init__(self):
        super().__init__('ball__subscriber')

        self.subscription = self.create_subscription(
            BallPos, 'ball_pos', self.listener_callback, 10
        )
    def listener_callback(self, msg):
        self.get_logger().info(
            f'收到:球{msg.id}坐标({msg.x}{msg.y})'
        )
def main():
    rclpy.init()
    node = BallSubscriber()
    rclpy.spin(node)
    node.sestroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()