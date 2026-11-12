#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ball_interfaces.msg import BallPos

class BallSupscriber(Node):
    def __init__(self):
        super().__init__('ball_subscriber')

        self.subscription = self.create_subscription(
            BallPos,
            'ball_pos',
            self.listener_callback,
            10
        )

    def listener_callback(self,msg):
        self.get_logger().info(
            f'收到球{msg.id}:x={msg.x},y={msg.y}'
        )

def main(args = None):
    rclpy.init(args=args)
    node = BallSupscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

