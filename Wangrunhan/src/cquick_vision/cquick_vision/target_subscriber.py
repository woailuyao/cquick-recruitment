#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.sub = self.create_subscription(TargetPosition, '/target_position', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'detected={msg.detected}, u={msg.u:.1f}, v={msg.v:.1f}, area={msg.area:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
