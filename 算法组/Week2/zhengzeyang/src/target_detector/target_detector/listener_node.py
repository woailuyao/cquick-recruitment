import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        self.count = 0
        self.create_subscription(
            TargetPosition, '/target_position', self.on_msg, 10)
        self.get_logger().info('listener_node ready')

    def on_msg(self, msg):
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                '#%d recv detected=True  u=%.1f v=%.1f area=%.0f'
                % (self.count, msg.u, msg.v, msg.area))
        else:
            self.get_logger().warn('#%d recv detected=False' % self.count)


def main():
    rclpy.init()
    node = ListenerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

