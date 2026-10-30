import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__("target_subscriber")
        self.count = 0
        self.sub = self.create_subscription(TargetPosition, "/target_position", self.cb, 10)

    def cb(self, msg):
        self.count += 1
        self.get_logger().info("[%d] detected=%s u=%.1f v=%.1f area=%.0f" % (self.count, msg.detected, msg.u, msg.v, msg.area))


def main():
    rclpy.init()
    node = TargetSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
