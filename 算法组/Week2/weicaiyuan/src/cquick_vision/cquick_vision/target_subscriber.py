import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition

class Subscriber(Node):
    def __init__(self):
        super().__init__("target_subscriber")
        self.create_subscription(TargetPosition, "/target_position", self.cb, 10)
        self.n = 0
        self.get_logger().info("等待接收...")

    def cb(self, m):
        self.n += 1
        self.get_logger().info(f"接收#{self.n:03d}: detected={m.detected}, u={m.u:.1f}, v={m.v:.1f}, area={m.area:.0f}")

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(Subscriber())
    rclpy.shutdown()
