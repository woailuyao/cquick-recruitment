import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition

class SubscriberNode(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.sub = self.create_subscription(TargetPosition, '/target_position', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'收到: detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()