import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'收到: detected={msg.detected}, u={msg.u:.1f}, v={msg.v:.1f}, area={msg.area:.1f}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
