import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(
            TargetPosition, '/target_position', self.listener_callback, 10)
        self.get_logger().info('订阅者已启动...')

    def listener_callback(self, msg):
        self.get_logger().info(f'收到: u={msg.u}, v={msg.v}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
