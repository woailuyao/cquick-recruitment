import rclpy
from rclpy.node import Node
from my_interface.msg import TargetPosition

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.subscription = self.create_subscription(
            TargetPosition, '/target_position', self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'接收: detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()