import rclpy
from rclpy.node import Node
from my_vision_interfaces.msg import TargetPosition

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        self.subscription = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Received: Detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
