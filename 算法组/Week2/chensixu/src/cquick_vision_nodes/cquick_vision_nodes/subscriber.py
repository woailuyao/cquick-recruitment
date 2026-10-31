import rclpy
from rclpy.node import Node
from cquick_interfaces.msg import TargetPosition

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        self.subscription = self.create_subscription(TargetPosition, '/target_position', self.listener_callback, 10)

    def listener_callback(self, msg):
        if msg.detected:
            self.get_logger().info(f'✅ 收到: u={msg.u}, v={msg.v}, area={msg.area}')
        else:
            self.get_logger().info('❌ 未检测到目标')

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
