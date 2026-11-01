import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        self.subscription = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        if msg.detected:
            self.get_logger().info(f'接收: 目标有效 | u={msg.u:.1f}, v={msg.v:.1f}, area={msg.area:.1f}')
        else:
            self.get_logger().info('接收: 未检测到目标 | u=0, v=0, area=0')

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()