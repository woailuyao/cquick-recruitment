import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition

class DetectSub(Node):
    def __init__(self):
        super().__init__('detect_sub')
        self.sub = self.create_subscription(
            TargetPosition,
            'detect_coord',
            self.callback,
            10
        )

    def callback(self, msg):
        if msg.detected:
            self.get_logger().info(f"收到目标：x={msg.x}, y={msg.y}")
        else:
            self.get_logger().info("未检测到目标")

def main(args=None):
    rclpy.init(args=args)
    node = DetectSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
