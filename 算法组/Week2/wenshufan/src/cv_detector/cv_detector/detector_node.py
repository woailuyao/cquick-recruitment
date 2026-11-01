import rclpy
from rclpy.node import Node

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.get_logger().info('✅ 检测器节点启动成功！')

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
