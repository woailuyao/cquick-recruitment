import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DetectSubNode(Node):
    def __init__(self):
        super().__init__('detect_sub_node')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'detect_coord',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        u = msg.data[0]
        v = msg.data[1]
        self.get_logger().info('收到目标坐标 u=%.2f , v=%.2f' % (u, v))

def main(args=None):
    rclpy.init(args=args)
    node = DetectSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
