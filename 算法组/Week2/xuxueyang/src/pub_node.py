import rclpy
from rclpy.node import Node
from target_interfaces.msg import TargetPosition

class DetectPub(Node):
    def __init__(self):
        super().__init__('detect_pub')
        self.pub = self.create_publisher(TargetPosition, 'detect_coord', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = TargetPosition()
        msg.detected = True
        msg.x = 240.0
        msg.y = 180.0
        self.pub.publish(msg)
        self.get_logger().info(f"发布坐标：x={msg.x}, y={msg.y}")

def main(args=None):
    rclpy.init(args=args)
    node = DetectPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
