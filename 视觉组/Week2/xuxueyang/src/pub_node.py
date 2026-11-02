import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

class DetectPub(Node):
    def __init__(self):
        super().__init__('detect_pub')
        self.pub = self.create_publisher(Float32MultiArray, 'detect_coord', 10)
        self.timer = self.create_timer(1.0, self.timer_cb)

    def timer_cb(self):
        msg = Float32MultiArray()
        # 模拟识别到红色色块，坐标 u=240, v=180
        msg.data = [240.0, 180.0]
        self.pub.publish(msg)
        self.get_logger().info('发布坐标：u=240.0 , v=180.0')

def main(args=None):
    rclpy.init(args=args)
    node = DetectPub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
