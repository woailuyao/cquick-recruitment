import rclpy
from rclpy.node import Node
from target_interface.msg import TargetPosition

class TargetPubNode(Node):
    def __init__(self):
        super().__init__('target_pub_node')
        self.publisher_ = self.create_publisher(TargetPosition, 'target_pos', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = TargetPosition()
        msg.detected = True
        msg.u = 240.0
        msg.v = 320.0
        msg.area = 1200.0
        self.publisher_.publish(msg)
        self.get_logger().info(f'发布：detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main(args=None):
    rclpy.init(args=args)
    node = TargetPubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
