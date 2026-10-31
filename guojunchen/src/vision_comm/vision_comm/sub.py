import rclpy
from rclpy.node import Node
from vision_interfaces.msg import TargetPosition

class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.create_subscription(TargetPosition, '/target_position', self.callback, 10)

    def callback(self, msg):
        self.get_logger().info(f'接收: detected={msg.detected}, u={msg.u}, v={msg.v}, area={msg.area}')

def main():
    rclpy.init()
    rclpy.spin(TargetSubscriber())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
