import rclpy
from rclpy.node import Node
from custom_interfaces.msg import TargetPosition

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        # 订阅 /target_position 话题
        self.subscription = self.create_subscription(
            TargetPosition,
            'target_position',
            self.listener_callback,
            10)
        self.get_logger().info('订阅节点已启动，等待接收坐标...')

    def listener_callback(self, msg):
        if msg.detected:
            self.get_logger().info(f'接收成功: u={msg.u}, v={msg.v}, 面积={msg.area}')
        else:
            self.get_logger().warning('接收成功，但未检测到目标')

def main(args=None):
    rclpy.init(args=args)
    node = VisionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()