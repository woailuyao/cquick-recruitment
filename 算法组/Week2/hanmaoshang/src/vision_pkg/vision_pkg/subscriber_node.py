import rclpy
from rclpy.node import Node
from interfaces.msg import TargetPosition

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        # 订阅 /target_position 话题
        self.subscription = self.create_subscription(
            TargetPosition,
            '/target_position',
            self.listener_callback,
            10)
        self.subscription  # 防止被垃圾回收
        self.get_logger().info('视觉订阅节点已启动，等待接收数据...')

    def listener_callback(self, msg):
        if msg.detected:
            self.get_logger().info(f'收到目标: 坐标(u={msg.u:.1f}, v={msg.v:.1f}), 面积={msg.area:.1f}')
        else:
            self.get_logger().warn(f'收到目标: 未检测到目标 (detected=False)')

def main(args=None):
    rclpy.init(args=args)
    vision_subscriber = VisionSubscriber()
    rclpy.spin(vision_subscriber)
    vision_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
