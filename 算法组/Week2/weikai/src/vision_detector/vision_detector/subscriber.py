"""目标位置订阅节点。

用法：
    ros2 run vision_detector subscriber

会打印收到的每一条 /target_position：
    - detected=True  -> 输出 u/v/area
    - detected=False -> 明确提示「未检测到」，且强调 u/v/area 是默认 0，
                        不可当作目标在左上角（任务3 红线）。
"""
import rclpy
from rclpy.node import Node
from target_position_msgs.msg import TargetPosition


class TargetSubscriber(Node):
    def __init__(self):
        super().__init__('target_subscriber')
        self.sub = self.create_subscription(
            TargetPosition, '/target_position', self.callback, 10)
        self.received = 0
        self.get_logger().info('TargetSubscriber 已启动，监听 /target_position')

    def callback(self, msg):
        self.received += 1
        if msg.detected:
            self.get_logger().info(
                '收到 #%d detected=True u=%.1f v=%.1f area=%.1f'
                % (self.received, msg.u, msg.v, msg.area))
        else:
            self.get_logger().warn(
                '收到 #%d detected=False -> 无效目标，u/v/area 为默认 0，'
                '不能当作运动指令！' % self.received)


def main(args=None):
    rclpy.init(args=args)
    node = TargetSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
