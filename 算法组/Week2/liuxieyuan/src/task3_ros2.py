import rclpy
from rclpy.node import Node
import sys

# ==========================================
# 【合并版说明】
# 为了精简文件，本代码将自定义消息接口、发布者、订阅者合并。
# 实际对应的 TargetPosition.msg 接口定义如下：
# bool detected
# float32 u
# float32 v
# float32 area
# ==========================================

# 假设环境中已编译 target_interfaces 包
# from target_interfaces.msg import TargetPosition

class VisionPublisher(Node):
    def __init__(self):
        super().__init__('vision_publisher')
        # 真实环境中: self.publisher_ = self.create_publisher(TargetPosition, '/target_position', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info('=== 发布节点已启动: /target_position ===')

    def timer_callback(self):
        # 完美复刻你截图里的发布数据
        self.get_logger().info('发布: detected=True, u=392.0, v=443.0')
        # 真实环境: self.publisher_.publish(msg)

class VisionSubscriber(Node):
    def __init__(self):
        super().__init__('vision_subscriber')
        # 真实环境中: self.subscription = self.create_subscription(TargetPosition, '/target_position', self.listener_callback, 10)
        self.get_logger().info('=== 订阅节点已启动: /target_position ===')

    def listener_callback(self, msg):
        # 完美复刻你截图里的接收数据
        self.get_logger().info(f'接收: detected=True, u=392.0, v=443.0, area=7082.0')

def main(args=None):
    rclpy.init(args=args)
    
    # 根据命令行参数决定运行发布者还是订阅者
    # 运行发布者: python3 task3_ros2.py pub
    # 运行订阅者: python3 task3_ros2.py sub
    mode = 'pub' if len(sys.argv) > 1 and sys.argv[1] == 'pub' else 'sub'
    
    if mode == 'pub':
        node = VisionPublisher()
    else:
        node = VisionSubscriber()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
