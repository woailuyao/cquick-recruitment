"""目标位置发布节点。

用法：
    # 1) 固定/演示数据验证通信（不依赖图片，开箱即用）：
    ros2 run vision_detector publisher

    # 2) 接入真实检测（给图片路径，阈值用默认白色足球范围）：
    ros2 run vision_detector publisher \
        --ros-args -p image_path:="/mnt/c/.../data/images/test_clear.jpg"

    # 3) 运行时改 HSV 阈值 / 最小面积（加分项：无需重编译）：
    ros2 run vision_detector publisher \
        --ros-args -p image_path:="/path/to/img.jpg" \
                   -p hsv_lower:="[0,0,170]" \
                   -p hsv_upper:="[180,80,255]" \
                   -p min_area:=800

话题：/target_position  (TargetPosition.msg)
    detected=false 时 u/v/area 全部为 0，订阅方靠 detected 判断有效性。
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from target_position_msgs.msg import TargetPosition

from vision_detector.detector import detect_from_file


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)

        # ---- 声明 ROS2 参数（可运行时调，加分项）----
        self.declare_parameter('image_path', '')           # 空=演示模式
        self.declare_parameter('hsv_lower', [0, 0, 170])
        self.declare_parameter('hsv_upper', [180, 80, 255])
        self.declare_parameter('min_area', 500)
        self.declare_parameter('rate', 10.0)               # 发布频率 Hz
        self.declare_parameter('demo', True)               # 演示模式开关

        rate = self.get_parameter('rate').value
        self.timer = self.create_timer(1.0 / max(rate, 0.1), self.timer_callback)
        self.count = 0
        self.get_logger().info('TargetPublisher 已启动，话题=/target_position')

    def timer_callback(self):
        msg = TargetPosition()
        image_path = self.get_parameter('image_path').value

        if image_path:
            # ---- 接入实际检测结果（任务3 最终要求）----
            try:
                res = detect_from_file(
                    image_path,
                    self.get_parameter('hsv_lower').value,
                    self.get_parameter('hsv_upper').value,
                    self.get_parameter('min_area').value,
                )
                if 'error' in res:
                    self.get_logger().error(res['error'])
                msg.detected = res['detected']
                msg.u = res['u']
                msg.v = res['v']
                msg.area = res['area']
            except Exception as e:  # noqa: BLE001
                self.get_logger().error('检测异常: ' + str(e))
                msg.detected = False
        else:
            # ---- 演示模式：固定数据，便于先验证通信 ----
            if self.get_parameter('demo').value and (self.count % 10 != 9):
                msg.detected = True
                msg.u = 320.0
                msg.v = 240.0
                msg.area = 26608.0
            else:
                # 每 10 条演示一次「未检测到」，展示 detected=false 约定
                msg.detected = False
                msg.u = 0.0
                msg.v = 0.0
                msg.area = 0.0

        self.pub.publish(msg)
        self.count += 1
        if msg.detected:
            self.get_logger().info(
                '发布 #%d detected=True u=%.1f v=%.1f area=%.1f'
                % (self.count, msg.u, msg.v, msg.area))
        else:
            self.get_logger().warn(
                '发布 #%d detected=False (u/v/area 为默认 0)' % self.count)


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
