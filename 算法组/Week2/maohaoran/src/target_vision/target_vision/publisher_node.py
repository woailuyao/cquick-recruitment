"""发布节点：对测试图片做 HSV 颜色检测，把结果发布到 /target_position。

数据来源：包内 images/ 目录下的测试图片，按文件名顺序逐张处理并循环。
HSV 阈值、最小面积都是 ROS2 参数，可以在运行时用 ros2 param set 修改。
"""

from pathlib import Path

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

from .detection import detect


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('target_publisher')

        share_dir = Path(get_package_share_directory('target_vision'))
        default_image_dir = str(share_dir / 'images')

        self.declare_parameter('image_dir', default_image_dir)
        self.declare_parameter('h_min', 100)
        self.declare_parameter('h_max', 130)
        self.declare_parameter('s_min', 80)
        self.declare_parameter('s_max', 255)
        self.declare_parameter('v_min', 40)
        self.declare_parameter('v_max', 255)
        self.declare_parameter('min_area', 2000.0)
        self.declare_parameter('publish_rate', 1.0)

        self.image_dir = str(self.get_parameter('image_dir').value)
        self.h_min = int(self.get_parameter('h_min').value)
        self.h_max = int(self.get_parameter('h_max').value)
        self.s_min = int(self.get_parameter('s_min').value)
        self.s_max = int(self.get_parameter('s_max').value)
        self.v_min = int(self.get_parameter('v_min').value)
        self.v_max = int(self.get_parameter('v_max').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.publish_rate = float(self.get_parameter('publish_rate').value)

        self.images = self._scan_images()
        self.index = 0
        self.frame_count = 0

        self.publisher_ = self.create_publisher(
            TargetPosition, '/target_position', 10)
        self.add_on_set_parameters_callback(self._on_parameters)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._on_timer)

        self.get_logger().info(
            f'发布节点已启动: topic=/target_position, 图片 {len(self.images)} 张, '
            f'频率 {self.publish_rate:.1f} Hz')
        self.get_logger().info(
            f'HSV 阈值: H[{self.h_min},{self.h_max}] '
            f'S[{self.s_min},{self.s_max}] V[{self.v_min},{self.v_max}] '
            f'最小面积 {self.min_area:.0f}')

    def _scan_images(self):
        """读取图片目录下所有 jpg/png，按文件名排序。"""
        directory = Path(self.image_dir)
        if not directory.is_dir():
            self.get_logger().error(f'图片目录不存在: {directory}')
            return []
        paths = sorted(p for p in directory.iterdir()
                       if p.suffix.lower() in {'.jpg', '.jpeg', '.png'})
        if not paths:
            self.get_logger().error(f'目录中没有图片: {directory}')
        return paths

    def _on_parameters(self, params):
        """参数回调：校验新阈值，通过后立即生效（加分项）。"""
        values = {p.name: p.value for p in params}

        h_min = int(values.get('h_min', self.h_min))
        h_max = int(values.get('h_max', self.h_max))
        s_min = int(values.get('s_min', self.s_min))
        s_max = int(values.get('s_max', self.s_max))
        v_min = int(values.get('v_min', self.v_min))
        v_max = int(values.get('v_max', self.v_max))
        min_area = float(values.get('min_area', self.min_area))
        rate = float(values.get('publish_rate', self.publish_rate))
        image_dir = str(values.get('image_dir', self.image_dir))

        if not (0 <= h_min < h_max <= 179):
            return SetParametersResult(
                successful=False, reason='H 需要满足 0 <= h_min < h_max <= 179')
        if not (0 <= s_min < s_max <= 255):
            return SetParametersResult(
                successful=False, reason='S 需要满足 0 <= s_min < s_max <= 255')
        if not (0 <= v_min < v_max <= 255):
            return SetParametersResult(
                successful=False, reason='V 需要满足 0 <= v_min < v_max <= 255')
        if min_area <= 0:
            return SetParametersResult(
                successful=False, reason='min_area 必须大于 0')
        if rate <= 0:
            return SetParametersResult(
                successful=False, reason='publish_rate 必须大于 0')

        self.h_min, self.h_max = h_min, h_max
        self.s_min, self.s_max = s_min, s_max
        self.v_min, self.v_max = v_min, v_max
        self.min_area = min_area

        if image_dir != self.image_dir:
            self.image_dir = image_dir
            self.images = self._scan_images()
            self.index = 0

        if rate != self.publish_rate:
            self.publish_rate = rate
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / rate, self._on_timer)

        self.get_logger().info(
            f'参数已更新: H[{h_min},{h_max}] S[{s_min},{s_max}] V[{v_min},{v_max}] '
            f'min_area={min_area:.0f} rate={self.publish_rate:.1f}Hz')
        return SetParametersResult(successful=True)

    def _on_timer(self):
        if not self.images:
            return

        path = self.images[self.index % len(self.images)]
        self.index += 1

        bgr = cv2.imread(str(path))
        if bgr is None:
            self.get_logger().error(f'图片读取失败: {path}')
            return

        result, _ = detect(
            bgr,
            lower=(self.h_min, self.s_min, self.v_min),
            upper=(self.h_max, self.s_max, self.v_max),
            min_area=self.min_area,
        )

        msg = TargetPosition()
        msg.detected = bool(result.detected)
        msg.u = float(result.u)
        msg.v = float(result.v)
        msg.area = float(result.area)
        self.publisher_.publish(msg)

        self.frame_count += 1
        if msg.detected:
            self.get_logger().info(
                f'[第 {self.frame_count} 条] {path.name}: detected=True '
                f'u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}')
        else:
            self.get_logger().info(
                f'[第 {self.frame_count} 条] {path.name}: detected=False '
                f'-> 未检测到目标')


def main(args=None):
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C 或外部 SIGTERM：rclpy 可能已经关闭了 context
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
