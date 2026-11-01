"""以实际检测结果发布 /target_position；只读图片，不调用摄像头。

例：ros2 run color_target_vision target_publisher --ros-args -p image_dir:=data/images
运行中调亮度：ros2 param set /target_publisher lower_v 40
运行中调面积：ros2 param set /target_publisher min_area 100.0
"""

import math
from pathlib import Path
import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from color_target_interfaces.msg import TargetPosition

from .core import DetectorConfig, detect
from .image_io import read_image


DETECTOR_PARAMETERS = {
    'lower_h': 95, 'lower_s': 80, 'lower_v': 70,
    'upper_h': 130, 'upper_s': 255, 'upper_v': 255,
    'min_area': 500.0, 'open_kernel': 3, 'close_kernel': 5,
}
IMAGE_SUFFIXES = {'.png', '.jpg', '.jpeg', '.bmp', '.tif', '.tiff', '.webp'}


def config_from_values(values):
    """把完整 ROS 参数集合转换为核心配置，统一检查上下限、面积和核。"""
    config = DetectorConfig(
        hsv_lower=tuple(values[name] for name in ('lower_h', 'lower_s', 'lower_v')),
        hsv_upper=tuple(values[name] for name in ('upper_h', 'upper_s', 'upper_v')),
        min_area=values['min_area'],
        open_kernel=values['open_kernel'],
        close_kernel=values['close_kernel'],
    )
    config.validate()
    return config


class TargetPublisher(Node):
    """按文件名循环读图，每次定时回调都重新检测，不重放固定坐标。"""

    def __init__(self):
        super().__init__('target_publisher')
        try:
            self._initialize()
        except Exception:
            # 构造失败时 main 尚未取得节点引用，需要在这里释放 ROS 资源。
            self.destroy_node()
            raise

    def _initialize(self):
        self.declare_parameter('image_dir', '', ParameterDescriptor(
            description='必填图片目录；只读取第一层图片，启动后不可修改', read_only=True))
        self.declare_parameter('publish_hz', 2.0, ParameterDescriptor(
            description='启动时发布频率，单位 Hz；运行中不可修改', read_only=True))
        for name, default in DETECTOR_PARAMETERS.items():
            self.declare_parameter(name, default, ParameterDescriptor(
                description='颜色检测参数；修改前校验完整配置'))

        image_dir = self.get_parameter('image_dir').value
        if not image_dir:
            raise ValueError('必须传入 image_dir，例如 --ros-args -p image_dir:=data/images')
        folder = Path(image_dir).expanduser()
        if not folder.is_dir():
            raise ValueError('image_dir 不存在或不是目录，请检查当前终端所在位置。')
        self.images = sorted(
            (path for path in folder.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES),
            key=lambda path: path.name.casefold(),
        )
        if not self.images:
            raise ValueError('image_dir 中没有支持的图片；不能用空目录完成验收。')
        # 提前检查全部输入，防止在某张坏图处才发现数据准备有误。
        for path in self.images:
            read_image(path)
        hz = self.get_parameter('publish_hz').value
        if isinstance(hz, bool) or not isinstance(hz, (int, float)) or not math.isfinite(hz) or hz <= 0:
            raise ValueError('publish_hz 必须是正的有限数，例如 2.0。')
        config_from_values(self._parameter_values())
        self.add_on_set_parameters_callback(self._validate_parameter_update)

        # 两端显式使用 reliable / keep_last / depth=10，避免 QoS 不匹配。
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST)
        self.publisher = self.create_publisher(TargetPosition, '/target_position', qos)
        self.sent_count = 0
        self.timer = self.create_timer(1.0 / hz, self._publish_next)
        self.get_logger().info(f'已检查 {len(self.images)} 张图片，按 {hz:g} Hz 循环检测并发布。')

    def _parameter_values(self):
        return {name: self.get_parameter(name).value for name in DETECTOR_PARAMETERS}

    def _validate_parameter_update(self, parameters):
        """只验证、不改变状态；拒绝更新后，定时器继续使用原有有效参数。

        一次请求中的所有检测参数合并后一起校验。需要同时改变多个上下限时，
        使用 ROS 的 set_parameters_atomically 服务；普通 ros2 param set 一次改一个。
        """
        values = self._parameter_values()
        for parameter in parameters:
            if parameter.name in ('image_dir', 'publish_hz'):
                return SetParametersResult(successful=False, reason='该参数仅允许启动时设置。')
            if parameter.name in values:
                values[parameter.name] = parameter.value
        try:
            config_from_values(values)
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _publish_next(self):
        path = self.images[self.sent_count % len(self.images)]
        # 参数由 ROS 完成提交后再读取，回调验证阶段不缓存或提前应用新值。
        config = config_from_values(self._parameter_values())
        try:
            detection = detect(read_image(path), config)
        except Exception as exc:
            # 读图/计算失败与“正常计算但未检出”不同：明确报错并停止，不发假结果。
            self.get_logger().error(f'处理 {path.name} 失败：{exc}')
            raise
        message = TargetPosition()
        message.detected = bool(detection.detected)
        message.u = float(detection.u) if message.detected else 0.0
        message.v = float(detection.v) if message.detected else 0.0
        message.area = float(detection.area) if message.detected else 0.0
        self.publisher.publish(message)
        self.sent_count += 1
        self.get_logger().info(
            f'发送 #{self.sent_count} image={path.name} detected={message.detected} '
            f'u={message.u:.3f} v={message.v:.3f} area={message.area:.3f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = None
    exit_code = 0
    try:
        node = TargetPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as exc:
        print(f'发布节点停止：{exc}', file=sys.stderr)
        exit_code = 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)


if __name__ == '__main__':
    main()
