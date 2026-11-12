"""任务 3：发布节点。

把「任务 1 的检测结果」按固定频率发布到 /target_position。
每到一个定时周期就换下一张图，检测 → 组装 TargetPosition → publish。

话题名和字段必须与 target_interfaces/msg/TargetPosition.msg 完全一致：
    bool detected / float32 u / float32 v / float32 area

运行：
    ros2 run color_target_detector target_publisher --ros-args \
        -p image_dir:=data -p rate_hz:=2.0

可选参数（运行时就能改，不用重新编译）：
    image_dir  图片目录           默认 data
    rate_hz    发布频率 Hz        默认 2.0
    lower_hsv  HSV 下界          默认 [35, 80, 60]
    upper_hsv  HSV 上界          默认 [85, 255, 255]
    min_area   最小轮廓面积      默认 500
    morph      形态学            默认 open
    topic      话题名            默认 /target_position
"""

import glob
import os
import sys

import cv2
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

from .detection import (DEFAULT_LOWER, DEFAULT_MIN_AREA, DEFAULT_UPPER,
                        Detection, detect)


class TargetPublisher(Node):

    def __init__(self):
        super().__init__('target_publisher')

        # ---- 所有阈值都做成 ROS2 参数，符合加分项「可运行时调整」----
        self.declare_parameter('image_dir', 'data')
        self.declare_parameter('rate_hz', 2.0)
        self.declare_parameter('lower_hsv', list(DEFAULT_LOWER))
        self.declare_parameter('upper_hsv', list(DEFAULT_UPPER))
        self.declare_parameter('min_area', float(DEFAULT_MIN_AREA))
        self.declare_parameter('morph', 'open')
        self.declare_parameter('topic', '/target_position')

        self.image_dir = self.get_parameter('image_dir').value
        self.rate_hz = float(self.get_parameter('rate_hz').value)
        self.lower = tuple(self.get_parameter('lower_hsv').value)
        self.upper = tuple(self.get_parameter('upper_hsv').value)
        self.min_area = float(self.get_parameter('min_area').value)
        self.morph = self.get_parameter('morph').value
        topic = self.get_parameter('topic').value

        self.images = sorted(
            glob.glob(os.path.join(self.image_dir, '*.png')) +
            glob.glob(os.path.join(self.image_dir, '*.jpg')) +
            glob.glob(os.path.join(self.image_dir, '*.jpeg'))
        )
        if not self.images:
            self.get_logger().error(
                f'目录 {self.image_dir} 里没有图片，请用 -p image_dir:=<绝对路径> 指定')
            raise SystemExit(1)

        self.pub = self.create_publisher(TargetPosition, topic, 10)
        self.index = -1
        self.count = 0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)

        self.get_logger().info(
            f'发布者已启动: topic={topic} type=target_interfaces/msg/TargetPosition '
            f'共 {len(self.images)} 张图，{self.rate_hz} Hz')

    def on_timer(self):
        self.index = (self.index + 1) % len(self.images)
        path = self.images[self.index]

        image = cv2.imread(path)
        if image is None:
            # 读图失败也发一帧「未检测到」，保证话题不断流
            det = Detection(detected=False)
        else:
            det = detect(image, self.lower, self.upper, self.min_area, self.morph)

        msg = TargetPosition()
        msg.detected = bool(det.detected)
        # 约定：没检测到目标时 u/v/area 一律填 0，绝不用图像左上角冒充目标位置
        msg.u = float(det.u) if det.detected else 0.0
        msg.v = float(det.v) if det.detected else 0.0
        msg.area = float(det.area) if det.detected else 0.0

        self.pub.publish(msg)
        self.count += 1

        if det.detected:
            self.get_logger().info(
                f'[发布 #{self.count}] {os.path.basename(path)} -> detected=True '
                f'u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}')
        else:
            self.get_logger().info(
                f'[发布 #{self.count}] {os.path.basename(path)} -> detected=False（未检测到目标）')


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TargetPublisher()
    except SystemExit:
        rclpy.shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
