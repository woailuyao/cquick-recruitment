#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""发布节点：图片 -> 蓝色目标检测 -> 发布到 /target_position。

数据来源（这点评审一定会问）
---------------------------
    * use_fixed_test_data:=false（默认，最终版）：逐张读取 data/images/photos 里的
      真实图片，每张图跑一次 detector.detect()，发的是**真实检测结果**。
      所以 u/v/area 会随图片变化，把 HSV 换成绿色区间时，四张会发 detected=false。
    * use_fixed_test_data:=true（只用于第 3.3 步"先验证通信"）：
      发 4 组写死的假数据，用来先确认话题能通，跟视觉算法无关。

可运行时调整的参数（加分项 +3）
-------------------------------
    ros2 param set /target_publisher hsv_lower "[110, 140, 80]"
    ros2 param set /target_publisher min_area 300.0
改了之后不用重启节点，下一个 timer 周期就用新阈值。
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node

from cquick_interfaces.msg import TargetPosition
from cquick_vision.detector import (DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                                    DEFAULT_MIN_AREA, BlueTargetDetector)

TOPIC_NAME = "/target_position"
IMG_EXTS = (".jpg", ".jpeg", ".png", ".bmp")

# 只在"先验证通信"阶段用。注意：这些数字是写死的，不是检测出来的！
FIXED_TEST_DATA = [
    (True, 320.0, 240.0, 4800.0),
    (True, 331.5, 245.0, 4912.0),
    (False, 0.0, 0.0, 0.0),
    (True, 342.0, 251.5, 5030.0),
]


def imread_bgr(path: Path) -> np.ndarray | None:
    """带中文路径也能读的 imread。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_bgr(path: Path, img: np.ndarray) -> bool:
    path.parent.mkdir(parents=True, exist_ok=True)
    ok, buf = cv2.imencode(path.suffix or ".png", img)
    if not ok:
        return False
    buf.tofile(str(path))
    return True


class TargetPublisher(Node):

    def __init__(self) -> None:
        super().__init__("target_publisher")

        # ---------------- 参数 ----------------
        self.declare_parameter("image_dir", "data/images/photos")
        self.declare_parameter("publish_rate", 2.0)
        self.declare_parameter("use_fixed_test_data", False)
        self.declare_parameter("hsv_lower", list(DEFAULT_HSV_LOWER))
        self.declare_parameter("hsv_upper", list(DEFAULT_HSV_UPPER))
        self.declare_parameter("min_area", float(DEFAULT_MIN_AREA))
        self.declare_parameter("kernel", 5)
        self.declare_parameter("morph", True)
        self.declare_parameter("select", "largest")
        self.declare_parameter("loop", True)
        self.declare_parameter("output_dir", "output/ros2")

        self.frame_index = 0
        self.msg_count = 0
        self.miss_count = 0
        self.finished = False

        self.timer = None
        self.detector = self._build_detector()
        self.images = self._collect_images()

        self.publisher = self.create_publisher(TargetPosition, TOPIC_NAME, 10)
        self._start_timer()
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            f"发布话题 {TOPIC_NAME}，类型 cquick_interfaces/msg/TargetPosition")
        self.get_logger().info(
            f"图片 {len(self.images)} 张，来自 {self.get_parameter('image_dir').value}")
        self.get_logger().info(
            f"HSV 下界={self.detector.hsv_lower} 上界={self.detector.hsv_upper} "
            f"min_area={self.detector.min_area:g} 形态学={self.detector.morph} "
            f"核={self.detector.kernel} 规则={self.detector.select}")
        if bool(self.get_parameter("use_fixed_test_data").value):
            self.get_logger().warning(
                "use_fixed_test_data=true：现在发的是写死的假坐标，只用于验证通信！")

    # ------------------------------------------------------------------ #
    def _build_detector(self) -> BlueTargetDetector:
        return BlueTargetDetector(
            hsv_lower=list(self.get_parameter("hsv_lower").value),
            hsv_upper=list(self.get_parameter("hsv_upper").value),
            min_area=float(self.get_parameter("min_area").value),
            morph=bool(self.get_parameter("morph").value),
            kernel=int(self.get_parameter("kernel").value),
            select=str(self.get_parameter("select").value),
        )

    def _collect_images(self) -> list[Path]:
        folder = Path(str(self.get_parameter("image_dir").value)).expanduser()
        if not folder.is_dir():
            self.get_logger().error(f"图片目录不存在：{folder.resolve()}")
            self.get_logger().error("请在项目根目录下运行，或用 image_dir:=/绝对/路径 指定")
            return []
        files = sorted(p for p in folder.iterdir() if p.suffix.lower() in IMG_EXTS)
        if not files:
            self.get_logger().error(f"目录里没有图片：{folder.resolve()}")
        return files

    def _start_timer(self) -> None:
        rate = float(self.get_parameter("publish_rate").value)
        rate = max(0.2, min(rate, 50.0))
        if self.timer is not None:
            self.destroy_timer(self.timer)
        self.timer = self.create_timer(1.0 / rate, self._on_timer)

    # ------------------------------------------------------------------ #
    def _on_param_change(self, params):
        """运行时改参数：改检测相关 -> 重建 detector；改频率 -> 重建 timer。"""
        rebuild = False
        restart_timer = False
        for p in params:
            if p.name in ("hsv_lower", "hsv_upper", "min_area", "kernel", "morph", "select"):
                rebuild = True
            elif p.name == "publish_rate":
                restart_timer = True
            elif p.name == "image_dir":
                self.images = self._collect_images()
                self.frame_index = 0

        if rebuild:
            try:
                self.detector = self._build_detector()
            except ValueError as exc:          # 例如上界小于下界
                self.get_logger().error(f"参数不合法，保持旧参数：{exc}")
                return SetParametersResult(successful=False, reason=str(exc))
            self.get_logger().info(
                f"检测参数已更新：HSV={self.detector.hsv_lower}~{self.detector.hsv_upper} "
                f"min_area={self.detector.min_area:g} 核={self.detector.kernel} "
                f"规则={self.detector.select}")
        if restart_timer:
            self._start_timer()
            self.get_logger().info(
                f"发布频率改为 {float(self.get_parameter('publish_rate').value):.2f} Hz")
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ #
    def _on_timer(self) -> None:
        if bool(self.get_parameter("use_fixed_test_data").value):
            self._publish_fixed()
        else:
            self._publish_detection()

    def _publish_fixed(self) -> None:
        detected, u, v, area = FIXED_TEST_DATA[self.frame_index % len(FIXED_TEST_DATA)]
        self.frame_index += 1
        msg = TargetPosition()
        msg.detected = bool(detected)
        msg.u = float(u)
        msg.v = float(v)
        msg.area = float(area)
        self.publisher.publish(msg)
        self.msg_count += 1
        self.get_logger().info(
            f"[FIXED #{self.msg_count}] detected={msg.detected} "
            f"u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}  (写死的假数据)")

    def _publish_detection(self) -> None:
        if not self.images:
            return
        # loop=false 时只把图片过一遍就停，方便"发 N 条就结束"的场合
        if not bool(self.get_parameter("loop").value) and self.frame_index >= len(self.images):
            if not self.finished:
                self.finished = True
                self.get_logger().info(
                    f"已处理完全部 {len(self.images)} 张图片（loop=false），停止发布")
            return
        path = self.images[self.frame_index % len(self.images)]
        self.frame_index += 1

        bgr = imread_bgr(path)
        if bgr is None:
            self.get_logger().error(f"读取失败，跳过：{path.name}")
            return

        result = self.detector.detect(bgr)

        # 没检测到时，u/v/area 明确写 0，并且 detected=False。
        # 绝不能把上一帧的结果继续发出去。
        msg = TargetPosition()
        msg.detected = bool(result.detected)
        msg.u = float(result.u) if result.detected else 0.0
        msg.v = float(result.v) if result.detected else 0.0
        msg.area = float(result.area) if result.detected else 0.0
        self.publisher.publish(msg)
        self.msg_count += 1
        if not msg.detected:
            self.miss_count += 1

        self.get_logger().info(
            f"[#{self.msg_count} 图{((self.frame_index - 1) % len(self.images)) + 1}/{len(self.images)}] {path.name} -> "
            f"detected={msg.detected} u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f} "
            f"(候选 {result.candidates})")

        out_dir = str(self.get_parameter("output_dir").value)
        if out_dir:
            annotated = self.detector.annotate(bgr, result)
            imwrite_bgr(Path(out_dir) / f"frame_{self.frame_index:03d}_{path.stem}.png", annotated)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = TargetPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
