#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 3：发布节点。把任务 1 的真实检测结果发布到 /target_position。

数据来源（README 里要说清楚的就是这一句）：
    - camera_index >= 0      -> cv2.VideoCapture 实时取帧（加分项）
    - camera_index == -1     -> 按顺序逐张读取 image_dir 里的测试图片并循环

所有 HSV / 面积 / 形态学参数都是 ROS2 参数，节点运行中可以用
`ros2 param set` 现场改（加分项「阈值做成 ROS2 参数」），改完下一帧立刻生效。
"""

from __future__ import annotations

import os

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from target_interfaces.msg import TargetPosition

from .detector import (DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER, DEFAULT_MIN_AREA,
                       DEFAULT_MORPH_KERNEL, DetectorConfig, annotate, detect,
                       imread_unicode, imwrite_unicode)

TOPIC = "/target_position"
IMAGE_EXTS = (".jpg", ".jpeg", ".png", ".bmp")

TUNABLE = {"hsv_lower", "hsv_upper", "min_area", "morph_kernel", "use_open", "use_close"}


class TargetPublisher(Node):

    def __init__(self) -> None:
        super().__init__("target_publisher")

        # ---- 参数声明（= 可运行时调整的颜色/面积参数）----
        self.declare_parameter("hsv_lower", list(DEFAULT_HSV_LOWER))
        self.declare_parameter("hsv_upper", list(DEFAULT_HSV_UPPER))
        self.declare_parameter("min_area", DEFAULT_MIN_AREA)
        self.declare_parameter("morph_kernel", DEFAULT_MORPH_KERNEL)
        self.declare_parameter("use_open", True)
        self.declare_parameter("use_close", True)
        # ---- 数据源与节奏 ----
        self.declare_parameter("camera_index", -1)      # -1 = 用图片
        self.declare_parameter("image_dir", "")         # 空 = 包的 share/test_images
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("save_annotated", False)
        self.declare_parameter("output_dir", "out_ros")

        self.cfg = self._cfg_from_params_safe()
        self.rate_hz = float(self.get_parameter("rate_hz").value)
        self.camera_index = int(self.get_parameter("camera_index").value)
        self.loop = bool(self.get_parameter("loop").value)
        self.save_annotated = bool(self.get_parameter("save_annotated").value)
        self.output_dir = str(self.get_parameter("output_dir").value)

        self.publisher = self.create_publisher(TargetPosition, TOPIC, 10)
        self.count = 0
        self.index = 0
        self.cap = None
        self.images = []
        self.camera_failures = 0

        if self.camera_index >= 0:
            self.cap = cv2.VideoCapture(self.camera_index)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            if not self.cap.isOpened():
                self.get_logger().error(
                    "打不开摄像头 index=%d，退回使用测试图片" % self.camera_index)
                self.cap = None
                self.camera_index = -1
        if self.cap is None:
            self.images = self._collect_images()
            if not self.images:
                self.get_logger().error(
                    "找不到测试图片。请用 --ros-args -p image_dir:=/path/to/imgs 指定目录，"
                    "或放图到 %s" % os.path.join(
                        get_package_share_directory("green_detector"), "test_images"))
            else:
                self.get_logger().info(
                    "数据源: %d 张测试图片, rate=%.1f Hz" % (len(self.images), self.rate_hz))

        self.add_on_set_parameters_callback(self._on_set_parameters)
        self.timer = self.create_timer(1.0 / max(self.rate_hz, 0.1), self.on_timer)
        self.get_logger().info("参数: %s" % self.cfg.describe())
        self.get_logger().info("发布话题: %s" % TOPIC)

    # ------------------------------------------------------------------
    def _cfg_from_params(self) -> DetectorConfig:
        lower = self.get_parameter("hsv_lower").value
        upper = self.get_parameter("hsv_upper").value
        return DetectorConfig(
            hsv_lower=tuple(int(v) for v in (lower or DEFAULT_HSV_LOWER)),
            hsv_upper=tuple(int(v) for v in (upper or DEFAULT_HSV_UPPER)),
            min_area=float(self.get_parameter("min_area").value),
            morph_kernel=int(self.get_parameter("morph_kernel").value),
            use_open=bool(self.get_parameter("use_open").value),
            use_close=bool(self.get_parameter("use_close").value),
        )

    def _cfg_from_params_safe(self) -> DetectorConfig:
        """启动时用：参数非法不要让节点直接崩，回落到默认值并打 error。"""
        try:
            return self._cfg_from_params()
        except (ValueError, TypeError) as exc:
            self.get_logger().error(
                "启动参数非法（%s），已回落到默认参数 %s" % (exc, DetectorConfig().describe()))
            return DetectorConfig()

    def _collect_images(self):
        folder = str(self.get_parameter("image_dir").value or "")
        if not folder:
            folder = os.path.join(get_package_share_directory("green_detector"), "test_images")
        if not os.path.isdir(folder):
            self.get_logger().error("图片目录不存在: %s" % folder)
            return []
        names = sorted(n for n in os.listdir(folder) if n.lower().endswith(IMAGE_EXTS))
        return [os.path.join(folder, n) for n in names]

    def _on_set_parameters(self, params):
        """运行时改参数：颜色/面积类即时生效；rate_hz 重建定时器；其余明确拒绝。

        注意：不在白名单里的参数如果直接 return successful=True，`ros2 param set`
        会显示成功但实际没有任何效果 —— 那比报错更难排查，所以这里显式拒绝。
        """
        names = {p.name for p in params}
        unknown = names - TUNABLE - {"rate_hz"}
        if unknown:
            return SetParametersResult(
                successful=False,
                reason="这些参数不支持运行时修改，请重启节点：%s" % sorted(unknown))

        if "rate_hz" in names:
            new_rate = float(next(p.value for p in params if p.name == "rate_hz"))
            if new_rate <= 0.0:
                return SetParametersResult(successful=False, reason="rate_hz 必须大于 0")
            self.rate_hz = new_rate
            self.destroy_timer(self.timer)
            self.timer = self.create_timer(1.0 / self.rate_hz, self.on_timer)
            self.get_logger().warn("发布频率已改为 %.2f Hz" % self.rate_hz)

        touched = [p for p in params if p.name in TUNABLE]
        if not touched:
            return SetParametersResult(successful=True)
        old = self.cfg
        try:
            cfg = self._cfg_from_params_with(params)
        except (ValueError, TypeError) as exc:
            self.cfg = old
            return SetParametersResult(successful=False, reason=str(exc))
        self.cfg = cfg
        self.get_logger().warn("参数已更新: %s" % cfg.describe())
        return SetParametersResult(successful=True)

    def _cfg_from_params_with(self, params):
        """把待生效的参数合并进来再校验（参数回调触发时 get_parameter 可能仍是旧值）。"""
        values = {
            "hsv_lower": list(self.get_parameter("hsv_lower").value),
            "hsv_upper": list(self.get_parameter("hsv_upper").value),
            "min_area": float(self.get_parameter("min_area").value),
            "morph_kernel": int(self.get_parameter("morph_kernel").value),
            "use_open": bool(self.get_parameter("use_open").value),
            "use_close": bool(self.get_parameter("use_close").value),
        }
        for p in params:
            if p.name in values:
                values[p.name] = p.value
        return DetectorConfig(
            hsv_lower=tuple(int(v) for v in values["hsv_lower"]),
            hsv_upper=tuple(int(v) for v in values["hsv_upper"]),
            min_area=float(values["min_area"]),
            morph_kernel=int(values["morph_kernel"]),
            use_open=bool(values["use_open"]),
            use_close=bool(values["use_close"]),
        )

    # ------------------------------------------------------------------
    def _next_frame(self):
        """返回 (frame, 数据来源描述)。取不到就返回 (None, 原因)。"""
        if self.cap is not None:
            ok, frame = self.cap.read()
            if ok:
                self.camera_failures = 0
                return frame, "camera:%d" % self.camera_index
            # 相机打开成功但中途掉线/被占用：连续失败 5 次就释放并回退到图片，
            # 否则节点会一直活着但永远发 detected=false（比启动失败更难排查）。
            self.camera_failures += 1
            if self.camera_failures >= 5:
                self.get_logger().error(
                    "摄像头连续 %d 次读取失败，已释放并回退到测试图片" % self.camera_failures)
                self.cap.release()
                self.cap = None
                self.camera_failures = 0
                self.images = self._collect_images()
            return None, "摄像头读取失败(%d/5)" % self.camera_failures

        if not self.images:
            return None, "没有可用图片"
        if self.index >= len(self.images):
            if not self.loop:
                return None, "图片已全部处理完（loop=false）"
            self.index = 0
        path = self.images[self.index]
        self.index += 1
        frame = imread_unicode(path)
        if frame is None:
            return None, "读图失败: %s" % path
        return frame, os.path.basename(path)

    def on_timer(self) -> None:
        msg = TargetPosition()
        frame, source = self._next_frame()

        if frame is None:
            # 没拿到图：detected=false + 约定默认值 0，绝不复用上一帧坐标
            self.count += 1
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.publisher.publish(msg)
            self.get_logger().warn("[#%d] 无图像(%s) -> detected=false u=0 v=0 area=0"
                                   % (self.count, source))
            return

        det = detect(frame, self.cfg)
        self.count += 1

        msg.detected = bool(det.detected)
        msg.u = float(det.u)
        msg.v = float(det.v)
        msg.area = float(det.area)
        self.publisher.publish(msg)

        if det.detected:
            self.get_logger().info(
                "[#%d] %s -> detected=true u=%.1f v=%.1f area=%.1f"
                % (self.count, source, det.u, det.v, det.area))
        else:
            self.get_logger().info(
                "[#%d] %s -> detected=false u=0 v=0 area=0（未检测到目标）"
                % (self.count, source))

        if self.save_annotated:
            stem = os.path.splitext(os.path.basename(source))[0] or "frame"
            out = os.path.join(self.output_dir, "%03d_%s.png" % (self.count, stem))
            imwrite_unicode(out, annotate(frame, det, self.cfg))

    def destroy_node(self) -> bool:
        if self.cap is not None:
            self.cap.release()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TargetPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("收到 Ctrl+C，退出。共发布 %d 条消息" % node.count)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
