#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""加分项（可选）：摄像头实时输入版本。

没有摄像头就**完全不用管这个文件**，静态图片版本已经满足考核要求。
有摄像头时：
    ros2 run cquick_vision camera_publisher --ros-args -p device_id:=0

注意：这个文件只能在有摄像头的机器上验证。WSL 里默认没有 /dev/video0，
想用 WSL 摄像头需要额外折腾 usbipd，不建议为了加分在这里耗时间。
"""
from __future__ import annotations

from pathlib import Path

import cv2
import rclpy
from rclpy.node import Node

from cquick_interfaces.msg import TargetPosition
from cquick_vision.detector import (DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                                    DEFAULT_MIN_AREA, BlueTargetDetector)

TOPIC_NAME = "/target_position"


class CameraPublisher(Node):

    def __init__(self) -> None:
        super().__init__("camera_publisher")
        self.declare_parameter("device_id", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("hsv_lower", list(DEFAULT_HSV_LOWER))
        self.declare_parameter("hsv_upper", list(DEFAULT_HSV_UPPER))
        self.declare_parameter("min_area", float(DEFAULT_MIN_AREA))
        self.declare_parameter("output_dir", "output/camera")

        self.detector = BlueTargetDetector(
            hsv_lower=list(self.get_parameter("hsv_lower").value),
            hsv_upper=list(self.get_parameter("hsv_upper").value),
            min_area=float(self.get_parameter("min_area").value),
        )

        device = int(self.get_parameter("device_id").value)
        self.cap = cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(
                f"打不开摄像头 /dev/video{device}。检查：摄像头是否插好、"
                f"当前用户是否在 video 组、WSL 是否直通了 USB 摄像头")
            raise RuntimeError("camera not available")

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("width").value))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("height").value))

        self.publisher = self.create_publisher(TargetPosition, TOPIC_NAME, 10)
        self.msg_count = 0
        rate = max(0.5, float(self.get_parameter("publish_rate").value))
        self.timer = self.create_timer(1.0 / rate, self.on_timer)
        self.get_logger().info(f"摄像头已打开，发布到 {TOPIC_NAME}，{rate:.1f} Hz")

    def on_timer(self) -> None:
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warning("读不到摄像头画面（跳帧）")
            return

        result = self.detector.detect(frame)
        msg = TargetPosition()
        msg.detected = bool(result.detected)
        msg.u = float(result.u) if result.detected else 0.0
        msg.v = float(result.v) if result.detected else 0.0
        msg.area = float(result.area) if result.detected else 0.0
        self.publisher.publish(msg)
        self.msg_count += 1
        if self.msg_count % 10 == 0:      # 摄像头 10Hz，别刷屏
            self.get_logger().info(
                f"[#{self.msg_count}] detected={msg.detected} "
                f"u={msg.u:.1f} v={msg.v:.1f} area={msg.area:.0f}")

        out_dir = str(self.get_parameter("output_dir").value)
        if out_dir and self.msg_count % 30 == 0:
            annotated = self.detector.annotate(frame, result)
            Path(out_dir).mkdir(parents=True, exist_ok=True)
            p = Path(out_dir) / f"camera_{self.msg_count:05d}.png"
            ok2, buf = cv2.imencode(".png", annotated)
            if ok2:
                buf.tofile(str(p))

    def destroy_node(self) -> None:
        if getattr(self, "cap", None) is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = CameraPublisher()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
