# -*- coding: utf-8 -*-
"""发布节点：逐张处理测试图片，把检测结果发到 /target_position。

HSV 上下界和最小面积都做成了 ROS2 参数，运行时可以改：
  ros2 run color_detector detect_node --ros-args -p s_min:=150 -p min_area:=800
"""

import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from pathlib import Path

from target_interfaces.msg import TargetPosition

from . import core, paths


def imread_u(path):
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return None if data.size == 0 else cv2.imdecode(data, cv2.IMREAD_COLOR)


class DetectNode(Node):
    def __init__(self):
        super().__init__("color_detect_node")

        # ---- 参数：颜色范围、面积、发布周期、图片目录 ----
        self.declare_parameter("image_dir", str(paths.image_dir()))
        self.declare_parameter("period", 1.0)      # 每张图发一次的间隔，秒
        self.declare_parameter("h_min", core.DEFAULT_LOWER[0])
        self.declare_parameter("h_max", core.DEFAULT_UPPER[0])
        self.declare_parameter("s_min", core.DEFAULT_LOWER[1])
        self.declare_parameter("v_min", core.DEFAULT_LOWER[2])
        self.declare_parameter("s_max", core.DEFAULT_UPPER[1])
        self.declare_parameter("v_max", core.DEFAULT_UPPER[2])
        self.declare_parameter("min_area", core.DEFAULT_MIN_AREA)
        self.declare_parameter("morph", True)

        p = self.get_parameter
        self.lower = (p("h_min").value, p("s_min").value, p("v_min").value)
        self.upper = (p("h_max").value, p("s_max").value, p("v_max").value)
        self.min_area = p("min_area").value
        self.morph = p("morph").value

        image_dir = p("image_dir").value
        self.images = sorted(
            [f for f in Path(image_dir).glob("*")
             if f.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp")]
        )
        self.index = 0

        if not self.images:
            self.get_logger().error("图片目录是空的: %s" % image_dir)
        else:
            self.get_logger().info("找到 %d 张图片, 目录 %s"
                                   % (len(self.images), image_dir))
        self.get_logger().info("HSV 下界=%s 上界=%s 最小面积=%d"
                               % (self.lower, self.upper, self.min_area))

        self.publisher = self.create_publisher(TargetPosition, "/target_position", 10)
        self.timer = self.create_timer(float(p("period").value), self.publish_once)

    def publish_once(self):
        msg = TargetPosition()

        if not self.images:
            # 没图也算“没检测到”，不能拿 0 当坐标用
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.publisher.publish(msg)
            return

        path = self.images[self.index % len(self.images)]
        self.index += 1

        img = imread_u(path)
        if img is None:
            target = None
        else:
            target, _, _ = core.detect(img, self.lower, self.upper,
                                       self.min_area, self.morph)

        if target is None:
            msg.detected = False
            msg.u = 0.0
            msg.v = 0.0
            msg.area = 0.0
            self.get_logger().info("[%s] 未检测到目标" % path.name)
        else:
            msg.detected = True
            msg.u = float(target.u)
            msg.v = float(target.v)
            msg.area = float(target.area)
            self.get_logger().info("[%s] u=%.1f v=%.1f area=%d"
                                   % (path.name, msg.u, msg.v, int(msg.area)))

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = DetectNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
