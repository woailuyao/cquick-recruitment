#!/usr/bin/env python3
"""发布节点：读取测试图片 → 颜色检测 → 发布 TargetPosition 到 /target_position。

数据来源：逐张处理 data/images/ 下的图片，按固定频率（1 Hz）循环发布，
          每个消息的 detected/u/v/area 都来自真实 HSV 检测，不是写死的坐标。
"""
import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from target_interfaces.msg import TargetPosition

# 每张测试图的 HSV 区间与最小面积（与 src/task1.py 的 CONFIG 完全一致）
# ranges 里每段 (lower, upper) 都会单独 inRange，再取并集；粉色跨 H=0/180 故用两段。
CONFIG = {
    "obj1.jpg": {"ranges": [((0, 90, 110), (8, 255, 255)),
                            ((170, 90, 110), (179, 255, 255))],
                 "min_area": 3000},
    "obj2.jpg": {"ranges": [((10, 120, 90), (25, 255, 255))], "min_area": 3000},
    "obj3.jpg": {"ranges": [((0, 0, 0), (179, 255, 100))], "min_area": 3000},
}


def detect(img, ranges, min_area):
    """与 src/task1.py 相同的检测逻辑，返回 (detected, u, v, area)。"""
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in ranges:
        m = cv2.inRange(hsv, np.array(lower), np.array(upper))
        mask = m if mask is None else cv2.bitwise_or(mask, m)
    k = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cands = [(cv2.contourArea(c), c) for c in contours if cv2.contourArea(c) >= min_area]
    if not cands:
        return False, 0.0, 0.0, 0.0
    cands.sort(key=lambda t: -t[0])
    area, best = cands[0]
    m = cv2.moments(best)
    if m["m00"] == 0:
        return False, 0.0, 0.0, 0.0
    return True, float(m["m10"] / m["m00"]), float(m["m01"] / m["m00"]), float(area)


class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.pub = self.create_publisher(TargetPosition, '/target_position', 10)
        self.image_dir = self.declare_parameter('image_dir', 'data/images').value
        if not os.path.isdir(self.image_dir):
            self.get_logger().error(
                '找不到图片目录: %s（请在 mositao 目录下启动 ros2 run，'
                '或加 --ros-args -p image_dir:=<目录>）' % self.image_dir)
            raise RuntimeError('image dir not found')
        self.images = [f for f in sorted(os.listdir(self.image_dir))
                       if f.lower().endswith(('.jpg', '.jpeg', '.png'))]
        if not self.images:
            self.get_logger().error('image_dir 里没有图片: %s' % self.image_dir)
            raise RuntimeError('no images')
        self.idx = 0
        self.count = 0
        self.get_logger().info('共 %d 张图，1Hz 循环发布到 /target_position' % len(self.images))
        self.timer = self.create_timer(1.0, self.callback)

    def callback(self):
        name = self.images[self.idx % len(self.images)]
        cfg = CONFIG.get(name, CONFIG["obj1.jpg"])
        img = cv2.imread(os.path.join(self.image_dir, name))
        # 与 src/task1.py 一致：长边缩到 800，保证两边坐标可比
        if img is not None:
            h, w = img.shape[:2]
            scale = 800 / max(h, w)
            if scale < 1:
                img = cv2.resize(img, None, fx=scale, fy=scale,
                                 interpolation=cv2.INTER_AREA)
        msg = TargetPosition()
        if img is None:
            msg.detected = False
            msg.u = msg.v = msg.area = 0.0
            self.get_logger().warn('读取失败: %s' % name)
        else:
            detected, u, v, area = detect(img, cfg["ranges"], cfg["min_area"])
            msg.detected = detected
            msg.u = u
            msg.v = v
            msg.area = area
        self.pub.publish(msg)
        self.count += 1
        self.get_logger().info(
            '[%d] %s: detected=%s u=%.1f v=%.1f area=%.0f'
            % (self.count, name, msg.detected, msg.u, msg.v, msg.area))
        self.idx += 1


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
