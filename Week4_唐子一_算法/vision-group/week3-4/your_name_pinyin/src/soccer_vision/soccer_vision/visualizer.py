"""左右分屏可视化：左侧 RGB 检测叠加，右侧场地地图定位。

考核要求最终界面：
  左侧 = 实际相机实时 RGB + 足球/标志检测框、置信度、几何特征点、FPS；
  右侧 = field.yaml 场地俯视图 + 相机光心位置图标 + 朝向箭头 + 坐标读数。
"""

from __future__ import annotations

from typing import List, Optional

import cv2
import numpy as np

from .field_map import FieldMap
from .localizer import PoseResult


def draw_detections(
    frame: np.ndarray,
    boxes,
    names,
    confs,
    centers,
    fps: float,
    resolution: tuple,
) -> np.ndarray:
    """在 RGB/BGR 帧上绘制检测框、类别、置信度、中心点。"""
    out = frame.copy()
    for box, name, conf, center in zip(boxes, names, confs, centers):
        x1, y1, x2, y2 = [int(v) for v in box]
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
        label = f"{name} {conf:.2f}"
        cv2.putText(
            out, label, (x1, max(0, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2,
        )
        cu, cv_ = int(center[0]), int(center[1])
        cv2.circle(out, (cu, cv_), 3, (0, 0, 255), -1)
    # 左上角信息
    info = f"{resolution[0]}x{resolution[1]}  FPS:{fps:.1f}"
    cv2.putText(out, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
    return out


def draw_map(
    field: FieldMap,
    pose: Optional[PoseResult],
    scale: float = 60.0,
    size: tuple = (500, 400),
) -> np.ndarray:
    """绘制场地俯视图 + 相机位置/朝向。scale 为像素/米。"""
    img = np.full((size[1], size[0], 3), (30, 30, 30), dtype=np.uint8)
    margin = 40

    def to_px(x, y):
        px = int(margin + x * scale)
        py = int(size[1] - margin - y * scale)  # Y 向上
        return px, py

    # 场地边界
    ox, oy = field.origin
    p1 = to_px(ox, oy)
    p2 = to_px(ox + field.length_x, oy + field.length_y)
    cv2.rectangle(img, p1, p2, (200, 200, 200), 2)

    # 原点与轴
    cv2.circle(img, to_px(ox, oy), 4, (0, 0, 255), -1)
    cv2.putText(img, "O", (p1[0] - 15, p1[1] + 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    # X 轴
    cv2.arrowedLine(img, to_px(ox, oy), to_px(ox + 1.0, oy), (0, 0, 255), 2)
    cv2.putText(img, "+X", to_px(ox + 1.0, oy),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    cv2.arrowedLine(img, to_px(ox, oy), to_px(ox, oy + 1.0), (0, 255, 0), 2)
    cv2.putText(img, "+Y", to_px(ox, oy + 1.0),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    # 场地标志点
    color_map = {"LCross": (0, 200, 255), "TCross": (0, 255, 200),
                 "XCross": (255, 200, 0), "PenaltyPoint": (255, 0, 255)}
    for m in field.markers:
        cv2.circle(img, to_px(m.x, m.y), 4,
                   color_map.get(m.type, (255, 255, 255)), -1)

    # 相机位置与朝向
    if pose is not None and pose.valid:
        cx, cy = to_px(pose.x, pose.y)
        cv2.circle(img, (cx, cy), 7, (0, 255, 255), -1)
        # 朝向箭头（长度 0.6 m）
        dx = 0.6 * np.cos(pose.theta_rad)
        dy = 0.6 * np.sin(pose.theta_rad)
        tip = to_px(pose.x + dx, pose.y + dy)
        cv2.arrowedLine(img, (cx, cy), tip, (0, 255, 255), 3)
        text = (
            f"x:{pose.x:.2f} y:{pose.y:.2f} yaw:{pose.yaw_deg:.1f}\n"
            f"n:{pose.marker_count} res:{pose.residual:.2f} {pose.status}"
        )
        for i, line in enumerate(text.split("\n")):
            cv2.putText(img, line, (10, 20 + i * 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return img


def combine(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    """左右拼接，统一高度。"""
    h = max(left.shape[0], right.shape[0])
    lw = left.shape[1]
    rw = int(h * right.shape[1] / right.shape[0])
    right_resized = cv2.resize(right, (rw, h))
    left_resized = left if left.shape[0] == h else cv2.resize(left, (lw, h))
    return np.hstack([left_resized, right_resized])
