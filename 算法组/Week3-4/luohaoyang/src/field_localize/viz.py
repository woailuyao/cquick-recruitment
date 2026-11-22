"""可视化：左侧 RGB 叠加 + 右侧场地俯视图（左右分屏）。"""

from __future__ import annotations

import math

import cv2
import numpy as np

TYPE_COLORS = {
    "LCross": (0, 255, 0),
    "TCross": (255, 160, 0),
    "XCross": (0, 0, 255),
    "PenaltyPoint": (255, 0, 255),
}


class MapPanel:
    """把 field.yaml 画成俯视图，并叠加相机位置/朝向/观测点。"""

    def __init__(self, field_map, size=(640, 720), margin=55, field_size=None,
                 strip_h=150):
        self.map = field_map
        self.W, self.H = size
        self.margin = margin
        self.strip_h = int(strip_h)
        self.map_h = self.H - self.strip_h
        x0, y0, x1, y1 = field_map.bbox(pad=1.0)
        if field_size:
            L = float(field_size.get("length", 0) or 0)
            Wd = float(field_size.get("width", 0) or 0)
            if L > 0 and Wd > 0:
                x0 = min(x0, -0.5)
                y0 = min(y0, -0.5)
                x1 = max(x1, L + 0.5)
                y1 = max(y1, Wd + 0.5)
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1
        self.field_size = field_size or {}
        sx = (self.W - 2 * margin) / max(1e-6, x1 - x0)
        sy = (self.map_h - 2 * margin) / max(1e-6, y1 - y0)
        self.scale = min(sx, sy)

    # ------------------------------------------------------------------
    def to_px(self, x, y):
        px = self.margin + (x - self.x0) * self.scale
        py = self.map_h - self.margin - (y - self.y0) * self.scale
        return int(round(px)), int(round(py))

    # ------------------------------------------------------------------
    def render(self, pose=None, obs_map=None, status_lines=None, title=""):
        img = np.full((self.H, self.W, 3), 32, np.uint8)

        # 网格
        step = 1.0
        x = math.floor(self.x0)
        while x <= self.x1:
            p1, p2 = self.to_px(x, self.y0), self.to_px(x, self.y1)
            cv2.line(img, p1, p2, (52, 52, 52), 1)
            cv2.putText(img, f"{x:.0f}", (p1[0] - 8, self.map_h - self.margin + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)
            x += step
        y = math.floor(self.y0)
        while y <= self.y1:
            p1, p2 = self.to_px(self.x0, y), self.to_px(self.x1, y)
            cv2.line(img, p1, p2, (52, 52, 52), 1)
            cv2.putText(img, f"{y:.0f}", (6, p1[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (110, 110, 110), 1)
            y += step

        # 场地边界
        L = float(self.field_size.get("length", 0) or 0)
        Wd = float(self.field_size.get("width", 0) or 0)
        if L > 0 and Wd > 0:
            p1 = self.to_px(0, 0)
            p2 = self.to_px(L, Wd)
            cv2.rectangle(img, p1, p2, (90, 90, 90), 1)

        # 地图标志
        for m in self.map.markers:
            c = TYPE_COLORS.get(m.type, (200, 200, 200))
            p = self.to_px(m.x, m.y)
            cv2.circle(img, p, 5, c, -1)
            cv2.putText(img, m.id, (p[0] + 7, p[1] - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, c, 1)

        # 观测点（用当前位姿换到地图系）与匹配连线
        if pose is not None and obs_map:
            x, y, yaw = pose
            c, s = math.cos(yaw), math.sin(yaw)
            for o in obs_map:
                px = c * o["x"] - s * o["y"] + x
                py = s * o["x"] + c * o["y"] + y
                p = self.to_px(px, py)
                cv2.drawMarker(img, p, (0, 255, 255), cv2.MARKER_TILTED_CROSS, 9, 1)
                if o.get("map_id"):
                    mm = self.map.get(o["map_id"])
                    if mm is not None:
                        cv2.line(img, p, self.to_px(mm.x, mm.y), (0, 200, 255), 1)

        # 相机位姿
        if pose is not None:
            x, y, yaw = pose
            p = self.to_px(x, y)
            tip = self.to_px(x + 0.9 * math.cos(yaw), y + 0.9 * math.sin(yaw))
            left = self.to_px(x + 0.32 * math.cos(yaw + 2.5),
                              y + 0.32 * math.sin(yaw + 2.5))
            right = self.to_px(x + 0.32 * math.cos(yaw - 2.5),
                               y + 0.32 * math.sin(yaw - 2.5))
            cv2.fillConvexPoly(img, np.array([tip, left, right]), (0, 220, 255))
            cv2.circle(img, p, 2, (0, 0, 0), -1)

        # 标题
        cv2.putText(img, title or "field map", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

        # 坐标轴（画在地图原点，原点不可见时退回左下角）
        if self.x0 < -0.2 < 0.2 < self.x1 and self.y0 < -0.2 < 0.2 < self.y1:
            ax, ay = 0.0, 0.0
        else:
            ax, ay = self.x0, self.y0
        o = self.to_px(ax, ay)
        cv2.arrowedLine(img, o, (o[0] + 45, o[1]), (150, 150, 150), 1, tipLength=0.3)
        cv2.putText(img, "+X", (o[0] + 48, o[1] + 4), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (150, 150, 150), 1)
        cv2.arrowedLine(img, o, (o[0], o[1] - 45), (150, 150, 150), 1, tipLength=0.3)
        cv2.putText(img, "+Y", (o[0] - 4, o[1] - 50), cv2.FONT_HERSHEY_SIMPLEX,
                    0.38, (150, 150, 150), 1)

        # 文字块（放在底部独立条带里，不遮挡地图）
        if status_lines:
            lines = [str(s) for s in status_lines]
            cv2.rectangle(img, (0, self.map_h), (self.W, self.H), (18, 18, 18), -1)
            cv2.line(img, (0, self.map_h), (self.W, self.map_h), (70, 70, 70), 1)
            for i, s in enumerate(lines):
                color = (60, 60, 255) if s.startswith("!") else (120, 255, 120)
                cv2.putText(img, s.lstrip("!"), (12, self.map_h + 24 + i * 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.46, color, 1)
        return img


# ----------------------------------------------------------------------
def compose(left, right):
    """左右拼成一张图（高度对齐）。"""
    if left.shape[0] != right.shape[0]:
        h = min(left.shape[0], right.shape[0])
        left = cv2.resize(left, (int(left.shape[1] * h / left.shape[0]), h))
        right = cv2.resize(right, (int(right.shape[1] * h / right.shape[0]), h))
    return np.hstack([left, right])


def draw_hud(frame, lines, origin=(10, 22), color=(255, 255, 255), scale=0.5):
    for i, s in enumerate(lines):
        y = origin[1] + i * 20
        cv2.putText(frame, s, (origin[0] + 10, y),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3)
        cv2.putText(frame, s, (origin[0], y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1)
    return frame