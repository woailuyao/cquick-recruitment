# -*- coding: utf-8 -*-
"""
可视化：左 RGB 实时画面 + 右 field.yaml 俯视地图（细则 2.11 要求的分屏布局）

注意：图上文字一律用 ASCII / 英文，因为 OpenCV 的 Hershey 字体不支持中文
（画中文会变成一串问号）。中文说明放在控制台和 README 里。
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

COLORS = {
    "LCross":       (0, 165, 255),    # BGR 橙
    "TCross":       (255, 170, 0),    # 蓝
    "XCross":       (0, 255, 170),    # 青绿
    "PenaltyPoint": (255, 0, 255),    # 品红
    "ball":         (0, 255, 0),      # 绿
    "unknown":      (200, 200, 200),
}

MAP_BG = (28, 30, 34)
FIELD_FILL = (46, 70, 52)
FIELD_LINE = (200, 210, 200)
GRID = (44, 48, 54)
TEXT = (235, 235, 235)
TEXT_DIM = (150, 156, 164)
OK_COLOR = (80, 220, 120)
BAD_COLOR = (70, 70, 235)


def color_of(name: str):
    return COLORS.get(name, COLORS["unknown"])


def put(img, text, org, scale=0.5, color=TEXT, thick=1, mono=False):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                (0, 0, 0), thick + 2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale,
                color, thick, cv2.LINE_AA)


def draw_symbol(img, cx, cy, kind, color, size=9, thick=2):
    """在地图上画一个标志符号，方便一眼分辨类别。"""
    s = size
    if kind == "LCross":
        cv2.line(img, (cx - s, cy + s), (cx - s, cy - s), color, thick, cv2.LINE_AA)
        cv2.line(img, (cx - s, cy + s), (cx + s, cy + s), color, thick, cv2.LINE_AA)
    elif kind == "TCross":
        cv2.line(img, (cx - s, cy - s), (cx + s, cy - s), color, thick, cv2.LINE_AA)
        cv2.line(img, (cx, cy - s), (cx, cy + s), color, thick, cv2.LINE_AA)
    elif kind == "XCross":
        cv2.line(img, (cx - s, cy - s), (cx + s, cy + s), color, thick, cv2.LINE_AA)
        cv2.line(img, (cx - s, cy + s), (cx + s, cy - s), color, thick, cv2.LINE_AA)
    elif kind == "PenaltyPoint":
        cv2.circle(img, (cx, cy), max(4, s - 2), color, thick, cv2.LINE_AA)
    else:
        cv2.circle(img, (cx, cy), max(3, s - 3), color, -1, cv2.LINE_AA)


@dataclass
class MapView:
    """地图 <-> 像素 的坐标映射，绘制和调试都用它。"""
    w: int
    h: int
    minx: float
    maxx: float
    miny: float
    maxy: float
    margin_px: int = 34

    def __post_init__(self):
        bw = max(1e-6, self.maxx - self.minx)
        bh = max(1e-6, self.maxy - self.miny)
        self.sx = (self.w - 2 * self.margin_px) / bw
        self.sy = (self.h - 2 * self.margin_px) / bh
        self.s = min(self.sx, self.sy)
        # 居中
        self.ox = self.margin_px + ((self.w - 2 * self.margin_px) - bw * self.s) / 2.0
        self.oy = self.margin_px + ((self.h - 2 * self.margin_px) - bh * self.s) / 2.0

    def to_px(self, x: float, y: float) -> tuple[int, int]:
        """地图 (x 右, y 上) -> 图像像素 (y 轴向下，所以翻转)。"""
        px = self.ox + (x - self.minx) * self.s
        py = self.oy + (self.maxy - y) * self.s
        return int(round(px)), int(round(py))

    def scale(self, meters: float) -> float:
        return meters * self.s


def make_map_base(w: int, h: int, field_map):
    """
    画地图的**静态部分**（背景、网格、场地框、坐标轴、所有特征符号），
    返回 (base_img, view)。这部分每帧都一样，应该缓存起来只画一次。

    为什么拆开：实测每帧重画整个地图面板要吃 20~30 ms，
    在 CPU 推理本来就只有 16 fps 的情况下，这是白白丢掉的帧率。
    """
    img = np.full((h, w, 3), MAP_BG, np.uint8)

    minx, maxx, miny, maxy = field_map.bounds()
    pad = max(1.0, 0.08 * max(maxx - minx, maxy - miny))
    view = MapView(w, h, minx - pad, maxx + pad, miny - pad, maxy + pad)

    # 场地范围
    x0, y0 = view.to_px(minx, maxy)
    x1, y1 = view.to_px(maxx, miny)
    cv2.rectangle(img, (x0, y0), (x1, y1), FIELD_FILL, -1)
    cv2.rectangle(img, (x0, y0), (x1, y1), FIELD_LINE, 1, cv2.LINE_AA)

    # 1 米网格
    g = 1.0 if view.s > 12 else 2.0
    gx = math.ceil(minx / g) * g
    while gx <= maxx:
        px, _ = view.to_px(gx, miny)
        cv2.line(img, (px, y0), (px, y1), GRID, 1)
        gx += g
    gy = math.ceil(miny / g) * g
    while gy <= maxy:
        _, py = view.to_px(minx, gy)
        cv2.line(img, (x0, py), (x1, py), GRID, 1)
        gy += g

    # 原点与坐标轴
    ox, oy = view.to_px(0.0, 0.0)
    put(img, "O", (ox + 4, oy - 4), 0.45, TEXT_DIM)
    ax_len = view.scale(0.6)
    cv2.arrowedLine(img, (ox, oy), (int(ox + ax_len), oy), (90, 160, 250), 2, tipLength=0.3)
    cv2.arrowedLine(img, (ox, oy), (ox, int(oy - ax_len)), (250, 160, 90), 2, tipLength=0.3)
    put(img, "+X", (int(ox + ax_len + 2), oy + 4), 0.4, (90, 160, 250))
    put(img, "+Y", (ox + 3, int(oy - ax_len - 4)), 0.4, (250, 160, 90))

    # 所有特征符号（静态）
    for f in field_map.features:
        px, py = view.to_px(f.x, f.y)
        draw_symbol(img, px, py, f.type, color_of(f.type), size=7, thick=1)

    put(img, f"MAP  {field_map.version}", (10, h - 10), 0.45, TEXT_DIM)
    return img, view


def draw_map_overlay(base: np.ndarray, view: "MapView", field_map,
                     matched_fids=(), pose: Optional[tuple] = None,
                     status_text: str = "", valid: bool = False,
                     extra_lines=()) -> np.ndarray:
    """在缓存的静态底图上叠加**每帧变化**的部分：命中高亮、相机位姿、文字。"""
    img = base.copy()

    for f in field_map.features:
        if f.fid not in matched_fids:
            continue
        px, py = view.to_px(f.x, f.y)
        cv2.circle(img, (px, py), 11, (255, 255, 255), 1, cv2.LINE_AA)
        draw_symbol(img, px, py, f.type, color_of(f.type), size=8, thick=2)
        put(img, f"#{f.fid}", (px + 12, py - 8), 0.38, (255, 255, 255))

    if pose is not None and valid and np.all(np.isfinite(pose)):
        px, py = view.to_px(pose[0], pose[1])
        yaw = math.radians(pose[2])
        L = max(16, view.scale(0.55))
        # 由于像素 y 轴向下，地图 yaw 逆时针 -> 像素向量 (cos, -sin)
        dx, dy = math.cos(yaw), -math.sin(yaw)
        tip = (int(px + L * dx), int(py + L * dy))
        for a in (-28, 28):
            ang = yaw + math.radians(a)
            cv2.line(img, (px, py),
                     (int(px + L * 1.7 * math.cos(ang)), int(py - L * 1.7 * math.sin(ang))),
                     (110, 190, 120), 1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 6, OK_COLOR, -1, cv2.LINE_AA)
        cv2.circle(img, (px, py), 9, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.arrowedLine(img, (px, py), tip, OK_COLOR, 3, tipLength=0.35)

    y = 22
    for line in extra_lines:
        put(img, line, (10, y), 0.46, TEXT)
        y += 20
    if status_text:
        put(img, status_text, (10, y), 0.46, OK_COLOR if valid else BAD_COLOR)

    return img


def render_map(w: int, h: int, field_map, matched_fids=(),
               pose: Optional[tuple] = None, status_text: str = "",
               valid: bool = False, extra_lines=()) -> np.ndarray:
    """一次性画整张地图（不缓存版本，给离线脚本用）。"""
    base, view = make_map_base(w, h, field_map)
    return draw_map_overlay(base, view, field_map, matched_fids, pose,
                            status_text, valid, extra_lines)


def draw_detections(img: np.ndarray, dets, feature_points=None, show_labels=True,
                    show_ball_center=True, show_ground_coord=True):
    """
    dets: 可迭代的 (bbox(x1,y1,x2,y2), label, conf, center_uv)
    feature_points: [(u, v, label[, x_b, y_b])]
        前三个是几何细化后的像素点；后两个（可选）是该点在载体坐标系中的地面米制坐标。
        show_ground_coord=True 时会把「像素 → 地面米制坐标」的结果直接画在图上，
        对应细则 2.4「展示标志从像素点到载体地面坐标的转换结果」。
    feature_points: [(u,v,label)] 用十字标出几何细化后的真实特征点

    show_ball_center: 在足球框中心画十字并标出像素坐标。
        考核细则要求录屏里必须能看到足球的「像素中心」，只画框不够。
    """
    for d in dets:
        x1, y1, x2, y2 = [int(v) for v in d[0]]
        label, conf = d[1], d[2]
        c = color_of(label)
        cv2.rectangle(img, (x1, y1), (x2, y2), c, 2, cv2.LINE_AA)
        if show_labels:
            txt = f"{label} {conf:.2f}"
            (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(img, (x1, max(0, y1 - th - 6)), (x1 + tw + 6, y1), c, -1)
            put(img, txt, (x1 + 3, max(12, y1 - 5)), 0.5, (20, 20, 20))

        # 足球的像素中心：细则要求实时画面里能看到 (u, v)
        if show_ball_center and label == "ball":
            cu, cv_ = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.drawMarker(img, (cu, cv_), (255, 255, 255), cv2.MARKER_CROSS, 12, 2)
            cv2.circle(img, (cu, cv_), 2, c, -1, cv2.LINE_AA)
            ctxt = f"({cu}, {cv_})"
            (tw2, th2), _ = cv2.getTextSize(ctxt, cv2.FONT_HERSHEY_SIMPLEX, 0.45, 1)
            ty = cv_ - 10 if cv_ > 40 else cv_ + th2 + 10
            cv2.rectangle(img, (cu + 6, ty - th2 - 3), (cu + 12 + tw2, ty + 3),
                          (0, 0, 0), -1)
            put(img, ctxt, (cu + 9, ty), 0.45, (255, 255, 255))

    if feature_points:
        for fp in feature_points:
            u, v, label = fp[0], fp[1], fp[2]
            xb = fp[3] if len(fp) > 3 else None
            yb = fp[4] if len(fp) > 4 else None
            if not (np.isfinite(u) and np.isfinite(v)):
                continue
            c = color_of(label)
            cv2.drawMarker(img, (int(u), int(v)), c, cv2.MARKER_CROSS, 14, 2)
            cv2.circle(img, (int(u), int(v)), 3, (255, 255, 255), -1, cv2.LINE_AA)
            # 像素 -> 载体地面米制坐标（细则 2.4 要求展示这一转换结果）
            if show_ground_coord and xb is not None and np.isfinite(xb) and np.isfinite(yb):
                txt = f"({xb:+.2f},{yb:+.2f})m"
                (tw, th), _ = cv2.getTextSize(txt, cv2.FONT_HERSHEY_SIMPLEX, 0.42, 1)
                ty = int(v) + 18
                if ty + th > img.shape[0] - 2:
                    ty = int(v) - 14
                cv2.rectangle(img, (int(u) - 2, ty - th - 2),
                              (int(u) + tw + 3, ty + 3), (0, 0, 0), -1)
                put(img, txt, (int(u), ty), 0.42, (200, 255, 200))


def compose_split(left: np.ndarray, right: np.ndarray, divider: int = 3) -> np.ndarray:
    """左右拼接；高度不一致时把左侧等比缩放到与右侧一致。"""
    if left.shape[0] != right.shape[0]:
        h = right.shape[0]
        scale = h / left.shape[0]
        left = cv2.resize(left, (int(left.shape[1] * scale), h), interpolation=cv2.INTER_AREA)
    canvas = np.zeros((left.shape[0], left.shape[1] + divider + right.shape[1], 3), np.uint8)
    canvas[:, :left.shape[1]] = left
    canvas[:, left.shape[1]:left.shape[1] + divider] = (70, 70, 70)
    canvas[:, left.shape[1] + divider:] = right
    return canvas
