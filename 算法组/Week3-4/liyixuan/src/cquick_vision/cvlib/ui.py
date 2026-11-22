"""左右分屏可视化：左 RGB 实时画面，右 field.yaml 俯视图 + 相机位姿。

注意：地图 +Y 向上，而图像 y 向下，所以像素映射必须 ppy = H - (y - ymin) * s。
"""
from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np

from .geom import wrap_deg

COLORS = {
    "LCross": (0, 165, 255),      # 橙
    "TCross": (255, 120, 0),      # 蓝
    "XCross": (255, 0, 255),      # 品红
    "PenaltyPoint": (0, 200, 0),  # 绿
    "ball": (0, 255, 0),
}
STATUS_COLOR = {
    "ok": (0, 200, 0),
    "ambiguous": (0, 200, 255),
    "high_residual": (0, 140, 255),
    "insufficient_markers": (0, 0, 255),
    "not_converged": (0, 0, 220),
    "lost": (60, 60, 200),
}


def put(frame, text, org, color=(255, 255, 255), scale=0.5, thick=1):
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0),
                thick + 2, cv2.LINE_AA)
    cv2.putText(frame, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color,
                thick, cv2.LINE_AA)


class MapView:
    """俯视地图画布：底图只画一次并缓存。"""

    def __init__(self, field_map, size: Tuple[int, int] = (640, 640), margin: int = 40):
        self.fm = field_map
        self.W, self.H = size
        self.margin = margin
        xmin, xmax, ymin, ymax = field_map.bounds
        self.xmin, self.xmax, self.ymin, self.ymax = xmin, xmax, ymin, ymax
        sx = (self.W - 2 * margin) / max(xmax - xmin, 1e-6)
        sy = (self.H - 2 * margin) / max(ymax - ymin, 1e-6)
        self.s = min(sx, sy)
        self.ox = (self.W - (xmax - xmin) * self.s) / 2.0
        self.oy = (self.H - (ymax - ymin) * self.s) / 2.0
        self.base = self._draw_base()

    # 地图米制 -> 画布像素（Y 轴翻转在这里）
    def to_px(self, x: float, y: float) -> Tuple[int, int]:
        px = self.ox + (x - self.xmin) * self.s
        py = self.H - (self.oy + (y - self.ymin) * self.s)
        return int(round(px)), int(round(py))

    def _draw_base(self) -> np.ndarray:
        img = np.full((self.H, self.W, 3), 250, np.uint8)
        # 1 m 网格
        gx = math.ceil(self.xmin)
        while gx <= self.xmax:
            p1 = self.to_px(gx, self.ymin)
            p2 = self.to_px(gx, self.ymax)
            cv2.line(img, p1, p2, (225, 225, 225), 1)
            gx += 1
        gy = math.ceil(self.ymin)
        while gy <= self.ymax:
            p1 = self.to_px(self.xmin, gy)
            p2 = self.to_px(self.xmax, gy)
            cv2.line(img, p1, p2, (225, 225, 225), 1)
            gy += 1
        # 场地外框
        cv2.rectangle(img, self.to_px(self.xmin, self.ymax),
                      self.to_px(self.xmax, self.ymin), (150, 150, 150), 2)
        # 坐标轴
        o = self.to_px(0, 0)
        np_ = self.to_px(1.0, 0)
        cv2.arrowedLine(img, o, np_, (0, 0, 200), 2, tipLength=0.25)
        put(img, "+X", (np_[0] + 4, np_[1] + 4), (0, 0, 200), 0.45)
        ny = self.to_px(0, 1.0)
        cv2.arrowedLine(img, o, ny, (200, 0, 0), 2, tipLength=0.25)
        put(img, "+Y", (ny[0] + 4, ny[1] - 4), (200, 0, 0), 0.45)
        cv2.circle(img, o, 3, (0, 0, 0), -1)
        put(img, "origin", (o[0] + 5, o[1] + 14), (90, 90, 90), 0.4)
        # 地图特征点
        for p in self.fm.points:
            c = COLORS.get(p.type, (0, 0, 0))
            px = self.to_px(p.x, p.y)
            cv2.drawMarker(img, px, c, cv2.MARKER_CROSS, 11, 2)
            put(img, p.mid, (px[0] + 6, px[1] - 4), c, 0.34)
        put(img, self.fm.summary()[:110], (8, 16), (60, 60, 60), 0.38)
        put(img, "map: +X right, +Y up, yaw CCW from +X", (8, self.H - 8),
            (120, 120, 120), 0.40)
        return img

    def render(self, carrier_pose: Optional[Tuple[float, float, float]],
               camera_pose: Optional[Tuple[float, float, float]],
               info: Dict[str, object]) -> np.ndarray:
        img = self.base.copy()
        status = str(info.get("status", "lost"))
        valid = bool(info.get("valid", False))
        col = STATUS_COLOR.get(status, (0, 0, 0)) if valid else STATUS_COLOR.get(
            status, (0, 0, 255))

        # 载体原点
        if carrier_pose is not None:
            cx, cy, cth = carrier_pose
            p = self.to_px(cx, cy)
            cv2.circle(img, p, 5, (90, 90, 90), -1)
            put(img, "carrier", (p[0] + 7, p[1] + 12), (90, 90, 90), 0.38)

        # 相机光心 + 朝向箭头
        if camera_pose is not None:
            cx, cy, cth = camera_pose
            p = self.to_px(cx, cy)
            L = 0.9 * self.s
            tip = (int(p[0] + L * math.cos(cth)), int(p[1] - L * math.sin(cth)))
            cv2.arrowedLine(img, p, tip, col, 3, tipLength=0.28)
            # 相机三角形（光轴方向）
            fwd = np.array([math.cos(cth), -math.sin(cth)])
            left = np.array([-fwd[1], fwd[0]])
            tri = np.array([
                p + fwd * 14,
                p - fwd * 8 + left * 8,
                p - fwd * 8 - left * 8,
            ], np.int32)
            cv2.fillPoly(img, [tri], col)
            cv2.circle(img, p, 3, (255, 255, 255), -1)

        # 读数
        y = 36
        lines = [
            (f"x_camera_map = {info.get('x_camera_map', float('nan')):.3f} m", (30, 30, 30)),
            (f"y_camera_map = {info.get('y_camera_map', float('nan')):.3f} m", (30, 30, 30)),
            (f"yaw_camera_map = {info.get('yaw_camera_map', float('nan')):.2f} deg", (30, 30, 30)),
            (f"marker_count = {info.get('marker_count', 0)}", (30, 30, 30)),
            (f"residual = {info.get('residual', float('nan')):.3f} m", (30, 30, 30)),
            (f"valid = {valid}  status = {status}", col),
            (f"obs = {info.get('n_obs', 0)}  hyps = {info.get('n_hypotheses', 0)}", (80, 80, 80)),
            (f"fps = {info.get('fps', 0.0):.1f}  loc = {info.get('loc_hz', 0.0):.1f} Hz", (80, 80, 80)),
        ]
        for txt, c in lines:
            put(img, txt, (14, y), c, 0.62, 1)
            y += 25

        if info.get("rival"):
            r = info["rival"]
            put(img, f"rival: ({r['x']:.2f},{r['y']:.2f}) "
                     f"{r['yaw_deg']:.1f}deg inl={r['inliers']}",
                (14, y), (0, 140, 255), 0.44)
            rp = self.to_px(r["x"], r["y"])
            cv2.drawMarker(img, rp, (0, 140, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
        return img


def draw_left(frame: np.ndarray, ball_dets: Sequence, dbg: Sequence[Dict],
              info: Dict[str, object], show_hud: bool = True) -> np.ndarray:
    out = frame.copy()
    for d in ball_dets:
        x1, y1, x2, y2 = (int(v) for v in (d.x1, d.y1, d.x2, d.y2))
        cv2.rectangle(out, (x1, y1), (x2, y2), COLORS["ball"], 2)
        u, v = d.center
        cv2.circle(out, (int(u), int(v)), 3, COLORS["ball"], -1)
        put(out, f"{d.cls_name} {d.conf:.2f} ({u:.0f},{v:.0f})",
            (x1, max(12, y1 - 6)), COLORS["ball"], 0.5)

    for e in dbg:
        x1, y1, x2, y2 = (int(v) for v in e["bbox"])
        c = COLORS.get(e["cls"], (200, 200, 200))
        cv2.rectangle(out, (x1, y1), (x2, y2), c, 2)
        u, v = e["pixel"]
        cu, cv_ = int(u), int(v)
        cv2.drawMarker(out, (cu, cv_), (255, 255, 255) if e["method"] != "bbox_center"
                       else (0, 0, 255), cv2.MARKER_CROSS, 13, 2)
        tag = f"{e['cls']} {e['conf']:.2f} {e['method']}"
        if e["distance"] is not None:
            tag += f" {e['distance']:.1f}m"
        if e["reason"]:
            tag += f" [{e['reason']}]"
        put(out, tag, (x1, min(out.shape[0] - 4, y2 + 14)), c, 0.42)

    if show_hud:
        h, w = out.shape[:2]
        put(out, f"{info.get('width', 0)}x{info.get('height', 0)}  "
                 f"frame {info.get('frame_id', 0)}  fps {info.get('fps', 0.0):.1f}  "
                 f"infer {info.get('infer_ms', 0.0):.1f}ms",
            (10, 20), (240, 240, 240), 0.5)
        put(out, f"ball={info.get('n_ball', 0)} marker={len(dbg)} "
                 f"obs={info.get('n_obs', 0)} refine_ok={info.get('n_refined', 0)}",
            (10, h - 12), (240, 240, 240), 0.5)
    return out


def compose_split(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    h = right.shape[0]
    scale = h / left.shape[0]
    lw = int(round(left.shape[1] * scale))
    l = cv2.resize(left, (lw, h), interpolation=cv2.INTER_AREA)
    return np.hstack([l, right])
