# -*- coding: utf-8 -*-
"""配置读取 + 右侧地图绘制（左右分屏的右边）。"""
import math
import os

import cv2
import numpy as np

try:
    import yaml
except ImportError:  # 保证提示清晰
    yaml = None

ALIASES = {"PenalityPoint": "PenaltyPoint", "penaltypoint": "PenaltyPoint"}


def _load_yaml(path):
    if yaml is None:
        raise SystemExit("缺少 PyYAML：pip install pyyaml")
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_camera(path):
    d = _load_yaml(path)
    K = np.array([
        [d["camera_matrix"]["fx"], 0.0, d["camera_matrix"]["cx"]],
        [0.0, d["camera_matrix"]["fy"], d["camera_matrix"]["cy"]],
        [0.0, 0.0, 1.0],
    ], dtype=float)
    dist = np.array(d.get("dist_coeffs", [0, 0, 0, 0, 0]), dtype=float).reshape(-1, 1)
    m = d.get("mount", {})
    return {
        "K": K, "dist": dist,
        "width": int(d.get("image_width", 1280)),
        "height": int(d.get("image_height", 720)),
        "height_m": float(m.get("height_m", 0.98)),
        "pitch_deg": float(m.get("pitch_deg", 30.0)),
        "roll_deg": float(m.get("roll_deg", 0.0)),
        "yaw_deg": float(m.get("yaw_deg", 0.0)),
        "proj": d.get("projection", {}),
        "loc": d.get("localization", {}),
        "field_frame": str(d.get("field_frame", "identity")),
        "models": d.get("models", {}),
    }


def _norm_type(t):
    return ALIASES.get(t, t)


def load_field(path, frame=None):
    """读取 field.yaml 并返回 (markers, bounds)。

    frame 用于「按组长 field_geometry.yaml 的定义」把 yaml 里的坐标换算成官方约定
    （官方：+X 指向球门，+Y 指向面向球门时的左侧）。yaml 文件本身不做任何修改。

    frame 取值：
        None 或 "identity"   : 直接使用 yaml 坐标
        "y,-x" / "xy" 等      : 例如 "y,-x" 表示
                                官方 X = yaml y，官方 Y = −yaml x
    """
    frame = (frame or "identity").strip().lower()
    """兼容三种常见写法，返回 ({type: [(x,y), ...]}, bounds)。"""
    d = _load_yaml(path)
    raw = d.get("markers") or d.get("field_markers") or d
    markers = {}
    for k, v in raw.items():
        if k in ("bounds",):
            continue
        t = _norm_type(k)
        pts = []
        if isinstance(v, dict):
            if "x" in v and "y" in v:
                pts.append((float(v["x"]), float(v["y"])))
            else:
                for kk, vv in v.items():
                    if isinstance(vv, dict) and "x" in vv and "y" in vv:
                        pts.append((float(vv["x"]), float(vv["y"])))
        elif isinstance(v, list):
            for item in v:
                if isinstance(item, dict):
                    pts.append((float(item["x"]), float(item["y"])))
                elif isinstance(item, (list, tuple)) and len(item) >= 2:
                    pts.append((float(item[0]), float(item[1])))
        if pts:
            markers[t] = pts
    b = d.get("bounds", {})
    bounds = (float(b.get("x_min", -1)), float(b.get("x_max", 10)),
              float(b.get("y_min", -1)), float(b.get("y_max", 10)))

    # 按组长定义把 yaml 坐标换算到官方约定（yaml 文件本身不改）
    if frame not in ("", "identity", "none", "xy"):
        parts = [p.strip() for p in frame.split(",")]
        if len(parts) >= 2:
            def conv(x, y):
                out = []
                for p in parts[:2]:
                    if p in ("x", "+x"):
                        out.append(x)
                    elif p == "-x":
                        out.append(-x)
                    elif p in ("y", "+y"):
                        out.append(y)
                    elif p == "-y":
                        out.append(-y)
                    else:
                        out.append(0.0)
                return out[0], out[1]
            markers = {t: [conv(x, y) for (x, y) in pts] for t, pts in markers.items()}
            cx = [conv(x, y)[0] for x in (bounds[0], bounds[1]) for y in (bounds[2], bounds[3])]
            cy = [conv(x, y)[1] for x in (bounds[0], bounds[1]) for y in (bounds[2], bounds[3])]
            bounds = (min(cx), max(cx), min(cy), max(cy))
    return markers, bounds


def flat_markers(markers):
    """{type: [(x,y)]} -> [{'type','x','y'}]"""
    out = []
    for t, pts in markers.items():
        for x, y in pts:
            out.append({"type": t, "x": x, "y": y})
    return out


class MapView:
    def __init__(self, bounds, size=(720, 720), margin=50, rot=0):
        self.xmin, self.xmax, self.ymin, self.ymax = bounds
        self.w, self.h = size
        self.margin = margin
        self.rot = rot % 360
        self.cx = 0.5 * (self.xmin + self.xmax)
        self.cy = 0.5 * (self.ymin + self.ymax)
        self._recompute()

    def _recompute(self):
        a = math.radians(self.rot)
        ca, sa = math.cos(a), math.sin(a)
        # 旋转后的四个角，用来确定绘图范围
        pts = []
        for x in (self.xmin, self.xmax):
            for y in (self.ymin, self.ymax):
                dx, dy = x - self.cx, y - self.cy
                pts.append((dx * ca - dy * sa, dx * sa + dy * ca))
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        self.rxmin, self.rxmax = min(xs), max(xs)
        self.rymin, self.rymax = min(ys), max(ys)
        spanx = max(1e-6, self.rxmax - self.rxmin)
        spany = max(1e-6, self.rymax - self.rymin)
        self.s = min((self.w - 2 * self.margin) / spanx,
                     (self.h - 2 * self.margin) / spany)
        self.ox = self.margin + ((self.w - 2 * self.margin) - spanx * self.s) / 2
        self.oy = self.h - self.margin - ((self.h - 2 * self.margin) - spany * self.s) / 2

    def to_px(self, x, y):
        a = math.radians(self.rot)
        ca, sa = math.cos(a), math.sin(a)
        dx, dy = x - self.cx, y - self.cy
        rx = dx * ca - dy * sa
        ry = dx * sa + dy * ca
        px = self.ox + (rx - self.rxmin) * self.s
        py = self.oy - (ry - self.rymin) * self.s     # 屏幕 y 向下，所以取负
        return int(round(px)), int(round(py))

    def render(self, markers, pose=None, title="field map"):
        img = np.full((self.h, self.w, 3), 245, np.uint8)
        cv2.rectangle(img, self.to_px(self.xmin, self.ymin),
                      self.to_px(self.xmax, self.ymax), (160, 160, 160), 2)
        for gx in np.arange(math.ceil(self.xmin), self.xmax + 1e-6, 1.0):
            cv2.line(img, self.to_px(gx, self.ymin), self.to_px(gx, self.ymax), (228, 228, 228), 1)
        for gy in np.arange(math.ceil(self.ymin), self.ymax + 1e-6, 1.0):
            cv2.line(img, self.to_px(self.xmin, gy), self.to_px(self.xmax, gy), (228, 228, 228), 1)

        o = self.to_px(0, 0)
        cv2.arrowedLine(img, o, self.to_px(1.2, 0), (0, 0, 255), 2, tipLength=0.3)
        cv2.arrowedLine(img, o, self.to_px(0, 1.2), (0, 160, 0), 2, tipLength=0.3)
        cv2.putText(img, "+X", self.to_px(1.35, -0.2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.putText(img, "+Y", self.to_px(-0.25, 1.35), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 160, 0), 2)

        colors = {"LCross": (255, 0, 0), "TCross": (0, 128, 255),
                  "XCross": (0, 0, 0), "PenaltyPoint": (128, 0, 128)}
        for t, pts in markers.items():
            c = colors.get(t, (0, 0, 0))
            for (x, y) in pts:
                p = self.to_px(x, y)
                cv2.circle(img, p, 6, c, -1)
                cv2.putText(img, t[0], (p[0] + 8, p[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)

        if pose is not None:
            p = self.to_px(pose["x"], pose["y"])
            cv2.circle(img, p, 9, (0, 0, 200), -1)
            th = math.radians(pose["yaw_deg"] + self.rot)
            tip = (int(p[0] + 55 * math.cos(th)), int(p[1] - 55 * math.sin(th)))
            cv2.arrowedLine(img, p, tip, (0, 0, 200), 3, tipLength=0.3)
            cv2.putText(img, f"({pose['x']:.2f}, {pose['y']:.2f}) yaw={pose['yaw_deg']:.1f}",
                        (int(p[0] + 12), int(p[1] + 22)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 0, 200), 2)

        cv2.putText(img, f"{title}  rot={self.rot}", (12, 26),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (120, 120, 120), 1)
        return img
