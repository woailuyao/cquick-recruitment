#!/usr/bin/env python3
"""场地模型加载（视觉定位包）。

职责
    读 config/field_geometry.yaml，产出：
        · dims    : 场地尺寸字典（长/宽、禁区、中圈、点球点距离）
        · markers : [(type, x, y, id), ...]，P 点单独放在 penalty_point
        · 便捷查询：by_type / id_of / half_filter

坐标系（与 field_geometry.yaml 一致）
    原点场地中心，+X 长边，+Y 短边，Z 向上，单位 m
"""

import math
import os

import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))          # xjx/
DEFAULT_PATH = os.path.join(ROOT, "config", "field_geometry.yaml")


class FieldMap:
    def __init__(self, path=None):
        p = path or DEFAULT_PATH
        with open(p, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f) or {}
        self.path = p
        self.meta = d.get("meta", {}) or {}
        self.dims = d.get("field", {}) or {}
        raw = d.get("markers", []) or []

        self.markers = []          # [(type, x, y, id)]
        self.penalty_point = None  # (x, y, id) 或 None
        for m in raw:
            t = m.get("type")
            x, y = float(m.get("x", 0.0)), float(m.get("y", 0.0))
            mid = m.get("id") or "%s%02d" % (t, len(self.markers) + 1)
            if t == "P":
                self.penalty_point = (x, y, mid)
            else:
                self.markers.append((t, x, y, mid))

    # ---------- 查询 ----------
    @property
    def length(self):
        return float(self.dims.get("length", 0.0))

    @property
    def width(self):
        return float(self.dims.get("width", 0.0))

    def by_type(self, types=("L", "T")):
        out = {}
        for t, x, y, mid in self.markers:
            if t in types:
                out.setdefault(t, []).append((x, y, mid))
        return out

    def id_of(self, x, y, tol=0.01):
        for t, mx, my, mid in self.markers:
            if abs(mx - x) < tol and abs(my - y) < tol:
                return mid
        if self.penalty_point and math.hypot(self.penalty_point[0] - x,
                                             self.penalty_point[1] - y) < tol:
            return self.penalty_point[2]
        return ""

    def half_filter(self, half, types=("L", "T")):
        """half: "" / "x<0" / "x>0" / "y<0" / "y>0"，只保留该半场的标志。"""
        marks = [(t, x, y, mid) for t, x, y, mid in self.markers if t in types]
        if half not in ("x<0", "x>0", "y<0", "y>0"):
            return marks
        idx = 1 if half[0] == "x" else 2
        want_neg = half[1] == "<"
        return [m for m in marks
                if (m[idx] < 0 if want_neg else m[idx] > 0)]

    def summary(self):
        cnt = {}
        for t, _, _, _ in self.markers:
            cnt[t] = cnt.get(t, 0) + 1
        return "场 %.1f×%.1f m | 标志 %d 个 %s | 点球点 %s | 版本 %s" % (
            self.length, self.width, len(self.markers),
            "/".join("%s×%d" % (k, v) for k, v in sorted(cnt.items())),
            "有" if self.penalty_point else "无",
            self.meta.get("version", "?"))
