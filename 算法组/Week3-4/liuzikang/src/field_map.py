#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场地地图加载 + 多特征匹配（任务 2 的核心算法）。"""
from __future__ import annotations

import math
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

try:
    import yaml
except ImportError:
    yaml = None

TYPE_ALIASES = {
    "PenalityPoint": "PenaltyPoint",
    "PenaltyPoint": "PenaltyPoint",
    "P": "PenaltyPoint",
    "LCross": "LCross",
    "L": "LCross",
    "TCross": "TCross",
    "T": "TCross",
    "XCross": "XCross",
    "X": "XCross",
}


def _canon_type(name: str) -> str:
    return TYPE_ALIASES.get(name, name)


@dataclass
class FieldMap:
    """场地标志地图。"""
    points: List[dict] = field(default_factory=list)
    origin_note: str = ""

    @classmethod
    def load(cls, path: Path | str) -> "FieldMap":
        text = Path(path).read_text(encoding="utf-8")
        if yaml is not None:
            data = yaml.safe_load(text)
        else:
            data = cls._simple_parse(text)
        if not isinstance(data, dict):
            raise ValueError("field.yaml 格式不对，应该是 '名字: [x, y]' 的字典")

        points = []
        for name, xy in data.items():
            if not isinstance(xy, (list, tuple)) or len(xy) < 2:
                continue
            base = "".join(ch for ch in str(name) if not ch.isdigit())
            ptype = _canon_type(base)
            points.append({"name": str(name), "type": ptype,
                           "x": float(xy[0]), "y": float(xy[1])})

        fm = cls(points=points)
        fm.origin_note = (f"{len(points)} 个标志，"
                          f"x∈[{min(p['x'] for p in points):.2f}, {max(p['x'] for p in points):.2f}], "
                          f"y∈[{min(p['y'] for p in points):.2f}, {max(p['y'] for p in points):.2f}]")
        return fm

    def by_type(self, ptype: str) -> List[dict]:
        return [p for p in self.points if p["type"] == ptype]

    @staticmethod
    def _simple_parse(text: str) -> dict:
        """没有 pyyaml 时的兜底解析：只认 `名字: [x, y]` 这种行。"""
        out = {}
        for line in text.splitlines():
            line = line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            name, val = line.split(":", 1)
            nums = re.findall(r"[-+]?\d*\.?\d+", val)
            if len(nums) >= 2:
                out[name.strip()] = [float(nums[0]), float(nums[1])]
        return out


@dataclass
class PoseResult:
    valid: bool
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    marker_count: int = 0
    inliers: int = 0
    residual: float = float("inf")
    status: str = "not_converged"
    matches: List[Tuple[int, int]] = field(default_factory=list)


def _rot(theta: float) -> Tuple[float, float, float, float]:
    c, s = math.cos(theta), math.sin(theta)
    return c, -s, s, c


def _apply(theta: float, tx: float, ty: float, x: float, y: float) -> Tuple[float, float]:
    c, _, s, _ = _rot(theta)
    return c * x - s * y + tx, s * x + c * y + ty


def _pose_from_pair(o1, o2, m1, m2):
    """由两对匹配点求 (x, y, theta)。"""
    ang_o = math.atan2(o2["y"] - o1["y"], o2["x"] - o1["x"])
    ang_m = math.atan2(m2["y"] - m1["y"], m2["x"] - m1["x"])
    theta = ang_m - ang_o
    c, _, s, _ = _rot(theta)
    tx = m1["x"] - (c * o1["x"] - s * o1["y"])
    ty = m1["y"] - (s * o1["x"] + c * o1["y"])
    return tx, ty, theta


def _score(observed: Sequence[dict], fmap: FieldMap, tx, ty, theta,
           tol: float) -> Tuple[int, float, List[Tuple[int, int]]]:
    """把所有观测点变换到地图，统计匹配上的数量和平均残差。"""
    used: set[int] = set()
    inliers, res_sum, matches = 0, 0.0, []
    for i, o in enumerate(observed):
        px, py = _apply(theta, tx, ty, o["x"], o["y"])
        best_d, best_j = float("inf"), -1
        for j, m in enumerate(fmap.points):
            if j in used or m["type"] != o["type"]:
                continue
            d = math.hypot(px - m["x"], py - m["y"])
            if d < best_d:
                best_d, best_j = d, j
        if best_j >= 0 and best_d <= tol:
            used.add(best_j)
            inliers += 1
            res_sum += best_d
            matches.append((i, best_j))
    return inliers, (res_sum / inliers if inliers else float("inf")), matches


def _in_bounds(x: float, y: float, fmap: FieldMap, margin: float) -> bool:
    """相机必须在场地范围（外扩 margin 米）内。"""
    xs = [p["x"] for p in fmap.points]
    ys = [p["y"] for p in fmap.points]
    return (min(xs) - margin <= x <= max(xs) + margin
            and min(ys) - margin <= y <= max(ys) + margin)


def match_pose(observed: Sequence[dict], fmap: FieldMap,
               tol: float = 0.35, min_markers: int = 2,
               ambiguity_margin: float = 0.6,
               near_pose: Optional[Tuple[float, float, float]] = None,
               side_hint: int = 0,
               bounds_margin: float = -1.0) -> PoseResult:
    """主入口：观测标志 → 地图位姿。"""
    obs = [o for o in observed if o.get("type") in
           ("LCross", "TCross", "XCross", "PenaltyPoint")]
    n = len(obs)
    if n < min_markers:
        return PoseResult(valid=False, marker_count=n, status="insufficient_markers")

    candidates = []
    for i in range(n):
        for j in range(n):
            if i == j:
                continue
            for a in range(len(fmap.points)):
                for b in range(len(fmap.points)):
                    if a == b:
                        continue
                    o1, o2 = obs[i], obs[j]
                    m1, m2 = fmap.points[a], fmap.points[b]
                    if o1["type"] != m1["type"] or o2["type"] != m2["type"]:
                        continue
                    do = math.hypot(o2["x"] - o1["x"], o2["y"] - o1["y"])
                    dm = math.hypot(m2["x"] - m1["x"], m2["y"] - m1["y"])
                    if do < 0.3 or abs(do - dm) > tol * 2:
                        continue
                    tx, ty, th = _pose_from_pair(o1, o2, m1, m2)
                    if bounds_margin >= 0 and not _in_bounds(tx, ty, fmap, bounds_margin):
                        continue
                    inl, res, matches = _score(obs, fmap, tx, ty, th, tol)
                    if inl >= 2:
                        candidates.append((inl, res, tx, ty, th, matches))

    if not candidates:
        return PoseResult(valid=False, marker_count=n, status="not_converged")

    candidates.sort(key=lambda c: (-c[0], c[1]))

    best = candidates[0]
    tied = [c for c in candidates
            if c[0] == best[0] and abs(c[1] - best[1]) < 0.15
            and math.hypot(c[2] - best[2], c[3] - best[3]) > ambiguity_margin]

    chosen = best
    if tied:
        if near_pose is not None:
            px, py, _ = near_pose
            chosen = min([best] + tied,
                         key=lambda c: math.hypot(c[2] - px, c[3] - py))
        elif side_hint in (1, -1):
            same_side = [c for c in [best] + tied if c[3] * side_hint > 0]
            if len(same_side) == 1:
                chosen = same_side[0]
            else:
                return PoseResult(valid=False, x=best[2], y=best[3],
                                  yaw_deg=math.degrees(best[4]), marker_count=n,
                                  inliers=best[0], residual=best[1],
                                  status="ambiguous", matches=best[5])
        else:
            return PoseResult(valid=False, x=best[2], y=best[3],
                              yaw_deg=math.degrees(best[4]), marker_count=n,
                              inliers=best[0], residual=best[1],
                              status="ambiguous", matches=best[5])

    inl, res, tx, ty, th, matches = chosen
    yaw = math.degrees(th)
    while yaw > 180:
        yaw -= 360
    while yaw <= -180:
        yaw += 360

    if res > tol:
        return PoseResult(valid=False, x=tx, y=ty, yaw_deg=yaw,
                          marker_count=n, inliers=inl, residual=res,
                          status="high_residual", matches=matches)

    return PoseResult(valid=True, x=tx, y=ty, yaw_deg=yaw,
                      marker_count=n, inliers=inl, residual=res,
                      status="ok", matches=matches)
