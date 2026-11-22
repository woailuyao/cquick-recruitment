# -*- coding: utf-8 -*-
"""
地图匹配与 SE(2) 位姿求解

输入：观测到的场地标志（载体坐标系下的地面米制坐标）
      field.yaml 中的已知特征（地图坐标系）
输出：载体在地图中的 (x, y, yaw) + 匹配内点 + 残差 + 状态

为什么不能"按类别找最近邻"（细则 2.8 明确禁止）：
    场地里同类 L/T/X 通常有多个，且球场本身接近对称。
    最近邻会把 A 点的 X 误配到 B 点的 X，位姿会整体偏移但残差很小，
    表现为"看起来一直很准，其实错了半个球场"。

本实现的思路：
    1. 用「同类观测点对 + 同类地图点对」枚举刚体假设（每对给出唯一 theta/t）；
    2. 对每个假设用最近同类型门限打分，统计内点与 RMS；
    3. 取最优假设，再用 Kabsch 对全部内点做最小二乘精化；
    4. 检查是否存在"同样好但位姿差很远"的第二个解 -> ambiguous；
    5. 内点不足 / 残差过大 -> 明确置 valid=false，不伪造位姿。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

from .geometry import kabsch_2d, pose_delta, rot2, transform_points, wrap_deg


# ---------------------------------------------------------------- 数据结构

@dataclass
class MarkerObs:
    """一个观测到的场地标志（载体坐标系，地面 Z=0）。"""
    uid: int
    type: str
    x: float
    y: float
    conf: float = 1.0
    refined: bool = True


@dataclass
class MapFeature:
    """field.yaml 里的一个已知特征（地图坐标系）。"""
    fid: int
    type: str
    x: float
    y: float
    # 官方 field.yaml 里的原始键名（TCross1 / LCross3 / PenalityPoint），
    # 用于现场卷尺核对时按名字引用；没有就留空。
    name: str = ""


@dataclass
class MatchResult:
    valid: bool
    status: str
    x: float = float('nan')
    y: float = float('nan')
    yaw: float = float('nan')          # deg，地图 +X 轴逆时针为正
    marker_count: int = 0              # 参与匹配的观测数
    inlier_count: int = 0
    residual: float = float('nan')
    matches: list = field(default_factory=list)   # [(obs_uid, map_fid, dist)]
    ambiguous_with: Optional[tuple] = None        # 第二个解的位姿
    hypotheses: int = 0

    def as_dict(self) -> dict:
        return {
            "valid": bool(self.valid),
            "status": self.status,
            "x_carrier_map": round(float(self.x), 3) if self.valid else None,
            "y_carrier_map": round(float(self.y), 3) if self.valid else None,
            "yaw_carrier_map": round(float(self.yaw), 2) if self.valid else None,
            "marker_count": int(self.marker_count),
            "inlier_count": int(self.inlier_count),
            "residual": round(float(self.residual), 3) if self.valid else None,
        }


# ---------------------------------------------------------------- 匹配器

class FieldMatcher:
    def __init__(self, features: list[MapFeature], gate: float = 0.6,
                 max_residual: float = 0.35, min_inliers: int = 2,
                 ambiguity_pos_m: float = 0.75, ambiguity_yaw_deg: float = 12.0,
                 max_hypotheses: int = 20000):
        self.features = list(features)
        self.gate = gate
        self.max_residual = max_residual
        self.min_inliers = min_inliers
        self.ambiguity_pos_m = ambiguity_pos_m
        self.ambiguity_yaw_deg = ambiguity_yaw_deg
        self.max_hypotheses = max_hypotheses
        self._by_type: dict[str, list[MapFeature]] = {}
        for f in self.features:
            self._by_type.setdefault(f.type, []).append(f)

    # ---------------------------------------------------------- 内部
    def _score(self, yaw_rad: float, t: np.ndarray, obs: list[MarkerObs]):
        """用最近同类型门限给一个位姿打分。"""
        R = rot2(yaw_rad)
        inliers = []
        used_fids = set()
        for o in obs:
            cands = self._by_type.get(o.type)
            if not cands:
                continue
            p = R @ np.array([o.x, o.y]) + t
            best = None
            best_d = float('inf')
            for f in cands:
                d = math.hypot(p[0] - f.x, p[1] - f.y)
                if d < best_d:
                    best_d, best = d, f
            if best is not None and best_d <= self.gate:
                inliers.append((o, best, best_d))
                used_fids.add(best.fid)
        if len(inliers) < self.min_inliers:
            return None
        rms = float(np.sqrt(np.mean([d ** 2 for _, _, d in inliers])))
        # 同一地图特征被多个观测重复占用 -> 明显是不可信的匹配
        if len(used_fids) < len(inliers):
            rms *= 1.5
        return {"inliers": inliers, "rms": rms, "n": len(inliers), "yaw": yaw_rad, "t": t}

    def _hypotheses(self, obs: list[MarkerObs]):
        """枚举刚体假设。"""
        hyps = []
        n = len(obs)
        for i in range(n):
            for j in range(i + 1, n):
                oi, oj = obs[i], obs[j]
                if oi.type != oj.type:
                    continue
                vec_o = np.array([oj.x - oi.x, oj.y - oi.y])
                lo = float(np.hypot(*vec_o))
                if lo < 0.30:                     # 观测点对太近，角度不可靠
                    continue
                ang_o = math.atan2(vec_o[1], vec_o[0])
                cands = self._by_type.get(oi.type, [])
                for a in range(len(cands)):
                    for b in range(len(cands)):
                        if a == b:
                            continue
                        fa, fb = cands[a], cands[b]
                        vec_m = np.array([fb.x - fa.x, fb.y - fa.y])
                        lm = float(np.hypot(*vec_m))
                        if lm < 0.30:
                            continue
                        # 尺度必须一致（地图与实际场地一致的强约束）
                        if abs(lm - lo) > max(0.5, 0.25 * lm):
                            continue
                        ang_m = math.atan2(vec_m[1], vec_m[0])
                        yaw = ang_m - ang_o
                        R = rot2(yaw)
                        t = np.array([fa.x, fa.y]) - R @ np.array([oi.x, oi.y])
                        hyps.append((yaw, t))
                        if len(hyps) >= self.max_hypotheses:
                            return hyps
        return hyps

    # ---------------------------------------------------------- 对外
    def solve(self, obs: list[MarkerObs]) -> MatchResult:
        n = len(obs)
        if n < self.min_inliers:
            return MatchResult(valid=False, status="insufficient_markers",
                               marker_count=n, inlier_count=0)

        # 同类观测少于 2 个时无法定朝向
        type_counts = {}
        for o in obs:
            type_counts[o.type] = type_counts.get(o.type, 0) + 1
        if not any(c >= 2 for c in type_counts.values()):
            return MatchResult(valid=False, status="insufficient_markers",
                               marker_count=n, inlier_count=0)

        hyps = self._hypotheses(obs)
        if not hyps:
            return MatchResult(valid=False, status="not_converged",
                               marker_count=n, inlier_count=0)

        scored = []
        for yaw, t in hyps:
            s = self._score(yaw, t, obs)
            if s is not None:
                scored.append(s)
        if not scored:
            return MatchResult(valid=False, status="high_residual",
                               marker_count=n, inlier_count=0)

        # 精化：内点 -> Kabsch 最小二乘
        refined = []
        for s in scored:
            inl = s["inliers"]
            if len(inl) >= 2:
                P = np.array([[o.x, o.y] for o, _, _ in inl])
                Q = np.array([[f.x, f.y] for _, f, _ in inl])
                try:
                    yaw2, t2, rms2 = kabsch_2d(P, Q)
                except Exception:
                    yaw2, t2, rms2 = s["yaw"], s["t"], s["rms"]
            else:
                yaw2, t2, rms2 = s["yaw"], s["t"], s["rms"]
            s2 = self._score(yaw2, t2, obs)
            if s2 is not None:
                refined.append(s2)
            else:
                refined.append({"inliers": inl, "rms": rms2, "n": len(inl),
                                "yaw": yaw2, "t": t2})

        # 排序：内点多优先，其次残差小
        refined.sort(key=lambda s: (-s["n"], s["rms"]))
        best = refined[0]

        x, y = float(best["t"][0]), float(best["t"][1])
        yaw_deg = wrap_deg(math.degrees(best["yaw"]))
        best_pose = (x, y, yaw_deg)

        # 歧义检查：找"内点同样多、残差接近、但位姿差很远"的第二个解
        amb = None
        for s in refined[1:]:
            if s["n"] < best["n"]:
                continue
            if s["rms"] > max(best["rms"] * 2.0 + 0.05, best["rms"] + 0.15):
                continue
            pose = (float(s["t"][0]), float(s["t"][1]), wrap_deg(math.degrees(s["yaw"])))
            dpos, dyaw = pose_delta(best_pose, pose)
            if dpos > self.ambiguity_pos_m or dyaw > self.ambiguity_yaw_deg:
                amb = pose
                break

        matches = [(o.uid, f.fid, round(d, 3)) for o, f, d in best["inliers"]]

        if best["n"] < self.min_inliers:
            return MatchResult(valid=False, status="insufficient_markers",
                               marker_count=n, inlier_count=best["n"],
                               residual=best["rms"], matches=matches,
                               hypotheses=len(hyps))

        if best["rms"] > self.max_residual:
            return MatchResult(valid=False, status="high_residual",
                               marker_count=n, inlier_count=best["n"],
                               residual=best["rms"], matches=matches,
                               hypotheses=len(hyps))

        if amb is not None:
            return MatchResult(valid=False, status="ambiguous",
                               marker_count=n, inlier_count=best["n"],
                               residual=best["rms"], matches=matches,
                               ambiguous_with=amb, hypotheses=len(hyps))

        return MatchResult(
            valid=True, status="ok",
            x=x, y=y, yaw=yaw_deg,
            marker_count=n, inlier_count=best["n"],
            residual=best["rms"], matches=matches,
            hypotheses=len(hyps),
        )
