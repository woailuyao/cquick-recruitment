"""地图匹配：成对假设 -> 类别约束内点打分 -> 加权 SE(2) 最小二乘 -> 歧义判定。

对应细则 2.8 的硬性要求：
  * 不用"按类别找最近点"，而是由两点对生成 SE(2) 候选假设，再用全场内点打分；
  * 内点匹配带类别约束、一对一占用、距离门限；
  * 显式保留次优假设用于 ambiguous 判定；
  * 失败分流：insufficient_markers / not_converged / high_residual / ambiguous。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field as dc_field
from itertools import combinations
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from .geom import angle_diff_deg, rot2, weighted_se2_ls
from .geom import wrap_deg
from .fieldmap import FieldPoint


@dataclass
class Observation:
    """载体系下的地面标志观测（米）。"""
    type: str
    x: float
    y: float
    sigma: float = 0.05
    confidence: float = 1.0
    weight: Optional[float] = None
    pixel: Optional[Tuple[float, float]] = None
    map_id: Optional[str] = None
    distance: Optional[float] = None

    def __post_init__(self):
        if self.weight is None:
            self.weight = 1.0 / max(self.sigma * self.sigma, 1e-6)


@dataclass
class Hypothesis:
    theta: float
    tx: float
    ty: float
    inliers: List[Tuple[int, FieldPoint, float]] = dc_field(default_factory=list)
    n_inliers: int = 0
    residual: float = float("inf")
    spread: float = 0.0

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.tx, self.ty, self.theta


@dataclass
class MatchResult:
    valid: bool
    status: str
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    residual: float = float("nan")
    marker_count: int = 0
    n_obs: int = 0
    n_hypotheses: int = 0
    inlier_ids: List[str] = dc_field(default_factory=list)
    spread_m: float = 0.0
    rival: Optional[dict] = None

    @property
    def theta(self) -> float:
        return math.radians(self.yaw_deg)

    @property
    def pose(self) -> Tuple[float, float, float]:
        return self.x, self.y, self.theta


# --------------------------------------------------------------- 内部工具

def _score(theta: float, t, obs: Sequence[Observation],
           by_type: Dict[str, List[FieldPoint]], gate: float
           ) -> List[Tuple[int, FieldPoint, float]]:
    """对一个候选位姿做类别约束的一对一最近邻匹配。"""
    t = np.asarray(t, dtype=float).reshape(2)
    R = rot2(theta)          # theta 为 map_from_base：P_map = R @ P_base + t
    cands: List[Tuple[float, int, FieldPoint]] = []
    for oi, o in enumerate(obs):
        q = R @ np.array([o.x, o.y], float) + t      # 观测点投到地图系
        for p in by_type.get(o.type, ()):
            d = math.hypot(q[0] - p.x, q[1] - p.y)
            if d <= gate:
                cands.append((d, oi, p))
    cands.sort(key=lambda c: c[0])

    used_o, used_m = set(), set()
    inl: List[Tuple[int, FieldPoint, float]] = []
    for d, oi, p in cands:
        if oi in used_o or p.mid in used_m:
            continue
        used_o.add(oi)
        used_m.add(p.mid)
        inl.append((oi, p, d))
    return inl


def _weighted_residual(obs: Sequence[Observation],
                       inl: Sequence[Tuple[int, FieldPoint, float]]) -> float:
    if not inl:
        return float("inf")
    w = np.array([float(obs[oi].weight) for oi, _, _ in inl])
    d = np.array([dd for _, _, dd in inl])
    return float(math.sqrt(float((w * d * d).sum() / max(w.sum(), 1e-9))))


def _spread(pts: Sequence[FieldPoint]) -> float:
    best = 0.0
    for a, b in combinations(pts, 2):
        best = max(best, math.hypot(a.x - b.x, a.y - b.y))
    return best


def _generate(obs: Sequence[Observation], points: Sequence[FieldPoint],
              mcfg: dict) -> List[Tuple[float, np.ndarray]]:
    """两点对生成 SE(2) 候选假设：同一对观测长与地图点对长一致 -> 唯一位姿。"""
    tol_ratio = float(mcfg.get("pair_len_tol_ratio", 0.10))
    tol_abs = float(mcfg.get("pair_len_tol_abs_m", 0.05))
    rmin = float(mcfg.get("min_obs_range_m", 0.4))
    rmax = float(mcfg.get("max_obs_range_m", 8.0))
    max_h = int(mcfg.get("max_hypotheses", 4000))

    map_pairs = []
    for a, b in combinations(points, 2):
        r = math.hypot(b.x - a.x, b.y - a.y)
        if rmin <= r <= rmax:
            map_pairs.append((a, b, r, math.atan2(b.y - a.y, b.x - a.x)))

    hyps: List[Tuple[float, np.ndarray]] = []
    seen: set = set()
    for i, j in combinations(range(len(obs)), 2):
        oi, oj = obs[i], obs[j]
        dx, dy = oj.x - oi.x, oj.y - oi.y
        r_obs = math.hypot(dx, dy)
        if not (rmin <= r_obs <= rmax):
            continue
        ang_obs = math.atan2(dy, dx)
        tol = max(tol_abs, tol_ratio * r_obs)
        for a, b, r_map, ang_map in map_pairs:
            if a.type != oi.type or b.type != oj.type:
                continue
            if abs(r_map - r_obs) > tol:
                continue
            theta = ang_map - ang_obs
            c, s = math.cos(theta), math.sin(theta)
            tx = a.x - (c * oi.x - s * oi.y)
            ty = a.y - (s * oi.x + c * oi.y)
            key = (int(round(theta / math.radians(1.0))),
                   int(round(tx / 0.02)), int(round(ty / 0.02)))
            if key in seen:
                continue
            seen.add(key)
            hyps.append((theta, np.array([tx, ty], float)))
            if len(hyps) >= max_h:
                return hyps
    return hyps


def _refit(obs: Sequence[Observation], hyp: Hypothesis,
           by_type: Dict[str, List[FieldPoint]], gate: float) -> Optional[Hypothesis]:
    """用内点做加权刚体最小二乘，并重新匹配一次。"""
    if len(hyp.inliers) < 2:
        return None
    src = np.array([[obs[oi].x, obs[oi].y] for oi, _, _ in hyp.inliers])
    dst = np.array([[p.x, p.y] for _, p, _ in hyp.inliers])
    w = np.array([float(obs[oi].weight) for oi, _, _ in hyp.inliers])
    tx, ty, theta = weighted_se2_ls(src, dst, w)
    inl = _score(theta, (tx, ty), obs, by_type, gate)
    if len(inl) < 2:
        return None
    h = Hypothesis(theta=theta, tx=tx, ty=ty, inliers=inl, n_inliers=len(inl))
    h.residual = _weighted_residual(obs, inl)
    h.spread = _spread([p for _, p, _ in inl])
    return h


def _evaluate(obs, by_type, pool, gate) -> Optional[List[Hypothesis]]:
    scored: List[Hypothesis] = []
    for theta, t in pool:
        inl = _score(theta, t, obs, by_type, gate)
        if not inl:
            continue
        h = Hypothesis(theta=float(theta), tx=float(t[0]), ty=float(t[1]),
                       inliers=inl, n_inliers=len(inl))
        h.residual = _weighted_residual(obs, inl)
        h.spread = _spread([p for _, p, _ in inl])
        scored.append(h)
    if not scored:
        return None

    scored.sort(key=lambda h: (-h.n_inliers, h.residual))
    refined = _refit(obs, scored[0], by_type, gate)
    if refined is not None:
        scored.append(refined)
        scored.sort(key=lambda h: (-h.n_inliers, h.residual))
    return scored


# ------------------------------------------------------------------ 主入口

def match(obs: Sequence[Observation], points: Sequence[FieldPoint], cfg: dict,
          prior: Optional[Tuple[float, float, float]] = None) -> MatchResult:
    """求解载体在地图中的位姿 (x, y, theta)。

    prior: 上一帧位姿 (x, y, theta_rad)，用于缩小搜索；None 表示全图搜索。
    """
    mcfg = cfg.get("matching", cfg)
    gate = float(mcfg.get("gate_m", 0.35))
    min_inl = int(mcfg.get("min_inliers", 2))
    res_max = float(mcfg.get("residual_max_m", 0.35))
    min_spread = float(mcfg.get("min_inlier_spread_m", 0.8))
    amb_d = float(mcfg.get("ambiguity_pose_dist_m", 0.5))
    amb_yaw = float(mcfg.get("ambiguity_yaw_deg", 10.0))
    amb_ratio = float(mcfg.get("ambiguity_residual_ratio", 1.3))
    track_r = float(mcfg.get("tracking_radius_m", 1.0))
    track_yaw = float(mcfg.get("tracking_yaw_deg", 30.0))

    by_type: Dict[str, List[FieldPoint]] = {}
    for p in points:
        by_type.setdefault(p.type, []).append(p)

    obs = list(obs)
    out = MatchResult(valid=False, status="insufficient_markers", n_obs=len(obs))
    if len(obs) < min_inl:
        return out
    if len(points) < min_inl:
        out.status = "map_too_small"
        return out

    hyps = _generate(obs, points, mcfg)
    out.n_hypotheses = len(hyps)
    if not hyps:
        out.status = "not_converged"
        return out

    scored: Optional[List[Hypothesis]] = None
    if prior is not None:
        px, py, pth = prior
        near = [h for h in hyps
                if math.hypot(h[1][0] - px, h[1][1] - py) <= track_r
                and abs(angle_diff_deg(math.degrees(h[0]), math.degrees(pth))) <= track_yaw]
        if near:
            scored = _evaluate(obs, by_type, near, gate)
        if not scored or scored[0].n_inliers < min_inl:
            scored = _evaluate(obs, by_type, hyps, gate)     # 回退全图搜索
    else:
        scored = _evaluate(obs, by_type, hyps, gate)

    if not scored:
        out.status = "not_converged"
        return out

    best = scored[0]
    out.x, out.y = best.tx, best.ty
    out.yaw_deg = wrap_deg(math.degrees(best.theta))
    out.residual = best.residual
    out.marker_count = best.n_inliers
    out.spread_m = best.spread
    out.inlier_ids = [p.mid for _, p, _ in best.inliers]
    for oi, p, _ in best.inliers:
        obs[oi].map_id = p.mid

    # ---- 有效性判定 ----
    if best.n_inliers < min_inl:
        out.status = "not_converged"
        return out
    if best.spread < min_spread:
        out.status = "insufficient_markers"     # 标志挤在一起，几何退化
        return out
    if best.residual > res_max:
        out.status = "high_residual"
        return out

    # ---- 歧义判定：找与最优解明显不同、支持度却相当的次优假设 ----
    # 注意：必须扫描完所有候选假设，不能遇到第一个"不够强"的就停，
    # 否则后面更强的对手会被漏掉（曾经因此让 1/3 的错误位姿被当成有效）。
    for h in scored[1:]:
        sep = math.hypot(h.tx - best.tx, h.ty - best.ty)
        dyaw = abs(angle_diff_deg(math.degrees(h.theta), math.degrees(best.theta)))
        if sep < amb_d and dyaw < amb_yaw:
            continue                             # 同一个解的近似重复
        if h.n_inliers >= best.n_inliers and h.residual <= best.residual * amb_ratio + 1e-9:
            out.rival = {"x": h.tx, "y": h.ty,
                         "yaw_deg": wrap_deg(math.degrees(h.theta)),
                         "inliers": h.n_inliers, "residual": h.residual}
            out.status = "ambiguous"
            return out

    out.valid = True
    out.status = "ok"
    return out
