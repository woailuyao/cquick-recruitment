from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
from itertools import product

import numpy as np

from .common import rot2d, se2_transform
from .field_map import FieldMap


@dataclass
class Observation:
    type: str
    x: float          # 载体坐标系地面 X（米）
    y: float          # 载体坐标系地面 Y（米）
    confidence: float = 1.0


@dataclass
class PoseResult:
    x: float
    y: float
    theta_rad: float
    valid: bool
    marker_count: int
    residual: float
    status: str
    inlier_count: int = 0

    @property
    def yaw_deg(self) -> float:
        return float(np.degrees(self.theta_rad))


def _fit_se2(src: np.ndarray, dst: np.ndarray) -> Optional[np.ndarray]:
    """最小二乘拟合 SE(2)：src(载体) -> dst(地图)，返回 (x, y, theta)。"""
    n = src.shape[0]
    if n < 2:
        return None
    src_c = src - src.mean(axis=0)
    dst_c = dst - dst.mean(axis=0)
    H = src_c.T @ dst_c
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    theta = np.arctan2(R[1, 0], R[0, 0])
    t = dst.mean(axis=0) - R @ src.mean(axis=0)
    return np.array([t[0], t[1], theta])


def _residual(pose: np.ndarray, src: np.ndarray, dst: np.ndarray) -> float:
    """给定位姿，计算 src 变换后与 dst 的均方根误差。"""
    pred = se2_transform(src, pose)
    err = np.linalg.norm(pred - dst, axis=1)
    return float(np.sqrt(np.mean(err ** 2)))


class Localizer:
    """基于场地标志匹配的单目自定位。"""

    def __init__(
        self,
        field_map: FieldMap,
        inlier_dist: float = 0.5,
        min_inliers: int = 3,
        max_residual: float = 1.0,
        approx_x: Optional[float] = None,
        approx_y: Optional[float] = None,
        approx_radius: float = 2.5,
    ):
        self.field = field_map
        self.inlier_dist = inlier_dist
        self.min_inliers = min_inliers
        self.max_residual = max_residual
        self.approx_x = approx_x
        self.approx_y = approx_y
        self.approx_radius = approx_radius

    def localize(self, observations: List[Observation]) -> PoseResult:
        """输入观测标志，输出地图二维位姿估计。"""
        n = len(observations)
        if n < self.min_inliers:
            return self._invalid(observations, "insufficient_markers")

        map_idx: Dict[str, List[int]] = {}
        for i, m in enumerate(self.field.markers):
            map_idx.setdefault(m.type, []).append(i)

        candidates: List[List[int]] = []
        for o in observations:
            cand = map_idx.get(o.type, [])
            if not cand:
                return self._invalid(observations, "ambiguous")
            candidates.append(cand)

        obs_xy = np.array([[o.x, o.y] for o in observations], dtype=float)

        best = None  # (pose, inlier_count, residual)
        for combo in product(*candidates):
            if len(set(combo)) != len(combo):
                continue
            dst = self.field.all_xy()[list(combo)]
            pose = _fit_se2(obs_xy, dst)
            if pose is None:
                continue

            # 先验位置约束：解出的位姿必须落在先验附近
            if self.approx_x is not None and self.approx_y is not None:
                d = float(np.hypot(pose[0] - self.approx_x, pose[1] - self.approx_y))
                if d > self.approx_radius:
                    continue

            res = _residual(pose, obs_xy, dst)
            pred = se2_transform(obs_xy, pose)
            all_map = self.field.all_xy()
            dists = np.linalg.norm(pred[:, None, :] - all_map[None, :, :], axis=2)
            inliers = int(np.sum(dists.min(axis=1) < self.inlier_dist))

            if best is None or (inliers, -res) > (best[1], -best[2]):
                best = (pose, inliers, res)

        if best is None:
            return self._invalid(observations, "ambiguous")

        pose, inliers, res = best
        if inliers < self.min_inliers:
            return self._invalid(observations, "insufficient_markers")
        if res > self.max_residual:
            return self._invalid(observations, "high_residual")

        if not self.field.contains(pose[0], pose[1], margin=2.0):
            return self._invalid(observations, "not_converged")

        return PoseResult(
            x=float(pose[0]),
            y=float(pose[1]),
            theta_rad=float(pose[2]),
            valid=True,
            marker_count=n,
            residual=res,
            status="ok",
            inlier_count=inliers,
        )

    def _invalid(self, observations, status: str) -> PoseResult:
        return PoseResult(
            x=0.0,
            y=0.0,
            theta_rad=0.0,
            valid=False,
            marker_count=len(observations),
            residual=0.0,
            status=status,
        )