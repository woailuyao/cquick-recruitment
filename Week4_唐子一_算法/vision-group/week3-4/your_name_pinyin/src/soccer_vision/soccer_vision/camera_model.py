"""相机模型：内参/外参加载、去畸变、像素 -> 地面平面(Z=0)求交。

核心几何：
  1. 像素点 (u,v) 先去畸变；
  2. 反投影得到相机坐标系射线方向 d_c = K^-1 [u,v,1]^T；
  3. 用外参把射线原点和方向转到载体坐标系；
  4. 与地面平面 Z=0 求交：P_b = O_b + lambda * d_b, lambda = -O_bz / d_bz。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np

from .common import load_yaml


@dataclass
class Intrinsics:
    camera_matrix: np.ndarray  # (3,3)
    dist_coeffs: np.ndarray    # (1,5) [k1,k2,p1,p2,k3]
    width: int
    height: int
    reproj_error: Optional[float] = None

    @classmethod
    def from_yaml(cls, path: str) -> "Intrinsics":
        d = load_yaml(path)
        K = np.array(d["camera_matrix"], dtype=float)
        dist = np.array(d["dist_coeffs"], dtype=float).reshape(1, -1)
        return cls(
            camera_matrix=K,
            dist_coeffs=dist,
            width=int(d["width"]),
            height=int(d["height"]),
            reproj_error=d.get("reproj_error"),
        )

    @property
    def fx(self) -> float:
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        return float(self.camera_matrix[1, 2])


@dataclass
class Extrinsics:
    """相机相对载体参考坐标系（base_link / rig_frame）的外参。

    - R: 相机坐标系 -> 载体坐标系的旋转矩阵 (3,3)
    - t: 相机光心在载体坐标系中的位置 (3,) （米）
    - 同时提供 height / pitch / roll 便于说明和调试（非独立标定量）。
    """

    R: np.ndarray
    t: np.ndarray
    height: float
    pitch_deg: float
    roll_deg: float

    @classmethod
    def from_yaml(cls, path: str) -> "Extrinsics":
        d = load_yaml(path)
        R = np.array(d["R"], dtype=float)
        t = np.array(d["t"], dtype=float).reshape(-1)
        return cls(
            R=R,
            t=t,
            height=float(d["height"]),
            pitch_deg=float(d.get("pitch_deg", 0.0)),
            roll_deg=float(d.get("roll_deg", 0.0)),
        )


class CameraGeometry:
    """把像素坐标投影到载体坐标系的 Z=0 地面平面。"""

    def __init__(self, intrinsics: Intrinsics, extrinsics: Extrinsics):
        self.intr = intrinsics
        self.extr = extrinsics
        self.K = intrinsics.camera_matrix
        self.K_inv = np.linalg.inv(self.K)

    def undistort_point(self, u: float, v: float) -> Tuple[float, float]:
        pts = np.array([[u, v]], dtype=np.float32)
        uu = cv2_undistort_points(pts, self.K, self.intr.dist_coeffs)
        # cv2.undistortPoints 返回形状因版本而异（(1,1,2)/(1,2)等），
        # 统一 reshape 成 (N,2) 再取第一个点，避免标量/数组混用报错。
        uu = np.asarray(uu, dtype=np.float64).reshape(-1, 2)
        return float(uu[0, 0]), float(uu[0, 1])

    def ray_in_camera(self, u: float, v: float) -> np.ndarray:
        """相机坐标系下的射线方向（未归一化亦可，这里归一化）。"""
        uu, vv = self.undistort_point(u, v)
        d = self.K_inv @ np.array([uu, vv, 1.0])
        return d / np.linalg.norm(d)

    def project_to_ground(self, u: float, v: float) -> Optional[np.ndarray]:
        """像素点 -> 载体坐标系 Z=0 地面点（米）。

        返回 None 表示无效：射线朝上（不交地面）、交点过远或数值非法。
        """
        d_c = self.ray_in_camera(u, v)
        # 相机坐标 -> 载体坐标
        R = self.extr.R
        d_b = R @ d_c
        O_b = self.extr.t  # 相机光心在载体坐标系中的位置

        # 地面平面 Z=0 求交
        if abs(d_b[2]) < 1e-6:
            return None  # 射线几乎平行于地面
        lam = -O_b[2] / d_b[2]
        if lam <= 0:
            return None  # 交点在相机后方（射线朝地平线上方）
        P = O_b + lam * d_b

        if not np.all(np.isfinite(P)):
            return None
        # 交点横向/纵向超出合理范围（比如 30 米）视为无效，避免伪造地面点
        if np.linalg.norm(P[:2]) > 30.0:
            return None
        return P


def cv2_undistort_points(pts: np.ndarray, K: np.ndarray, dist: np.ndarray) -> np.ndarray:
    return cv2.undistortPoints(pts, K, dist,None,None,K)
