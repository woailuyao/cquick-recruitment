# -*- coding: utf-8 -*-
"""
几何核心：相机模型、外参、像素 -> 地面、SE(2) 位姿工具

坐标约定（**整个工程统一，别改**）
--------------------------------------------------
载体坐标系 (carrier / base_link)：
    +X 前进, +Y 向左, +Z 向上；地面为 Z = 0
相机坐标系 (OpenCV)：
    +X 右, +Y 下, +Z 光轴向前
地图坐标系 (field map)：
    +X 向右, +Y 向上；yaw 从 +X 轴逆时针为正（细则 C0 约定）

变换链：
    P_carrier = R_carrier_camera @ P_camera + t_carrier_camera
    P_map     = R2(yaw_map) @ P_carrier_xy + (x_map, y_map)
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

import numpy as np


# ---------------------------------------------------------------- 角度 / 旋转

def wrap_deg(a: float) -> float:
    """把角度归一化到 (-180, 180]。"""
    a = (a + 180.0) % 360.0 - 180.0
    return 180.0 if a == -180.0 else a


def wrap_rad(a: float) -> float:
    a = (a + math.pi) % (2 * math.pi) - math.pi
    return a


def rot2(theta_rad: float) -> np.ndarray:
    c, s = math.cos(theta_rad), math.sin(theta_rad)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def rot_x(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


# ---------------------------------------------------------------- 相机模型

@dataclass
class CameraModel:
    """针孔相机 + 畸变。fx/fy/cx/cy 单位像素。"""
    fx: float
    fy: float
    cx: float
    cy: float
    dist: np.ndarray = field(default_factory=lambda: np.zeros(5, np.float64))
    width: int = 1280
    height: int = 720

    @property
    def K(self) -> np.ndarray:
        return np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0, 1.0]], dtype=np.float64)

    def undistort(self, pts: np.ndarray) -> np.ndarray:
        """
        pts: (N,2) 原始像素 -> 去畸变后的**归一化像平面坐标** (N,2)，即 (x/z, y/z)。
        注意：必须始终返回归一化坐标。畸变系数全零时也不能直接返回原始像素，
        否则后续 [x/z, y/z, 1] 的构造会把像素当方向，地面交点会整体算错。
        """
        import cv2
        pts = np.asarray(pts, dtype=np.float64).reshape(-1, 1, 2)
        # cv2.undistortPoints 自身会做 (u-cx)/fx 的归一化；dist 全零时同样成立
        und = cv2.undistortPoints(pts, self.K, self.dist).reshape(-1, 2)
        return und

    def pixel_to_ray_camera(self, pts: np.ndarray) -> np.ndarray:
        """
        像素 -> 相机坐标系下的射线方向（未归一化为单位向量，Z 分量恒为 1 方向）。
        返回 (N,3)，每个是 [x_n, y_n, 1] 的方向向量。
        """
        und = self.undistort(pts)          # 归一化像平面坐标 (x/z, y/z)
        return np.hstack([und, np.ones((und.shape[0], 1))])

    def project(self, pts_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """相机系 3D 点 -> 像素。返回 (uv, 可见 mask)，只在 Z>0 时有效。"""
        pts_cam = np.asarray(pts_cam, dtype=np.float64).reshape(-1, 3)
        z = pts_cam[:, 2]
        ok = z > 1e-6
        uv = np.full((pts_cam.shape[0], 2), np.nan)
        if ok.any():
            p = pts_cam[ok]
            uv[ok, 0] = self.fx * p[:, 0] / p[:, 2] + self.cx
            uv[ok, 1] = self.fy * p[:, 1] / p[:, 2] + self.cy
        return uv, ok


# ---------------------------------------------------------------- 外参

@dataclass
class Extrinsics:
    """
    P_carrier = R_carrier_camera @ P_camera + t_carrier_camera
    R: (3,3)；t: (3,) 相机光心在载体坐标系中的位置（通常 z = 相机高度）
    """
    R: np.ndarray
    t: np.ndarray

    @property
    def camera_height(self) -> float:
        return float(self.t[2])

    def ray_to_carrier(self, dirs_cam: np.ndarray, origins_cam: Optional[np.ndarray] = None) -> tuple[np.ndarray, np.ndarray]:
        dirs_b = (self.R @ np.asarray(dirs_cam, dtype=np.float64).T).T
        if origins_cam is None:
            orig_b = np.broadcast_to(self.t, dirs_b.shape)
        else:
            origins_cam = np.asarray(origins_cam, dtype=np.float64)
            orig_b = (self.R @ origins_cam.T).T + self.t
        return orig_b, dirs_b

    def ground_intersection(self, origins_b: np.ndarray, dirs_b: np.ndarray,
                            z_ground: float = 0.0,
                            max_range: float = 30.0,
                            require_front=True) -> tuple[np.ndarray, np.ndarray]:
        """
        射线与地面平面 Z = z_ground 求交。
        返回 (pts_xyz (N,3) 含 NaN 表示无效, valid mask (N,))
        无效情形：射线水平/朝上、交点在身后、距离超限 —— 细则要求不能产生 NaN/无穷大假点。
        """
        origins_b = np.atleast_2d(origins_b)
        dirs_b = np.atleast_2d(dirs_b)
        n = dirs_b.shape[0]
        pts = np.full((n, 3), np.nan, dtype=np.float64)
        valid = np.zeros(n, dtype=bool)

        dz = dirs_b[:, 2]
        for i in range(n):
            if abs(dz[i]) < 1e-9:
                continue                      # 射线平行于地面
            lam = (z_ground - origins_b[i, 2]) / dz[i]
            if lam <= 0:
                continue                      # 交点在相机后方
            if require_front and dz[i] >= 0:
                continue                      # 射线朝上，不可能打到地面
            p = origins_b[i] + lam * dirs_b[i]
            if not np.all(np.isfinite(p)):
                continue
            if math.hypot(p[0] - origins_b[i, 0], p[1] - origins_b[i, 1]) > max_range:
                continue
            pts[i] = p
            valid[i] = True
        return pts, valid

    @staticmethod
    def from_mount(height_m: float, pitch_deg: float = 0.0, roll_deg: float = 0.0,
                   yaw_offset_deg: float = 0.0, tx: float = 0.0, ty: float = 0.0) -> "Extrinsics":
        """
        用"安装高度 + 俯仰 + 滚转 + 偏航"构造外参。
        pitch_deg < 0 表示镜头向下压（最常见的机器人头部安装方式）。
        """
        # 相机 -> 载体 的基准旋转（相机水平朝前、无姿态角时）
        #   carrier X(前) = camera Z(光轴)
        #   carrier Y(左) = -camera X(右)
        #   carrier Z(上) = -camera Y(下)
        base = np.array([[0.0, 0.0, 1.0],
                         [-1.0, 0.0, 0.0],
                         [0.0, -1.0, 0.0]], dtype=np.float64)
        # 俯仰绕载体 Y 轴（+Y 向左），正角度把机头上抬
        R = rot_z(yaw_offset_deg) @ rot_y(-pitch_deg) @ rot_x(roll_deg) @ base
        t = np.array([tx, ty, height_m], dtype=np.float64)
        return Extrinsics(R=R, t=t)


# ---------------------------------------------------------------- SE(2)

def transform_points(pts_b: np.ndarray, yaw_rad: float, t_xy: np.ndarray) -> np.ndarray:
    """P_map = R(yaw) @ P_b + t"""
    pts_b = np.asarray(pts_b, dtype=np.float64).reshape(-1, 2)
    return (rot2(yaw_rad) @ pts_b.T).T + np.asarray(t_xy, dtype=np.float64).reshape(1, 2)


def kabsch_2d(P: np.ndarray, Q: np.ndarray) -> tuple[float, np.ndarray, float]:
    """
    最小二乘刚体变换：找 (yaw, t) 使 R(yaw) P_i + t ≈ Q_i。
    P: 观测点（载体系），Q: 地图点。返回 (yaw_rad, t(2,), rms)
    """
    P = np.asarray(P, dtype=np.float64).reshape(-1, 2)
    Q = np.asarray(Q, dtype=np.float64).reshape(-1, 2)
    if P.shape[0] < 2:
        raise ValueError("kabsch_2d 需要至少 2 组对应点")
    pc, qc = P.mean(0), Q.mean(0)
    H = (P - pc).T @ (Q - qc)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(2)
    if np.linalg.det(Vt.T @ U.T) < 0:
        D[1, 1] = -1.0
    R = Vt.T @ D @ U.T
    yaw = math.atan2(R[1, 0], R[0, 0])
    t = qc - R @ pc
    res = (R @ P.T).T + t - Q
    rms = float(np.sqrt(np.mean(np.sum(res ** 2, axis=1))))
    return yaw, t, rms


def pose_delta(pose_a, pose_b) -> tuple[float, float]:
    """两个 (x, y, yaw_deg) 位姿的位置差(m) 与航向差(deg)。"""
    da = math.hypot(pose_a[0] - pose_b[0], pose_a[1] - pose_b[1])
    dyaw = abs(wrap_deg(pose_a[2] - pose_b[2]))
    return da, dyaw
