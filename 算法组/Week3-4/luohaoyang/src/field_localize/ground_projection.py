"""像素 -> 去畸变 -> 相机射线 -> 载体系 -> 与地面 Z=0 求交。

坐标约定
--------
相机系（OpenCV）: x 右, y 下, z 前
载体/底板系     : x 前, y 左, z 上, 地面为 z=0

外参两种写法（推荐第一种，由 calib_extrinsics.py 用 solvePnP 生成）:

    R_base_camera: [9 个数，行优先]            # base <- camera 旋转
    camera_origin_base_m: [x, y, z]            # 相机光心在 base 下的位置

    # 或者（手工粗标定时用）
    camera_height_m / pitch_deg / roll_deg / yaw_deg
    translation_x_m / translation_y_m / translation_z_m
"""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import yaml


# 相机轴 -> 底板轴（无俯仰、无滚转、朝向一致时）
R_BASE_CAMERA0 = np.array([
    [0.0, 0.0, 1.0],   # base_x = cam_z
    [-1.0, 0.0, 0.0],  # base_y = -cam_x
    [0.0, -1.0, 0.0],  # base_z = -cam_y
], dtype=np.float64)


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=np.float64)


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=np.float64)


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float64)


def euler_to_R(pitch_deg, roll_deg, yaw_deg):
    """pitch > 0 表示镜头朝下。"""
    return (rot_z(math.radians(yaw_deg))
            @ rot_y(math.radians(pitch_deg))
            @ rot_x(math.radians(roll_deg))
            @ R_BASE_CAMERA0)


def R_to_euler(R):
    """R = Rz(yaw) Ry(pitch) Rx(roll) R0 的逆解，返回 (pitch, roll, yaw) 度。"""
    rel = R @ R_BASE_CAMERA0.T
    # rel = Rz(yaw) Ry(pitch) Rx(roll)
    sp = -rel[2, 0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    roll = math.atan2(rel[2, 1], rel[2, 2])
    yaw = math.atan2(rel[1, 0], rel[0, 0])
    return math.degrees(pitch), math.degrees(roll), math.degrees(yaw)


class GroundProjector:
    """把像素点投影到地面（载体系，z=0）。"""

    def __init__(self, K, D, R_base_camera, origin_base,
                 min_range=0.15, max_range=20.0, min_down_sin=0.02):
        self.K = np.asarray(K, dtype=np.float64)
        self.D = np.asarray(D, dtype=np.float64).reshape(-1, 1)
        self.R = np.asarray(R_base_camera, dtype=np.float64).reshape(3, 3)
        self.O = np.asarray(origin_base, dtype=np.float64).reshape(3)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.min_down_sin = float(min_down_sin)
        if self.O[2] <= 0:
            raise SystemExit(f"[proj] camera height must be > 0, got origin={self.O}")

    # ------------------------------------------------------------------
    @classmethod
    def from_yaml(cls, intrinsics_path, extrinsics_path, **kw):
        with open(intrinsics_path, "r", encoding="utf-8") as f:
            intr = yaml.safe_load(f)
        with open(extrinsics_path, "r", encoding="utf-8") as f:
            extr = yaml.safe_load(f) or {}

        K = np.array(intr["camera_matrix"], dtype=np.float64)
        D = np.array(intr["distortion_coefficients"], dtype=np.float64)

        if "R_base_camera" in extr and "camera_origin_base_m" in extr:
            R = np.array(extr["R_base_camera"], dtype=np.float64).reshape(3, 3)
            O = np.array(extr["camera_origin_base_m"], dtype=np.float64).reshape(3)
            method = extr.get("method", "solvePnP")
        else:
            h = float(extr.get("camera_height_m", 0.0))
            pitch = float(extr.get("pitch_deg", 0.0))
            roll = float(extr.get("roll_deg", 0.0))
            yaw = float(extr.get("yaw_deg", 0.0))
            tx = float(extr.get("translation_x_m", 0.0))
            ty = float(extr.get("translation_y_m", 0.0))
            tz = float(extr.get("translation_z_m", h))
            R = euler_to_R(pitch, roll, yaw)
            O = np.array([tx, ty, tz], dtype=np.float64)
            method = "euler(height/pitch/roll/yaw)"
            if h <= 0:
                    raise SystemExit("[proj] missing camera_height_m in extrinsics, or it is <= 0")

        obj = cls(K, D, R, O, **kw)
        obj.method = method
        obj.image_size = (intr.get("image_width"), intr.get("image_height"))
        obj.extrinsics = extr
        obj.intrinsics = intr
        return obj

    # ------------------------------------------------------------------
    def ray_base(self, u, v):
        """像素 -> 载体系单位射线方向。"""
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        n = cv2.undistortPoints(pts, self.K, self.D).reshape(2)
        d_c = np.array([n[0], n[1], 1.0], dtype=np.float64)
        d_b = self.R @ d_c
        norm = np.linalg.norm(d_b)
        if norm < 1e-9:
            return None
        return d_b / norm

    def project(self, u, v):
        """返回 dict(x, y, range_m, angle_deg)；不可靠时返回 None。"""
        d = self.ray_base(u, v)
        if d is None:
            return None
        if d[2] > -self.min_down_sin:      # 射线没在往下走（地平线以上）
            return None
        lam = -self.O[2] / d[2]
        if not math.isfinite(lam) or lam <= 0:
            return None
        P = self.O + lam * d
        rng = math.hypot(P[0] - self.O[0], P[1] - self.O[1])
        if rng < self.min_range or rng > self.max_range:
            return None
        if abs(P[2]) > 1e-6:               # 数值兜底
            return None
        return {
            "x": float(P[0]),
            "y": float(P[1]),
            "range_m": float(rng),
            "angle_deg": float(math.degrees(math.atan2(d[1], d[0]))),
        }

    def horizon_v(self):
        """地平线像素行（用于界面提示）。"""
        # 找 d_b[2] == 0 的 pixel v：d_b = R @ [nx, ny, 1]
        r = self.R
        # d_b[2] = r20*nx + r21*ny + r22
        best = None
        for v in range(0, 2000, 2):
            for u in (self.K[0, 2],):
                d = self.ray_base(u, v)
                if d is None:
                    continue
                if d[2] > 0:
                    return v
        return best

    def describe(self):
        lines = [
            f"intrinsics: fx={self.K[0,0]:.2f} fy={self.K[1,1]:.2f} "
            f"cx={self.K[0,2]:.2f} cy={self.K[1,2]:.2f}  size={self.image_size}",
            f"distortion: {np.round(self.D.ravel(), 5).tolist()}",
            f"extrinsics source: {self.method}",
            f"camera origin (base): {np.round(self.O, 4).tolist()}  m",
        ]
        pitch, roll, yaw = R_to_euler(self.R)
        lines.append(f"equivalent mount angles: pitch_down={pitch:.2f}  roll={roll:.2f}  yaw={yaw:.2f} deg")
        return "\n".join(lines)