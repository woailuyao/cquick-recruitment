#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单目地面投影：像素坐标 (u,v) → 载体坐标系地面点 (x, y) 米。"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class GroundProjection:
    K: np.ndarray
    dist: np.ndarray
    height: float
    pitch_deg: float = 0.0
    roll_deg: float = 0.0
    offset: Tuple[float, float] = (0.0, 0.0)
    min_dist: float = 0.3
    max_dist: float = 12.0

    def _R_cam_to_carrier(self) -> np.ndarray:
        """相机系(右/下/前) → 载体系(前/左/上)，叠加俯仰和滚转。"""
        swap = np.array([[0, 0, 1],
                         [-1, 0, 0],
                         [0, -1, 0]], dtype=np.float64)

        p = math.radians(self.pitch_deg)
        cp, sp = math.cos(p), math.sin(p)
        R_pitch = np.array([[cp, 0, sp],
                            [0, 1, 0],
                            [-sp, 0, cp]], dtype=np.float64)

        r = math.radians(self.roll_deg)
        cr, sr = math.cos(r), math.sin(r)
        R_roll = np.array([[1, 0, 0],
                           [0, cr, -sr],
                           [0, sr, cr]], dtype=np.float64)

        return R_pitch @ R_roll @ swap

    def pixel_to_ground(self, u: float, v: float) -> Optional[Tuple[float, float]]:
        """单个像素 → 载体系地面坐标。无法确定时返回 None（不许编坐标）。"""
        pts = np.array([[[u, v]]], dtype=np.float64)
        und = cv2.undistortPoints(pts, self.K, self.dist, P=self.K)
        x_n = (und[0, 0, 0] - self.K[0, 2]) / self.K[0, 0]
        y_n = (und[0, 0, 1] - self.K[1, 2]) / self.K[1, 1]

        d_cam = np.array([x_n, y_n, 1.0], dtype=np.float64)
        d = self._R_cam_to_carrier() @ d_cam
        norm = np.linalg.norm(d)
        if norm < 1e-9:
            return None
        d /= norm

        if d[2] > -1e-3:
            return None

        lam = -self.height / d[2]
        if lam <= 0:
            return None

        P = np.array([0.0, 0.0, self.height]) + lam * d
        gx = P[0] + self.offset[0]
        gy = P[1] + self.offset[1]

        dist = math.hypot(gx, gy)
        if dist < self.min_dist or dist > self.max_dist:
            return None
        return float(gx), float(gy)

    def ground_to_pixel(self, x: float, y: float) -> Optional[Tuple[float, float]]:
        """反过来：地面点 → 像素（画可视化/自测用）。"""
        P = np.array([x - self.offset[0], y - self.offset[1], 0.0])
        O = np.array([0.0, 0.0, self.height])
        d = P - O
        R = self._R_cam_to_carrier()
        d_cam = R.T @ d
        if d_cam[2] <= 1e-6:
            return None
        uv = self.K @ (d_cam / d_cam[2])
        return float(uv[0]), float(uv[1])


def make_default(width: int = 1280, height: int = 720,
                 fov_deg: float = 60.0, cam_height: float = 1.2,
                 pitch_deg: float = 10.0) -> GroundProjection:
    """按"水平视场角 + 分辨率"造一个近似内参，用于**自测**（真实标定后要替换）。"""
    f = (width / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
    K = np.array([[f, 0, width / 2.0],
                  [0, f, height / 2.0],
                  [0, 0, 1]], dtype=np.float64)
    dist = np.zeros(5, dtype=np.float64)
    return GroundProjection(K=K, dist=dist, height=cam_height, pitch_deg=pitch_deg)
