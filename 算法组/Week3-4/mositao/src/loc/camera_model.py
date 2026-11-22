# -*- coding: utf-8 -*-
"""单目相机模型：内参 / 畸变 / 外参 / 像素<->地面 双向换算（纯 numpy 实现）。

坐标约定（与考核细则 2.1 一致）：
- 载体坐标系 base：+X 向前、+Y 向左、+Z 向上，地面为 Z=0（ROS REP-103）；
- 相机坐标系 cam：+Z 沿光轴向前、+X 向右、+Y 向下（OpenCV）；
- 外参保存的是「相机 -> 载体」的刚性变换：p_base = R @ p_cam + t，t 为相机光心在 base 中的位置。

不依赖 cv2：畸变求逆用数值牛顿迭代，便于在无图形环境下做单元验证。
"""

from __future__ import annotations

import math

import numpy as np

from ..common.config_io import load_yaml, pick

EPS = 1e-9


def _rot(axis: str, deg: float) -> np.ndarray:
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


class CameraModel:
    """针孔相机 + OpenCV 五参数畸变模型 [k1, k2, p1, p2, k3]。"""

    def __init__(self, fx, fy, cx, cy, dist=(0, 0, 0, 0, 0), width=1280, height=720):
        self.K = np.array([[float(fx), 0.0, float(cx)],
                           [0.0, float(fy), float(cy)],
                           [0.0, 0.0, 1.0]], float)
        d = list(dist) + [0.0] * (5 - len(list(dist)))
        self.dist = np.array(d[:5], float)
        self.width = int(width)
        self.height = int(height)

    # ---------- 构造 ----------
    @classmethod
    def from_dict(cls, data: dict) -> "CameraModel":
        intr = data.get("camera_intrinsics", data)
        dist = pick(intr, "dist_coeffs", "distortion", "dist", default=[0, 0, 0, 0, 0])
        if isinstance(dist, dict):
            dist = [pick(dist, "k1", default=0.0), pick(dist, "k2", default=0.0),
                    pick(dist, "p1", default=0.0), pick(dist, "p2", default=0.0),
                    pick(dist, "k3", default=0.0)]
        return cls(
            fx=pick(intr, "fx"),
            fy=pick(intr, "fy"),
            cx=pick(intr, "cx"),
            cy=pick(intr, "cy"),
            dist=dist,
            width=pick(intr, "width", "image_width", default=1280),
            height=pick(intr, "height", "image_height", default=720),
        )

    @classmethod
    def from_yaml(cls, path) -> "CameraModel":
        return cls.from_dict(load_yaml(path))

    # ---------- 畸变 ----------
    def distort(self, x: np.ndarray, y: np.ndarray):
        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        radial = 1.0 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return xd, yd

    def undistort(self, xd: np.ndarray, yd: np.ndarray, iters: int = 25):
        """归一化平面：畸变坐标 -> 无畸变坐标（牛顿迭代，向量化）。"""
        xd = np.asarray(xd, float)
        yd = np.asarray(yd, float)
        x, y = xd.copy(), yd.copy()
        for _ in range(iters):
            xp, yp = self.distort(x, y)
            ex, ey = xp - xd, yp - yd
            if np.all(np.abs(ex) < 1e-14) and np.all(np.abs(ey) < 1e-14):
                break
            h = 1e-7
            xpx, ypx = self.distort(x + h, y)
            xpy, ypy = self.distort(x, y + h)
            j11, j21 = (xpx - xp) / h, (ypx - yp) / h
            j12, j22 = (xpy - xp) / h, (ypy - yp) / h
            det = j11 * j22 - j12 * j21
            det = np.where(np.abs(det) < 1e-12, 1e-12, det)
            dx = (j22 * ex - j12 * ey) / det
            dy = (-j21 * ex + j11 * ey) / det
            x -= dx
            y -= dy
            if float(np.max(np.hypot(dx, dy))) < 1e-14:
                break
        return x, y

    # ---------- 视场门限 ----------
    @property
    def max_offaxis_deg(self) -> float:
        """可信任的最大偏轴角（度）：以画面四角中最远的偏轴角为基准，留 15% 余量。

        多项式畸变只在标定视场内有效；超出该角度后模型可能「折返」，
        即把 60° 以外的点算回画面中央，反投影会得到完全错误的地面坐标。
        这个门限由相机自身参数推导，不写死，换镜头后自动跟随。
        """
        fx, fy = self.K[0, 0], self.K[1, 1]
        dx = max(self.K[0, 2], self.width - self.K[0, 2])
        dy = max(self.K[1, 2], self.height - self.K[1, 2])
        corner = math.degrees(math.atan2(math.hypot(dx, dy), min(abs(fx), abs(fy))))
        return corner * 1.15

    # ---------- 像素 <-> 相机系 ----------
    def rays_cam(self, uv) -> np.ndarray:
        """像素坐标 -> 相机系单位射线。"""
        uv = np.asarray(uv, float).reshape(-1, 2)
        xd = (uv[:, 0] - self.K[0, 2]) / self.K[0, 0]
        yd = (uv[:, 1] - self.K[1, 2]) / self.K[1, 1]
        x, y = self.undistort(xd, yd)
        rays = np.stack([x, y, np.ones_like(x)], axis=1)
        return rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def project(self, p_cam):
        """相机系点 -> 像素（含畸变）。返回 (uv, valid)，valid=False 表示点在相机后方。"""
        p = np.asarray(p_cam, float).reshape(-1, 3)
        z = p[:, 2]
        ok = z > 1e-6
        zs = np.where(ok, z, 1.0)
        x, y = p[:, 0] / zs, p[:, 1] / zs
        xd, yd = self.distort(x, y)
        u = self.K[0, 0] * xd + self.K[0, 2]
        v = self.K[1, 1] * yd + self.K[1, 2]
        return np.stack([u, v], axis=1), ok


class Extrinsics:
    """相机 -> 载体的刚性外参。"""

    def __init__(self, R: np.ndarray, t: np.ndarray):
        self.R = np.asarray(R, float).reshape(3, 3)
        self.t = np.asarray(t, float).reshape(3)

    @classmethod
    def from_params(cls, height: float, pitch_deg: float, roll_deg: float = 0.0,
                    yaw_offset_deg: float = 0.0, forward_m: float = 0.0,
                    lateral_m: float = 0.0) -> "Extrinsics":
        """pitch_deg：光轴俯角，向下为正；yaw_offset_deg：光轴相对载体 +X 的水平偏角，向左为正。"""
        # 光轴水平朝前时的相机基：x_c=右=-y_b, y_c=下=-z_b, z_c=前=+x_b
        r0 = np.array([[0.0, 0.0, 1.0],
                       [-1.0, 0.0, 0.0],
                       [0.0, -1.0, 0.0]], float)
        R = _rot("z", yaw_offset_deg) @ _rot("y", pitch_deg) @ _rot("x", roll_deg) @ r0
        t = np.array([float(forward_m), float(lateral_m), float(height)], float)
        return cls(R, t)

    @classmethod
    def from_dict(cls, data: dict) -> "Extrinsics":
        ext = data.get("camera_extrinsics", data)
        if "R" in ext and "t" in ext:
            return cls(ext["R"], ext["t"])
        return cls.from_params(
            height=pick(ext, "camera_height_m", "height_m", "z_m", default=0.0),
            pitch_deg=pick(ext, "pitch_deg", "pitch", default=0.0),
            roll_deg=pick(ext, "roll_deg", "roll", default=0.0),
            yaw_offset_deg=pick(ext, "yaw_offset_deg", "yaw_deg", default=0.0),
            forward_m=pick(ext, "forward_offset_m", "forward_m", default=0.0),
            lateral_m=pick(ext, "lateral_offset_m", "lateral_m", default=0.0),
        )

    @classmethod
    def from_yaml(cls, path) -> "Extrinsics":
        return cls.from_dict(load_yaml(path))

    # ---------- 变换 ----------
    def cam_to_base(self, p_cam) -> np.ndarray:
        p = np.asarray(p_cam, float)
        return p @ self.R.T + self.t

    def base_to_cam(self, p_base) -> np.ndarray:
        p = np.asarray(p_base, float)
        return (p - self.t) @ self.R


def offaxis_deg(p_cam) -> np.ndarray:
    """点/射线偏离光轴的角度（度）。"""
    p = np.asarray(p_cam, float).reshape(-1, 3)
    return np.degrees(np.arctan2(np.hypot(p[:, 0], p[:, 1]), np.abs(p[:, 2])))


def pixels_to_ground(cam: CameraModel, ext: Extrinsics, uv,
                     z_eps: float = 1e-6, min_range: float = 0.0,
                     max_range: float | None = None,
                     max_offaxis_deg: float | None = None):
    """像素点 -> 载体坐标系中的地面交点（Z=0）。

    返回 (pts_base, valid)：
    - pts_base: (n, 3) 交点，无效行为 NaN；
    - valid: (n,) bool，以下情形一律置 False：
        射线朝上 / 在相机后方 / 超出距离范围 / 偏轴角超过 max_offaxis_deg / 出现 NaN。

    细则 2.7 要求不得为无效观测伪造地面点，因此这里显式掩码，绝不用 0 填充。

    max_offaxis_deg 是必要的安全门限：多项式畸变模型只在标定视场内有效，
    视场外会出现「折返」（偏轴 60°+ 的点可能被算回画面中央），若不加门限，
    这类像素会被反投影到完全错误的地面位置，且表面上看不出异常。
    """
    uv = np.asarray(uv, float).reshape(-1, 2)
    n = uv.shape[0]
    out = np.full((n, 3), np.nan)
    if n == 0:
        return out, np.zeros(0, bool)

    rays_cam = cam.rays_cam(uv)            # (n,3) 相机系单位射线
    dirs = rays_cam @ ext.R.T              # R @ r  (n,3)
    origin = ext.t
    dz = dirs[:, 2]

    valid = np.isfinite(dz) & (dz < -z_eps)   # 必须朝下才可能与 Z=0 相交
    if max_offaxis_deg is None:
        max_offaxis_deg = cam.max_offaxis_deg
    valid &= offaxis_deg(rays_cam) <= max_offaxis_deg
    lam = np.full(n, np.nan)
    lam[valid] = -origin[2] / dz[valid]
    valid &= np.isfinite(lam) & (lam > 0.0)

    pts = origin[None, :] + lam[:, None] * dirs
    rng = np.linalg.norm(pts[:, :2] - origin[None, :2], axis=1)
    valid &= np.isfinite(rng)
    if min_range > 0:
        valid &= rng >= min_range
    if max_range is not None:
        valid &= rng <= max_range

    out[valid] = pts[valid]
    return out, valid


def ground_to_pixels(cam: CameraModel, ext: Extrinsics, pts_base,
                     max_offaxis_deg: float | None = None):
    """载体系地面点 -> 像素（仿真与自检用，与 pixels_to_ground 互为逆过程）。

    与 pixels_to_ground 使用同一个偏轴门限，保证正反变换的定义域一致，
    否则会出现「能投影出去、却反投影不回来」的伪一致性。
    """
    p = np.asarray(pts_base, float).reshape(-1, 3)
    p_cam = ext.base_to_cam(p)
    uv, ok = cam.project(p_cam)
    ok &= p_cam[:, 2] > 0.05
    if max_offaxis_deg is None:
        max_offaxis_deg = cam.max_offaxis_deg
    ok &= offaxis_deg(p_cam) <= max_offaxis_deg
    return uv, ok


def carrier_to_camera_pose_2d(x: float, y: float, yaw_rad: float, ext: Extrinsics):
    """载体二维位姿 -> 相机光心二维位姿（细则 2.9 的平面简化版）。

    返回 (x_cam_map, y_cam_map, yaw_cam_map[rad])。
    """
    c, s = math.cos(yaw_rad), math.sin(yaw_rad)
    # 相机光心在载体系中的水平位置（含前向/侧向安装偏置）
    t = ext.t
    dx, dy = float(t[0]), float(t[1])
    x_cam = x + c * dx - s * dy
    y_cam = y + s * dx + c * dy
    # 光轴相对载体 +X 的水平偏角
    yaw_off = math.atan2(float(ext.R[1, 2]), float(ext.R[0, 2]))
    return x_cam, y_cam, yaw_rad + yaw_off
