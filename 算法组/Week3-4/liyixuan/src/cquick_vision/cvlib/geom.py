"""SE(2) 几何、射线-地面求交、误差传播。

本模块只依赖 numpy，便于在没有 OpenCV 的环境下单测。
坐标约定：
  base   : x 前, y 左, z 上（右手系，原点在地面）
  camera : x 右, y 下, z 前（OpenCV）
  map    : +X 向右, +Y 向上，yaw 从 +X 逆时针为正
位姿统一用 (x, y, theta[rad])，含义为 "A_from_B"：P_A = R(theta) @ P_B + t
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np


# ---------------------------------------------------------------- 角度工具

def wrap_rad(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


def angle_diff_deg(a: float, b: float) -> float:
    return wrap_deg(a - b)


# ---------------------------------------------------------------- 旋转

def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def Rz(deg: float) -> np.ndarray:
    """绕 +Z 轴旋转 deg 度（2D 旋转矩阵）。"""
    return rot2(math.radians(deg))


def _rot3(axis: str, deg: float) -> np.ndarray:
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)
    raise ValueError(axis)


# 相机水平朝向时（pitch=0, roll=0）： camera -> base
R_CAM_HORIZONTAL = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
], float)


def rotation_from_mount(pitch_down_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """由安装俯角/滚角构造 R_base_camera（近似，快速版外参）。"""
    return R_CAM_HORIZONTAL @ _rot3("x", -pitch_down_deg) @ _rot3("z", roll_deg)


# ---------------------------------------------------------------- SE(2)

def transform_points(theta: float, t, pts) -> np.ndarray:
    pts = np.asarray(pts, dtype=float).reshape(-1, 2)
    return pts @ rot2(theta).T + np.asarray(t, dtype=float).reshape(2,)


def invert_pose(x: float, y: float, theta: float) -> Tuple[float, float, float]:
    """A_from_B -> B_from_A"""
    Rt = rot2(theta).T
    t = -Rt @ np.array([x, y], dtype=float)
    return float(t[0]), float(t[1]), float(wrap_rad(-theta))


def compose_pose(p_map_a, p_a_b) -> Tuple[float, float, float]:
    """map_from_B = map_from_A * A_from_B"""
    xa, ya, ta = p_map_a
    xb, yb, tb = p_a_b
    t = rot2(ta) @ np.array([xb, yb], float) + np.array([xa, ya], float)
    return float(t[0]), float(t[1]), float(wrap_rad(ta + tb))


def weighted_se2_ls(src, dst, w=None) -> Tuple[float, float, float]:
    """已知对应关系的加权刚体最小二乘（尺度固定为 1，不是相似变换）。

    求解 min sum w_i * |R(theta) p_i + t - q_i|^2
    """
    src = np.asarray(src, float).reshape(-1, 2)
    dst = np.asarray(dst, float).reshape(-1, 2)
    n = len(src)
    if n == 0 or len(dst) != n:
        raise ValueError("src/dst 数量不匹配")
    w = np.ones(n) if w is None else np.asarray(w, float).reshape(-1)
    w = np.clip(w, 1e-9, None)
    W = float(w.sum())

    ps = (src * w[:, None]).sum(0) / W
    qs = (dst * w[:, None]).sum(0) / W
    p0, q0 = src - ps, dst - qs

    cross = float((w * (p0[:, 0] * q0[:, 1] - p0[:, 1] * q0[:, 0])).sum())
    dot = float((w * (p0[:, 0] * q0[:, 0] + p0[:, 1] * q0[:, 1])).sum())
    theta = math.atan2(cross, dot)

    R = rot2(theta)
    t = qs - R @ ps
    return float(t[0]), float(t[1]), float(theta)


# ---------------------------------------------------------------- 相机射线

def pixel_ray(u: float, v: float, K: np.ndarray) -> np.ndarray:
    """像素 -> 相机系单位射线方向（输入必须是去畸变后的像素）。"""
    d = np.linalg.solve(np.asarray(K, float), np.array([u, v, 1.0], float))
    n = np.linalg.norm(d)
    if n <= 0:
        raise ValueError("退化射线")
    return d / n


def ray_to_ground(d_b, O_b, min_down: float = 1e-3,
                  max_range: float = 8.0) -> Tuple[Optional[np.ndarray], Optional[str]]:
    """载体坐标系射线与地面 Z=0 求交。

    返回 (地面点, None) 或 (None, 失败原因)。
    失败原因：ray_not_downward / behind / range_xx / nan
    """
    d_b = np.asarray(d_b, float).reshape(3)
    O_b = np.asarray(O_b, float).reshape(3)

    if not np.all(np.isfinite(d_b)) or not np.all(np.isfinite(O_b)):
        return None, "nan"
    if d_b[2] > -min_down:            # 射线不朝下，永远不会交到地面
        return None, "ray_not_downward"

    lam = -O_b[2] / d_b[2]
    if not np.isfinite(lam) or lam <= 0.0:
        return None, "behind"

    P = O_b + lam * d_b
    D = math.hypot(P[0] - O_b[0], P[1] - O_b[1])   # 地面距离
    if not np.isfinite(D) or D <= 1e-3:
        return None, "nan"
    if D > max_range:
        return None, f"range_{D:.2f}"
    P[2] = 0.0
    return P, None


def ground_distance(P_b) -> float:
    return float(math.hypot(float(P_b[0]), float(P_b[1])))


def sigma_from_range(D: float, delta_px: float, fy: float, h_cam: float) -> float:
    """地面距离 D 处、像素误差 delta_px 引起的地面位置误差（米）。

    sigma ~= D^2 * delta_px / (fy * h)
    """
    if fy <= 0 or h_cam <= 0:
        return 1.0
    s = (D * D) * delta_px / (fy * h_cam)
    return float(max(s, 1e-3))


def weight_from_sigma(sigma: float) -> float:
    return float(1.0 / max(sigma * sigma, 1e-6))
