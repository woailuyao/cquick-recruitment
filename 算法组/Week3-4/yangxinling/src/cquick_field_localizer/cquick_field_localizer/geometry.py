#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
geometry.py —— 单目相机几何：去畸变、视线、地面投影、坐标系变换

这是任务2 的数学核心，考核文档要求的那条链路就实现在这里：

    像素 (u,v)
      --undistort-->  去畸变像素
      --K^-1-->       归一化相机坐标 d_c = (x, y, 1)
      --R_bc-->       载体(车体)坐标 d_b
      --与 Z=0 地面求交-->  P_b = O_b + λ·d_b,  λ = -O_bz / d_bz
      --(X, Y, ψ)-->  map 坐标

坐标系约定（务必和 README 写一致）：

  相机坐标系 C：x 向右，y 向下，z 向前（OpenCV 标准）
  载体系      B：x 向前，y 向左，z 向上（机器人常用）
  map 坐标系  M：x 向右，y 向上（俯视图），yaw 逆时针为正，单位米

  相机水平安装时两套轴的关系：x_b = z_c, y_b = -x_c, z_b = -y_c
  即 R_bc0 = [[0, 0, 1],
              [-1,0, 0],
              [0, -1,0]]

  俯仰（pitch，向下为正）绕 y_b 转；滚转（roll）绕光轴 z_c 转。

  俯视图里 map 的 +X 向右、+Y 向上；机器人朝向 yaw 从 +X 起逆时针量。
  于是车体前向 = (cosψ, sinψ)，车体左向 = (-sinψ, cosψ)。
"""

import numpy as np


# ---------------------------------------------------------------------------
# 外参构造
# ---------------------------------------------------------------------------
def rotation_body_from_camera(pitch_deg=0.0, roll_deg=0.0):
    """
    返回 R_bc（3×3），把相机坐标系的向量旋到载体系。

    pitch_deg : 镜头向下俯的角度（正数表示"低头看地"）
    roll_deg  : 相机绕自身光轴的滚转（正数按右手定则，绕 z_c 正向）
    """
    p = np.deg2rad(float(pitch_deg))
    r = np.deg2rad(float(roll_deg))

    R_bc0 = np.array([[0.0, 0.0, 1.0],
                      [-1.0, 0.0, 0.0],
                      [0.0, -1.0, 0.0]])

    Ry = np.array([[np.cos(p), 0.0, np.sin(p)],
                   [0.0, 1.0, 0.0],
                   [-np.sin(p), 0.0, np.cos(p)]])
    Rz = np.array([[np.cos(r), -np.sin(r), 0.0],
                   [np.sin(r), np.cos(r), 0.0],
                   [0.0, 0.0, 1.0]])
    return Ry @ R_bc0 @ Rz


def extrinsics_from_mount(height_m, pitch_deg, roll_deg, dx=0.0, dy=0.0):
    """
    由"安装高度 + 俯仰 + 滚转 + 平移"生成 R_bc 与相机光心在载体系中的位置。

    返回 (R_bc, O_b)，O_b = (dx, dy, height_m)：
      dx 为相机相对车体原点的前向偏移，dy 为左向偏移。
    车体原点取在"地面上的车体中心"（z=0 就是地面）。
    """
    R_bc = rotation_body_from_camera(pitch_deg, roll_deg)
    O_b = np.array([float(dx), float(dy), float(height_m)])
    return R_bc, O_b


def optical_axis_ground_point(R_bc, O_b):
    """光轴与地面的交点，用来检查"相机到底在看多远"，单位米（载体系 x,y）。"""
    d_b = R_bc @ np.array([0.0, 0.0, 1.0])
    if abs(d_b[2]) < 1e-9:
        return None
    lam = -O_b[2] / d_b[2]
    if lam <= 0:
        return None
    P = O_b + lam * d_b
    return float(P[0]), float(P[1]), float(lam)


# ---------------------------------------------------------------------------
# 相机模型
# ---------------------------------------------------------------------------
class CameraModel:
    """把内参 + 畸变 + 外参打包，提供像素→地面的批量投影。"""

    def __init__(self, K, D, R_bc, O_b, image_size=None,
                 pitch_deg=float("nan"), roll_deg=float("nan")):
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        # 安装参数只用于显示/记录（实际投影用 R_bc），不参与计算
        self.pitch_deg = float(pitch_deg)
        self.roll_deg = float(roll_deg)
        self.D = np.asarray(D, dtype=np.float64).ravel()
        while self.D.size < 5:
            self.D = np.append(self.D, 0.0)
        self.R_bc = np.asarray(R_bc, dtype=np.float64).reshape(3, 3)
        self.O_b = np.asarray(O_b, dtype=np.float64).ravel()
        self.image_size = tuple(image_size) if image_size else None
        self.K_inv = np.linalg.inv(self.K)

    # ---- 坐标轴/工具 ----
    @property
    def height(self):
        return float(self.O_b[2])

    def undistort_pixels(self, uv):
        """去畸变，返回去畸变后的像素坐标（可用于把检测点画到干净图上）。"""
        pts = np.asarray(uv, dtype=np.float64).reshape(-1, 1, 2)
        if np.allclose(self.D, 0):
            return pts.reshape(-1, 2)
        out = cv2_undistort_points(pts, self.K, self.D)
        return out.reshape(-1, 2)

    def normalize(self, uv_undist):
        """去畸变像素 → 归一化相机坐标（z=1 平面）。"""
        pts = np.asarray(uv_undist, dtype=np.float64).reshape(-1, 2)
        homo = np.hstack([pts, np.ones((len(pts), 1))])
        d = homo @ self.K_inv.T           # (N,3)
        return d / d[:, 2:3]

    def bearings_body(self, uv_undist):
        """
        像素 → 载体系方向向量（未归一化，z 分量 <0 表示朝下）。
        注意：只对方向做旋转，不做平移。
        """
        d_c = self.normalize(uv_undist)
        return d_c @ self.R_bc.T

    def ground_from_pixels(self, uv, min_fwd=0.02):
        """
        像素 → 地面上一点（载体系 x 前 / y 左，单位米，原点在车体正下方地面）。

        返回 (points_Nx2, valid_N, depth_N)：
          valid=False 表示该像素在地平线以上（d_bz >= 0），没有交点。
          depth 是该点相对相机光心的前向距离（≈地面上到相机的斜距在光轴上的投影）。
        """
        uv = np.asarray(uv, dtype=np.float64).reshape(-1, 2)
        if len(uv) == 0:
            return np.zeros((0, 2)), np.zeros(0, dtype=bool), np.zeros(0)
        und = self.undistort_pixels(uv)
        d_b = self.bearings_body(und)

        z = d_b[:, 2]
        valid = z < -1e-6                      # 必须朝下
        lam = np.full(len(d_b), np.inf)
        lam[valid] = -self.O_b[2] / z[valid]   # λ = -O_bz / d_bz

        P = self.O_b[None, :] + lam[:, None] * d_b
        pts = P[:, :2]
        # 交点跑到相机背后（λ<=0）也算无效
        valid &= lam > 0
        # 太近的点数值不稳定，也剔除
        valid &= pts[:, 0] > min_fwd
        depth = lam
        return pts, valid, depth

    # ---- 位姿 ----
    def body_to_map(self, pts_b, pose_xy, yaw_deg):
        """载体系二维点 → map。pose_xy=(X,Y)，yaw_deg 逆时针为正。"""
        pts_b = np.asarray(pts_b, dtype=np.float64).reshape(-1, 2)
        psi = np.deg2rad(yaw_deg)
        c, s = np.cos(psi), np.sin(psi)
        R = np.array([[c, -s], [s, c]])
        return pts_b @ R.T + np.asarray(pose_xy, dtype=np.float64).reshape(1, 2)

    def map_to_body(self, pts_m, pose_xy, yaw_deg):
        pts_m = np.asarray(pts_m, dtype=np.float64).reshape(-1, 2)
        psi = np.deg2rad(yaw_deg)
        c, s = np.cos(psi), np.sin(psi)
        R = np.array([[c, -s], [s, c]])
        return (pts_m - np.asarray(pose_xy, dtype=np.float64).reshape(1, 2)) @ R


# 延迟导入 cv2，方便在没有 OpenCV 的纯数学环境里做单测
def cv2_undistort_points(pts, K, D):
    import cv2
    return cv2.undistortPoints(pts, K, D, P=K)


# ---------------------------------------------------------------------------
# 从 YAML 字典构造（给 localizer 用，避免依赖 yaml 的写法到处重复）
# ---------------------------------------------------------------------------
def camera_from_configs(intr_cfg, extr_cfg):
    """intr_cfg: camera_intrinsics.yaml 解析后的 dict；extr_cfg: camera_extrinsics.yaml。"""
    cm = intr_cfg.get("camera_matrix", {})
    if "K" in intr_cfg and not cm:
        K = np.array(intr_cfg["K"], dtype=np.float64)
    else:
        K = np.array([[cm["fx"], 0.0, cm["cx"]],
                      [0.0, cm["fy"], cm["cy"]],
                      [0.0, 0.0, 1.0]], dtype=np.float64)
    if "D" in intr_cfg:
        D = np.array(intr_cfg["D"], dtype=np.float64).ravel()
    else:
        dc = intr_cfg.get("dist_coeffs", {})
        D = np.array([dc.get("k1", 0.0), dc.get("k2", 0.0),
                      dc.get("p1", 0.0), dc.get("p2", 0.0),
                      dc.get("k3", 0.0)], dtype=np.float64)
    _pitch = float(extr_cfg.get("pitch_deg", float("nan")))
    _roll = float(extr_cfg.get("roll_deg", float("nan")))
    if "R_bc" in extr_cfg:
        R_bc = np.array(extr_cfg["R_bc"], dtype=np.float64).reshape(3, 3)
    else:
        R_bc = rotation_body_from_camera(extr_cfg.get("pitch_deg", 0.0),
                                         extr_cfg.get("roll_deg", 0.0))
    if "O_b" in extr_cfg:
        O_b = np.array(extr_cfg["O_b"], dtype=np.float64).ravel()
    else:
        O_b = np.array([extr_cfg.get("dx", 0.0), extr_cfg.get("dy", 0.0),
                        extr_cfg["height_m"]], dtype=np.float64)
    size = (intr_cfg.get("image_width"), intr_cfg.get("image_height"))
    return CameraModel(K, D, R_bc, O_b, size,
                       pitch_deg=_pitch, roll_deg=_roll)
