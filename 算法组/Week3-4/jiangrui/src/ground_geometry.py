# -*- coding: utf-8 -*-
"""单目相机几何：内参、去畸变、射线、地面(Z=0)投影、外参构建。

坐标约定
--------
camera  : OpenCV 光学坐标系，x 向右，y 向下，z 沿光轴向前。
carrier : 载体坐标系，x 向前（相机水平朝向），y 向左，z 向上。
          本次考核把载体原点定义为“相机光心在地面上的垂直投影”，
          因此 carrier 位姿与 camera 光心的二维地图位姿一致（见 README）。
map     : field.yaml 的场地坐标系，Z=0 为地面。

外参只需要“相机相对地平面的姿态”：
    高度 h、俯仰 pitch（相机往下看为正）、滚转 roll。
航向 yaw 由定位结果给出，不放进外参，避免重复计数。
"""

import math
import os

import cv2
import numpy as np

import minyaml

GROUND_Z = 0.0


def _R0():
    """camera 光轴 -> 载体前向 的固定旋转（不含俯仰/滚转）。"""
    return np.array([[0.0, 0.0, 1.0],
                     [-1.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0]], dtype=float)


def rot_x(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(deg):
    r = math.radians(deg)
    c, s = math.cos(r), math.sin(r)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rotation_from_angles(pitch_down_deg, roll_deg, yaw_deg=0.0):
    """由安装角构造 camera -> carrier 旋转。pitch 向下为正。"""
    return rot_z(yaw_deg) @ rot_x(roll_deg) @ rot_y(pitch_down_deg) @ _R0()


class CameraModel:
    """从 calibration/camera_intrinsics.yaml 读取内参。"""

    def __init__(self, intrinsics_path):
        data = minyaml.load(intrinsics_path)
        self.path = intrinsics_path
        self.width = int(data.get("image_width", 1280))
        self.height = int(data.get("image_height", 720))
        raw_k = data["camera_matrix"]["data"]
        self.K = np.array(raw_k, dtype=float).reshape(3, 3)
        dist = data.get("dist_coeffs") or [0.0] * 5
        self.D = np.array(dist, dtype=float).reshape(-1, 1)
        self.rms = data.get("rms_reprojection_error")

    def normalized_rays(self, uv):
        """像素 -> 相机坐标系单位射线（先按畸变模型去畸变）。"""
        pts = np.asarray(uv, dtype=float).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, self.K, self.D).reshape(-1, 2)
        rays = np.hstack([norm, np.ones((len(norm), 1))])
        return rays / np.linalg.norm(rays, axis=1, keepdims=True)

    def project_points(self, points_b, R_bc, t_bc):
        """载体坐标点 -> 像素（调试/画图用，可能与检测点不完全重合）。"""
        pts = np.asarray(points_b, dtype=float).reshape(-1, 3)
        cam = (pts - t_bc.reshape(1, 3)) @ R_bc  # R^T (P - t)
        z = cam[:, 2]
        ok = z > 1e-6
        uv = np.full((len(pts), 2), np.nan)
        if ok.any():
            proj = cam[ok] / z[ok, None]
            uv[ok] = cv2.projectPoints(proj.reshape(-1, 1, 3),
                                       np.zeros(3), np.zeros(3),
                                       self.K, self.D)[0].reshape(-1, 2)
        return uv, ok


class Extrinsics:
    """camera -> carrier 的固定外参。

    carrier 原点 = 相机光心在地面上的垂直投影，因此平移为 (0, 0, h)。
    如果将来相机装到机器人底座上，可用 from_matrix() 直接给完整位姿。
    """

    def __init__(self, R_bc, t_bc, height, pitch_down, roll, method, source):
        self.R_bc = np.asarray(R_bc, dtype=float)
        self.t_bc = np.asarray(t_bc, dtype=float).reshape(3)
        self.height = float(height)
        self.pitch_down = float(pitch_down)
        self.roll = float(roll)
        self.method = method
        self.source = source

    @classmethod
    def from_angles(cls, height_m, pitch_down_deg, roll_deg, source="命令行/配置"):
        h = float(height_m)
        R = rotation_from_angles(pitch_down_deg, roll_deg)
        return cls(R, [0.0, 0.0, h], h, pitch_down_deg, roll_deg,
                   "手工测量高度 + 安装角", source)

    @classmethod
    def from_matrix(cls, R_bc, t_bc, height, pitch_down, roll, source="yaml"):
        return cls(R_bc, t_bc, height, pitch_down, roll, "旋转矩阵 + 平移向量", source)

    @classmethod
    def from_yaml(cls, path):
        """读取已标定外参；status 不是 calibrated 时返回 None。"""
        if not os.path.exists(path):
            return None
        data = minyaml.load(path)
        if str(data.get("status", "")).lower() != "calibrated":
            return None
        if data.get("rotation_matrix") and data.get("translation_m"):
            R = np.array(data["rotation_matrix"], dtype=float).reshape(3, 3)
            t = np.array(data["translation_m"], dtype=float).reshape(3)
            return cls.from_matrix(R, t, data.get("camera_height_m", t[2]),
                                   data.get("pitch_deg", 0.0),
                                   data.get("roll_deg", 0.0), source=path)
        if data.get("camera_height_m") is not None:
            return cls.from_angles(data["camera_height_m"],
                                   data.get("pitch_deg", 0.0) or 0.0,
                                   data.get("roll_deg", 0.0) or 0.0,
                                   source=path)
        return None

    def camera_yaw_offset_deg(self):
        """相机光轴水平方向相对 carrier +X 的夹角（标准装法为 0）。"""
        fwd = self.R_bc @ np.array([0.0, 0.0, 1.0])
        return math.degrees(math.atan2(fwd[1], fwd[0]))

    def project_to_ground(self, uv, camera):
        """像素 -> carrier 坐标系地面点。

        返回 dict:
            points : (N,2) 地面坐标，无效点为 nan
            valid  : (N,) bool
            dist   : (N,) 沿射线距离（米），无效点为 inf
        """
        uv = np.asarray(uv, dtype=float).reshape(-1, 2)
        n = len(uv)
        pts = np.full((n, 2), np.nan)
        dist = np.full(n, np.inf)
        valid = np.zeros(n, dtype=bool)
        if n == 0:
            return {"points": pts, "valid": valid, "dist": dist}

        d_c = camera.normalized_rays(uv)
        d_b = (self.R_bc @ d_c.T).T
        den = d_b[:, 2]
        hits = den < -1e-4  # 射线必须朝下
        if not hits.any():
            return {"points": pts, "valid": valid, "dist": dist}

        lam = np.full(n, np.inf)
        lam[hits] = -self.t_bc[2] / den[hits]  # 地面是 Z=0
        good = hits & np.isfinite(lam) & (lam > 0.05) & (lam < 30.0)
        pts[good] = self.t_bc[:2] + lam[good, None] * d_b[good, :2]
        dist[good] = lam[good]
        valid = good & np.isfinite(pts).all(axis=1)
        return {"points": pts, "valid": valid, "dist": dist}
