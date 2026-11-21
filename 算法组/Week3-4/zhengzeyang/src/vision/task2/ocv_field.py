
"""
坐标系转换
"""
import math
import numpy as np
import cv2
import yaml

def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


class CameraModel:
    def __init__(self, cfg_path):
        cfg = load_yaml(cfg_path)
        self.width = int(cfg.get("image_width", 1280))
        self.height = int(cfg.get("image_height", 720))
        self.K = np.array(cfg["K"], np.float64).reshape(3, 3)
        self.dist = np.array(cfg.get("dist", [0, 0, 0, 0, 0]), np.float64).reshape(-1, 1, 1)

        ex = cfg.get("extrinsics", {}) or {}
        self.cam_height = float(ex.get("height", 0.45))          # 光心离地高度 m
        self.pitch = math.radians(float(ex.get("pitch_deg", 20.0)))  # 俯仰，向下为正

    # ---------- 像素 -> 相机系方向（z 分量固定 1，不做归一化，比例会自动约掉）
    def pixel_ray(self, u, v):
        """像素 -> 相机系方向向量 (d1 右, d2 下, d3 前)。
        注意：cv2.undistortPoints 返回的已经是“归一化相机坐标”
        (即 (u-cx)/fx 经去畸变后的值)，不能再减主点、除焦距。"""
        pts = np.array([[[float(u), float(v)]]], np.float64)
        xn, yn = cv2.undistortPoints(pts, self.K, self.dist)[0, 0]
        return np.array([xn,        # d1: 右
                         yn,        # d2: 下
                         1.0])      # d3: 前

    # ---------- 像素 -> 地面点（相机局部水平系 a 前向 / b 左向）
    def pixel_to_local_ground(self, u, v, max_range=12.0):
        """返回 (a, b, ok, reason)。a/b 单位 m。此结果与相机 yaw 无关。"""
        d = self.pixel_ray(u, v)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)

        # 相机系(x右,y下,z前) -> 水平系(x右,y前,z上)：
        #   相机 y 轴(下)   在水平系里 = (0, -sin p, -cos p)
        #   相机 z 轴(光轴) 在水平系里 = (0,  cos p, -sin p)
        d_h = np.array([d[0],
                        d[2] * cp - d[1] * sp,
                        -d[2] * sp - d[1] * cp])

        if d_h[2] >= -1e-9:
            return 0.0, 0.0, False, "ray_up"        # 射线朝上/平行，落在地平线外
        lam = -self.cam_height / d_h[2]            # 射线从高度 h 落到 z=0
        if lam <= 0 or lam > max_range:
            return 0.0, 0.0, False, "out_of_range"
        a = lam * d_h[1]      # 前向距离
        b = -lam * d_h[0]     # 左向距离（相机右为正，所以取负）
        return float(a), float(b), True, "ok"

    # ---------- 批量：像素 -> 地面点（向量化，供检测器一次处理几千个点）
    def pixel_to_ground_many(self, us, vs, max_range=12.0):
        """返回 (a, b)，无效处为 nan。与 pixel_to_local_ground 完全同一套公式。"""
        us = np.atleast_1d(np.asarray(us, np.float64))
        vs = np.atleast_1d(np.asarray(vs, np.float64))
        pts = np.stack([us, vs], -1).reshape(-1, 1, 2)
        d = cv2.undistortPoints(pts, self.K, self.dist).reshape(-1, 2)
        d1, d2 = d[:, 0], d[:, 1]
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        fwd = cp - d2 * sp            # 光轴前向分量（d3 归一化为 1）
        up = -sp - d2 * cp            # 竖直分量，必须为负才打得到地面
        with np.errstate(divide="ignore", invalid="ignore"):
            lam = -self.cam_height / up
            ok = (up < -1e-9) & (lam > 0) & (lam <= max_range)
            a = np.where(ok, lam * fwd, np.nan)
            b = np.where(ok, -lam * d1, np.nan)
        return a, b

    # ---------- 地面点 -> 像素（含畸变，用于把地面系结果画回画面）
    def ground_to_pixel(self, a, b):
        """(a 前向, b 左向) -> (u, v)。地面点已在相机系原点上，rvec/tvec 取零。"""
        a = np.atleast_1d(np.asarray(a, np.float64))
        b = np.atleast_1d(np.asarray(b, np.float64))
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        x_c = -b                              # 相机 x 向右，b 向左
        y_c = -sp * a + cp * self.cam_height  # 相机 y 向下
        z_c = cp * a + sp * self.cam_height   # 相机 z 向前
        P = np.stack([x_c, y_c, z_c], -1).reshape(-1, 1, 3)
        uv, _ = cv2.projectPoints(P, np.zeros(3), np.zeros(3), self.K, self.dist)
        uv = uv.reshape(-1, 2)
        return uv[:, 0], uv[:, 1]

    # ---------- 局部系 -> 场地系（已知 yaw 时用；匹配阶段由矩阵直接完成）
    def local_to_field(self, a, b, x_cam, y_cam, yaw_deg):
        """场地坐标 = R(yaw)·(a,b) + (x_cam, y_cam)"""
        c, s = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
        return (x_cam + a * c - b * s, y_cam + a * s + b * c)


class GroundModel:
    """批量正投影 / 反投影 / 局部雅可比的薄封装，隔离检测与可视化模块对 CameraModel 的直接依赖。"""

    def __init__(self, cam, eps=2.0):
        self.cam = cam
        self.eps = float(eps)

    def to_ground(self, us, vs):
        return self.cam.pixel_to_ground_many(us, vs)

    def to_pixel(self, a, b):
        return self.cam.ground_to_pixel(a, b)

    def jac(self, us, vs):
        """J = ∂(a,b)/∂(u,v)，m/px，形状 (N,2,2)，无效点为 nan。"""
        e = self.eps
        us = np.atleast_1d(np.asarray(us, np.float64))
        vs = np.atleast_1d(np.asarray(vs, np.float64))
        a_r, b_r = self.to_ground(us + e, vs)
        a_l, b_l = self.to_ground(us - e, vs)
        a_d, b_d = self.to_ground(us, vs + e)
        a_u, b_u = self.to_ground(us, vs - e)
        J = np.empty((us.size, 2, 2), np.float64)
        J[:, 0, 0] = (a_r - a_l) / (2 * e)
        J[:, 0, 1] = (a_d - a_u) / (2 * e)
        J[:, 1, 0] = (b_r - b_l) / (2 * e)
        J[:, 1, 1] = (b_d - b_u) / (2 * e)
        with np.errstate(invalid="ignore"):
            bad = ~np.isfinite(J).all(axis=(1, 2))
            bad |= np.abs(np.linalg.det(J)) < 1e-12
        J[bad] = np.nan
        return J
