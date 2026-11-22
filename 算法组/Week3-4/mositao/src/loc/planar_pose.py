# -*- coding: utf-8 -*-
"""路线 B：由「已知地面点 <-> 像素」直接解相机在场地地图中的位姿（PnP）。

与 matcher.py 的区别（两条路线并存，互为校验）：

| 对比项 | 路线 A：观测点地图匹配 | 路线 B：PnP 直接解算 |
| --- | --- | --- |
| 最少标志数 | 3 | 4（单应要求共面四点非共线） |
| 是否需要外参 | 需要（高度/俯仰/滚转） | **不需要**，一次解出全部 6DoF |
| 输出 | x/y/yaw | x/y/yaw + 高度 + 俯仰 + 滚转 |
| 歧义 | 靠多假设残差 | 共面解天然给出双解，靠高度>0 剔除镜像解 |
| 精度 | 受外参误差影响 | 只受内参与像素误差影响 |

考核细则允许自选实现路线，因此两条都留着：现场用 B 取位姿与姿态，
用 A 做交叉校验；任一给出的位姿差异过大时报警。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera_model import CameraModel, Extrinsics, _rot


@dataclass
class PlanarPoseResult:
    valid: bool = False
    x: float = float("nan")            # 相机光心在地图中的 X (m)
    y: float = float("nan")
    yaw_deg: float = float("nan")      # 光轴水平投影相对地图 +X 的角度（逆时针为正）
    height_m: float = float("nan")     # 光心离地高度
    pitch_deg: float = float("nan")    # 俯角，向下为正
    roll_deg: float = float("nan")
    rms_px: float = float("nan")       # 重投影 RMS（像素）
    n_points: int = 0
    n_inliers: int = 0
    status: str = "lost"
    R: np.ndarray | None = None        # 地图 -> 相机
    t: np.ndarray | None = None
    alternatives: list = field(default_factory=list)

    def as_dict(self) -> dict:
        ok = self.valid
        # 同 matcher.LocalizationResult.as_dict：相机光心即载体参考原点，两组同值。
        x = round(self.x, 4) if ok else None
        y = round(self.y, 4) if ok else None
        yaw = round(self.yaw_deg, 3) if ok else None
        return {
            "valid": ok,
            "x_carrier_map": x,
            "y_carrier_map": y,
            "yaw_carrier_map": yaw,
            "x_camera_map": x,
            "y_camera_map": y,
            "yaw_camera_map": yaw,
            "height_m": round(self.height_m, 4) if ok else None,
            "pitch_deg": round(self.pitch_deg, 3) if ok else None,
            "roll_deg": round(self.roll_deg, 3) if ok else None,
            "rms_px": round(self.rms_px, 3) if ok else None,
            "marker_count": self.n_inliers,
            "status": self.status,
        }


def _decompose(R_cam_from_map: np.ndarray, t_cam_from_map: np.ndarray):
    """由 p_cam = R p_map + t 解出相机在地图中的位置与姿态角。

    与我们 Extrinsics 的定义对齐：M = R^T 为相机各轴在地图中的方向，
    M[:,2] 是光轴，M[:,1] 是相机 +Y（画面向下）。
    """
    M = R_cam_from_map.T
    cam_pos = -R_cam_from_map.T @ t_cam_from_map
    axis = M[:, 2]                                   # 光轴在 map 中的方向
    height = float(cam_pos[2])

    cos_p = float(np.clip(np.linalg.norm(axis[:2]), -1.0, 1.0))
    pitch = math.degrees(math.asin(float(np.clip(-axis[2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(float(axis[1]), float(axis[0])))
    _ = cos_p  # 保留：cos(pitch)，供后续扩展使用

    # 零滚转时的相机 +Y 方向：Rz(psi) Ry(p) (0,0,-1) = (-sin p cos psi, -sin p sin psi, -cos p)
    psi, p = math.radians(yaw), math.radians(pitch)
    ideal_y = np.array([-math.sin(p) * math.cos(psi),
                        -math.sin(p) * math.sin(psi),
                        -math.cos(p)])
    actual_y = M[:, 1]
    cross = np.cross(ideal_y, actual_y)
    roll = math.degrees(math.atan2(float(np.dot(cross, axis)), float(np.dot(ideal_y, actual_y))))
    return float(cam_pos[0]), float(cam_pos[1]), yaw, pitch, roll, height


def _reproject_rms(obj_pts, img_pts, cam: CameraModel, rvec, tvec) -> float:
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, cam.K, cam.dist.reshape(-1, 1))
    d = np.linalg.norm(proj.reshape(-1, 2) - np.asarray(img_pts, float), axis=1)
    return float(np.sqrt(np.mean(d * d)))


def solve_planar_pose(map_points, pixels, cam: CameraModel,
                      reproj_thresh_px: float = 3.0,
                      min_points: int = 4,
                      require_above_ground: bool = True) -> PlanarPoseResult:
    """用已知场地标志的地图坐标与像素坐标解相机位姿。

    map_points : [(x, y), ...] 地图坐标（米，地面 Z=0）
    pixels     : [(u, v), ...] 对应像素坐标（可含畸变，内参里已含畸变系数）
    """
    res = PlanarPoseResult()
    obj = np.asarray(map_points, float)
    img = np.asarray(pixels, float)
    if obj.ndim != 2 or obj.shape[0] < min_points or obj.shape != (img.shape[0], 2):
        res.status = "insufficient_markers"
        res.n_points = int(obj.shape[0]) if obj.ndim == 2 else 0
        return res

    obj3 = np.hstack([obj, np.zeros((obj.shape[0], 1))]).astype(np.float64)
    img_f = img.astype(np.float64)

    ok, rvec, tvec, inliers = cv2.solvePnPRansac(
        obj3, img_f, cam.K, cam.dist.reshape(-1, 1),
        reprojectionError=reproj_thresh_px, iterationsCount=200,
        confidence=0.995, flags=cv2.SOLVEPNP_IPPE)
    if not ok or inliers is None or len(inliers) < min_points:
        res.status = "not_converged"
        res.n_points = int(obj3.shape[0])
        return res

    rvec, tvec = cv2.solvePnPRefineLM(
        obj3[inliers.ravel()], img_f[inliers.ravel()], cam.K, cam.dist.reshape(-1, 1), rvec, tvec)

    R, _ = cv2.Rodrigues(rvec)
    x, y, yaw, pitch, roll, height = _decompose(R, tvec.reshape(3))
    rms = _reproject_rms(obj3[inliers.ravel()], img_f[inliers.ravel()], cam, rvec, tvec)

    res.x, res.y, res.yaw_deg = x, y, yaw
    res.pitch_deg, res.roll_deg, res.height_m = pitch, roll, height
    res.rms_px = rms
    res.n_points = int(obj3.shape[0])
    res.n_inliers = int(len(inliers))
    res.R, res.t = R, tvec.reshape(3)

    if require_above_ground and height < 0.05:
        # 共面 PnP 的镜像解会把相机放到地面以下，直接判无效
        res.status = "mirror_solution"
        res.valid = False
        return res

    res.valid = True
    res.status = "ok"
    return res


def planar_pose_to_extrinsics(result: PlanarPoseResult,
                              forward_m: float = 0.0,
                              lateral_m: float = 0.0) -> Extrinsics:
    """把 PnP 解出的相机姿态转成项目统一使用的 Extrinsics（相机 -> 载体）。

    若载体参考原点直接取相机光心，则 forward/lateral 保持 0。
    """
    if not result.valid:
        raise ValueError("PnP 位姿无效，不能用于生成外参")
    return Extrinsics.from_params(
        height=result.height_m,
        pitch_deg=result.pitch_deg,
        roll_deg=result.roll_deg,
        yaw_offset_deg=0.0,
        forward_m=forward_m,
        lateral_m=lateral_m,
    )


def check_extrinsics_consistency(result: PlanarPoseResult, tol_deg: float = 0.5) -> dict:
    """自检：把解出的 (高度, 俯角, 滚转, 朝向) 重新组装成外参，看能否还原 PnP 的旋转。

    注意自由度的划分：Extrinsics 只描述「相机相对载体」的安装姿态，
    相机在地图中的绝对朝向属于载体位姿，因此重建时把 result.yaw_deg 作为安装偏角传入，
    这样比较的才是同一件事。
    """
    ext = Extrinsics.from_params(height=result.height_m, pitch_deg=result.pitch_deg,
                                 roll_deg=result.roll_deg, yaw_offset_deg=result.yaw_deg)
    R_expected = ext.R                 # 相机轴在地图中的方向
    R_actual = result.R.T
    cos_angle = (np.trace(R_expected.T @ R_actual) - 1.0) / 2.0
    angle = math.degrees(math.acos(float(np.clip(cos_angle, -1.0, 1.0))))
    return {"rotation_diff_deg": angle, "ok": angle <= tol_deg,
            "height_diff_m": abs(ext.t[2] - result.height_m)}
