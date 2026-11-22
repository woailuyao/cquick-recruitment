"""
像素 -> 载体系地面米制坐标（ground_projection）
=============================================
链路（细则 2.7）：
  1. 像素点 (u,v) 去畸变；
  2. 用内参 K 的逆求相机系射线方向 d_c = K^-1 [u,v,1]^T；
  3. 经外参 R/t 把射线原点和方向转到载体系；
  4. 射线与地面 Z=0 求交：P_b = O_b + λ d_b，λ = -O_bz / d_bz；
  5. 射线朝水平/上方、交点 NaN/inf 或越界时标记无效，不产生伪地面点。
"""
import cv2
import numpy as np


class GroundProjector:
    def __init__(self, intrinsics, extrinsics, field_half=(3.9, 3.0)):
        self.intr = intrinsics
        self.ext = extrinsics
        self.half_x, self.half_y = field_half   # 场地半长/半宽（越界判断）

    def project_one(self, u, v):
        """单个像素 -> 载体系地面点。返回 dict。"""
        # 外参尚未实测时，不能产出可信地面点
        if not self.ext.measured:
            return {"x_b": None, "y_b": None, "valid": False,
                    "reason": "extrinsics_not_measured"}

        # 1. 去畸变 + 归一化：P=None 时输出归一化射线坐标 (xn, yn)
        pts = np.array([[[u, v]]], dtype=np.float64)
        norm = cv2.undistortPoints(pts, self.intr.K, self.intr.dist, None, None)
        xn, yn = norm[0, 0]

        # 2. 相机系射线方向
        d_c = np.array([xn, yn, 1.0])

        # 3. 转到载体系：方向 R·d_c；原点 = 外参平移 t
        R = self.ext.rotation_matrix()
        d_b = R @ d_c
        O_b = self.ext.t

        # 4. 与 Z=0 平面求交
        d_bz = d_b[2]
        if abs(d_bz) < 1e-6:
            return {"x_b": None, "y_b": None, "valid": False,
                    "reason": "ray_horizontal"}

        lam = -O_b[2] / d_bz
        if lam <= 0:
            # 交点在相机后方或射线朝上，不可用
            return {"x_b": None, "y_b": None, "valid": False,
                    "reason": "intersection_behind"}

        p_b = O_b + lam * d_b
        x_b, y_b = float(p_b[0]), float(p_b[1])

        # 5. 越界 / 非有限值检查
        if not (np.isfinite(x_b) and np.isfinite(y_b)):
            return {"x_b": None, "y_b": None, "valid": False,
                    "reason": "non_finite"}
        if abs(x_b) > self.half_x or abs(y_b) > self.half_y:
            return {"x_b": x_b, "y_b": y_b, "valid": False,
                    "reason": "out_of_field"}

        return {"x_b": x_b, "y_b": y_b, "valid": True, "reason": "ok"}

    def project_many(self, landmarks):
        """批量投影。输入 landmark 列表，补 x_b/y_b/valid/reason 字段。"""
        results = []
        for lm in landmarks:
            proj = self.project_one(lm["u"], lm["v"])
            item = dict(lm)
            item.update(proj)
            results.append(item)
        return results
