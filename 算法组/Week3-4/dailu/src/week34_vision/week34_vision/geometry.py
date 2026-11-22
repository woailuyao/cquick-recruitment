# -*- coding: utf-8 -*-
"""单目地面几何：像素 <-> 地面(载体坐标系)，以及 SE(2) 位姿求解。

只依赖 numpy，便于单独测试。
坐标系约定（很重要，写代码前先统一）：
  载体坐标系 rig/base：原点在相机正下方的地面上，X 向前，Y 向左，Z 向上；
  相机坐标系 camera(OpenCV)：X 向右，Y 向下，Z 沿光轴向前；
  pitch_deg > 0 表示镜头向下俯视（你这次的安装角 = 30°）。
"""
from __future__ import annotations

import math

import numpy as np


def camera_to_vehicle(pitch_deg: float, roll_deg: float = 0.0, yaw_deg: float = 0.0) -> np.ndarray:
    """相机安装角 -> 旋转矩阵 R_vc（相机系向量 -> 载体系向量）。"""
    p, r, y = (math.radians(a) for a in (pitch_deg, roll_deg, yaw_deg))
    # 光轴在载体系的方向：绕 Z 转 yaw，再向下俯 pitch
    zc = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), -math.sin(p)])
    # 图像右方向（水平，且与光轴垂直）
    right = np.array([math.sin(y), -math.cos(y), 0.0])
    # 图像下方向 = 光轴 x 右方向
    yc = np.cross(zc, right)
    if r:  # roll：绕光轴旋转
        cr, sr = math.cos(r), math.sin(r)
        right, yc = cr * right + sr * yc, -sr * right + cr * yc
    right /= np.linalg.norm(right)
    yc /= np.linalg.norm(yc)
    zc /= np.linalg.norm(zc)
    # 列向量 = 相机三轴在载体系中的表示
    return np.column_stack([right, yc, zc])


class GroundProjector:
    """把去畸变后的像素点投影到地面平面，得到载体坐标系下的 (X, Y)。"""

    def __init__(self, K, height_m: float, pitch_deg: float,
                 roll_deg: float = 0.0, yaw_deg: float = 0.0,
                 max_range_m: float = 20.0, min_range_m: float = 0.0):
        self.K = np.asarray(K, dtype=float).reshape(3, 3)
        self.Kinv = np.linalg.inv(self.K)
        self.height = float(height_m)
        self.R = camera_to_vehicle(pitch_deg, roll_deg, yaw_deg)
        self.C = np.array([0.0, 0.0, self.height])  # 相机光心在载体系的位置
        self.max_range = float(max_range_m)
        self.min_range = float(min_range_m)

    # ---- 像素 -> 地面 -------------------------------------------------
    def ground_from_pixel(self, u: float, v: float):
        """返回 (X, Y, 距离)；射线朝上或超出量程时返回 None。"""
        d_c = self.Kinv @ np.array([u, v, 1.0])   # 相机系下的射线方向
        d_v = self.R @ d_c                        # 转到载体系
        if d_v[2] >= -1e-9:                       # 与地面平行或朝上
            return None
        lam = -self.C[2] / d_v[2]
        if lam <= 0:
            return None
        P = self.C + lam * d_v
        dist = math.hypot(P[0], P[1])
        if dist < self.min_range or dist > self.max_range:
            return None
        return float(P[0]), float(P[1]), float(dist)

    # ---- 地面 -> 像素（画图/自检用） ---------------------------------
    def pixel_from_ground(self, X: float, Y: float):
        P_c = self.R.T @ (np.array([X, Y, 0.0]) - self.C)
        if P_c[2] <= 1e-9:
            return None
        p = self.K @ P_c
        return float(p[0] / p[2]), float(p[1] / p[2])

    # ---- 视场覆盖范围（帮你判断标志有没有进画面） ---------------------
    def ground_band(self, width: int, height: int):
        """返回画面上下边缘对应地面距离，单位米。"""
        out = []
        for v in (0, height - 1):
            d = self.Kinv @ np.array([width / 2.0, v, 1.0])
            dv = self.R @ d
            if dv[2] >= -1e-9:
                out.append(float("inf"))
            else:
                lam = -self.C[2] / dv[2]
                P = self.C + lam * dv
                out.append(float(math.hypot(P[0], P[1])))
        return out[0], out[1]


def fit_se2(obs_xy: np.ndarray, map_xy: np.ndarray, weights=None):
    """求解 2D 刚体变换 map ≈ R(θ)·obs + t（不含缩放）。

    weights 可选：每个点的权重。远处标志的地面坐标误差大，
    给它较小权重，但仍参与求解（不会因为"太远"而丢掉一个观测）。
    返回 (theta_rad, t(2,), residual_m, rms)。
    """
    obs = np.asarray(obs_xy, dtype=float).reshape(-1, 2)
    mp = np.asarray(map_xy, dtype=float).reshape(-1, 2)
    if len(obs) != len(mp) or len(obs) < 2:
        raise ValueError("至少需要 2 组观测/地图点，且数量相同")
    if weights is None:
        w = np.ones(len(obs))
    else:
        w = np.asarray(weights, dtype=float).reshape(-1)
        if len(w) != len(obs) or w.sum() <= 0:
            w = np.ones(len(obs))
    w = w / w.sum()
    oc = (obs * w[:, None]).sum(axis=0)
    mc = (mp * w[:, None]).sum(axis=0)
    H = ((obs - oc) * w[:, None]).T @ (mp - mc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    t = mc - R @ oc
    pred = (R @ obs.T).T + t
    err = np.linalg.norm(pred - mp, axis=1)
    theta = math.atan2(R[1, 0], R[0, 0])
    return theta, t, float(err.mean()), float(np.sqrt((err ** 2).mean()))


def _canon_type(t) -> str:
    """把各种写法归一到 L / T / X / P。

    'L' / 'LCross' / 'l_cross' -> 'L'；'PenalityPoint' / 'PenaltyPoint' -> 'P'
    """
    up = str(t).strip().upper()
    if up.startswith("PENAL"):
        return "P"
    for k in ("L", "T", "X"):
        if up.startswith(k):
            return k
    return up


def solve_pose_with_matching(observations, field_points, max_residual_m=0.6,
                             max_range_m=6.0, max_combos=20000,
                             max_ambiguity_margin=0.20, max_ambiguity_pose=0.5,
                             max_ambiguity_yaw=15.0,
                             include_ambiguous_candidates=False):
    """带同类标志歧义处理的位姿求解。

    observations: [{'type': 'L'/'T'/'X', 'x': ..., 'y': ...}, ...]（载体坐标系）
    field_points: [{'type': 'LCROSS'/'TCROSS'/... , 'x': ..., 'y': ...}]（地图坐标系）
    返回 dict：valid/status/x/y/yaw_deg/marker_count/residual/assignments

    注意：两个入参的标志类型写法可以不同（L 与 LCross），内部会归一化。
    """
    import itertools

    obs_by_type = {}
    for o in observations:
        obs_by_type.setdefault(_canon_type(o["type"]), []).append(o)
    map_by_type = {}
    for fp in field_points:
        map_by_type.setdefault(_canon_type(fp["type"]), []).append(fp)

    result = {"valid": False, "status": "insufficient_markers", "x": None, "y": None,
              "yaw_deg": None, "marker_count": len(observations), "residual": None,
              "assignments": []}

    # 每种类型、观测数 n 需要从地图同类点里挑 n 个（排列，因为有身份歧义）
    per_type_choices = {}
    total = 1
    for t, obs_list in obs_by_type.items():
        cand = map_by_type.get(t, [])
        k = len(obs_list)
        if len(cand) < k:
            result["status"] = "insufficient_markers"
            return result
        perms = list(itertools.permutations(range(len(cand)), k))
        per_type_choices[t] = perms
        total *= len(perms)
    if total > max_combos:
        # 组合太多时只按顺序对齐（一般自己场地不会走到这里）
        per_type_choices = {t: [tuple(range(len(v[0])))] for t, v in per_type_choices.items()}

    sols = []
    for combo in itertools.product(*[per_type_choices[t] for t in per_type_choices]):
        obs_list, map_list, adm = [], [], []
        for t, idxs in zip(per_type_choices.keys(), combo):
            for o, j in zip(obs_by_type[t], idxs):
                cand = map_by_type[t][j]
                if math.hypot(o["x"], o["y"]) > max_range_m:
                    continue
                obs_list.append([o["x"], o["y"]])
                map_list.append([cand["x"], cand["y"]])
                adm.append({"type": t, "obs": o, "map": cand})
        if len(obs_list) < 2:
            continue
        try:
            # 权重按观测距离衰减：1 m ≈ 0.8，2 m ≈ 0.5，3 m ≈ 0.31
            # 远处的点仍然参与求解，只是不让它主导位姿
            ws = [1.0 / (1.0 + (math.hypot(o[0], o[1]) / 2.0) ** 2) for o in obs_list]
            theta, tr, mean_err, rms = fit_se2(obs_list, map_list, weights=ws)
        except ValueError:
            continue
        sols.append((float(rms), theta, tr, mean_err, adm))

    if not sols:
        result["status"] = "insufficient_markers"
        return result

    sols.sort(key=lambda s: s[0])
    rms, theta, tr, mean_err, adm = sols[0]
    result.update({
        "x": float(tr[0]), "y": float(tr[1]),
        "yaw_deg": float(math.degrees(theta) % 360.0),
        "residual": float(rms),
        "marker_count": len(adm),
        "assignments": adm,
    })

    # —— 对称歧义判定 ——
    # 场地左右近似对称（L1/T5 与 L2/T6 关于 x≈2.99 镜像），只有 2 个标志时
    # 往往存在一个"镜像解"残差几乎一样。这里检查次优解：
    #   残差差距 < amb_margin_m 且 位姿差 > amb_pose_m 或 > amb_yaw_deg -> ambiguous
    if len(sols) > 1:
        rms2, theta2, tr2, _, adm2 = sols[1]
        result["second_residual"] = float(rms2)
        dx = float(math.hypot(tr2[0] - tr[0], tr2[1] - tr[1]))
        dyaw = abs((math.degrees(theta2 - theta) + 180) % 360 - 180)
        result["second_dx"] = dx
        result["second_dyaw"] = float(dyaw)
        # 判据（相对门槛）：
        #   门槛 = max(0.06, 0.5 × 最优残差)
        # 即：最优解本身很准时（残差小），允许与次优解差距较小；
        #     最优解本身很差时（残差大），要求次优解明显更差才算唯一确定。
        gap = rms2 - rms
        thr = max(0.06, 0.5 * rms)
        ambiguous = (gap < thr and
                     (dx > max_ambiguity_pose or dyaw > max_ambiguity_yaw))
        result["ambiguity_gap"] = float(gap)
        result["ambiguity_threshold"] = float(thr)
        if rms <= max_residual_m and ambiguous:
            result["status"] = "ambiguous"
            if include_ambiguous_candidates:
                result["candidates"] = [
                    {"x": float(s[2][0]), "y": float(s[2][1]),
                     "yaw_deg": float(math.degrees(s[1]) % 360.0),
                     "residual": float(s[0]),
                     "assignments": s[4]}
                    for s in sols[:5]
                ]
            return result

    if rms > max_residual_m:
        result["status"] = "high_residual"
    else:
        result["valid"] = True
        result["status"] = "ok"
    return result
