"""几何库：内参/场地加载、坐标变换、射线-地面求交。纯 numpy + cv2，无重依赖。"""
from pathlib import Path

import cv2
import numpy as np
import yaml

# field.yaml 类别映射：检测类别名 -> 标志点命名前缀/别名
FIELD_CLASSES = {
    "L_cross": ("LCross",),
    "T_cross": ("TCross",),
    "X_cross": ("XCross",),
    "penalty_point": ("PenalityPoint", "PenaltyPoint"),  # 官方少一个 t，做别名
}


def distortion_fold_warning(K, dist):
    """检查畸变多项式在画面全域内是否单调、不变号（折叠 = 灾难）。

    返回 (ok, msg)。r_u→r_d 映射若过零或非单调，undistortPoints 会收敛到
    镜像分支（本项目 2031 近距标定的 k3=-12.5 就踩过这个坑）。
    """
    if dist is None or (len(dist) and not np.any(dist)):
        return True, "无畸变"
    fx = K[0, 0]
    r_max = np.hypot(K[0, 2], K[1, 2]) / fx          # 最远角归一化半径再留 50% 余量
    rs = np.linspace(1e-4, r_max * 1.5, 3000)
    k1, k2 = dist[0], dist[1]
    k3 = dist[4] if len(dist) > 4 else 0.0
    f = 1 + k1 * rs ** 2 + k2 * rs ** 4 + k3 * rs ** 6
    if (f <= 0).any():
        i = int(np.argmax(f <= 0))
        return False, (f"畸变多项式在 r_u={rs[i]:.3f} 处过零（折叠），"
                       f"画面内即可出现镜像点，该组系数不可用")
    rd = rs * f
    if (np.diff(rd) <= 0).any():
        return False, "畸变映射非单调，去畸变求逆可能不唯一"
    return True, f"畸变映射健康（检查域 r_u≤{r_max*1.5:.2f}）"


def load_intrinsics(path):
    """读内参 yaml -> (K 3x3, dist 5,)。兼容 camera_intrinsics*.yaml 全家。"""
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"],
                    dtype=np.float64).reshape(-1)
    return K, dist


def load_field(path):
    """读 field.yaml -> {点名: np.array([x, y])}（米，Z=0）。"""
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    pts = {}
    for k, v in d.items():
        if (isinstance(v, (list, tuple)) and len(v) == 2
                and all(isinstance(x, (int, float)) for x in v)):
            pts[str(k)] = np.array([float(v[0]), float(v[1])], dtype=np.float64)
    if not pts:
        raise ValueError(f"field.yaml 里没读到坐标点：{path}")
    return pts


def field_points_by_class(field_pts):
    """{点名: xy} -> {类别: [(点名, xy), ...]} 供 RANSAC 候选检索。"""
    by_cls = {c: [] for c in FIELD_CLASSES}
    for name, xy in field_pts.items():
        for cls, prefixes in FIELD_CLASSES.items():
            if any(name.startswith(p) for p in prefixes):
                by_cls[cls].append((name, xy))
                break
    return {c: v for c, v in by_cls.items() if v}


def camera_R(yaw_deg, pitch_deg):
    """位姿参数 -> 场地系->相机系旋转矩阵 R_CF（行 = right/down/forward，均按场地系表示）。

    约定：fwd 前向光轴，right = fwd × world_up（注意 +Y 是左，故指向 -Y 侧），
          down = fwd × right，pitch 下俯为正。
    """
    yaw, pitch = np.radians(yaw_deg), np.radians(pitch_deg)
    fwd = np.array([np.cos(pitch) * np.cos(yaw),
                    np.cos(pitch) * np.sin(yaw),
                    -np.sin(pitch)])
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    down = np.cross(fwd, right)
    return np.vstack([right, down, fwd])


def pose_from_Rt(R_CF, tvec):
    """R_CF + t（相机系下光心平移）-> (C_F 光心场地坐标, yaw_deg, pitch_deg)。"""
    C = -R_CF.T @ np.asarray(tvec, dtype=np.float64).reshape(3)
    fwd = R_CF[2, :]
    pitch = np.degrees(np.arcsin(np.clip(-fwd[2], -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(fwd[1], fwd[0]))
    return C, float(yaw), float(pitch)


def project_field_points(K, dist, R_CF, C_F, names_xy):
    """把 [(name, xy)] 的场地点投影到像素 -> {name: (u,v)}。"""
    obj = np.array([[*xy, 0.0] for _, xy in names_xy],
                   dtype=np.float64).reshape(-1, 1, 3)
    rvec, _ = cv2.Rodrigues(R_CF)
    tvec = -R_CF @ np.asarray(C_F, dtype=np.float64).reshape(3)
    uv, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    uv = uv.reshape(-1, 2)
    return {name: (float(uv[i, 0]), float(uv[i, 1]))
            for i, (name, _) in enumerate(names_xy)}


def undistort_ray(K, dist, uvs):
    """像素 -> 去畸变归一化射线方向 (N,3)，z=1。"""
    p = np.asarray(uvs, dtype=np.float64).reshape(-1, 1, 2)
    out = cv2.undistortPoints(p, K, dist).reshape(-1, 2)
    return np.hstack([out, np.ones((len(out), 1))])


def ground_intersect(C_F, d_F):
    """光心 C_F 沿方向 d_F（场地系）与 Z=0 求交 -> (x,y)，射线朝上返回 None。"""
    if d_F[2] >= 0:
        return None
    t = -C_F[2] / d_F[2]
    p = C_F + t * d_F
    return p[:2].copy()


def pixel_to_ground(R_CF, C_F, K, dist, uv, ball_radius=0.0):
    """像素 -> 地面世界坐标 (x,y[, contact])。

    ball_radius>0 时把「球心射线落地交点」换算成「球与地面的接触点」：
    接触点 = 交点 - d_xy * r/|d_z|（球心在地面上方 r 处引起的水平偏移）。
    """
    d_C = undistort_ray(K, dist, uv)[0]          # 相机系归一化方向
    d_F = R_CF.T @ d_C                           # 转到场地系
    p = ground_intersect(C_F, d_F)
    if p is None:
        return None
    if ball_radius > 0 and d_F[2] < 0:
        p = p - d_F[:2] * (ball_radius / abs(d_F[2]))
    return p
