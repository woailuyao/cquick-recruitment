# -*- coding: utf-8 -*-
"""任务 2 几何链路离线自测：只用 numpy + 合成数字，不需要任何照片或视频。

覆盖 4 段公式，全部可在没相机的现在验证符号是否正确:
  1. 像素 -> 归一化相机射线 (K 的逆)
  2. 相机射线 -> 载体/测试架坐标系 (外参 R)
  3. 射线与地面平面 Z=0 求交，得到米制地面坐标
  4. 多个地面观测点 <-> field.yaml 地图点的 SE(2) 位姿拟合 (Kabsch)

运行: python geometry_selftest.py
预期: 正投影/反投影误差 < 1e-9 m；加 3 cm 噪声后位姿误差在厘米/度级。

注意: 这里的 K / 高度 / 俯仰角都是占位示例值，
      实拍后必须换成你自己的 camera_intrinsics.yaml 与外参。
"""
import numpy as np

RNG = np.random.default_rng(20261114)

# ---- 示例参数（占位，实拍标定后必须替换） ----
K = np.array([[1000.0, 0.0, 640.0],
              [0.0, 1000.0, 360.0],
              [0.0, 0.0, 1.0]])
K_INV = np.linalg.inv(K)


def camera_to_body(pitch_deg):
    """相机系 -> 载体系的外参旋转。

    约定: 载体系 X 右 / Y 前 / Z 上；相机系 x 右 / y 图像向下 / z 光轴向前。
    pitch 为光轴相对水平面下俯的角度。pitch=0 时光轴水平朝 +Y，
    pitch=60 时光轴朝前下方，画面中心落在身前约 0.7 m 的地面上。
    """
    t = np.deg2rad(pitch_deg)
    s, c = np.sin(t), np.cos(t)
    return np.array([[1.0, 0.0, 0.0],
                     [0.0, -s, c],
                     [0.0, -c, -s]])


# 相机向下俯 60 度，相机光心离地 1.2 m
R_CAM_TO_BODY = camera_to_body(60.0)
O_BODY = np.array([0.0, 0.0, 1.2])   # 相机光心在载体系中的位置


def ground_to_pixel(point_ground_xy, K_mat=K, R=R_CAM_TO_BODY, O=O_BODY):
    """地面点(载体系 XY, Z=0) -> 像素。用于生成合成观测。"""
    p_body = np.array([point_ground_xy[0], point_ground_xy[1], 0.0])
    p_cam = R.T @ (p_body - O)
    if p_cam[2] <= 0:
        return None
    uvw = K_mat @ (p_cam / p_cam[2])
    return uvw[:2]


def pixel_to_ground(u, v, K_inv=K_INV, R=R_CAM_TO_BODY, O=O_BODY,
                    max_range=30.0):
    """像素 -> 地面点。返回 (x, y, status)，status 非 OK 时不得使用结果。

    说明: 这里假设像素已先做过去畸变（cv2.undistortPoints）。
          实拍后请把 K_inv 换成你自己的标定内参。
    """
    d_cam = K_inv @ np.array([u, v, 1.0])
    d_body = R @ d_cam
    if d_body[2] >= -1e-9:
        return None, None, "ray_not_downward"     # 射线不与地面相交
    lam = -O[2] / d_body[2]
    if lam <= 0:
        return None, None, "negative_scale"
    p = O + lam * d_body
    if abs(p[2]) > 1e-6:
        return None, None, "plane_mismatch"
    if np.hypot(p[0], p[1]) > max_range:
        return None, None, "out_of_field"
    return p[0], p[1], "OK"


def fit_se2(observed_body, map_points):
    """从 >=3 个非共线对应点求 body -> map 的刚体变换 (Kabsch 2D)。"""
    if len(observed_body) < 3 or len(observed_body) != len(map_points):
        return None, None, None, "insufficient_markers"
    obs = np.asarray(observed_body, dtype=float)
    mp = np.asarray(map_points, dtype=float)
    c_obs, c_map = obs.mean(axis=0), mp.mean(axis=0)
    H = (obs - c_obs).T @ (mp - c_map)
    U, S, Vt = np.linalg.svd(H)
    Rm = Vt.T @ U.T
    if np.linalg.det(Rm) < 0:                    # 防止出现镜像反射
        Vt[-1, :] *= -1
        Rm = Vt.T @ U.T
    if S[-1] < 1e-6:
        return None, None, None, "degenerate_collinear"
    t = c_map - Rm @ c_obs
    yaw = float(np.arctan2(Rm[1, 0], Rm[0, 0]))
    residual = float(np.mean(np.linalg.norm((Rm @ obs.T).T + t - mp, axis=1)))
    return t, yaw, residual, "OK"


def main():
    print("=== 1. 正投影 / 反投影一致性 ===")
    samples = np.array([[0.3, 0.7], [-0.4, 1.0], [0.5, 0.9], [0.0, 1.2]])
    max_err = 0.0
    for p in samples:
        uv = ground_to_pixel(p)
        assert uv is not None, "合成点在相机后方，检查俯仰角与高度"
        x, y, st = pixel_to_ground(uv[0], uv[1])
        assert st == "OK", st
        err = np.hypot(x - p[0], y - p[1])
        max_err = max(max_err, err)
        print("  ground({:5.2f},{:5.2f}) -> px({:7.2f},{:7.2f}) -> "
              "ground({:5.2f},{:5.2f})  err={:.2e} m".format(
                  p[0], p[1], uv[0], uv[1], x, y, err))
    print("  最大往返误差 = {:.2e} m  -> {}\n".format(
        max_err, "PASS" if max_err < 1e-8 else "FAIL"))

    print("=== 2. 射线不与地面相交时必须判无效（不得伪造位姿）===")
    x, y, st = pixel_to_ground(640, 360, K_inv=K_INV, R=np.eye(3), O=O_BODY)
    print("  平视相机 px(640,360) -> {}".format(st))
    assert st == "ray_not_downward", st
    t, yaw, res, st2 = fit_se2([[0.0, 0.0]], [[1.0, 1.0]])
    print("  只有 1 个特征点 -> {}".format(st2))
    assert st2 == "insufficient_markers"
    print("  -> PASS\n")

    print("=== 3. 用合成场地点 + 噪声估计地图位姿 ===")
    x_true, y_true, yaw_true = 1.50, -0.80, np.deg2rad(25.0)
    c, s = np.cos(yaw_true), np.sin(yaw_true)
    R_BODY_TO_MAP = np.array([[c, -s], [s, c]])
    t_true = np.array([x_true, y_true])

    # 6 个不共线的 L/T/X / 点球点，模拟 field.yaml 里的米制坐标
    map_points = np.array([
        [0.0, 0.0], [0.0, 3.0], [3.0, 0.0],
        [3.0, 3.0], [1.5, 1.5], [-1.0, 2.0],
    ], dtype=float)
    # 地图点 -> 载体系观测（真实位姿的逆变换）
    body_points = (R_BODY_TO_MAP.T @ (map_points - t_true).T).T
    noisy = body_points + RNG.normal(0.0, 0.03, body_points.shape)

    t_est, yaw_est, residual, status = fit_se2(noisy, map_points)
    assert status == "OK", status
    pos_err = float(np.linalg.norm(t_est - t_true))
    yaw_err = abs(np.rad2deg(np.arctan2(np.sin(yaw_est - yaw_true),
                                         np.cos(yaw_est - yaw_true))))
    print("  真实 : x={:.3f} m  y={:.3f} m  yaw={:.2f} deg".format(
        x_true, y_true, np.rad2deg(yaw_true)))
    print("  估计 : x={:.3f} m  y={:.3f} m  yaw={:.2f} deg".format(
        t_est[0], t_est[1], np.rad2deg(yaw_est)))
    print("  位置误差={:.4f} m  航向误差={:.3f} deg  平均残差={:.4f} m".format(
        pos_err, yaw_err, residual))
    print("  现场参考阈值: 位置 <= 0.5 m, 航向 <= 10 deg  -> {}\n".format(
        "PASS" if pos_err <= 0.5 and yaw_err <= 10 else "FAIL"))

    print("=== 4. 特征不足 / 共线 -> valid=false ===")
    print("  2 个点:", fit_se2(noisy[:2], map_points[:2])[3])
    collinear = np.array([[0., 0.], [1., 0.], [2., 0.], [3., 0.]], dtype=float)
    print("  4 个共线点:", fit_se2(collinear, map_points[:4])[3])
    print("  -> PASS")


if __name__ == "__main__":
    main()
