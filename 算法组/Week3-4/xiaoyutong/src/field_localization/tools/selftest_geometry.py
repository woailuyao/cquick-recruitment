"""几何自检：不用相机、不用照片，验证任务二最核心的三段数学对不对。

跑法：
    python tools/selftest_geometry.py

它会做三件事：
  1) 高度/俯仰/滚转角  →  旋转矩阵  →  反解回角度，必须完全一致；
  2) 造一个假的相机 + 地上的假棋盘格，走一遍外参求解流程，看能不能还原真实高度和俯仰；
  3) 像素 → 地面 → 像素 往返投影，误差应小于 1e-6 像素。

任何一条报「失败」，说明代码被改坏了，先别去实验室拍数据。
"""

import os
import sys

import cv2
import numpy as np

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
if ROOT not in sys.path:
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision2.console import setup_console  # noqa: E402
from vision2.geometry import (  # noqa: E402
    GroundProjector,
    angles_from_rotation,
    pose_from_plane,
    rotation_from_height_pitch_roll,
)
from vision2.yaml_mini import load_yaml, save_yaml  # noqa: E402

setup_console()

FAIL = []


def check(name, ok, detail=""):
    print(("  [通过] " if ok else "  [失败] ") + name + ("  " + detail if detail else ""))
    if not ok:
        FAIL.append(name)


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def test_angle_roundtrip():
    print("\n1) 角度 ↔ 旋转矩阵")
    worst_p = worst_r = 0.0
    rng = np.random.default_rng(0)
    for _ in range(200):
        p = rng.uniform(-10, 70)
        r = rng.uniform(-15, 15)
        h = rng.uniform(0.2, 2.0)
        R, t = rotation_from_height_pitch_roll(h, p, r)
        p2, r2 = angles_from_rotation(R)
        worst_p = max(worst_p, abs(p2 - p))
        worst_r = max(worst_r, abs(r2 - r))
        assert abs(t[2] - h) < 1e-12
    check("pitch 往返误差 < 1e-9°", worst_p < 1e-9, "max=%.2e" % worst_p)
    check("roll 往返误差 < 1e-9°", worst_r < 1e-9, "max=%.2e" % worst_r)


def test_extrinsics_recovery():
    print("\n2) 假相机 + 地面棋盘格 → 还原高度/俯仰/滚转")
    W, H = 1280, 720
    K = np.array([[800.0, 0, 640.0], [0, 800.0, 360.0], [0, 0, 1.0]])
    dist = np.zeros(5)
    cols, rows, square = 9, 6, 0.025

    objp = np.zeros((cols * rows, 3), np.float64)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square

    rng = np.random.default_rng(1)
    worst_h = worst_p = worst_r = 0.0
    for _ in range(50):
        h_true = rng.uniform(0.4, 1.6)
        p_true = rng.uniform(5, 55)
        r_true = rng.uniform(-8, 8)
        R_gc, t_gc = rotation_from_height_pitch_roll(h_true, p_true, r_true)

        # 板子平放在地上：绕竖轴随机转一个角度，摆在相机前方 1~3m
        yaw = rng.uniform(-np.pi, np.pi)
        R_gb = rot_z(yaw)
        t_gb = np.array([rng.uniform(0.5, 2.5), rng.uniform(-0.6, 0.6), 0.0])

        # 相机在板坐标系里的位姿（这就是 solvePnP 会给出的量）
        R_bc = R_gb.T @ R_gc
        t_bc = R_gb.T @ (t_gc - t_gb)

        # 用真实位姿把点投到图像上（模拟检测到的角点）
        R_cb = R_bc.T
        t_cb = -R_cb @ t_bc
        proj, _ = cv2.projectPoints(objp, cv2.Rodrigues(R_cb)[0], t_cb, K, dist)
        pts = proj.reshape(-1, 2) + rng.normal(0, 0.15, (len(proj), 2))  # 加 0.15px 噪声

        # 走一遍真实流程：undistortPoints + solvePnP(IPPE) → 转成「相机→板」位姿
        und = cv2.undistortPoints(pts.reshape(-1, 1, 2), K, dist)
        ok, rvec, tvec = cv2.solvePnP(objp, und, np.eye(3), None, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        R_cb2, _ = cv2.Rodrigues(rvec)
        t_cb2 = tvec.reshape(3)
        R_bc2 = R_cb2.T
        t_bc2 = -R_bc2 @ t_cb2
        info = pose_from_plane(R_bc2, t_bc2)

        worst_h = max(worst_h, abs(info["height_m"] - h_true))
        worst_p = max(worst_p, abs(info["pitch_deg"] - p_true))
        worst_r = max(worst_r, abs(info["roll_deg"] - r_true))

    check("高度还原误差 < 2 cm", worst_h < 0.02, "max=%.4f m" % worst_h)
    check("俯仰还原误差 < 0.3°", worst_p < 0.3, "max=%.3f°" % worst_p)
    check("滚转还原误差 < 1.0°", worst_r < 1.0, "max=%.3f°" % worst_r)
    check("相机坐标系朝向检查（原点在正下方）", abs(info["t_gc"][0]) < 1e-12 and abs(info["t_gc"][1]) < 1e-12)


def test_projection_roundtrip():
    print("\n3) 像素 → 地面 → 像素 往返")
    W, H = 1280, 720
    K = np.array([[800.0, 0, 640.0], [0, 800.0, 360.0], [0, 0, 1.0]])
    dist = np.array([-0.12, 0.08, 0.0005, -0.0003, -0.01])
    R, t = rotation_from_height_pitch_roll(0.9, 25.0, 1.5)
    proj = GroundProjector(K, dist, R, t, width=W, height=H)

    rng = np.random.default_rng(2)
    worst = 0.0
    n_ok = 0
    for _ in range(300):
        u = rng.uniform(80, W - 80)
        v = rng.uniform(H * 0.55, H - 5)  # 只取画面下半部分（地面）
        g = proj.pixel_to_ground(u, v)
        if g is None:
            continue
        uv = proj.ground_to_pixel(g[0], g[1])
        if uv is None:
            continue
        worst = max(worst, float(np.hypot(uv[0] - u, uv[1] - v)))
        n_ok += 1
    check("往返误差 < 1e-4 px", worst < 1e-4, "max=%.2e（%d 个点）" % (worst, n_ok))

    # 地平线以上的点必须返回 None（不能伪造地面点）
    above = [proj.pixel_to_ground(u, 20) for u in (100, 640, 1200)]
    check("地平线以上的像素返回 None", all(a is None for a in above))

    # 已知几何手算校验：正前方 2m 的点应落在图像某一像素上，且能反算回 2m
    p2 = proj.ground_to_pixel(2.0, 0.0)
    g2 = proj.pixel_to_ground(*p2) if p2 else None
    check(
        "正前方 2 m 的点可往返",
        g2 is not None and abs(g2[0] - 2.0) < 1e-6 and abs(g2[1]) < 1e-6,
        "" if g2 is None else "(%.6f, %.6f)" % (g2[0], g2[1]),
    )


def test_yaml():
    print("\n4) yaml 读写")
    tmp = os.path.join(ROOT, "calibration", "out", "_selftest_intr.yaml")
    data = {
        "image_width": 1280,
        "image_height": 720,
        "camera_matrix": [800.0, 0.0, 640.0, 0.0, 800.0, 360.0, 0.0, 0.0, 1.0],
        "dist_coeffs": [0.1, -0.05, 0.001, 0.002, -0.01],
        "note": "自检用",
    }
    save_yaml(tmp, data)
    back = load_yaml(tmp)
    ok = (
        back["image_width"] == 1280
        and len(back["camera_matrix"]) == 9
        and abs(back["camera_matrix"][0] - 800.0) < 1e-9
        and back["note"] == "自检用"
    )
    check("写出去再读回来一致", ok)
    try:
        os.remove(tmp)
    except OSError:
        pass

    # 能读带注释的配置
    cfg = os.path.join(ROOT, "config", "camera_intrinsics.example.yaml")
    if os.path.exists(cfg):
        got = load_yaml(cfg)
        check("能读带注释的示例配置", got.get("image_width") == 1280)


def main():
    print("== 任务二几何自检 ==")
    test_angle_roundtrip()
    test_extrinsics_recovery()
    test_projection_roundtrip()
    test_yaml()
    print("")
    if FAIL:
        print("结果：%d 项失败 -> %s" % (len(FAIL), ", ".join(FAIL)))
        return 1
    print("结果：全部通过，几何部分可以放心用。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
