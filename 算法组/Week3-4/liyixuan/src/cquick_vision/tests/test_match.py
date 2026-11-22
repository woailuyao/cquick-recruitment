"""合成数据自测：不需要相机 / OpenCV / YOLO，只验证几何与匹配数学。

运行：
    python tests/test_match.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from cvlib.fieldmap import load_field                     # noqa: E402
from cvlib.geom import (                                  # noqa: E402
    invert_pose, ray_to_ground, rotation_from_mount, sigma_from_range,
    transform_points, weighted_se2_ls,
)
from cvlib.matcher import Observation, match              # noqa: E402

CFG = {
    "matching": {
        "pair_len_tol_ratio": 0.10,
        "pair_len_tol_abs_m": 0.05,
        "min_obs_range_m": 0.4,
        "max_obs_range_m": 8.0,
        "gate_m": 0.35,
        "residual_max_m": 0.35,
        "min_inliers": 2,
        "min_inlier_spread_m": 0.8,
        "ambiguity_pose_dist_m": 0.5,
        "ambiguity_yaw_deg": 10.0,
        "ambiguity_residual_ratio": 1.3,
        "tracking_radius_m": 1.0,
        "tracking_yaw_deg": 30.0,
    }
}

FIELD = load_field(os.path.join(ROOT, "config", "field.yaml"),
                   os.path.join(ROOT, "config", "field_geometry.yaml"))
RNG = np.random.default_rng(7)


def visible_obs(pose, max_range=7.0, half_fov_deg=45.0, sigma=0.05, keep=None):
    """给定真实载体位姿，生成"相机能看到"的载体系观测（带噪声）。"""
    x, y, th = pose
    c, s = math.cos(-th), math.sin(-th)      # map -> base: p_b = R(-th)(q - t)
    obs = []
    for p in FIELD.points:
        dx, dy = p.x - x, p.y - y
        bx = c * dx - s * dy
        by = s * dx + c * dy
        r = math.hypot(bx, by)
        if r > max_range or r < 0.4:
            continue
        if abs(math.degrees(math.atan2(by, bx))) > half_fov_deg:
            continue
        nx = bx + RNG.normal(0, sigma)
        ny = by + RNG.normal(0, sigma)
        rr = math.hypot(nx, ny)
        obs.append(Observation(type=p.type, x=nx, y=ny,
                               sigma=sigma_from_range(rr, 1.0, 900.0, 0.40),
                               confidence=0.9))
    if keep is not None:
        obs = obs[:keep]
    return obs


def check(name, cond, detail=""):
    print(("PASS  " if cond else "FAIL  ") + name + ((" | " + detail) if detail else ""))
    if not cond:
        raise SystemExit(1)


def main():
    print(FIELD.summary())
    print("bounds:", FIELD.bounds)

    # ---- 1. 正常场景：多标志 ----
    truth = (3.0, 0.6, math.radians(35.0))
    obs = visible_obs(truth, sigma=0.03)
    print(f"\n[1] 可见标志 {len(obs)} 个: {[o.type for o in obs]}")
    r = match(obs, FIELD.points, CFG)
    err_xy = math.hypot(r.x - truth[0], r.y - truth[1])
    err_yaw = abs((math.degrees(r.theta) - math.degrees(truth[2]) + 180) % 360 - 180)
    print(f"    valid={r.valid} status={r.status} n_in={r.marker_count} "
          f"res={r.residual:.3f} hyp={r.n_hypotheses} "
          f"err={err_xy*100:.1f}cm {err_yaw:.2f}deg")
    check("正常场景定位成功", r.valid and err_xy < 0.20 and err_yaw < 3.0,
          f"err={err_xy*100:.1f}cm/{err_yaw:.2f}deg")

    # ---- 2. 只有一个标志 -> 必须无效 ----
    obs1 = visible_obs(truth, sigma=0.03, keep=1)
    r1 = match(obs1, FIELD.points, CFG)
    print(f"\n[2] 单标志: valid={r1.valid} status={r1.status}")
    check("单标志判无效", (not r1.valid) and r1.status == "insufficient_markers")

    # ---- 3. 混入一个错误观测（模拟误检）----
    obs3 = visible_obs(truth, sigma=0.03)
    obs3.append(Observation(type="TCross", x=4.2, y=1.8, sigma=0.05,
                            confidence=0.4))
    r3 = match(obs3, FIELD.points, CFG)
    e3 = math.hypot(r3.x - truth[0], r3.y - truth[1])
    print(f"\n[3] 含误检: valid={r3.valid} status={r3.status} "
          f"n_in={r3.marker_count} res={r3.residual:.3f} err={e3*100:.1f}cm")
    check("误检不破坏定位", r3.valid and e3 < 0.25, f"err={e3*100:.1f}cm")

    # ---- 4. 无观测 ----
    r4 = match([], FIELD.points, CFG)
    print(f"\n[4] 无观测: valid={r4.valid} status={r4.status}")
    check("无观测判无效", (not r4.valid) and r4.status == "insufficient_markers")

    # ---- 5. 跟踪先验 ----
    r5 = match(obs, FIELD.points, CFG, prior=(3.05, 0.55, math.radians(33.0)))
    e5 = math.hypot(r5.x - truth[0], r5.y - truth[1])
    print(f"\n[5] 带先验: valid={r5.valid} status={r5.status} err={e5*100:.1f}cm")
    check("先验搜索正常", r5.valid and e5 < 0.20)

    # ---- 6. 射线-地面求交 + 误差传播 ----
    R = rotation_from_mount(pitch_down_deg=12.0)
    O = np.array([0.0, 0.0, 0.40])
    K = np.array([[900.0, 0, 640.0], [0, 900.0, 360.0], [0, 0, 1.0]])
    # 光轴方向（像素中心）在 base 下的方向
    from cvlib.geom import pixel_ray
    d_b = R @ pixel_ray(640.0, 360.0, K)
    P, why = ray_to_ground(d_b, O, max_range=8.0)
    print(f"\n[6] 光轴落点: {None if P is None else np.round(P, 3)} why={why}")
    check("光轴求交成功", P is not None and P[2] == 0.0)
    # 朝上的射线必须被拒绝
    up = R @ np.array([0.0, -1.0, 0.0])     # 相机 -y = 上
    P2, why2 = ray_to_ground(up, O)
    print(f"    朝上射线: why={why2}")
    check("朝上射线被拒绝", P2 is None and why2 == "ray_not_downward")

    # 误差传播量级：3 m 与 6 m 处 1 px 的地面误差
    s3 = sigma_from_range(3.0, 1.0, 900.0, 0.40)
    s6 = sigma_from_range(6.0, 1.0, 900.0, 0.40)
    print(f"    sigma(3m)={s3*100:.1f}cm  sigma(6m)={s6*100:.1f}cm  权重比={(s6/s3)**2:.0f}x")
    check("远点误差显著更大", s6 > 3.0 * s3)

    # ---- 7. 加权最小二乘正确性 ----
    true_t = (1.5, -2.0, math.radians(70.0))
    src = RNG.normal(0, 2.0, (6, 2))
    dst = transform_points(true_t[2], np.array(true_t[:2]), src)
    tx, ty, th = weighted_se2_ls(src, dst)
    check("加权 LS 还原位姿",
          abs(tx - true_t[0]) < 1e-9 and abs(ty - true_t[1]) < 1e-9
          and abs(th - true_t[2]) < 1e-9,
          f"({tx:.6f},{ty:.6f},{math.degrees(th):.6f}deg)")

    print("\n全部通过。")


if __name__ == "__main__":
    main()
