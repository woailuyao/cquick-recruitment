"""端到端几何自测（不需要相机/OpenCV）：地面点 -> 像素 -> 反向投影 -> 匹配。

这一步专门用来抓坐标约定错误（Y 轴上下翻转、yaw 正方向、相机 x/y/z 轴向）。
运行：
    python tests/test_pipeline.py
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from cvlib.fieldmap import load_field                                  # noqa: E402
from cvlib.geom import (                                               # noqa: E402
    pixel_ray, ray_to_ground, rotation_from_mount, sigma_from_range,
)
from cvlib.matcher import Observation, match                           # noqa: E402

CFG = {"matching": {
    "pair_len_tol_ratio": 0.10, "pair_len_tol_abs_m": 0.05,
    "min_obs_range_m": 0.4, "max_obs_range_m": 8.0, "gate_m": 0.35,
    "residual_max_m": 0.35, "min_inliers": 2, "min_inlier_spread_m": 0.8,
    "ambiguity_pose_dist_m": 0.5, "ambiguity_yaw_deg": 10.0,
    "ambiguity_residual_ratio": 1.3, "tracking_radius_m": 1.0,
    "tracking_yaw_deg": 30.0,
}}

W, H = 1280, 720
K = np.array([[900.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]])
FIELD = load_field(os.path.join(ROOT, "config", "field.yaml"),
                   os.path.join(ROOT, "config", "field_geometry.yaml"))
RNG = np.random.default_rng(11)


def project(P_b, R, t):
    """载体系地面点 -> 像素（与 pixel_ray 互为逆运算）。"""
    d_c = R.T @ (np.asarray(P_b, float) - t)
    if d_c[2] <= 1e-6:
        return None
    u = K[0, 0] * d_c[0] / d_c[2] + K[0, 2]
    v = K[1, 1] * d_c[1] / d_c[2] + K[1, 2]
    if not (0 <= u < W and 0 <= v < H):
        return None
    return float(u), float(v)


def build_observations(truth, R, t, noise_px=0.0, max_range=7.0):
    x, y, th = truth
    c, s = math.cos(-th), math.sin(-th)
    obs, details = [], []
    for p in FIELD.points:
        dx, dy = p.x - x, p.y - y
        bx, by = c * dx - s * dy, s * dx + c * dy
        D = math.hypot(bx, by)
        if D > max_range or bx <= 0.2 or abs(math.degrees(math.atan2(by, bx))) > 42.0:
            continue
        uv = project((bx, by, 0.0), R, t)
        if uv is None:
            continue
        u = uv[0] + RNG.normal(0, noise_px)
        v = uv[1] + RNG.normal(0, noise_px)
        d_b = R @ pixel_ray(u, v, K)
        P, why = ray_to_ground(d_b, t, max_range=max_range + 1.0)
        if P is None:
            details.append((p.mid, "ray_fail:" + str(why)))
            continue
        sigma = sigma_from_range(float(math.hypot(P[0], P[1])), 1.0, K[1, 1], float(t[2]))
        obs.append(Observation(type=p.type, x=float(P[0]), y=float(P[1]),
                               sigma=sigma, confidence=0.9, pixel=(u, v)))
        details.append((p.mid, f"base=({P[0]:.2f},{P[1]:.2f}) D={math.hypot(P[0],P[1]):.2f}"))
    return obs, details


def check(name, cond, detail=""):
    print(("PASS  " if cond else "FAIL  ") + name + ((" | " + detail) if detail else ""))
    if not cond:
        raise SystemExit(1)


def run_case(name, truth, R, t, noise_px, tol_xy, tol_yaw):
    obs, details = build_observations(truth, R, t, noise_px=noise_px)
    print(f"\n[{name}] truth=({truth[0]:.2f},{truth[1]:.2f},"
          f"{math.degrees(truth[2]):.1f}deg) 观测 {len(obs)} 个")
    for mid, d in details:
        print(f"    {mid:14s} {d}")
    r = match(obs, FIELD.points, CFG)
    exy = math.hypot(r.x - truth[0], r.y - truth[1]) if r.valid else float("inf")
    eyaw = abs((math.degrees(r.theta) - math.degrees(truth[2]) + 180) % 360 - 180) \
        if r.valid else float("inf")
    print(f"    -> valid={r.valid} status={r.status} inl={r.marker_count} "
          f"res={r.residual:.3f} err={exy*100:.1f}cm {eyaw:.2f}deg")
    check(name, r.valid and exy < tol_xy and eyaw < tol_yaw,
          f"err={exy*100:.1f}cm/{eyaw:.2f}deg")
    return r


def main():
    R = rotation_from_mount(pitch_down_deg=15.0)
    t = np.array([0.03, -0.01, 0.42])

    # 1. 无噪声：应几乎精确复原
    run_case("无噪声 正前方", (3.0, 0.6, math.radians(35.0)), R, t, 0.0, 0.02, 0.3)

    # 2. 1 px 像素噪声：远点误差放大，验证加权有效
    run_case("1px 噪声", (3.0, 0.6, math.radians(35.0)), R, t, 1.0, 0.50, 5.0)

    # 3. 朝 +Y 方向（yaw=90）：用来抓 Y 轴上下翻转
    run_case("朝向 +Y", (2.0, -1.5, math.radians(90.0)), R, t, 0.0, 0.05, 0.5)

    # 4. 朝 -X 方向（yaw≈190，跨 180 度边界）
    run_case("朝向 -X", (6.9, 2.4, math.radians(190.0)), R, t, 0.0, 0.05, 0.5)

    # 5. 朝 +Y 时，看到的地面点必须落在 base 的 +x（不是 -x）
    obs, _ = build_observations((2.0, -1.5, math.radians(90.0)), R, t)
    check("前方观测在 base +x 侧", all(o.x > 0 for o in obs),
          f"x={[round(o.x,2) for o in obs]}")

    print("\n全部通过：坐标约定一致。")


if __name__ == "__main__":
    main()
