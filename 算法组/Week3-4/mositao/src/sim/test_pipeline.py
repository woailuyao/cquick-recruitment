# -*- coding: utf-8 -*-
"""仿真自检：在**没有相机、没有场地、没有 field.yaml 真文件**的情况下，
先证明「像素 -> 地面 -> 地图匹配 -> 二维位姿」这条链路在数学上是通的。

用例：
  A  几何往返一致性：地面点 -> 像素 -> 地面点，误差应接近机器精度（限定在视场内的点）
  A2 地图文件解析：读取 config/field.sample.yaml 与 field_geometry.sample.yaml
  B  随机位姿定位精度（像素噪声 + 遮挡 + 离群误检）
  B2 理想条件上限（无噪声无遮挡）
  B3 穷举假设 vs 随机 RANSAC 假设的对比（对应细则加分项）
  C  完全对称场地下应输出 ambiguous
  C2 只给 2 个标志时应输出 insufficient_markers
  C3 含离群误检时的稳健性
  C4 用上一帧位姿作先验时，可消掉对称场地 180° 翻转歧义

运行（项目根目录）：
    python src/sim/test_pipeline.py
    python src/sim/test_pipeline.py --trials 300 --noise 2.0
"""

from __future__ import annotations

import argparse
import math
import platform
import statistics
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

# 允许直接以 `python src/sim/test_pipeline.py` 运行
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.common.paths import OUTPUT_DIR, ensure_runtime_dirs, rel          # noqa: E402
from src.loc.camera_model import (                                          # noqa: E402
    CameraModel, Extrinsics, ground_to_pixels, pixels_to_ground,
)
from src.loc.field_map import FieldMap, FieldMarker                         # noqa: E402
from src.loc.matcher import (                                               # noqa: E402
    MatchParams, ObservedMarker, locate, rot2, wrap_pi,
)
from src.loc.planar_pose import solve_planar_pose                           # noqa: E402

# ----------------------------------------------------------------------------
# 虚拟场地：与 config/field.yaml 一致（单球门、14 个标志点）
# 单球门使罚球区只出现在 +X 一端，布局天然非对称 -> 180° 翻转解不成立
# ----------------------------------------------------------------------------
VIRTUAL_FIELD = [
    # 场地四角
    ("LCross", -7.00, -4.50), ("LCross", -7.00, 4.50),
    ("LCross", 7.00, -4.50), ("LCross", 7.00, 4.50),
    # 中线与边线交点
    ("TCross", 0.00, -4.50), ("TCross", 0.00, 4.50),
    # 球门线与罚球区两侧线交点（仅 +X 一端）
    ("TCross", 7.00, -1.50), ("TCross", 7.00, 1.50),
    # 罚球区前沿两角（仅 +X 一端）
    ("LCross", 5.50, -2.00), ("LCross", 5.50, 2.00),
    # 场地中心与中圈交点
    ("XCross", 0.00, 0.00), ("XCross", 0.00, -1.50), ("XCross", 0.00, 1.50),
    # 点球点（仅 +X 一端）
    ("PenaltyPoint", 5.00, 0.00),
]

# 完全对称的迷你场地：绕原点旋转 180° 与自身重合，用于验证歧义识别
SYMMETRIC_FIELD = [
    ("TCross", -3.00, 0.00), ("TCross", 3.00, 0.00), ("XCross", 0.00, 0.00),
]

CAM_HEIGHT_M = 0.50
CAM_PITCH_DEG = 22.0
IMAGE_W, IMAGE_H = 1280, 720
FOCAL_PX = 650.0          # 约 89° 水平视场，接近常见广角 USB 相机

# 真实量级的畸变系数：只在小半径内有效，不会在视场外把点「折叠」回画面
DIST_COEFFS = (-0.15, 0.05, 0.0008, -0.0004, -0.008)


def make_camera() -> CameraModel:
    return CameraModel(fx=FOCAL_PX, fy=FOCAL_PX, cx=IMAGE_W / 2, cy=IMAGE_H / 2,
                       dist=DIST_COEFFS, width=IMAGE_W, height=IMAGE_H)


def make_extrinsics() -> Extrinsics:
    return Extrinsics.from_params(height=CAM_HEIGHT_M, pitch_deg=CAM_PITCH_DEG)


def build_map(spec) -> FieldMap:
    return FieldMap(markers=[FieldMarker(t, x, y, name=f"sim_{i}")
                             for i, (t, x, y) in enumerate(spec)])


def map_to_base(px: float, py: float, pose) -> np.ndarray:
    """地图点 -> 载体坐标系（与 P_map = R(theta) P_base + t 互逆）。"""
    x, y, yaw = pose
    return rot2(-yaw) @ (np.array([px, py]) - np.array([x, y]))


# ----------------------------------------------------------------------------
# A. 几何往返一致性
# ----------------------------------------------------------------------------
def test_roundtrip(cam, ext, log) -> float:
    log("A. 几何往返一致性（地面点 -> 像素 -> 地面点；仅统计落在画面内的点）")
    grid = [(gx, gy) for gx in np.arange(1.0, 12.1, 0.5) for gy in np.arange(-4.0, 4.1, 0.5)]
    pts = np.array([(gx, gy, 0.0) for gx, gy in grid])
    uv, ok_fwd = ground_to_pixels(cam, ext, pts)

    inside = ok_fwd.copy()
    inside &= (uv[:, 0] >= 0) & (uv[:, 0] < IMAGE_W) & (uv[:, 1] >= 0) & (uv[:, 1] < IMAGE_H)
    pts_in, uv_in = pts[inside], uv[inside]

    back, ok_back = pixels_to_ground(cam, ext, uv_in)
    err = np.full(len(pts_in), np.nan)
    err[ok_back] = np.linalg.norm(back[ok_back][:, :2] - pts_in[ok_back][:, :2], axis=1)
    worst = float(np.nanmax(err)) if np.isfinite(err).any() else float("nan")
    log(f"   参与往返的地面点: {int(inside.sum())} 个（画面内且可反投影）")
    log(f"   最大往返误差   : {worst:.3e} m（应 < 1e-6）")
    log(f"   有效反投影比例 : {float(ok_back.mean()):.3f}")
    return worst


# ----------------------------------------------------------------------------
# B. 随机位姿定位精度
# ----------------------------------------------------------------------------
def simulate_observations(cam, ext, field_map: FieldMap, pose, rng,
                          noise_px: float, dropout: float, n_outliers: int,
                          max_range_m: float = 9.0):
    """从真值位姿生成一帧观测：投影 -> 加噪 -> 遮挡 -> 误检。"""
    truth = []
    for m in field_map.markers:
        pb = map_to_base(m.x, m.y, pose)
        if np.hypot(pb[0], pb[1]) > max_range_m:
            continue
        truth.append((m.type, float(pb[0]), float(pb[1])))
    if not truth:
        return [], []

    pts = np.array([(x, y, 0.0) for _, x, y in truth])
    uv, ok = ground_to_pixels(cam, ext, pts)
    ok &= (uv[:, 0] >= 0) & (uv[:, 0] < IMAGE_W) & (uv[:, 1] >= 0) & (uv[:, 1] < IMAGE_H)

    visible = [(truth[i], uv[i]) for i in range(len(truth)) if ok[i]]
    if not visible:
        return [], []

    uv_vis = np.array([uv for _, uv in visible]) + rng.normal(0.0, noise_px, (len(visible), 2))
    ground, valid = pixels_to_ground(cam, ext, uv_vis)

    obs = []
    for i, (meta, _) in enumerate(visible):
        if not valid[i] or not np.all(np.isfinite(ground[i])):
            continue
        if rng.random() < dropout:
            continue
        obs.append(ObservedMarker(meta[0], float(ground[i][0]), float(ground[i][1]),
                                  confidence=float(rng.uniform(0.55, 0.99))))

    types = [m.type for m in field_map.markers]
    for _ in range(n_outliers):
        if not obs:
            break
        r = float(rng.uniform(1.5, max_range_m))
        a = float(rng.uniform(-math.pi, math.pi))
        obs.append(ObservedMarker(str(rng.choice(types)), r * math.cos(a), r * math.sin(a),
                                  confidence=float(rng.uniform(0.25, 0.60))))
    return obs, visible


def simulate_pixels(cam, ext, field_map, pose, rng, noise_px, dropout,
                    max_range_m: float = 9.0, n_outliers: int = 0):
    """生成一帧「检测结果」：类别 + 地图坐标 + 像素坐标（可含误检）。

    两条定位路线共用同一批像素，对比才公平。
    """
    out = []
    for m in field_map.markers:
        pb = map_to_base(m.x, m.y, pose)
        if np.hypot(pb[0], pb[1]) > max_range_m:
            continue
        uv, ok = ground_to_pixels(cam, ext, [(pb[0], pb[1], 0.0)])
        if not ok[0]:
            continue
        u = float(uv[0][0] + rng.normal(0.0, noise_px))
        v = float(uv[0][1] + rng.normal(0.0, noise_px))
        if not (0 <= u < IMAGE_W and 0 <= v < IMAGE_H):
            continue
        if rng.random() < dropout:
            continue
        out.append({"type": m.type, "map_x": m.x, "map_y": m.y,
                    "u": u, "v": v, "is_outlier": False})

    types = [m.type for m in field_map.markers]
    for _ in range(n_outliers):
        out.append({"type": str(rng.choice(types)), "map_x": float("nan"),
                    "map_y": float("nan"), "u": float(rng.uniform(0, IMAGE_W)),
                    "v": float(rng.uniform(0, IMAGE_H)), "is_outlier": True})
    return out


def run_trials_both(cam, ext, field_map, n_trials, noise_px, dropout, n_outliers,
                    rng, params: MatchParams):
    """同一批像素上同时跑路线 A（匹配）与路线 B（PnP），记录精度与耗时。"""
    rows = []
    for _ in range(n_trials):
        pose = (float(rng.uniform(-6.0, 6.0)), float(rng.uniform(-3.5, 3.5)),
                float(rng.uniform(-math.pi, math.pi)))
        dets = simulate_pixels(cam, ext, field_map, pose, rng, noise_px, dropout,
                               n_outliers=n_outliers)
        truth = [d for d in dets if not d["is_outlier"]]

        # 路线 A：像素反投影到地面 -> 观测标志 -> 地图匹配
        res_a, t_a = None, float("nan")
        if len(truth) >= params.min_markers:
            g, okg = pixels_to_ground(cam, ext, [(d["u"], d["v"]) for d in truth])
            obs = [ObservedMarker(d["type"], float(gg[0]), float(gg[1]), 0.9)
                   for d, gg, o in zip(truth, g, okg) if o]
            if len(obs) >= params.min_markers:
                t0 = time.perf_counter()
                res_a = locate(obs, field_map, params=params)
                t_a = (time.perf_counter() - t0) * 1000

        # 路线 B：已知地图坐标 + 像素 直接解位姿（误检没有地图坐标，真实系统同样拿不到）
        res_b, t_b = None, float("nan")
        if len(truth) >= 4:
            t0 = time.perf_counter()
            res_b = solve_planar_pose(
                [(d["map_x"], d["map_y"]) for d in truth],
                [(d["u"], d["v"]) for d in truth], cam)
            t_b = (time.perf_counter() - t0) * 1000

        rows.append({"pose": pose, "n_vis": len(truth), "n_det": len(dets),
                     "A": res_a, "B": res_b, "tA": t_a, "tB": t_b})
    return rows


def summarize_both(rows, log) -> dict:
    def collect(key, getter):
        errs_p, errs_y, ts = [], [], []
        for r in rows:
            res = r[key]
            if res is None or not res.valid:
                continue
            px, py, pyaw = r["pose"]
            errs_p.append(math.hypot(res.x - px, res.y - py))
            yaw = res.yaw_deg if key == "A" else res.yaw_deg
            errs_y.append(abs(math.degrees(wrap_pi(math.radians(yaw) - pyaw))))
            ts.append(r["tA"] if key == "A" else r["tB"])
        n_ok = len(errs_p)
        return {
            "valid_rate": n_ok / max(len(rows), 1),
            "median_pos": statistics.median(errs_p) if errs_p else float("nan"),
            "p95_pos": sorted(errs_p)[int(0.95 * (n_ok - 1))] if errs_p else float("nan"),
            "median_yaw": statistics.median(errs_y) if errs_y else float("nan"),
            "median_ms": statistics.median(ts) if ts else float("nan"),
        }

    a, b = collect("A", None), collect("B", None)
    log(f"     样本 {len(rows)} 帧 | 路线 A 有效率 {a['valid_rate']:.1%}，"
        f"位置中位 {a['median_pos']:.3f} m / P95 {a['p95_pos']:.3f} m，"
        f"航向中位 {a['median_yaw']:.2f}°，耗时中位 {a['median_ms']:.1f} ms")
    log(f"     {'':16s}| 路线 B 有效率 {b['valid_rate']:.1%}，"
        f"位置中位 {b['median_pos']:.3f} m / P95 {b['p95_pos']:.3f} m，"
        f"航向中位 {b['median_yaw']:.2f}°，耗时中位 {b['median_ms']:.1f} ms")
    return {"A": a, "B": b}


def test_extrinsics_recovery(cam, field_map, log) -> bool:
    """桌边外参自标定：用一帧的已知点对应解出高度/俯角/滚转，与真值比对。"""
    true_h, true_pitch, true_roll = CAM_HEIGHT_M, CAM_PITCH_DEG, 0.0
    ext_true = Extrinsics.from_params(height=true_h, pitch_deg=true_pitch, roll_deg=true_roll)
    pose = (-1.2, 0.6, math.radians(28.0))
    rng = np.random.default_rng(31)
    dets = simulate_pixels(cam, ext_true, field_map, pose, rng, 0.5, 0.0)
    truth = [d for d in dets if not d["is_outlier"]]
    log(f"   使用 {len(truth)} 个已知标志的地图坐标与像素坐标求解")
    res = solve_planar_pose([(d["map_x"], d["map_y"]) for d in truth],
                            [(d["u"], d["v"]) for d in truth], cam)
    if not res.valid:
        log(f"   求解失败: status={res.status}")
        return False
    log(f"   解出外参: 高度 {res.height_m:.3f} m（真值 {true_h:.3f}），"
        f"俯角 {res.pitch_deg:.2f}°（真值 {true_pitch:.1f}），滚转 {res.roll_deg:.2f}°")
    log(f"   重投影 RMS {res.rms_px:.3f} px，内点 {res.n_inliers}/{res.n_points}")
    ok = (abs(res.height_m - true_h) < 0.02 and abs(res.pitch_deg - true_pitch) < 0.5
          and res.rms_px < 1.0)
    log(f"   期望高度差 <0.02 m 且俯角差 <0.5° -> {'通过' if ok else '未通过'}")
    return ok


def run_trials(cam, ext, field_map, n_trials, noise_px, dropout, n_outliers, rng,
               params: MatchParams, sampling: str = "exhaustive"):
    rows = []
    for _ in range(n_trials):
        pose = (float(rng.uniform(-6.0, 6.0)), float(rng.uniform(-3.5, 3.5)),
                float(rng.uniform(-math.pi, math.pi)))
        obs, visible = simulate_observations(cam, ext, field_map, pose, rng,
                                             noise_px, dropout, n_outliers)
        if len(visible) < params.min_markers:
            continue
        res = locate(obs, field_map, params=params, sampling=sampling,
                     seed=int(rng.integers(1 << 30)))
        pos_err = yaw_err = float("nan")
        if res.valid:
            pos_err = math.hypot(res.x - pose[0], res.y - pose[1])
            yaw_err = abs(math.degrees(wrap_pi(math.radians(res.yaw_deg) - pose[2])))
        rows.append({"n_visible": len(visible), "n_obs": len(obs), "status": res.status,
                     "valid": res.valid, "pos_err": pos_err, "yaw_err": yaw_err,
                     "residual": res.residual, "marker_count": res.marker_count,
                     "candidates": res.candidates})
    return rows


def run_trials_paired(cam, ext, field_map, n_trials, noise_px, dropout, n_outliers, rng,
                      params: MatchParams):
    """同一批观测帧上同时跑「穷举假设」与「随机 RANSAC 假设」，保证对比公平。"""
    rows = []
    for _ in range(n_trials):
        pose = (float(rng.uniform(-6.0, 6.0)), float(rng.uniform(-3.5, 3.5)),
                float(rng.uniform(-math.pi, math.pi)))
        obs, visible = simulate_observations(cam, ext, field_map, pose, rng,
                                             noise_px, dropout, n_outliers)
        if len(visible) < params.min_markers:
            continue
        for sampling in ("exhaustive", "random"):
            res = locate(obs, field_map, params=params, sampling=sampling, seed=12345)
            pos_err = yaw_err = float("nan")
            if res.valid:
                pos_err = math.hypot(res.x - pose[0], res.y - pose[1])
                yaw_err = abs(math.degrees(wrap_pi(math.radians(res.yaw_deg) - pose[2])))
            rows.append({"sampling": sampling, "status": res.status, "valid": res.valid,
                         "pos_err": pos_err, "yaw_err": yaw_err,
                         "candidates": res.candidates})
    return rows


def summarize_paired(rows, log) -> dict:
    out = {}
    for sampling in ("exhaustive", "random"):
        sub = [r for r in rows if r["sampling"] == sampling]
        valid = [r for r in sub if r["valid"]]
        errs = sorted(r["pos_err"] for r in valid)
        cands = statistics.median([r["candidates"] for r in sub]) if sub else 0
        out[sampling] = {
            "valid_rate": len(valid) / max(len(sub), 1),
            "median_pos": statistics.median(errs) if errs else float("nan"),
            "median_cand": cands,
        }
    log(f"     穷举假设: 有效率 {out['exhaustive']['valid_rate']:.1%}，"
        f"中位位置误差 {out['exhaustive']['median_pos']:.3f} m，"
        f"中位候选数 {out['exhaustive']['median_cand']:.0f}")
    log(f"     随机假设: 有效率 {out['random']['valid_rate']:.1%}，"
        f"中位位置误差 {out['random']['median_pos']:.3f} m，"
        f"中位候选数 {out['random']['median_cand']:.0f}")
    return out


def test_tracking(cam, ext, field_map, params: MatchParams, log,
                  n_frames: int = 60, noise_px: float = 1.5) -> bool:
    """连续帧跟踪：沿半径 2.5 m 的圆周运动，用上一帧位姿作先验。

    这是实际运行时的工作模式（细则 2.10「连续位姿融合」），
    也是消掉场地 180° 对称歧义的主要手段。
    """
    log(f"  圆周运动 {n_frames} 帧，约每帧 6°，像素噪声 σ={noise_px}px")
    rng = np.random.default_rng(7)
    prior = None
    n_valid, n_eligible, errs, yerrs, statuses = 0, 0, [], [], {}
    for k in range(n_frames):
        phi = 2 * math.pi * k / n_frames
        pose = (2.5 * math.cos(phi), 2.5 * math.sin(phi), phi + math.pi / 2)
        obs, visible = simulate_observations(cam, ext, field_map, pose, rng,
                                             noise_px, 0.15, 1)
        if len(visible) < params.min_markers:
            # 看不到足够标志：保留先验但放宽窗口，等下一帧再确认（不伪造位姿）
            if prior:
                prior = (prior[0], prior[1], prior[2], 2.0, 50.0)
            continue
        if len(obs) < params.min_markers:
            # 丢帧后观测不足 3 个：按细则必须报无效，不计入「可定位帧」分母
            if prior:
                prior = (prior[0], prior[1], prior[2], 2.0, 50.0)
            continue
        n_eligible += 1
        res = locate(obs, field_map, params=params, prior=prior)
        statuses[res.status] = statuses.get(res.status, 0) + 1
        if res.valid:
            n_valid += 1
            errs.append(math.hypot(res.x - pose[0], res.y - pose[1]))
            yerrs.append(abs(math.degrees(wrap_pi(math.radians(res.yaw_deg) - pose[2]))))
            prior = (res.x, res.y, math.radians(res.yaw_deg), 1.0, 35.0)
        else:
            prior = (prior[0], prior[1], prior[2], 2.0, 50.0) if prior else None
    log("     状态分布: " + ", ".join(f"{k}={v}" for k, v in sorted(statuses.items())))
    rate = n_valid / max(n_eligible, 1)
    log(f"     可定位帧 {n_eligible}/{n_frames}（其余帧画面内不足 3 个标志，按细则应报无效），"
        f"其中有效 {n_valid} 帧（有效率 {rate:.1%}）")
    if errs:
        log(f"     位置中位误差 {statistics.median(errs):.3f} m，"
            f"P95 {sorted(errs)[int(0.95 * (len(errs) - 1))]:.3f} m")
        log(f"     航向中位误差 {statistics.median(yerrs):.2f}°，"
            f"P95 {sorted(yerrs)[int(0.95 * (len(yerrs) - 1))]:.2f}°")
    ok = rate >= 0.85 and errs and statistics.median(errs) < 0.3
    log(f"   期望：可定位帧有效率 ≥85% 且中位误差 <0.3 m -> {'通过' if ok else '未通过'}")
    return ok


def summarize(rows, log) -> dict:
    valid = [r for r in rows if r["valid"]]
    log(f"     样本 {len(rows)} 帧，定位有效 {len(valid)} 帧"
        f"（有效率 {len(valid) / max(len(rows), 1):.1%}）")
    status: dict = {}
    for r in rows:
        status[r["status"]] = status.get(r["status"], 0) + 1
    log("     状态分布: " + ", ".join(f"{k}={v}" for k, v in sorted(status.items())))
    if rows:
        vis = [r["n_visible"] for r in rows]
        log(f"     每帧可见标志数: 中位 {statistics.median(vis):.0f}，最少 {min(vis)}，最多 {max(vis)}")
        log(f"     候选位姿数: 中位 {statistics.median([r['candidates'] for r in rows]):.0f}")
    if not valid:
        return {"valid_rate": 0.0}
    pe = sorted(r["pos_err"] for r in valid)
    ye = sorted(r["yaw_err"] for r in valid)
    med_p, p95_p = statistics.median(pe), pe[int(0.95 * (len(pe) - 1))]
    med_y, p95_y = statistics.median(ye), ye[int(0.95 * (len(ye) - 1))]
    ok_p = sum(1 for e in pe if e <= 0.5) / len(pe)
    ok_y = sum(1 for e in ye if e <= 10.0) / len(ye)
    log(f"     位置误差: 中位 {med_p:.3f} m，P95 {p95_p:.3f} m，≤0.5m 占 {ok_p:.1%}")
    log(f"     航向误差: 中位 {med_y:.2f}°，P95 {p95_y:.2f}°，≤10° 占 {ok_y:.1%}")
    return {"valid_rate": len(valid) / max(len(rows), 1), "median_pos": med_p, "p95_pos": p95_p,
            "median_yaw": med_y, "p95_yaw": p95_y, "within_0_5m": ok_p, "within_10deg": ok_y}


# ----------------------------------------------------------------------------
# C. 歧义 / 无效输出 / 稳健性 / 先验
# ----------------------------------------------------------------------------
def obs_from_truth(field_map, pose, types=None):
    obs = []
    for t, x, y in field_map:
        if types is not None and t not in types:
            continue
        pb = map_to_base(x, y, pose)
        obs.append(ObservedMarker(t, float(pb[0]), float(pb[1]), 0.9))
    return obs


def test_ambiguous(log, params: MatchParams) -> bool:
    log("C. 完全对称场地的歧义识别（3 个标志，绕原点 180° 与自身重合）")
    fm = build_map(SYMMETRIC_FIELD)
    res = locate(obs_from_truth(SYMMETRIC_FIELD, (0.0, 0.0, 0.0)), fm, params=params)
    log(f"   status={res.status}  valid={res.valid}  marker_count={res.marker_count}  "
        f"候选位姿={res.candidates}")
    if res.second:
        log(f"   次优解: x={res.second['x']:.2f} y={res.second['y']:.2f} "
            f"yaw={res.second['yaw_deg']:.1f}° residual={res.second['residual']:.4f} "
            f"(与最优相差 {res.second['pose_delta_m']:.2f} m / {res.second['yaw_delta_deg']:.1f}°)")
    ok = res.status == "ambiguous" and not res.valid
    log(f"   期望 status=ambiguous -> {'通过' if ok else '未通过'}")
    return ok


def test_insufficient(log, params: MatchParams) -> bool:
    log("C2. 标志不足时应输出无效（只给 2 个标志）")
    fm = build_map(VIRTUAL_FIELD)
    obs = [ObservedMarker("XCross", 2.0, 0.0, 0.9), ObservedMarker("TCross", 3.0, 1.0, 0.9)]
    res = locate(obs, fm, params=params)
    ok = (not res.valid) and res.status == "insufficient_markers"
    log(f"   status={res.status}  valid={res.valid}  marker_count={res.marker_count}"
        f" -> {'通过' if ok else '未通过'}")
    return ok


def test_outliers(log, params: MatchParams) -> bool:
    log("C3. 含离群误检的稳健性（3 个真标志 + 2 个假阳性）")
    fm = build_map(VIRTUAL_FIELD)
    pose = (1.0, 0.5, math.radians(30.0))
    truth = [("LCross", 7.0, 4.5), ("XCross", 0.0, 0.0), ("PenaltyPoint", 5.0, 0.0)]
    obs = obs_from_truth(truth, pose)
    obs.append(ObservedMarker("TCross", 5.0, 3.0, 0.5))
    obs.append(ObservedMarker("XCross", 6.0, -2.0, 0.4))
    res = locate(obs, fm, params=params)
    ok = res.valid and math.hypot(res.x - pose[0], res.y - pose[1]) < 0.3
    log(f"   status={res.status}  x={res.x:.2f} y={res.y:.2f} yaw={res.yaw_deg:.1f}° "
        f"marker_count={res.marker_count} residual={res.residual:.4f}"
        f" -> {'通过' if ok else '未通过'}")
    return ok


def test_prior(log, params: MatchParams) -> bool:
    log("C4. 用上一帧位姿作先验，消掉对称场地的 180° 翻转歧义")
    fm = build_map(SYMMETRIC_FIELD)
    pose = (0.3, -0.2, math.radians(20.0))
    obs = obs_from_truth(SYMMETRIC_FIELD, pose)
    bare = locate(obs, fm, params=params)
    primed = locate(obs, fm, params=params,
                    prior=(0.3, -0.2, math.radians(20.0), 1.5, 25.0))
    ok = (bare.status == "ambiguous") and primed.valid and \
        math.hypot(primed.x - pose[0], primed.y - pose[1]) < 0.05
    log(f"   无先验: status={bare.status}；有先验: status={primed.status} "
        f"x={primed.x:.3f} y={primed.y:.3f} yaw={primed.yaw_deg:.2f}°"
        f" -> {'通过' if ok else '未通过'}")
    return ok


# ----------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description="单目场地自定位 仿真自检")
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--noise", type=float, default=1.5, help="像素噪声标准差")
    ap.add_argument("--dropout", type=float, default=0.15, help="观测随机丢失概率")
    ap.add_argument("--outliers", type=int, default=1, help="每帧误检个数")
    ap.add_argument("--seed", type=int, default=20261106)
    args = ap.parse_args()

    ensure_runtime_dirs()
    lines: list[str] = []

    def log(msg: str) -> None:
        print(msg)
        lines.append(msg)

    log("=" * 78)
    log("单目场地自定位 · 仿真自检（不依赖相机 / 场地 / field.yaml 真文件）")
    log(f"时间: {datetime.now():%Y-%m-%d %H:%M:%S}")
    log(f"环境: Python {platform.python_version()} / numpy {np.__version__} / {platform.system()}")
    log(f"相机: fx=fy={FOCAL_PX:.0f} cx={IMAGE_W / 2:.0f} cy={IMAGE_H / 2:.0f} "
        f"畸变{DIST_COEFFS} @ {IMAGE_W}x{IMAGE_H}")
    log(f"外参: 高度 {CAM_HEIGHT_M} m，俯角 {CAM_PITCH_DEG}°（固定安装）")
    log("=" * 78)

    cam, ext = make_camera(), make_extrinsics()
    params = MatchParams()
    fm = build_map(VIRTUAL_FIELD)

    worst = test_roundtrip(cam, ext, log)
    log("")

    log("A2. 地图文件解析（config/field.yaml，自建地图）")
    root = Path(__file__).resolve().parents[2]
    try:
        geo = root / "config" / "field_geometry.yaml"
        loaded = FieldMap.from_files(root / "config" / "field.yaml",
                                     geo if geo.exists() else None)
        log("   " + loaded.summary().replace("\n", "\n   "))
        parse_ok = len(loaded.markers) == len(VIRTUAL_FIELD)
        log(f"   解析出 {len(loaded.markers)} 个标志点（样例共 {len(VIRTUAL_FIELD)} 个）"
            f" -> {'通过' if parse_ok else '数量不符'}")
    except Exception as exc:                                   # noqa: BLE001
        log(f"   失败：{type(exc).__name__}: {exc}")
        parse_ok = False
    log("")

    log(f"B. 随机位姿定位精度（{args.trials} 次，像素噪声 σ={args.noise}px，"
        f"丢失率 {args.dropout:.0%}，误检 {args.outliers} 个/帧）")
    rows = run_trials(cam, ext, fm, args.trials, args.noise, args.dropout, args.outliers,
                      np.random.default_rng(args.seed), params)
    stats = summarize(rows, log)
    log("")

    log("B2. 理想条件上限（无噪声、无遮挡、无误检）")
    ideal = summarize(run_trials(cam, ext, fm, max(args.trials // 4, 25), 0.0, 0.0, 0,
                                 np.random.default_rng(args.seed + 1), params), log)
    log("")

    log("B3. 穷举假设 vs 随机 RANSAC 假设（同一批观测帧上对比，对应细则加分项 +2 的证据）")
    paired = run_trials_paired(cam, ext, fm, max(args.trials // 2, 40), args.noise,
                               args.dropout, args.outliers,
                               np.random.default_rng(args.seed + 2), params)
    summarize_paired(paired, log)
    log("")

    log("B4. 路线对比：观测点地图匹配（A） vs PnP 直接解算（B）——同一批像素")
    rows_both = run_trials_both(cam, ext, fm, max(args.trials // 2, 40), args.noise,
                                args.dropout, args.outliers,
                                np.random.default_rng(args.seed + 3), params)
    both = summarize_both(rows_both, log)
    log("")

    checks = {
        "roundtrip": worst < 1e-6,
        "parse": parse_ok,
        "ambiguous": test_ambiguous(log, params),
    }
    log("")
    checks["insufficient"] = test_insufficient(log, params)
    log("")
    checks["outliers"] = test_outliers(log, params)
    log("")
    checks["prior"] = test_prior(log, params)
    log("")

    log("D. 连续帧跟踪（圆周运动，用上一帧位姿作先验——实际运行的工作模式）")
    checks["tracking"] = test_tracking(cam, ext, fm, params, log)
    log("")

    log("E. 桌边外参自标定（用场地标志解出高度/俯角/滚转，替代量角器）")
    checks["extrinsics"] = test_extrinsics_recovery(cam, fm, log)
    log("")

    log("=" * 78)
    log("结论")
    for name, label in [("roundtrip", "几何往返误差 < 1e-6 m"), ("parse", "地图文件解析"),
                        ("ambiguous", "对称场地歧义识别"), ("insufficient", "标志不足输出无效"),
                        ("outliers", "离群误检稳健性"), ("prior", "先验消歧"),
                        ("tracking", "连续帧跟踪有效率"),
                        ("extrinsics", "桌边外参自标定")]:
        log(f"  {label:<22}: {'通过' if checks[name] else '未通过'}")
    log(f"  {'常规条件有效率':<22}: {stats.get('valid_rate', 0):.1%}")
    log(f"  {'位置误差中位数':<22}: {stats.get('median_pos', float('nan')):.3f} m（验收目标 ≤ 0.5 m）")
    log(f"  {'航向误差中位数':<22}: {stats.get('median_yaw', float('nan')):.2f}°（验收目标 ≤ 10°）")
    log("  说明: 位姿误差为仿真统计值，真实验收以 C0 现场测试为准。")
    log("=" * 78)

    out_file = OUTPUT_DIR / f"sim_selftest_{datetime.now():%Y%m%d_%H%M%S}.txt"
    out_file.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"\n结果已保存: {rel(out_file)}")
    return 0 if all(checks.values()) else 1


if __name__ == "__main__":
    raise SystemExit(main())
