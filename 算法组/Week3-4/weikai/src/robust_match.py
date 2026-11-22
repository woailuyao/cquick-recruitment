# -*- coding: utf-8 -*-
"""robust_match.py —— 地图匹配的稳健算法与对比（加分项）

在 locator_fixed.solve_pose_fixed（穷举 2 对同源点假设 → 内点投票）基础上，
实现并对比三种地图匹配策略：

  1. exhaustive  现有基线：枚举全部 2 对假设，确定性最强，假设数 O(n^2*m^2)；
  2. ransac      标准 RANSAC：随机采样 2 对假设 + 自适应迭代次数
                 （按内点率估计，置信度 0.999），假设数与内点率挂钩；
  3. irls        稳健位姿优化：在初始解上做迭代重加权 Kabsch（Tukey biweight），
                 对内点集中的长尾残差降权，压低定位残差。

两种实验：
  --mode synthetic  合成数据蒙特卡洛：随机位姿投影地图标志 + 高斯噪声，
                    再注入 k 个假标志（外点），对比三种方法的成功率 /
                    误差 / 单帧耗时——考察对外点的稳健性；
  --mode real       真实视频：M10 等整段检测一次，同一批观测分别用三种
                    方法解算并跨帧聚合，对比真值误差与耗时。

两种方法输出与 solve_pose_fixed 完全同构（dict 列表），可无缝替换。
"""
import argparse
import math
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2  # noqa: E402

from locator import load_field, load_intrinsics, field_type  # noqa: E402
from locator_fixed import (load_extrinsics, camera_to_base,  # noqa: E402
                           pixels_to_ground, fit_rigid, aggregate)


# ---------------------------------------------------------------- #
# 2. 标准 RANSAC                                                    #
# ---------------------------------------------------------------- #
def solve_pose_ransac(obs_g, map_pts, obs_types, map_names,
                      iters=400, hyp_tol=0.35, final_tol=0.30,
                      min_inliers=4, confidence=0.999, seed=0):
    """随机采样版 RANSAC 位姿解算，接口与 solve_pose_fixed 相同。

    与穷举版的区别：假设由随机采样产生（固定种子可复现）。实验发现
    经典自适应停止准则在本问题里不可靠——指派错误但内点数满额的假设
    会把所需轮数估计得过低，导致正解未被采样，故这里用固定轮数，
    每轮成本极低（纯 numpy 小矩阵），默认 400 轮仍比穷举快一个量级。
    """
    rng = np.random.default_rng(seed)
    n, m = len(obs_g), len(map_pts)
    if n < 2 or m < 2:
        return []
    mt = [field_type(s) for s in map_names]
    compat = np.array([[ot is not None and ot == mt[k] for k in range(m)]
                       for ot in obs_types], bool)
    # 每个观测可匹配的地图点索引（预先算好，采样时只从合法对里挑）
    obs_ok = [[k for k in range(m) if compat[i, k]] for i in range(n)]

    best = []                       # 按内点数保留的 top-K 假设
    pairs_all = [(i, j) for i in range(n) for j in range(i + 1, n)
                 if np.linalg.norm(obs_g[i] - obs_g[j]) >= 0.12]
    if not pairs_all:
        return []

    for it in range(iters):
        i, j = pairs_all[rng.integers(len(pairs_all))]
        if not (obs_ok[i] and obs_ok[j]):
            continue
        k = obs_ok[i][rng.integers(len(obs_ok[i]))]
        l_cands = [l for l in obs_ok[j] if l != k]
        if not l_cands:
            continue
        l = l_cands[rng.integers(len(l_cands))]
        A, B = obs_g[[i, j]], map_pts[[k, l]]
        if np.linalg.norm(B[0] - B[1]) < 0.12:
            continue
        R, t = fit_rigid(A, B)
        pred = obs_g @ R.T + t
        diff = np.linalg.norm(pred[:, None, :] - map_pts[None, :, :], axis=2)
        diff = np.where(compat, diff, np.inf)
        nn = diff.min(axis=1)
        inl = nn < hyp_tol
        ni = int(inl.sum())
        if ni < min_inliers:
            continue
        best.append((ni, inl.copy(), diff.argmin(axis=1)))
        if len(best) > 12:
            best.sort(key=lambda z: -z[0])
            best = best[:12]

    if not best:
        return []
    best.sort(key=lambda z: -z[0])
    # 与穷举版相同：每个候选假设都用全部内点重解 + 更紧的 final_tol 复核，
    # 再按（内点数、残差）排序去重
    out = []
    for ni, inl, idx in best[:12]:
        R2, t2 = fit_rigid(obs_g[inl], map_pts[idx[inl]])
        # 两轮"重指派→重解"：消除假设阶段最近邻指派的歧义
        inl2, idx2 = None, None
        for _round in range(2):
            pred2 = obs_g @ R2.T + t2
            diff2 = np.linalg.norm(pred2[:, None, :] - map_pts[None, :, :],
                                   axis=2)
            diff2 = np.where(compat, diff2, np.inf)
            nn2 = diff2.min(axis=1)
            idx2 = diff2.argmin(axis=1)
            inl2 = nn2 < final_tol
            if inl2.sum() < min_inliers:
                break
            R2, t2 = fit_rigid(obs_g[inl2], map_pts[idx2[inl2]])
        if inl2 is None or inl2.sum() < min_inliers:
            continue
        nn2 = np.linalg.norm((obs_g @ R2.T + t2)[:, None, :]
                             - map_pts[None, :, :], axis=2)
        nn2 = np.where(compat, nn2, np.inf).min(axis=1)
        yaw = math.degrees(math.atan2(R2[1, 0], R2[0, 0]))
        rms = float(np.sqrt(np.mean(nn2[inl2] ** 2)))
        x, y = float(t2[0]), float(t2[1])
        if not (abs(x) <= 4.6 and abs(y) <= 3.6):   # 场内约束（同穷举版）
            continue
        pairs = [(map_names[int(idx2[q])], int(q)) for q in np.where(inl2)[0]]
        out.append({"x": x, "y": y, "yaw": yaw, "rms": rms,
                    "n": int(inl2.sum()), "pairs": pairs})
    if not out:
        return []
    final = []
    for s in sorted(out, key=lambda z: (-z["n"], z["rms"])):
        if any(abs(s["x"] - f["x"]) < 0.6 and abs(s["y"] - f["y"]) < 0.6
               and abs((s["yaw"] - f["yaw"] + 180) % 360 - 180) < 12
               for f in final):
            continue
        final.append(s)
        if len(final) >= 3:
            break
    return final


# ---------------------------------------------------------------- #
# 3. IRLS 稳健位姿优化（Tukey biweight）                             #
# ---------------------------------------------------------------- #
def _fit_rigid_w(A, B, w):
    """加权 2D Kabsch：min sum w_i ||R A_i + t - B_i||^2。"""
    sw = w.sum()
    if sw <= 1e-9:
        return fit_rigid(A, B)
    ca = (w[:, None] * A).sum(axis=0) / sw
    cb = (w[:, None] * B).sum(axis=0) / sw
    H = ((A - ca) * w[:, None]).T @ (B - cb)
    U, _S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    return R, cb - R @ ca


def refine_irls(obs_g, map_pts, obs_types, map_names, R, t,
                iters=12, tukey_c=0.25, final_tol=0.30):
    """从初始 (R, t) 出发的迭代重加权精化。

    每轮：观测→最近相容地图点残差 r → Tukey 权 w=(1-(r/c)^2)^2 (r<c) →
    加权 Kabsch 重解。长尾残差（假配对）权重趋 0，等效自动剔除外点。
    """
    n, m = len(obs_g), len(map_pts)
    mt = [field_type(s) for s in map_names]
    compat = np.array([[ot is not None and ot == mt[k] for k in range(m)]
                       for ot in obs_types], bool)
    for _ in range(iters):
        pred = obs_g @ R.T + t
        diff = np.linalg.norm(pred[:, None, :] - map_pts[None, :, :], axis=2)
        diff = np.where(compat, diff, np.inf)
        nn = diff.min(axis=1)
        idx = diff.argmin(axis=1)
        w = np.where(nn < tukey_c,
                     (1 - (np.minimum(nn, tukey_c) / tukey_c) ** 2) ** 2,
                     0.0)
        if w.sum() < 2:
            break
        R, t = _fit_rigid_w(obs_g, map_pts[idx], w)
    diff = np.linalg.norm((obs_g @ R.T + t)[:, None, :] - map_pts[None, :, :],
                          axis=2)
    diff = np.where(compat, diff, np.inf)
    nn = diff.min(axis=1)
    idx = diff.argmin(axis=1)
    inl = nn < final_tol
    if inl.sum() < 2:
        return None
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    rms = float(np.sqrt(np.mean(nn[inl] ** 2)))
    pairs = [(map_names[int(idx[q])], int(q)) for q in np.where(inl)[0]]
    return {"x": float(t[0]), "y": float(t[1]), "yaw": yaw, "rms": rms,
            "n": int(inl.sum()), "pairs": pairs}


def solve_pose_fixed_irls(obs_g, map_pts, obs_types, map_names, **kw):
    """穷举基线 + IRLS 精化：先用 solve_pose_fixed 出初始解，再逐一精化。"""
    from locator_fixed import solve_pose_fixed
    sols = solve_pose_fixed(obs_g, map_pts, obs_types, map_names)
    out = []
    for s in sols:
        yaw = math.radians(s["yaw"])
        R = np.array([[math.cos(yaw), -math.sin(yaw)],
                      [math.sin(yaw), math.cos(yaw)]])
        t = np.array([s["x"], s["y"]])
        r = refine_irls(obs_g, map_pts, obs_types, map_names, R, t)
        if r is not None:
            out.append(r)
    return out


# ---------------------------------------------------------------- #
# 4. 对比实验                                                        #
# ---------------------------------------------------------------- #
def _mc_trial(rng, map_pts, names, n_obs, n_out, sigma):
    """合成一轮观测：随机位姿 + 地标投影噪声 + n_out 个均匀外点。"""
    m = len(map_pts)
    xy = np.array([rng.uniform(-3.5, 3.5), rng.uniform(-2.5, 2.5)])
    yaw = rng.uniform(-180, 180)
    th = math.radians(yaw)
    Rm = np.array([[math.cos(th), -math.sin(th)],
                   [math.sin(th), math.cos(th)]])
    vis = rng.choice(m, size=min(n_obs, m), replace=False)
    carrier = (np.linalg.inv(Rm) @ (map_pts[vis] - xy).T).T
    carrier += rng.normal(0, sigma, carrier.shape)
    obs = carrier
    types = [names[i][0] for i in vis]
    if n_out:
        out_pts = rng.uniform(-5, 5, (n_out, 2))
        out_types = [rng.choice(["L", "T"]) for _ in range(n_out)]
        obs = np.vstack([obs, out_pts])
        types += out_types
    return obs, types, xy, yaw


def run_synthetic(field, trials=200, sigma=0.04, seed=42, ransac_iters=400):
    from functools import partial
    from locator_fixed import solve_pose_fixed
    names = [k for k in field if field_type(k) is not None]
    map_pts = np.array([field[k] for k in names], float)
    rng = np.random.default_rng(seed)
    methods = {
        "exhaustive(基线)": solve_pose_fixed,
        "ransac": partial(solve_pose_ransac, iters=ransac_iters),
        "exhaustive+irls": solve_pose_fixed_irls,
    }
    res = {k: {"ok": 0, "err": [], "t": 0.0, "runs": 0}
           for k in methods}
    print("合成蒙特卡洛: %d 轮, 地面噪声 sigma=%.2f m, 外点 0~3 个/轮, "
          "RANSAC iters=%d" % (trials, sigma, ransac_iters))
    for _ in range(trials):
        n_obs = int(rng.integers(5, 9))
        n_out = int(rng.integers(0, 4))
        obs, types, xy, yaw = _mc_trial(rng, map_pts, names, n_obs, n_out,
                                        sigma)
        for name, fn in methods.items():
            t0 = time.perf_counter()
            sols = fn(obs, map_pts, types, names)
            dt = time.perf_counter() - t0
            r = res[name]
            r["t"] += dt
            r["runs"] += 1
            if not sols:
                continue
            s = sols[0]
            err = math.hypot(s["x"] - xy[0], s["y"] - xy[1])
            dy = abs((s["yaw"] - yaw + 180) % 360 - 180)
            if err <= 0.15 and dy <= 5.0:
                r["ok"] += 1
                r["err"].append(err)
    print("-" * 72)
    print("%-18s %8s %10s %12s %12s"
          % ("方法", "成功率", "平均误差/m", "单帧耗时/ms", "成功轮耗时"))
    for name, r in res.items():
        succ = 100 * r["ok"] / r["runs"]
        emean = (np.mean(r["err"]) if r["err"] else float("nan"))
        print("%-18s %7.1f%% %10.3f %12.2f %12.2f"
              % (name, succ, emean, 1e3 * r["t"] / r["runs"],
                 1e3 * r["t"] / max(1, r["ok"])))


def run_scale(n_landmarks=12, n_obs=10, trials=15, seed=7):
    """规模实验：地图标志数增加时穷举与 RANSAC 的耗时对比。

    当前场地只有 7 个标志，穷举假设数 C(n,2)*C(m,2) 很小；本实验用
    合成地图把规模拉到 10~16 个标志，验证 RANSAC 的 O(iters) 优势。
    """
    from locator_fixed import solve_pose_fixed
    rng = np.random.default_rng(seed)
    print("规模实验: 观测 %d 个, 每个规模 %d 轮（含 1 个假标志）"
          % (n_obs, trials))
    print("%-10s %10s %14s %14s %10s"
          % ("标志数", "假设数", "穷举/ms", "ransac/ms", "加速比"))
    for m in (8, 12, 16):
        names = ["L%d" % i if i % 2 == 0 else "T%d" % i for i in range(m)]
        map_pts = rng.uniform([-4, -3], [4, 3], (m, 2))
        t_ex = t_ra = 0.0
        for _ in range(trials):
            obs, types, _xy, _yaw = _mc_trial(rng, map_pts, names, n_obs, 1,
                                              0.03)
            t0 = time.perf_counter()
            solve_pose_fixed(obs, map_pts, types, names)
            t_ex += time.perf_counter() - t0
            t0 = time.perf_counter()
            solve_pose_ransac(obs, map_pts, types, names)
            t_ra += time.perf_counter() - t0
        hyp = n_obs * (n_obs - 1) // 2 * m * (m - 1) // 2
        print("%-10d %10d %14.1f %14.1f %9.1fx"
              % (m, hyp, 1e3 * t_ex / trials, 1e3 * t_ra / trials,
                 t_ex / max(t_ra, 1e-9)))


def run_real(args):
    from locator_fixed import solve_pose_fixed
    from realtime_locate import detect_boxes, get_model
    field = load_field(args.field)
    K, dist, _wh = load_intrinsics(args.intr)
    h, pitch, roll = load_extrinsics(args.extr)
    R_cb = camera_to_base(pitch, roll)
    names = [k for k in field if field_type(k) is not None]
    map_pts = np.array([field[k] for k in names], float)

    parts = [float(v) for v in args.truth.split(",")]
    truth, truth_yaw = (parts[0], parts[1]), (parts[2] if len(parts) > 2
                                              else None)
    model = get_model(args.weights)

    cap = cv2.VideoCapture(args.video)
    frames = []
    fi = 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        fi += 1
        if (fi - 1) % args.stride:
            continue
        boxes = detect_boxes(model, bgr, args.conf, args.imgsz, "none")
        if len(boxes) < 3:
            continue
        obs = np.array([[b[1], b[2]] for b in boxes])
        types = [b[3] for b in boxes]
        g, okm = pixels_to_ground(obs, K, dist, R_cb, h)
        if okm.sum() >= 2:
            frames.append((g[okm],
                           [types[i] for i in np.where(okm)[0]]))
    cap.release()
    print("视频 %s: 采样 %d 帧(检出>=3 标志)" % (args.video, len(frames)))

    from locator_fixed import solve_pose_fixed as _spf  # noqa: F811
    methods = {
        "exhaustive(基线)": _spf,
        "ransac": solve_pose_ransac,
        "exhaustive+irls": solve_pose_fixed_irls,
    }
    for name, fn in methods.items():
        t0 = time.perf_counter()
        sols_by_frame = [fn(o, map_pts, t, names) for o, t in frames]
        dt = time.perf_counter() - t0
        res = aggregate(sols_by_frame)
        if res is None:
            print("%-18s 无有效定位" % name)
            continue
        x, y, yw, n_in, rms = res
        err = math.hypot(x - truth[0], y - truth[1])
        dyaw = ("-" if truth_yaw is None else
                "%+.2f°" % ((yw - truth_yaw + 180) % 360 - 180))
        print("%-18s x=%+.3f y=%+.3f yaw=%+.2f°  位置误差=%.3f m  "
              "航向误差=%s  主簇=%d帧 rms=%.3f  总耗时=%.0f ms"
              % (name, x, y, yw, err, dyaw, n_in, rms, 1e3 * dt))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--mode", choices=["synthetic", "real", "scale"],
                    default="synthetic")
    ap.add_argument("--field", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "templates/field.yaml"))
    ap.add_argument("--intr", default="/mnt/d/captures/calib/intrinsics.yaml")
    ap.add_argument("--extr", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "templates/camera_extrinsics.yaml"))
    ap.add_argument("--video")
    ap.add_argument("--truth", help="真值 x,y[,yaw]")
    ap.add_argument("--weights", default="/mnt/d/captures/landmarks/"
                                        "weights/best_facing_v3.pt")
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--stride", type=int, default=20)
    ap.add_argument("--trials", type=int, default=200)
    ap.add_argument("--sigma", type=float, default=0.04)
    ap.add_argument("--iters", type=int, default=400,
                    help="RANSAC 采样轮数")
    args = ap.parse_args()

    field = load_field(args.field)
    if args.mode == "synthetic":
        run_synthetic(field, args.trials, args.sigma,
                      ransac_iters=args.iters)
    elif args.mode == "scale":
        run_scale()
    else:
        if not args.video or not args.truth:
            ap.error("--mode real 需要 --video 和 --truth")
        args.field = args.field if os.path.isabs(args.field) else \
            os.path.join(os.path.dirname(os.path.abspath(__file__)),
                         os.path.basename(args.field))
        run_real(args)


if __name__ == "__main__":
    main()
