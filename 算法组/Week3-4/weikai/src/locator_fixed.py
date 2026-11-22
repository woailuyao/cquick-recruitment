# -*- coding: utf-8 -*-
"""locator_fixed.py —— 固定外参版单目场地自定位（任务 2）

与原 locator.py 的区别（细则 2.6/2.7）：
  原版：每帧把 (x, y, z, yaw, pitch) 当未知量做 6DoF PnP 反解 —— 外参随帧变化，
        与"相机刚性安装、外参固定"的要求不符；
  本版：z(高度)、pitch(俯仰)、roll(滚转) 是【固定外参】，从
        camera_extrinsics.yaml 读入且整段视频不变；每帧只做：
          像素 → 去畸变射线 → 转到载体系 → 与地面 Z=0 求交
          → 得到载体系米制地面点 → 与 field.yaml 做 2D 刚体配准
          → 输出 (x_camera_map, y_camera_map, yaw_camera_map)
        地面投影的自由度只有 3 个（x/y/yaw），求解退化为 2D Kabsch，
        比 PnP 快一个数量级且不会有镜像假解。

载体约定：原点为相机光心的地面投影，+X 为相机朝向的水平投影，+Y 为左侧，+Z 向上；
          与地图 +X→球门、+Y→面向球门左侧一致，故 yaw 直接是地图航向。

用法：
  python locator_fixed.py --video /mnt/d/captures/M1.mp4 --truth=0,2.99,-36.94 \\
      --detector yolo --weights /mnt/d/captures/landmarks/weights/best_facing_v3.pt \\
      --conf 0.05 --imgsz 640 --enhance none --stride 10 \\
      --intr /mnt/d/captures/calib/intrinsics.yaml \\
      --extr templates/camera_extrinsics.yaml
"""
import argparse
import itertools
import math
import os
import sys

import cv2
import numpy as np

from locator import (load_field, load_intrinsics, field_type,
                     detect_yolo, detect)


# ---------- 固定外参读取 ----------
def load_extrinsics(path):
    """从 camera_extrinsics.yaml 读 h / pitch / roll（简单键值解析，不依赖 yaml）。"""
    vals = {}
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            v = v.strip().strip("'\"")
            try:
                vals[k.strip()] = float(v)
            except ValueError:
                pass
    h = vals.get("height_median_m", vals.get("height_m", 0.743))
    pitch = vals.get("pitch_median_deg", vals.get("pitch_deg", 14.5))
    roll = vals.get("roll_nominal_deg", vals.get("roll_deg", 0.0))
    return float(h), float(pitch), float(roll)


def camera_to_base(pitch_deg, roll_deg=0.0):
    """相机系(x右,y下,z前) → 载体系(+X前,+Y左,+Z上) 的 3x3 旋转。

    无俯仰时：相机 x→载体 -Y、相机 y→载体 -Z、相机 z→载体 +X。
    俯仰 pitch 绕载体 +Y(左) 轴旋转，正值=光轴向下；
    滚转 roll 绕相机 z 轴（先于俯仰施加）。
    """
    th = math.radians(pitch_deg)
    ph = math.radians(roll_deg)
    R_level = np.array([[0., 0., 1.],
                        [-1., 0., 0.],
                        [0., -1., 0.]], float)
    Ry = np.array([[math.cos(th), 0., math.sin(th)],
                   [0., 1., 0.],
                   [-math.sin(th), 0., math.cos(th)]], float)
    Rz = np.array([[math.cos(ph), -math.sin(ph), 0.],
                   [math.sin(ph), math.cos(ph), 0.],
                   [0., 0., 1.]], float)
    return Ry @ R_level @ Rz


def pixels_to_ground(pts, K, dist, R_cb, h, max_range=40.0):
    """像素 → 载体系地面点 (xb, yb)，返回 (N,2) 与有效掩码。

    d_c = K^-1 [u,v,1]；d_b = R_cb d_c；λ = -h / d_b,z；P_b = O_b + λ d_b。
    地平线以上 / 交点过远 / 非有限值一律标为无效，不产生 NaN 或伪造点。
    """
    uv = np.asarray(pts, float).reshape(-1, 1, 2)
    zn = cv2.undistortPoints(uv, K, dist)       # (N,1,2) 归一化射线
    d_c = np.concatenate([zn[:, 0, :], np.ones((len(uv), 1))], axis=1)
    d_b = (R_cb @ d_c.T).T
    dz = d_b[:, 2]
    ok = dz < -1e-6                             # 射线必须朝下
    lam = np.where(ok, -h / np.where(ok, dz, -1.0), -1.0)
    P = lam[:, None] * d_b[:, :2]
    valid = (ok & (lam > 0) & (lam < max_range)
             & np.isfinite(P).all(axis=1) & (np.abs(P) < max_range).all(axis=1))
    return P, valid


# ---------- 2D 刚体配准 ----------
def fit_rigid(A, B):
    """最小二乘求 map = R @ carrier + t（Kabsch 2D）。"""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    U, _S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    return R, cb - R @ ca


def solve_pose_fixed(obs_g, map_pts, obs_types, map_names,
                     hyp_tol=0.35, final_tol=0.30, min_inliers=4):
    """固定外参下的位姿解算：枚举 2 对同源点 → Kabsch → 全体内点投票。

    obs_g: (N,2) 载体系地面点；map_pts: (M,2) 地图坐标；
    obs_types: N 个 'L'/'T'；map_names: M 个地图标志名。
    返回与 locator.solve_pose 兼容的 dict 列表（按内点数、残差排序）。
    """
    n, m = len(obs_g), len(map_pts)
    if n < 2 or m < 2:
        return []
    mt = [field_type(s) for s in map_names]
    # 类型相容矩阵（None 类型不参与）
    compat = np.array([[ot is not None and ot == mt[k] for k in range(m)]
                       for ot in obs_types], bool)

    out = []
    for i, j in itertools.combinations(range(n), 2):
        for k, l in itertools.combinations(range(m), 2):
            if not (compat[i, k] and compat[j, l]):
                continue
            A = obs_g[[i, j]]
            B = map_pts[[k, l]]
            if np.linalg.norm(A[0] - A[1]) < 0.12:
                continue                        # 观测点太近，方向病态
            R, t = fit_rigid(A, B)
            pred = obs_g @ R.T + t
            diff = np.linalg.norm(pred[:, None, :] - map_pts[None, :, :],
                                  axis=2)
            diff = np.where(compat, diff, np.inf)
            nn = diff.min(axis=1)
            inl = nn < hyp_tol
            if inl.sum() < min_inliers:
                continue
            idx = diff.argmin(axis=1)
            # 用全部内点重解一次
            R2, t2 = fit_rigid(obs_g[inl], map_pts[idx[inl]])
            pred2 = obs_g @ R2.T + t2
            diff2 = np.linalg.norm(pred2[:, None, :] - map_pts[None, :, :],
                                   axis=2)
            diff2 = np.where(compat, diff2, np.inf)
            nn2 = diff2.min(axis=1)
            inl2 = nn2 < final_tol
            if inl2.sum() < min_inliers:
                continue
            yaw = math.degrees(math.atan2(R2[1, 0], R2[0, 0]))
            rms = float(np.sqrt(np.mean(nn2[inl2] ** 2)))
            x, y = float(t2[0]), float(t2[1])
            if not (abs(x) <= 4.6 and abs(y) <= 3.6):   # 场内约束
                continue
            pairs = [(map_names[int(idx[q])], int(q)) for q in np.where(inl2)[0]]
            out.append({"x": x, "y": y, "yaw": yaw, "rms": rms,
                        "n": int(inl2.sum()), "pairs": pairs})

    if not out:
        return []
    # 去重：0.6m + 12° 视为同一解
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


# ---------- 跨帧聚合 ----------
def aggregate(sols_by_frame, radius=0.8):
    """每帧取最优解，0.8m 内聚簇，返回 (x, y, yaw, n_frames, rms)。"""
    pts = [s[0] for s in sols_by_frame if s]
    if not pts:
        return None
    best = None
    for p in pts:
        cluster = [q for q in pts
                   if math.hypot(q["x"] - p["x"], q["y"] - p["y"]) < radius]
        if best is None or len(cluster) > best[0]:
            best = (len(cluster), cluster, p)
    cluster = best[1]
    x = float(np.median([q["x"] for q in cluster]))
    y = float(np.median([q["y"] for q in cluster]))
    yaws = np.array([q["yaw"] for q in cluster])
    yaw = float(np.median((yaws + 180) % 360 - 180) % 360)
    rms = float(np.median([q["rms"] for q in cluster]))
    return x, y, yaw, len(cluster), rms


# ---------- CLI ----------
def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--image")
    ap.add_argument("--video")
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--truth", help="真值 'x,y' 或 'x,y,yaw'")
    ap.add_argument("--field", default="templates/field.yaml")
    ap.add_argument("--intr", default="/mnt/d/captures/calib/intrinsics.yaml")
    ap.add_argument("--extr", default="templates/camera_extrinsics.yaml")
    ap.add_argument("--detector", choices=["classic", "yolo"], default="yolo")
    ap.add_argument("--weights", default="/mnt/d/captures/landmarks/"
                                        "weights/best_facing_v3.pt")
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--enhance", choices=["none", "clahe"], default="none")
    ap.add_argument("--selftest", action="store_true")
    args = ap.parse_args()

    base = os.path.dirname(os.path.abspath(__file__))
    fp = lambda p: p if os.path.isabs(p) else os.path.join(base, p)
    field = load_field(fp(args.field))
    K, dist, _wh = load_intrinsics(args.intr)
    h, pitch, roll = load_extrinsics(fp(args.extr))
    R_cb = camera_to_base(pitch, roll)
    print("固定外参: z=%.3f m, pitch=%.2f°, roll=%.2f°（整段视频不变）"
          % (h, pitch, roll))

    names = [k for k in field if field_type(k) is not None]
    map_pts = np.array([field[k] for k in names], float)

    if args.selftest:
        # 合成数据：已知位姿 → 投影成像素 → 反解，验证数学链路
        rng = np.random.default_rng(0)
        truth_xy = np.array([0.4, -1.2])
        truth_yaw = 22.0
        Rm = np.array([[math.cos(math.radians(truth_yaw)),
                        -math.sin(math.radians(truth_yaw))],
                       [math.sin(math.radians(truth_yaw)),
                        math.cos(math.radians(truth_yaw))]])
        # 载体点 = R_map^-1 (地图点 - t)
        carrier = (np.linalg.inv(Rm) @ (map_pts - truth_xy).T).T
        # 载体 → 相机：先抬高 h，再用 R_cb^T 转回相机系
        pts3 = np.concatenate([carrier, np.full((len(carrier), 1), -h)], axis=1)
        cam = (R_cb.T @ pts3.T).T
        norm = cam[:, :2] / cam[:, 2:3]              # (N,2) 归一化像平面
        px = (K @ np.vstack([norm.T, np.ones(norm.shape[0])])).T[:, :2]
        obs_g, ok = pixels_to_ground(px, K, dist, R_cb, h)
        sols = solve_pose_fixed(obs_g[ok], map_pts,
                                [field_type(n) for n in names], names)
        print("selftest: 有效投影 %d/%d, 解=%s"
              % (ok.sum(), len(ok), sols[0] if sols else None))
        if sols:
            s = sols[0]
            print("真值 x=%.3f y=%.3f yaw=%.2f° → 误差 %.4f m / %.2f°"
                  % (truth_xy[0], truth_xy[1], truth_yaw,
                     math.hypot(s["x"] - truth_xy[0], s["y"] - truth_xy[1]),
                     s["yaw"] - truth_yaw))
        return

    def run_detect(bgr):
        if args.enhance == "clahe":
            lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = cv2.createCLAHE(2.0, (8, 8)).apply(l)
            bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
        if args.detector == "yolo":
            return detect_yolo(bgr, args.weights, args.conf, args.imgsz)
        return detect(bgr)

    def one_frame(bgr):
        pts = run_detect(bgr)
        if len(pts) < 2:
            return [], 0
        px = np.array([[x, y] for x, y, _ in pts], float)
        types = [t for _, _, t in pts]
        obs_g, ok = pixels_to_ground(px, K, dist, R_cb, h)
        if ok.sum() < 2:
            return [], len(pts)
        sols = solve_pose_fixed(obs_g[ok], map_pts,
                                [types[i] for i in np.where(ok)[0]], names)
        return sols, len(pts)

    if args.image:
        bgr = cv2.imread(args.image)
        sols, nd = one_frame(bgr)
        print("检测到 %d 个标志" % nd)
        if sols:
            s = sols[0]
            print("x=%+.3f y=%+.3f yaw=%+.2f° 内点=%d rms=%.3f m"
                  % (s["x"], s["y"], s["yaw"], s["n"], s["rms"]))
        return

    if not args.video:
        ap.error("需要 --image 或 --video（或 --selftest）")

    truth, truth_yaw = None, None
    if args.truth:
        parts = [float(p) for p in args.truth.replace("，", ",").split(",")]
        truth = (parts[0], parts[1])
        if len(parts) >= 3:
            truth_yaw = parts[2]

    cap = cv2.VideoCapture(args.video)
    n_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    all_sols = []
    fi = 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        fi += 1
        if (fi - 1) % args.stride:
            continue
        sols, nd = one_frame(bgr)
        all_sols.append(sols)
        if fi % (args.stride * 20) == 0:
            print("  帧 %d/%d 检出 %d 解 %d" % (fi, n_frames, nd, len(sols)))
    cap.release()

    print("-" * 62)
    print("采样帧 %d，有解帧 %d" % (len(all_sols), sum(bool(s) for s in all_sols)))
    res = aggregate(all_sols)
    if res is None:
        print("无有效定位")
        return
    x, y, yaw, n_in, rms = res
    print("整段结果: x_camera_map=%+.3f m  y_camera_map=%+.3f m  "
          "yaw_camera_map=%+.2f°" % (x, y, yaw))
    print("主簇 %d 帧, 残差 %.3f m" % (n_in, rms))
    if truth:
        err = math.hypot(x - truth[0], y - truth[1])
        print("位置误差 = %.3f m  %s" % (err, "达标" if err <= 0.5 else "超差"))
        if truth_yaw is not None:
            dy = (yaw - truth_yaw + 180) % 360 - 180
            print("航向误差 = %+.2f°  %s" % (dy, "达标" if abs(dy) <= 10 else "超差"))


if __name__ == "__main__":
    main()
