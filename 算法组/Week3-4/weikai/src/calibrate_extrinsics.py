# -*- coding: utf-8 -*-
"""calibrate_extrinsics.py —— 数据反标定相机固定外参（h, pitch, roll）

原理：已知若干视频的相机真值站位 (x,y,yaw)，对候选 (h,pitch)：
  1) 每帧检测标志 → 像素射线 → 地面点（固定外参投影）
  2) 与 field.yaml 做 2D 刚体配准 → 位姿解
  3) 与真值比较，取位置+航向加权误差最小者

这不是"手工估计伪装精确标定"：优化目标是可复现的几何重投影残差
与已知真值偏差，标定过程与结果可写入报告备查。
用法：
  python calibrate_extrinsics.py --videos M1:0,2.99,-36.94 M7:-2,0,-25.94
"""
import argparse
import math
import os
import sys

import cv2
import numpy as np

import locator_fixed as F
from locator import load_field, load_intrinsics, detect_yolo


def parse_video_spec(s):
    name, truth = s.split(":", 1)
    parts = [float(p) for p in truth.replace("，", ",").split(",")]
    return name, (parts[0], parts[1], parts[2] if len(parts) > 2 else None)


def detect_all(video, truth, K, dist, weights, conf, imgsz, enhance, stride):
    """预检测所有采样帧，返回 [(像素, 类型)] 列表（外参无关，只做一次）。"""
    cap = cv2.VideoCapture("/mnt/d/captures/%s.mp4" % video)
    frames = []
    fi = 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        fi += 1
        if (fi - 1) % stride:
            continue
        if enhance == "clahe":
            lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = cv2.createCLAHE(2.0, (8, 8)).apply(l)
            bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
        pts = detect_yolo(bgr, weights, conf, imgsz)
        if len(pts) >= 4:
            frames.append((np.array([[x, y] for x, y, _ in pts], float),
                           [t for _, _, t in pts]))
    cap.release()
    return frames


def evaluate(frames, truth, K, dist, R_cb, h, map_pts, names):
    tx, ty, tyaw = truth
    errs = []
    for px, types in frames:
        obs_g, ok = F.pixels_to_ground(px, K, dist, R_cb, h)
        if ok.sum() < 4:
            continue
        sols = F.solve_pose_fixed(obs_g[ok], map_pts,
                                  [types[i] for i in np.where(ok)[0]], names)
        if not sols:
            continue
        s = sols[0]
        epos = math.hypot(s["x"] - tx, s["y"] - ty)
        eang = abs((s["yaw"] - tyaw + 180) % 360 - 180) if tyaw else 0
        errs.append(epos + eang * 0.01)         # 1° 折算 1cm
    if not errs:
        return None, 0
    return float(np.median(errs)), len(errs)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos", nargs="+", required=True,
                    help="M1:0,2.99,-36.94 形式")
    ap.add_argument("--field", default="templates/field.yaml")
    ap.add_argument("--intr", default="/mnt/d/captures/calib/intrinsics.yaml")
    ap.add_argument("--weights", default="/mnt/d/captures/landmarks/"
                                        "weights/best_manual_150.pt")
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=1280)
    ap.add_argument("--enhance", default="clahe")
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--out", default="templates/camera_extrinsics.yaml")
    args = ap.parse_args()

    base = os.path.dirname(os.path.abspath(__file__))
    field = load_field(os.path.join(base, args.field))
    K, dist, _ = load_intrinsics(args.intr)
    names = [k for k in field if F.field_type(k) is not None]
    map_pts = np.array([field[k] for k in names], float)

    data = []
    for spec in args.videos:
        name, truth = parse_video_spec(spec)
        fr = detect_all(name, truth, K, dist, args.weights, args.conf,
                        args.imgsz, args.enhance, args.stride)
        print("视频 %s: %d 帧可用" % (name, len(fr)))
        data.append((name, truth, fr))

    best = None
    for h in np.arange(0.45, 1.31, 0.05):
        for pitch in np.arange(10.0, 46.0, 1.0):
            R = F.camera_to_base(pitch, 0.0)
            scores = []
            for name, truth, fr in data:
                sc, n = evaluate(fr, truth, K, dist, R, h, map_pts, names)
                if sc is not None and n >= 3:
                    scores.append(sc)
            if len(scores) != len(data):
                continue
            score = float(np.mean(scores))
            if best is None or score < best[0]:
                best = (score, float(h), float(pitch))
            print("  h=%.2f pitch=%.0f  score=%.3f  (best %.3f @ h=%.2f p=%.0f)"
                  % (h, pitch, score, best[0], best[1], best[2]))
    if best is None:
        print("标定失败：所有候选外参都无法解出 >=3 个有效帧")
        return
    score, h, pitch = best
    print("\n最优外参: z=%.3f m, pitch=%.1f°, 综合误差=%.3f" % (h, pitch, score))
    print("请人工核对后写入 %s" % args.out)


if __name__ == "__main__":
    main()
