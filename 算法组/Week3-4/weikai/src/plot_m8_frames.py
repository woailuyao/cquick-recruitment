#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""M8 逐帧解算散点分析：看系统偏差的模式（沿视线 vs 横向）与簇结构。

用法（WSL）：
  ~/vision_ws/k1_soccer_vision/.venv/bin/python plot_m8_frames.py
"""
import math
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from locator import load_field, load_intrinsics, detect_yolo, solve_pose, field_type

VIDEO = "M8"
TRUTH = (-3.00, 0.00)
CONF, IMGSZ, STRIDE = 0.05, 1280, 5


def clahe(bgr):
    lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
    l, a, b = cv2.split(lab)
    l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
    return cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)


def main():
    base = os.path.dirname(os.path.abspath(__file__))
    FIELD = load_field(os.path.join(base, "templates/field.yaml"))
    K, dist, _ = load_intrinsics("/mnt/d/captures/calib/intrinsics.yaml")
    W = "/mnt/d/captures/landmarks/weights/best_manual_150.pt"

    cap = cv2.VideoCapture("/mnt/d/captures/%s.mp4" % VIDEO)
    k = 0
    xs, ys, yaws, fnos = [], [], [], []
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        if k % STRIDE == 0:
            pts = detect_yolo(clahe(bgr), W, CONF, IMGSZ)
            sols = []
            if len(pts) >= 4:
                sols = solve_pose(
                    np.array([(x, y) for x, y, _ in pts], float), FIELD, K, dist,
                    obs_types=[t for _, _, t in pts])
            if sols:
                s = sols[0]
                xs.append(s["x"]); ys.append(s["y"])
                yaws.append(s["yaw"]); fnos.append(k)
        k += 1
    cap.release()
    xs, ys = np.array(xs), np.array(ys)
    print("解算帧 %d 个" % len(xs))

    out_dir = "/mnt/d/captures/_locate/figs"
    os.makedirs(out_dir, exist_ok=True)

    half_l, half_w = 7.954 / 2, 5.982 / 2
    goal_half = 2.886 / 2
    fig, ax = plt.subplots(figsize=(9.5, 7.5))
    ax.add_patch(plt.Rectangle((-half_l, -half_w), 2 * half_l, 2 * half_w,
                               fill=False, ec="#444", lw=1.6))
    ax.plot([half_l, half_l], [-goal_half, goal_half], color="#c0392b", lw=3.2)
    ax.plot([-half_l, half_l], [0, 0], ls="--", color="#bbb", lw=0.8)
    ax.plot([0, 0], [-half_w, half_w], ls="--", color="#bbb", lw=0.8)
    for n, p in FIELD.items():
        if field_type(n) is None:
            continue
        ax.plot(p[0], p[1], "s", ms=5, color="#2c7fb8")
        ax.annotate(n.replace("Cross", ""), (p[0], p[1]), fontsize=7,
                    textcoords="offset points", xytext=(4, 4), color="#2c7fb8")
    med = (np.median(xs), np.median(ys))
    # 主簇（真值 0.8m 内）
    near = np.hypot(xs - TRUTH[0], ys - TRUTH[1]) < 0.8
    far = ~near
    ax.plot(xs[far], ys[far], ".", ms=7, color="#e67e22", alpha=0.7,
            label="frame solution (outside 0.8m)")
    ax.plot(xs[near], ys[near], ".", ms=9, color="#27ae60", alpha=0.8,
            label="frame solution (within 0.8m)")
    ax.plot(TRUTH[0], TRUTH[1], "x", ms=15, mew=2.8, color="#c0392b",
            label="ground truth (-3, 0)", zorder=6)
    ax.plot(med[0], med[1], "o", ms=11, mfc="none", mew=2.4, color="#27ae60",
            label="median", zorder=6)
    ax.add_patch(plt.Circle(TRUTH, 0.5, fill=False, ls="--", color="#c0392b",
                            lw=1.4, label="0.5m spec"))
    # 偏差向量放大标注
    dx, dy = med[0] - TRUTH[0], med[1] - TRUTH[1]
    ax.annotate("", xy=(med[0], med[1]), xytext=TRUTH,
                arrowprops=dict(arrowstyle="->", color="#8e44ad", lw=1.8))
    ax.text(TRUTH[0] + dx / 2, TRUTH[1] + dy / 2 - 0.25,
            "bias (%+.2f, %+.2f)m" % (dx, dy), color="#8e44ad", fontsize=9,
            ha="center")
    ax.set_xlim(-half_l - 0.4, half_l + 0.4)
    ax.set_ylim(-half_w - 0.4, half_w + 0.4)
    ax.set_aspect("equal")
    ax.grid(alpha=0.25)
    ax.set_xlabel("X (m, toward goal)")
    ax.set_ylabel("Y (m)")
    err = math.hypot(dx, dy)
    ax.set_title("%s: %d frame solutions, median err=%.3fm, "
                 "yaw med=%.1f deg" % (VIDEO, len(xs), err,
                                       np.median(yaws)), fontsize=11)
    ax.legend(fontsize=8, loc="lower left")
    fig.tight_layout()
    out = os.path.join(out_dir, "fig_m8_scatter.png")
    fig.savefig(out, dpi=150)
    print("散点图 ->", out)

    # 偏差分解
    print("\n中位数偏差分解（真值 -> 中位数）:")
    print("  dx = %+.3f m（沿 +X，即沿视线方向）" % dx)
    print("  dy = %+.3f m（横向）" % dy)
    yaw_m = np.median(yaws)
    print("  yaw 中位数 %+.1f°（真值 0°）" % yaw_m)
    # 相机高度分布
    zs = []
    cap = cv2.VideoCapture("/mnt/d/captures/%s.mp4" % VIDEO)
    k2 = 0
    while True:
        ok, bgr = cap.read()
        if not ok:
            break
        if k2 % STRIDE == 0:
            pts = detect_yolo(clahe(bgr), W, CONF, IMGSZ)
            if len(pts) >= 4:
                sols = solve_pose(
                    np.array([(x, y) for x, y, _ in pts], float), FIELD, K, dist,
                    obs_types=[t for _, _, t in pts])
            else:
                sols = []
            if sols:
                zs.append(sols[0]["z"])
        k2 += 1
    cap.release()
    if zs:
        print("  相机高度中位数 %.2f m（范围 %.2f~%.2f）"
              % (np.median(zs), min(zs), max(zs)))


if __name__ == "__main__":
    main()
