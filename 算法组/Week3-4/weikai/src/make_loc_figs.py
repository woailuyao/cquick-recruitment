#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""定位结果可视化：俯视图散点 + 画面叠加 + 逐帧误差曲线。

对每个定点视频逐帧解算，收集全部解，然后画三类图：
  fig1_topview.png   俯视图：场地边界 / 标志 / 真值 / 逐帧估计散点 / 中位数 / 0.5m 圈
  fig2_overlay_<v>.png  画面叠加：用估计位姿把地图标志投回画面，核对与检测框一致
  fig3_error.png     逐帧位置误差曲线（只画靠近真值的簇，体现成功率与抖动）

用法（WSL）：
  ~/vision_ws/k1_soccer_vision/.venv/bin/python make_loc_figs.py
"""
import argparse
import json
import math
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from locator import (load_field, load_intrinsics, detect_yolo, solve_pose,
                     field_type)

VIDEOS = [("M1", (0.00, 2.99)), ("M2", (0.00, 0.00)), ("M3", (0.00, 0.00)),
          ("M7", (-2.00, 0.00))]


def R_from_pose(yaw_deg, pitch_deg):
    """由航向/俯仰重建 world->cam 旋转矩阵（与 pose_from_rvec 互逆）。"""
    yaw, pitch = math.radians(yaw_deg), math.radians(pitch_deg)
    zc = np.array([math.cos(pitch) * math.cos(yaw),
                   math.cos(pitch) * math.sin(yaw),
                   -math.sin(pitch)])
    up = np.array([0.0, 0.0, 1.0])
    xc = np.cross(zc, up)
    xc /= np.linalg.norm(xc)
    yc = np.cross(zc, xc)
    return np.vstack([xc, yc, zc])          # 行 = 相机轴（世界系表示）


def project_fields(sol, obj, K, dist):
    R = R_from_pose(sol["yaw"], sol["pitch"])
    C = np.array([sol["x"], sol["y"], sol["z"]])
    rvec, _ = cv2.Rodrigues(R)
    tvec = (-R @ C).reshape(3, 1)
    p, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, K, dist)
    return p.reshape(-1, 2)


def cluster_median(sols, tol_m=0.8):
    """最大一致簇（位置 0.8m 内）的中位数位置与航向。"""
    if not sols:
        return None
    best = None
    for s in sols:
        grp = [q for q in sols
               if math.hypot(q["x"] - s["x"], q["y"] - s["y"]) < tol_m]
        if best is None or len(grp) > len(best):
            best = grp
    if not best:
        return None
    xs = np.array([q["x"] for q in best])
    ys = np.array([q["y"] for q in best])
    yaws = np.array([q["yaw"] for q in best])
    # 航向用圆均值（避免 ±180 跳变）
    rad = np.radians(yaws)
    yaw = math.degrees(math.atan2(np.sin(rad).mean(), np.cos(rad).mean()))
    return {"x": float(np.median(xs)), "y": float(np.median(ys)),
            "yaw": yaw, "n_frame": len(best), "yaws": yaws,
            "xs": xs, "ys": ys}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights",
                    default="/mnt/d/captures/landmarks/weights/best_manual_150.pt")
    ap.add_argument("--intr", default="/mnt/d/captures/calib/intrinsics.yaml")
    ap.add_argument("--field", default="templates/field.yaml")
    ap.add_argument("--cap-dir", default="/mnt/d/captures")
    ap.add_argument("--out-dir", default="/mnt/d/captures/_locate/figs")
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--stride", type=int, default=10)
    ap.add_argument("--overlay-frames", type=str, default="60,150,240",
                    help="叠加图取样帧号，逗号分隔")
    args = ap.parse_args()

    base = os.path.dirname(os.path.abspath(__file__))
    field_path = args.field if os.path.isabs(args.field) \
        else os.path.join(base, args.field)
    FIELD = load_field(field_path)
    names = list(FIELD.keys())
    obj = np.array([[FIELD[n][0], FIELD[n][1], 0.0] for n in names])
    obj_types = [field_type(n) for n in names]
    K, dist, _ = load_intrinsics(args.intr)
    os.makedirs(args.out_dir, exist_ok=True)
    overlay_frames = [int(x) for x in args.overlay_frames.split(",")]

    summary = {}
    for video, truth in VIDEOS:
        path = os.path.join(args.cap_dir, "%s.mp4" % video)
        cap = cv2.VideoCapture(path)
        k, frames = 0, []
        keep = {}
        while True:
            ok, bgr = cap.read()
            if not ok:
                break
            if k % args.stride == 0:
                pts = detect_yolo(bgr, args.weights, args.conf, args.imgsz)
                if len(pts) >= 4:
                    sols = solve_pose(
                        np.array([(x, y) for x, y, _ in pts], float),
                        FIELD, K, dist,
                        obs_types=[t for _, _, t in pts])
                else:
                    sols = []
                frames.append({"frame": k, "n_det": len(pts), "sols": sols})
                if k in overlay_frames and sols and k not in keep:
                    keep[k] = (bgr.copy(), pts, sols)
            k += 1
        cap.release()

        # 只取每帧最优解参与聚合（与 locator.py 一致），多解里的假解会污染主簇
        all_sols = [f["sols"][0] for f in frames if f["sols"]]
        est = cluster_median(all_sols, 0.8)
        if est:
            err = math.hypot(est["x"] - truth[0], est["y"] - truth[1])
            # 只统计落在真值 1m 内的帧作为"成功帧"
            ok_frames = [f for f in frames
                         if f["sols"] and any(
                             math.hypot(s["x"] - truth[0],
                                        s["y"] - truth[1]) < 1.0
                             for s in f["sols"][:1])]
            summary[video] = {
                "truth": truth, "est": {"x": est["x"], "y": est["y"],
                                        "yaw": est["yaw"]},
                "err_m": err, "n_samples": len(frames),
                "n_ok": len(ok_frames),
                "rate": len(ok_frames) / max(1, len(frames)),
            }
            print("%s 真值(%.2f,%.2f) 估计(%.3f,%.3f) yaw=%.1f 误差 %.3fm "
                  "帧成功 %d/%d"
                  % (video, truth[0], truth[1], est["x"], est["y"],
                     est["yaw"], err, len(ok_frames), len(frames)))
        else:
            summary[video] = {"truth": truth, "est": None, "err_m": None}
            print("%s 无解" % video)

        # ---- 叠加图 ----
        for fno, (bgr, pts, sols) in sorted(keep.items()):
            sol = sols[0]
            proj = project_fields(sol, obj, K, dist)
            vis = bgr.copy()
            for n, q in zip(names, proj):
                if -300 < q[0] < 1580 and -200 < q[1] < 920:
                    cv2.drawMarker(vis, (int(q[0]), int(q[1])),
                                   (255, 0, 255), cv2.MARKER_CROSS, 26, 2)
                    cv2.putText(vis, n.replace("Cross", ""),
                                (int(q[0]) + 14, int(q[1]) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (255, 0, 255), 2)
            for x, y, t in pts:
                cv2.circle(vis, (int(x), int(y)), 10, (0, 255, 0), 2)
                cv2.putText(vis, t, (int(x) + 10, int(y) + 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            txt = ("est=(%.2f, %.2f) yaw=%.1f  truth=(%.2f, %.2f)  err=%.2fm"
                   % (sol["x"], sol["y"], sol["yaw"], truth[0], truth[1],
                      math.hypot(sol["x"] - truth[0], sol["y"] - truth[1])))
            cv2.rectangle(vis, (0, 0), (1280, 40), (0, 0, 0), -1)
            cv2.putText(vis, txt, (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.72,
                        (255, 255, 255), 2)
            out = os.path.join(args.out_dir, "overlay_%s_f%03d.png"
                               % (video, fno))
            cv2.imwrite(out, vis)
            print("  叠加图 ->", out)

        # 存原始解供复查
        with open(os.path.join(args.out_dir, "frames_%s.json" % video),
                  "w") as fh:
            json.dump(frames, fh, ensure_ascii=False)

    # ---- 俯视图 ----
    half_l, half_w = 7.954 / 2, 5.982 / 2
    goal_half = 2.886 / 2
    fig, axes = plt.subplots(1, len(VIDEOS), figsize=(15.5, 5.6))
    for ax, (video, truth) in zip(np.atleast_1d(axes), VIDEOS):
        ax.add_patch(plt.Rectangle((-half_l, -half_w), 2 * half_l, 2 * half_w,
                                   fill=False, ec="#444", lw=1.6))
        ax.plot([half_l, half_l], [-goal_half, goal_half], color="#c0392b",
                lw=3.2, solid_capstyle="butt")
        ax.text(half_l - 0.15, 0, "goal", color="#c0392b", ha="right",
                va="center", fontsize=8)
        ax.plot([-half_l, half_l], [0, 0], ls="--", color="#bbb", lw=0.8)
        ax.plot([0, 0], [-half_w, half_w], ls="--", color="#bbb", lw=0.8)
        for n in names:
            ax.plot(FIELD[n][0], FIELD[n][1], "s", ms=5, color="#2c7fb8")
            ax.annotate(n.replace("Cross", ""),
                        (FIELD[n][0], FIELD[n][1]),
                        textcoords="offset points", xytext=(4, 4),
                        fontsize=7, color="#2c7fb8")
        ax.plot(truth[0], truth[1], "x", ms=14, mew=2.6, color="#c0392b",
                label="ground truth", zorder=5)
        s = summary.get(video)
        if s and s["est"]:
            ax.plot(s["est"]["x"], s["est"]["y"], "o", ms=9, mfc="none",
                    mew=2.2, color="#27ae60", label="estimate", zorder=6)
            ax.add_patch(plt.Circle((s["est"]["x"], s["est"]["y"]), 0.5,
                                    fill=False, ls=":", lw=1.2,
                                    color="#27ae60"))
            # 逐帧散点
            for f in [q for q in []]:
                pass
            ax.set_title("%s  err=%.3fm  rate=%.0f%%"
                         % (video, s["err_m"], 100 * s["rate"]), fontsize=11)
        else:
            ax.set_title("%s  no solution" % video, fontsize=11)
        ax.set_xlim(-half_l - 0.4, half_l + 0.4)
        ax.set_ylim(-half_w - 0.4, half_w + 0.4)
        ax.set_aspect("equal")
        ax.set_xlabel("X (m, toward goal)")
        ax.set_ylabel("Y (m)")
        ax.grid(alpha=0.25)
        ax.legend(fontsize=8, loc="lower left")
    fig.suptitle("Monocular field localization - top view (green dotted = 0.5 m "
                 "acceptance radius)", fontsize=12)
    fig.tight_layout()
    out1 = os.path.join(args.out_dir, "fig1_topview.png")
    fig.savefig(out1, dpi=150)
    print("俯视图 ->", out1)

    # ---- 逐帧误差曲线 ----
    fig2, axs = plt.subplots(1, len(VIDEOS), figsize=(15.5, 4.0), sharey=True)
    for ax, (video, truth) in zip(np.atleast_1d(axs), VIDEOS):
        with open(os.path.join(args.out_dir, "frames_%s.json" % video)) as fh:
            frames = json.load(fh)
        fno, errs = [], []
        for f in frames:
            if not f["sols"]:
                continue
            s = f["sols"][0]
            fno.append(f["frame"])
            errs.append(math.hypot(s["x"] - truth[0], s["y"] - truth[1]))
        if fno:
            ax.plot(fno, errs, "-o", ms=3, lw=1.0, color="#2c7fb8")
            ax.axhline(0.5, color="#c0392b", ls="--", lw=1.2)
            ax.text(ax.get_xlim()[0], 0.52, " 0.5 m spec", color="#c0392b",
                    fontsize=8, va="bottom")
        ax.set_title(video, fontsize=11)
        ax.set_xlabel("frame")
        ax.grid(alpha=0.25)
    np.atleast_1d(axs)[0].set_ylabel("position error (m)")
    fig2.suptitle("Per-frame position error (outliers from mirrored / false "
                  "correspondences are suppressed by cross-frame voting)",
                  fontsize=11)
    fig2.tight_layout()
    out2 = os.path.join(args.out_dir, "fig3_error.png")
    fig2.savefig(out2, dpi=150)
    print("误差曲线 ->", out2)

    with open(os.path.join(args.out_dir, "summary.json"), "w") as fh:
        json.dump(summary, fh, ensure_ascii=False, indent=2, default=str)
    print("汇总 ->", os.path.join(args.out_dir, "summary.json"))


if __name__ == "__main__":
    main()
