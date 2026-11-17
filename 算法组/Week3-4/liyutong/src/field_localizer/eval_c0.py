# -*- coding: utf-8 -*-
"""
C0 「现场测试」表自动生成器

考核要求交一张表：3 个以上测试点的参考坐标 / 估计坐标 / 位置误差 / 航向误差。
手工从 demo 界面读数既慢又抖（每点要取 10 帧平均），所以这里直接回放
现场采集的 gt 视频，逐帧解算后取中位数，一次把表填好。

参考坐标的来源：录制时三脚架摆在卷尺量出的位置上，并且「转动相机让球门中心
落在画面水平正中」——所以位姿是已知的，写在 现场录制清单.md 的表格里。

    P1 gt1  (0.0, 0.0)       yaw   0.00 deg
    P2 gt2  (+2.5, -1.2)     yaw +38.66 deg
    P3 gt3  (-2.5, +1.5)     yaw -12.99 deg

用法
    python eval_c0.py                                  # 用 gt1_1/gt2_1/gt3_1/gt3_2
    python eval_c0.py --height 0.39                    # 试另一个相机高度
    python eval_c0.py --stride 3                       # 每 3 帧取 1 帧，跑得快些
    python eval_c0.py --clip gt3_2                     # 只跑一段
    python eval_c0.py --markdown c0_table.md           # 顺手把表格写成文件

输出
    终端打印一段 markdown，直接粘进 README_提交.md 的「C0 现场测试」小节。
"""
import argparse
import math
import os

import cv2
import numpy as np

import localize as L

# (片段, 参考 x, 参考 y, 参考 yaw_deg)
DEFAULT_CLIPS = [
    ("gt1_1", 0.0, 0.0, 0.0),
    ("gt2_1", 2.5, -1.2, 38.66),
    ("gt3_1", -2.5, 1.5, -12.99),
    ("gt3_2", -2.5, 1.5, -12.99),
    # 2026-11-17 实验室重拍（按补拍计划三机位）。下面坐标是摆位**目标值**，
    # 真值以现场卷尺实测为准，回来后替换这三个数。
    ("gt1_new", 2.00, 2.50, -16.70),
    ("gt2_new", -2.00, 2.50, -163.30),
    ("gt3_new", 2.00, -2.50, 16.70),
]


def yaw_diff(a, b):
    """带符号的最小角度差，范围 (-180, 180]"""
    return (a - b + 180.0) % 360.0 - 180.0


def run_clip(path, mk, K, dist, R_gc, height_m, markers, goal, field_dims,
             tol, stride, conf, imgsz=640):
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        return None

    xs, ys, yws, ns, rs = [], [], [], [], []
    n_frame = n_solvable = n_flip = 0
    prev = None
    idx = 0

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        idx += 1
        if stride > 1 and idx % stride:
            continue
        n_frame += 1

        r = mk.predict(frame, conf=conf, imgsz=imgsz, verbose=False)[0]
        det = []
        for b in r.boxes:
            x1, y1, x2, y2 = (float(b.xyxy[0][0]), float(b.xyxy[0][1]),
                              float(b.xyxy[0][2]), float(b.xyxy[0][3]))
            det.append((mk.names[int(b.cls[0])], (x1 + x2) / 2.0, (y1 + y2) / 2.0,
                        float(b.conf[0])))
        if len(det) >= 2:
            n_solvable += 1

        H, W = frame.shape[:2]
        res = L.localize(det, K, dist, R_gc, height_m, markers, goal=goal,
                         tol_m=tol, require_margin=(H, W), field_dims=field_dims,
                         prev=prev)
        if res["valid"]:
            xs.append(res["x_camera_map"])
            ys.append(res["y_camera_map"])
            yws.append(res["yaw_camera_map"])
            ns.append(res["marker_count"])
            if res["residual"] is not None:
                rs.append(res["residual"])
            prev = dict(x=res["x_camera_map"], y=res["y_camera_map"],
                        yaw=res["yaw_camera_map"])
        elif res.get("marker_count", 0) < 2:
            prev = None
        if res.get("status") == "JUMP_REJECTED":
            n_flip += 1

    cap.release()
    if not xs:
        return dict(name=os.path.basename(path), n_frame=n_frame,
                    n_solvable=n_solvable, n_ok=0, n_flip=n_flip, fail=True)

    # 航向是多值的，取中位数前要先统一到同一支（否则 179 与 -179 的中位数会算成 0）
    yaws = np.array(yws, float)
    y0 = yaws[0]
    yaws = np.array([y0 + yaw_diff(v, y0) for v in yaws])

    return dict(
        name=os.path.basename(path),
        n_frame=n_frame, n_solvable=n_solvable, n_ok=len(xs), n_flip=n_flip,
        fail=False,
        x=float(np.median(xs)), y=float(np.median(ys)), yaw=float(np.median(yaws)),
        x_std=float(np.std(xs)), y_std=float(np.std(ys)),
        yaw_std=float(np.std(yaws)),
        marker_count=float(np.median(ns)) if ns else 0.0,
        residual=float(np.median(rs)) if rs else float("nan"),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--marker-model", default="runs_marker/baseline/weights/best.pt")
    ap.add_argument("--videos", default="videos")
    ap.add_argument("--clip", action="append", default=None,
                    help="只跑指定片段名（可重复）；不给就跑默认四段")
    ap.add_argument("--height", type=float, default=None,
                    help="覆盖相机高度（米）；默认用 camera_extrinsics.yaml 里的值")
    ap.add_argument("--tol", type=float, default=0.45)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--stride", type=int, default=1, help="每 N 帧取 1 帧")
    ap.add_argument("--markdown", default=None, help="把表格也写进这个文件")
    ap.add_argument("--imgsz", type=int, default=640,
                    help="推理分辨率；交点偏小时用 1280（离线评测不要求实时）")
    args = ap.parse_args()

    if not os.path.exists(args.marker_model):
        raise SystemExit(f"找不到标志模型 {args.marker_model}\n"
                         f"先跑 train_marker.py")

    from ultralytics import YOLO
    mk = YOLO(args.marker_model)

    markers, goal, fld = L.load_field()
    R_gc, h_extr = L.load_extrinsics()
    K, dist = L.load_intrinsics()
    height_m = args.height if args.height else h_extr
    field_dims = (float(fld["length_m"]), float(fld["width_m"]))
    print(f"相机高度 {height_m:.4f} m   tol {args.tol} m   stride {args.stride}")

    clips = DEFAULT_CLIPS
    if args.clip:
        want = set(args.clip)
        clips = [c for c in DEFAULT_CLIPS if c[0] in want]

    rows, detail = [], []
    for name, rx, ry, ryaw in clips:
        path = os.path.join(args.videos, name + ".mp4")
        if not os.path.exists(path):
            print(f"[skip] {path} 不存在")
            continue
        print(f"\n=== {name} ===  参考 ({rx:+.2f}, {ry:+.2f}, {ryaw:+.2f})")
        out = run_clip(path, mk, K, dist, R_gc, height_m, markers, goal,
                       field_dims, args.tol, args.stride, args.conf, args.imgsz)
        if out is None or out["fail"]:
            print(f"  无有效解（帧 {out['n_frame']}，可见点够的 {out['n_solvable']}）")
            rows.append(f"| {name} | {rx:+.2f} / {ry:+.2f} / {ryaw:+.2f} | 无有效解 | — | — |")
            continue

        ex = out["x"] - rx
        ey = out["y"] - ry
        ep = math.hypot(ex, ey)
        eyaw = yaw_diff(out["yaw"], ryaw)

        print(f"  帧 {out['n_frame']}  可见点够 {out['n_solvable']}  "
              f"有效解 {out['n_ok']}  跳变否决 {out['n_flip']}")
        print(f"  估计 ({out['x']:+.3f}, {out['y']:+.3f}, {out['yaw']:+.2f} deg)"
              f"   抖动 sd ({out['x_std']:.3f}, {out['y_std']:.3f}, {out['yaw_std']:.2f})")
        print(f"  误差 位置 {ep*100:.1f} cm   航向 {eyaw:+.2f} deg"
              f"   |  内点 {out['marker_count']:.0f}   残差 {out['residual']:.2f} px")
        if abs(eyaw) > 90:
            print("  !! 航向差超过 90 deg：锁到了对称的另一支，检查首帧是否对着球门")

        detail.append(dict(name=name, ep=ep, eyaw=abs(eyaw), n=out["n_ok"]))
        rows.append(
            f"| {name} | {rx:+.2f} / {ry:+.2f} / {ryaw:+.2f} "
            f"| {out['x']:+.2f} / {out['y']:+.2f} / {out['yaw']:+.2f} "
            f"| {ep*100:.1f} cm | {eyaw:+.2f} deg |")

    if not rows:
        raise SystemExit("一条都没跑出来，检查 models / videos 路径")

    md = ["| 测试点 | 参考 (x / y / yaw) | 估计 (x / y / yaw) | 平面位置误差 | 航向误差 |",
          "|---|---|---|---|---|"] + rows
    if detail:
        e_p = np.median([d["ep"] for d in detail]) * 100
        e_y = np.median([d["eyaw"] for d in detail])
        md += ["", f"四个点位中位误差：**平面 {e_p:.1f} cm / 航向 {e_y:.2f} deg**"
                   f"（各点取全部有效解的中位数，抖动见上面 sd）"]
    text = "\n".join(md)
    print("\n" + text)

    if args.markdown:
        with open(args.markdown, "w", encoding="utf-8") as f:
            f.write(text + "\n")
        print(f"\n已写入 {args.markdown}")


if __name__ == "__main__":
    main()
