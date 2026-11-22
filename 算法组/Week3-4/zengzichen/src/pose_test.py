#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_test.py —— 三姿态定位测试(录像, 现场也可)
对 1~3 段录像各采 N 帧做定位, 报 x/y/yaw 的均值±标准差, 并统计 ok/ambiguous/insufficient。
可与真值对比给出误差(诚实报告, 不造假)。

用法:
  三姿态(无真值):
    /root/yolo_env/bin/python pose_test.py --video v1.mp4 v2.mp4 v3.mp4 --frames 20
  三姿态(带真值, 顺序和 video 一一对应):
    /root/yolo_env/bin/python pose_test.py --video v1.mp4 v2.mp4 v3.mp4 \
        --truth "2.0,-1.5,30" "1.5,-1.0,-60" "2.5,0.5,10" --frames 20
  (真值格式 "x,y,yaw", yaw 单位度)
"""
import argparse, os, sys, math, csv
import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import localize


def parse_truth(s):
    x, y, yaw = [float(v) for v in s.split(",")]
    return x, y, yaw


def mean_angle(degs):
    r = np.radians(np.asarray(degs, float))
    return math.degrees(math.atan2(np.sin(r).mean(), np.cos(r).mean()))


def wrap180(d):
    return (d + 180) % 360 - 180


def circ_median(degs):
    """圆中位数: 取使所有样本到它的圆距离之和最小的那个样本值。
    对'两个候选位姿来回跳'(镜像歧义)的场景, 比均值稳健得多。"""
    a = np.asarray(degs, float)
    if a.size == 0:
        return 0.0
    best, best_cost = float(a[0]), None
    for cand in a:
        cost = float(np.sum(np.abs(wrap180(a - cand))))
        if best_cost is None or cost < best_cost:
            best, best_cost = float(cand), cost
    return best


def process(source, K, dist, mp, W, H, nframes, conf, tol, refine):
    cap = cv2.VideoCapture(source)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 0
    step = max(1, total // max(1, nframes)) if total > nframes else 1
    rows = []
    fidx = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        if fidx % step == 0:
            sol, dets, obs = localize.localize_image(img, K, dist, mp, W, H,
                                                     conf=conf, tol=tol, refine=refine)
            rows.append(dict(frame=fidx, x=sol["x"], y=sol["y"], yaw=sol["yaw"],
                             n=sol["marker_count"], res=sol["residual"],
                             valid=sol["valid"], status=sol["status"]))
        fidx += 1
    cap.release()
    return rows


def summarize(name, rows, truth):
    est = [r for r in rows if r["status"] in ("ok", "ambiguous", "poor")]  # 可采信解(出界/不足不计)
    print(f"\n=== {name} ===")
    print(f"  采样 {len(rows)} 帧: ok {sum(r['status']=='ok' for r in rows)}, "
          f"ambiguous {sum(r['status']=='ambiguous' for r in rows)}, "
          f"insufficient {sum(r['status']=='insufficient' for r in rows)}, "
          f"out_of_field {sum(r['status']=='out_of_field' for r in rows)}, "
          f"poor {sum(r['status']=='poor' for r in rows)}")
    if not est:
        print("  没有可采信的位姿帧(匹配不足或解超出场地边界), 检查相机是否拍到标志")
        return rows
    xs = [r["x"] for r in est]; ys = [r["y"] for r in est]; ysaw = [r["yaw"] for r in est]
    # 稳健聚合: 用中位数作报告值(考核允许滤波, 见细则2.10); 均值±std 仅作参考
    mx, my = float(np.median(xs)), float(np.median(ys))
    myaw = circ_median(ysaw)
    ax, ay = float(np.mean(xs)), float(np.mean(ys))
    ayaw = mean_angle(ysaw)
    print(f"  x   = {mx:+.3f} m   (均值 {ax:+.3f} ± {np.std(xs):.3f})")
    print(f"  y   = {my:+.3f} m   (均值 {ay:+.3f} ± {np.std(ys):.3f})")
    print(f"  yaw = {myaw:+.1f}°  (圆均值 {ayaw:+.1f} ± "
          f"{np.std([wrap180(a-ayaw) for a in ysaw]):.2f}°)")
    if truth:
        ex = mx - truth[0]; ey = my - truth[1]
        eyaw = wrap180(myaw - truth[2])
        print(f"  [真值 x={truth[0]} y={truth[1]} yaw={truth[2]}] "
              f"误差(按中位数) dx={ex:+.3f} dy={ey:+.3f} dyaw={eyaw:+.2f}°")
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", nargs="+", required=True, help="1~3 段录像")
    ap.add_argument("--truth", nargs="+", help="与 video 一一对应的 \"x,y,yaw\"")
    ap.add_argument("--frames", type=int, default=20)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--tol", type=float, default=0.20)
    ap.add_argument("--norefine", action="store_true")
    ap.add_argument("--out", default="pose_result.csv")
    args = ap.parse_args()

    if args.truth and len(args.truth) != len(args.video):
        print("错误: --truth 个数要和 --video 个数一致"); sys.exit(1)

    K, dist, W, H = localize.load_intrinsics()
    mp = localize.load_map()

    all_rows = []
    for i, src in enumerate(args.video):
        truth = parse_truth(args.truth[i]) if args.truth else None
        rows = process(src, K, dist, mp, W, H, args.frames, args.conf, args.tol,
                       refine=not args.norefine)
        for r in rows:
            r["pose"] = i + 1
            r["source"] = os.path.basename(src)
        all_rows.extend(rows)
        summarize(os.path.basename(src), rows, truth)

    with open(args.out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["pose", "source", "frame", "x", "y", "yaw",
                                          "n", "res", "valid", "status"])
        w.writeheader()
        for r in all_rows:
            w.writerow(r)
    print(f"\n已写出明细: {args.out}")


if __name__ == "__main__":
    main()