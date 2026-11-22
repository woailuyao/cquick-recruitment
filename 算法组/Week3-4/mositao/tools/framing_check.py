# -*- coding: utf-8 -*-
"""机位取景自检：一条命令判断「当前画面能不能用」。

为什么需要：
    0.73 m 低机位下反复出现「要么识别不出来、要么识别错」。真实原因是几何——
    镜头离地 0.73 m 时地面入射角只有 5~10°，白线在画面里挤成窄带、标志交点被压扁，
    模型与几何法都会失效。与其一次次试，不如先用这个工具量出「当前画面够不够格」。

判据（三条同时满足才算合格）：
    1) 绿茵占比：画面下半部里草皮像素 ≥ 45%  —— 说明镜头真的对着场地而不是墙
       （判据见 tools/frame_metrics.py：HSV，不再用 RGB 绝对亮度差，室内暗光也稳）
    2) 白线像素：≥ 1.2%                        —— 说明画面里有足够多的场地线
    3) 标志点数：v11 去重后 ≥ 4                 —— 说明交点够做位姿

用法（项目根目录）：
    run.bat tools\\framing_check.py
    run.bat tools\\framing_check.py --index 1 --conf 0.25
    run.bat tools\\framing_check.py --frame outputs\\framing_check\\frame_191817.png
        （离线复核一张已保存的帧，不开相机）

输出：outputs/framing_check/frame_<时间>.png（当前帧）与终端结论。
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, WEIGHTS_DIR                        # noqa: E402
from tools.frame_metrics import grass_ratio, is_gray                       # noqa: E402
from tools.realtime_field import dedup_dets, white_mask                     # noqa: E402

CLASSES = ["LCross", "TCross", "XCross"]


def main() -> int:
    ap = argparse.ArgumentParser(description="机位取景自检")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_marker_v11.pt"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--dedup", type=float, default=35.0)
    ap.add_argument("--device", default="0")
    ap.add_argument("--frames", type=int, default=15)
    ap.add_argument("--frame", default=None,
                    help="用已保存的帧代替相机（离线复核/回归用）")
    args = ap.parse_args()

    if args.frame:
        frame = cv2.imread(args.frame)
        if frame is None:
            print(f"读不到帧：{args.frame}")
            return 2
        sharp = cv2.Laplacian(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
    else:
        cap = cv2.VideoCapture(args.index, cv2.CAP_MSMF)
        if not cap.isOpened():
            cap = cv2.VideoCapture(args.index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if not cap.isOpened():
            print(f"打不开相机 index={args.index}（可能被其他程序占用，或设备号不是 {args.index}）")
            return 2

        best = None
        for _ in range(args.frames):
            ok, fr = cap.read()
            if not ok or fr is None or fr.shape[1] != 1280:
                continue
            s = cv2.Laplacian(cv2.cvtColor(fr, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
            if best is None or s > best[0]:
                best = (s, fr)
        cap.release()
        if best is None:
            print("抓不到 1280×720 的帧：检查相机是否被其他窗口占用")
            return 1
        sharp, frame = best
    H, W = frame.shape[:2]

    # 1) 绿茵占比（下半部）——HSV 判据，理由见 tools/frame_metrics.py 顶部注释
    lower = frame[int(H * 0.45):, :]
    grass = grass_ratio(lower)
    gray_frame = is_gray(frame)

    # 2) 白线像素占比
    wm = white_mask(frame)
    white_frac = float((wm > 0).mean())

    # 3) 标志点数（v11 + 去重）
    from ultralytics import YOLO
    model = YOLO(args.weights)
    res = model.predict(frame, conf=args.conf, imgsz=640, device=args.device, verbose=False)[0]
    dets = []
    for bx in res.boxes:
        t = res.names[int(bx.cls[0])]
        if t not in CLASSES:
            continue
        x1, y1, x2, y2 = [float(v) for v in bx.xyxy[0]]
        dets.append({"type": t, "conf": float(bx.conf[0]), "u": (x1 + x2) / 2,
                     "v": (y1 + y2) / 2, "w": max(x2 - x1, y2 - y1)})
    dets = dedup_dets(dets, radius=args.dedup)

    ok_grass = grass >= 0.45
    ok_white = white_frac >= 0.012
    ok_marks = len(dets) >= 4
    ok = ok_grass and ok_white and ok_marks

    out_dir = OUTPUT_DIR / "framing_check"
    out_dir.mkdir(parents=True, exist_ok=True)
    if args.frame:
        p = Path(args.frame)
    else:
        stamp = datetime.now().strftime("%H%M%S")
        p = out_dir / f"frame_{stamp}.png"
        cv2.imwrite(str(p), frame)

    src = f"帧文件 {p.name}" if args.frame else f"相机 index={args.index}"
    print("=" * 62)
    print(f"取景自检｜{src}｜{W}×{H}｜清晰度 {sharp:.0f}")
    print("=" * 62)
    print(f"  绿茵占比（下半部） : {grass:6.1%}   门槛 45%   {'✅' if ok_grass else '❌'}")
    print(f"  白线像素占比       : {white_frac:6.2%}   门槛 1.2%  {'✅' if ok_white else '❌'}")
    print(f"  标志点数（去重后） : {len(dets):6d}    门槛 4     {'✅' if ok_marks else '❌'}")
    if gray_frame:
        print("  ⚠ 这一帧几乎没有颜色（灰度采集或过曝）：颜色类判据对它无效，"
              "先查采集格式与曝光")
    if dets:
        print("  检出：" + "  ".join(f"{d['type'][:1]}{d['conf']:.2f}@({d['u']:.0f},{d['v']:.0f})"
                                    for d in sorted(dets, key=lambda z: -z["conf"])))
    print("-" * 62)
    if ok:
        print("结论：✅ 取景合格，可以做外参标定")
        print("下一步：run.bat tools\\annotate.py --dir dataset\\manucalib --mode click "
              "--classes LCross,TCross,XCross")
    else:
        print("结论：❌ 取景不合格")
        if not ok_grass:
            print("  · 绿茵太少：镜头没对着场地，或俯角太小/太大；把镜头转向场内，"
                  "让画面下半部是草皮")
            print("    （若画面确实是草皮仍被判少，先看上面有没有灰度帧警告，再看曝光是否过暗）")
        if not ok_white:
            print("  · 白线太少：场地线不在视野里；靠近白线密集处（中圈、罚球区、球门区）")
        if not ok_marks:
            print("  · 交点不足 4 个：机位过低时视野被地面吃掉，建议抬高机位，或后退 0.5~1 m")
    try:
        shown = p.relative_to(ROOT)
    except ValueError:
        shown = p
    print(f"当前帧：{shown}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
