# -*- coding: utf-8 -*-
"""曝光扫描：自动压低相机曝光，找既不过曝、又能检出标志的那一档。

背景：现场过曝像素占 17% 时，白线边缘饱和，检测置信度会掉到 0.0x，
      任何对比度增强都救不回来。只能从采集端压曝光。

    python exp_sweep.py --source 1

输出：每档的 过曝占比 / 平均亮度 / 检出数 / 最高置信度，
      以及 D:\\vision_data\\diag\\exp_XX.jpg 供人工核对。
"""

import argparse
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="1")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--model",
                    default=os.path.join(ROOT, "weights", "marker_v3_best.pt"))
    ap.add_argument("--imgsz", type=int, default=1920)
    ap.add_argument("--conf", type=float, default=0.01)
    ap.add_argument("--out-dir", default=r"D:\vision_data\diag")
    ap.add_argument("--values", default="-3,-4,-5,-6,-7,-8",
                    help="DSHOW 曝光档位（log2 秒，越小越暗）")
    return ap.parse_args()


def open_cam(idx, w, h):
    """优先 DSHOW（手动曝光支持最好），失败退 MSMF。"""
    for name, api in (("DSHOW", cv2.CAP_DSHOW), ("MSMF", cv2.CAP_MSMF)):
        cap = cv2.VideoCapture(idx, api)
        if not cap.isOpened():
            cap.release()
            continue
        try:
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        except Exception:
            pass
        ok_n = 0
        for _ in range(12):
            try:
                ok, _ = cap.read()
            except Exception:
                ok = False
            ok_n += 1 if ok else 0
        if ok_n >= 6:
            gw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            gh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            print(f"后端 {name}  {gw}x{gh}")
            return cap
        cap.release()
    raise SystemExit(f"摄像头 {idx} 打不开或读不到帧")


def grab(cap, n=15):
    f = None
    for _ in range(n):
        try:
            ok, x = cap.read()
        except Exception:
            ok = False
        if ok:
            f = x
    return f


def stats(img):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return float(g.mean()), float((g >= 250).mean() * 100.0)


def detect(model, img, imgsz, conf):
    r = model.predict(img, imgsz=imgsz, conf=conf, verbose=False)[0]
    top = max((float(b.conf) for b in r.boxes), default=0.0)
    return len(r.boxes), top


def main():
    args = parse_args()
    os.makedirs(args.out_dir, exist_ok=True)
    from ultralytics import YOLO
    model = YOLO(args.model)

    cap = open_cam(int(args.source), args.width, args.height)

    # 1) 当前自动曝光基线
    f = grab(cap)
    if f is None:
        raise SystemExit("读帧失败")
    m, clip = stats(f)
    n, top = detect(model, f, args.imgsz, args.conf)
    print(f"{'auto':>8}  亮度 {m:6.1f}  过曝 {clip:5.2f}%  n={n}  max_conf={top:.3f}")
    cv2.imwrite(os.path.join(args.out_dir, "exp_auto.jpg"), f)

    results = []
    for mode in (0.25, 1.0):   # 两种手动曝光枚举值，不同后端不一样
        for v in [float(x) for x in args.values.split(",")]:
            try:
                cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, mode)
                cap.set(cv2.CAP_PROP_EXPOSURE, v)
            except Exception:
                continue
            f = grab(cap)
            if f is None:
                continue
            m, clip = stats(f)
            n, top = detect(model, f, args.imgsz, args.conf)
            tag = f"{v:g}@m{mode:g}"
            print(f"{tag:>8}  亮度 {m:6.1f}  过曝 {clip:5.2f}%  n={n}  max_conf={top:.3f}")
            cv2.imwrite(os.path.join(args.out_dir, f"exp_{v:g}_m{mode:g}.jpg"), f)
            results.append((top, clip, m, v, mode))

    # 恢复自动曝光
    try:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
    except Exception:
        pass
    cap.release()

    if not results:
        print("手动曝光设置无效，相机可能不支持。改用转角度避开亮背景。")
        return
    best = max(results, key=lambda r: r[0])
    print(f"\n最佳: 曝光={best[3]:g} (mode={best[4]:g})  "
          f"max_conf={best[0]:.3f}  过曝={best[1]:.2f}%  亮度={best[2]:.1f}")
    if best[0] >= 0.30:
        print("结论: 压曝光后可以检出，把该档位写进采集脚本即可，无需重训。")
    else:
        print("结论: 压曝光仍不足，转手点方案 / 补数据重训。")


if __name__ == "__main__":
    main()
