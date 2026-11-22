# -*- coding: utf-8 -*-
"""检测器外观域诊断：同一张图用多种预处理各推一遍，打印框数与置信度。

用途：当实时画面检测置信度极低（0.0x）时，判断是“尺度问题”还是
“光照/对比度域差”，并找到能救回来的预处理方式。

    python diag_detect.py --image D:\\vision_data\\diag\\frame.jpg

输出：
  - 终端打印每种变体的框数、类别、置信度、框尺寸
  - D:\\vision_data\\diag\\enh_<变体>.jpg 供人工核对
"""

import argparse
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=r"D:\vision_data\diag\frame.jpg")
    ap.add_argument("--model",
                    default=os.path.join(ROOT, "weights", "marker_v3_best.pt"))
    ap.add_argument("--imgsz", type=int, default=1920)
    ap.add_argument("--conf", type=float, default=0.01)
    ap.add_argument("--out-dir", default=r"D:\vision_data\diag")
    return ap.parse_args()


def gamma(img, g):
    lut = np.array([((i / 255.0) ** g) * 255 for i in range(256)]).astype("uint8")
    return cv2.LUT(img, lut)


def build_variants(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    variants = {
        "raw": bgr,
        "gray": cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR),
        "clahe2": cv2.cvtColor(cv2.createCLAHE(2.0, (8, 8)).apply(gray),
                               cv2.COLOR_GRAY2BGR),
        "clahe4": cv2.cvtColor(cv2.createCLAHE(4.0, (8, 8)).apply(gray),
                               cv2.COLOR_GRAY2BGR),
        "gamma06": gamma(bgr, 0.6),
        "gamma06_clahe": cv2.cvtColor(
            cv2.createCLAHE(2.0, (8, 8)).apply(
                cv2.cvtColor(gamma(bgr, 0.6), cv2.COLOR_BGR2GRAY)),
            cv2.COLOR_GRAY2BGR),
    }
    return variants


def main():
    args = parse_args()
    img = cv2.imread(args.image)
    if img is None:
        raise SystemExit(f"读不到图片: {args.image}")
    h, w = img.shape[:2]
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print(f"图片 {w}x{h}  亮度 mean={gray.mean():.1f} std={gray.std():.1f}"
          f"  过曝像素(>=250)占比 {(gray >= 250).mean() * 100:.2f}%")

    from ultralytics import YOLO
    model = YOLO(args.model)
    os.makedirs(args.out_dir, exist_ok=True)

    best = None
    for name, v in build_variants(img).items():
        r = model.predict(v, imgsz=args.imgsz, conf=args.conf, verbose=False)[0]
        boxes = r.boxes
        print(f"\n[{name}] n={len(boxes)}")
        for b in boxes:
            cls = r.names[int(b.cls)]
            conf = float(b.conf)
            cx, cy, bw, bh = [float(x) for x in b.xywh[0]]
            print(f"   {cls:<7} {conf:.3f}  [{cx:6.0f},{cy:6.0f}] {bw:4.0f}x{bh:4.0f}")
        cv2.imwrite(os.path.join(args.out_dir, f"enh_{name}.jpg"), v)
        top = max((float(b.conf) for b in boxes), default=0.0)
        if best is None or top > best[1]:
            best = (name, top)

    print(f"\n最高置信度出现在变体: {best[0]}  ({best[1]:.3f})")
    print("判定: 若某变体最高置信度 >= 0.30，说明加该预处理即可，无需重训；")
    print("      若全部 < 0.10，说明当前机型/光照与训练集域差过大，需要重训或改方案。")


if __name__ == "__main__":
    main()
