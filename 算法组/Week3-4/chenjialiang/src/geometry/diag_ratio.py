
import os
import sys

import cv2
import numpy as np
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from localize import (CONF, CORNERS, DUP_DIST, K, MODEL,
                      PITCH_DEG, CAM_HEIGHT, pixel_depth)

def main():
    src = sys.argv[1]
    h = float(sys.argv[4]) if len(sys.argv) > 4 else CAM_HEIGHT
    pitch = np.radians(PITCH_DEG)
    model = YOLO(MODEL)
    cap = cv2.VideoCapture(src)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    ratios, classes = [], {}
    step = max(1, total // 40)
    for i in range(0, total, step):
        cap.set(cv2.CAP_PROP_POS_FRAMES, i)
        ok, frame = cap.read()
        if not ok:
            continue
        if frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))
        res = model(frame, conf=CONF, verbose=False)[0]
        raw = []
        for b in res.boxes:
            nm = model.names[int(b.cls[0])]
            if nm not in CORNERS:
                continue
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            raw.append([nm, (x1 + x2) / 2, (y1 + y2) / 2, float(b.conf[0])])
        raw.sort(key=lambda r: -r[3])
        keep = []
        for r in raw:
            if any(r[0] == q[0] and (r[1] - q[1]) ** 2 + (r[2] - q[2]) ** 2 < DUP_DIST ** 2
                   for q in keep):
                continue
            keep.append(r)
        if len(keep) != 2:
            continue

        n0, n1 = keep[0][0], keep[1][0]
        if not (n0.endswith("_left") != n1.endswith("_left")):
            continue
        fam = n0[:-5] if n0.endswith("_left") else n0[:-6]
        if (n1[:-5] if n1.endswith("_left") else n1[:-6]) != fam:
            continue

        d0 = pixel_depth(keep[0][1], keep[0][2], pitch, h)
        d1 = pixel_depth(keep[1][1], keep[1][2], pitch, h)
        if d0 is None or d1 is None:
            continue
        depth = 0.5 * (d0 + d1)
        p3 = np.array([CORNERS[n0][:2], CORNERS[n1][:2]], float)
        world_sep = float(np.hypot(*(p3[0] - p3[1])))
        expect = world_sep * K[0, 0] / depth
        actual = abs(keep[0][1] - keep[1][1])
        ratios.append(actual / max(expect, 1.0))
        classes[fam] = classes.get(fam, 0) + 1

    cap.release()
    if not ratios:
        print("没有左右成对的帧")
        return
    r = np.array(ratios)
    print(f"成对帧 {len(r)}，类别分布 {classes}")
    print(f"比值  min {r.min():.2f}  p10 {np.percentile(r,10):.2f}  "
          f"中位 {np.median(r):.2f}  p90 {np.percentile(r,90):.2f}  max {r.max():.2f}")
    for lo, hi in [(0, .4), (.4, .62), (.62, 1.55), (1.55, 2.5), (2.5, 9)]:
        n = int(((r >= lo) & (r < hi)).sum())
        print(f"  [{lo:.2f}, {hi:.2f}) : {n:4d} 帧  {100*n/len(r):5.1f}%  实际间距均值 {np.mean(r[(r>=lo)&(r<hi)]) if n else float('nan'):.2f}")

if __name__ == "__main__":
    main()
