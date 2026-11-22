# -*- coding: utf-8 -*-
"""直接法外参标定：已知相机真值位姿时，最小化像素→地面→地图的重投影残差。
互补于 calibrate_extrinsics.py（后者经 2D 配准，会被假解污染）。"""
import argparse
import math
import os
import numpy as np
import cv2
import locator_fixed as F
from locator import load_field, load_intrinsics, detect_yolo


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True)
    ap.add_argument("--truth", required=True, help="x,y,yaw")
    ap.add_argument("--weights", default="/mnt/d/captures/landmarks/weights/best_manual_150.pt")
    a = ap.parse_args()
    tx, ty, tyaw = [float(v) for v in a.truth.split(",")]
    field = F.load_field("templates/field.yaml")
    K, dist, _ = load_intrinsics("/mnt/d/captures/calib/intrinsics.yaml")
    names = [k for k in field if F.field_type(k) is not None]
    mpts = {k: np.array(field[k], float) for k in names}
    types = {k: F.field_type(k) for k in names}

    cap = cv2.VideoCapture("/mnt/d/captures/%s.mp4" % a.video)
    data = []
    fi = 0
    while True:
        ok, bgr = cap.read()
        if not ok: break
        fi += 1
        if (fi - 1) % 10: continue
        lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
        l, aa, bb = cv2.split(lab)
        l = cv2.createCLAHE(2.0, (8, 8)).apply(l)
        bgr = cv2.cvtColor(cv2.merge([l, aa, bb]), cv2.COLOR_LAB2BGR)
        det = detect_yolo(bgr, a.weights, 0.05, 1280)
        if len(det) >= 4:
            data.append((np.array([[x, y] for x, y, _ in det], float),
                         [t for _, _, t in det]))
    cap.release()
    print("帧数", len(data))

    yr = math.radians(tyaw)
    Rm = np.array([[math.cos(yr), -math.sin(yr)], [math.sin(yr), math.cos(yr)]])
    best = None
    rows = []
    for h in np.arange(0.4, 1.51, 0.05):
        for pitch in np.arange(8.0, 51.0, 1.0):
            R = F.camera_to_base(pitch, 0.0)
            errs = []
            for px, det_types in data:
                g, ok = F.pixels_to_ground(px, K, dist, R, h)
                g = g[ok]
                dt = [det_types[i] for i in np.where(ok)[0]]
                if len(g) < 4: continue
                m = g @ Rm.T + np.array([tx, ty])   # 载体系→地图（已知真值）
                dmin = []
                for p, t in zip(m, dt):
                    ds = [np.linalg.norm(p - mpts[k]) for k in names
                          if types[k] == t]
                    dmin.append(min(ds) if ds else 99)
                errs.append(float(np.median(sorted(dmin)[:max(3, len(dmin)-1)])))
            if not errs: continue
            sc = float(np.median(errs))
            rows.append((sc, h, pitch))
            if best is None or sc < best[0]: best = (sc, h, pitch)
    rows.sort()
    print("TOP 10:")
    for sc, h, p in rows[:10]:
        print("  residual=%.3f m  h=%.2f  pitch=%.1f" % (sc, h, p))


if __name__ == "__main__":
    main()
