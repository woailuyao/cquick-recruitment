# 俯角搜索：给定站位扫 0-40 度取重投影误差最小
import os
import sys

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
CFG = yaml.safe_load(open(os.path.join(ROOT, "config", "markers_c0.yaml"), encoding="utf-8"))
CORNERS = CFG["corners"]
MODEL = os.path.join(ROOT, "runs", "detect", "corner4", "weights", "best.pt")

K = np.array([[1653.274, 0.0, 646.380],
              [0.0, 1642.710, 307.337],
              [0.0, 0.0, 1.0]])

CONF = 0.40
DUP = 60.0

def pixel_to_body(u, v, pitch, h):
    x = (u - K[0, 2]) / K[0, 0]
    y = (v - K[1, 2]) / K[1, 1]
    d_cam = np.array([x, y, 1.0])
    f = np.array([np.cos(pitch), 0.0, -np.sin(pitch)])
    r = np.array([0.0, -1.0, 0.0])
    dn = np.cross(f, r)
    R = np.vstack([r, dn, f])
    d = R.T @ d_cam
    if d[2] >= -1e-6:
        return None
    lam = -h / d[2]
    p = lam * d
    return np.array([p[0], p[1]])

def solve(pts3, pts2, h, pitch_deg):
    P = np.asarray(pts3, float)[:, :2]
    body = []
    for u, v in pts2:
        p = pixel_to_body(u, v, np.radians(pitch_deg), h)
        if p is None:
            return None
        body.append(p)
    B = np.array(body)
    bc, wc = B - B.mean(0), P - P.mean(0)
    num = np.sum(bc[:, 0] * wc[:, 1] - bc[:, 1] * wc[:, 0])
    den = np.sum(bc[:, 0] * wc[:, 0] + bc[:, 1] * wc[:, 1])
    yaw = np.arctan2(num, den)
    c, s = np.cos(yaw), np.sin(yaw)
    C = P.mean(0) - np.array([[c, -s], [s, c]]) @ B.mean(0)
    return C

def main():
    if len(sys.argv) < 4:
        print("用法: fit_pitch.py <视频> <站位x> <站位y> [相机高度]")
        return
    src, gx, gy = sys.argv[1], float(sys.argv[2]), float(sys.argv[3])
    h = float(sys.argv[4]) if len(sys.argv) > 4 else 1.07

    model = YOLO(MODEL)
    cap = cv2.VideoCapture(src)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    samples = []
    hist = {}
    best_conf = 0.0
    step = max(1, total // 60)
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
            name = model.names[int(b.cls[0])]
            if name not in CORNERS:
                continue
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            raw.append([name, (x1 + x2) / 2, (y1 + y2) / 2, float(b.conf[0])])
        raw.sort(key=lambda r: -r[3])
        keep = []
        for r in raw:
            if any(r[0] == q[0] and (r[1] - q[1]) ** 2 + (r[2] - q[2]) ** 2 < DUP ** 2 for q in keep):
                continue
            keep.append(r)
        if keep:
            hist[len(keep)] = hist.get(len(keep), 0) + 1
            best_conf = max(best_conf, max(k[3] for k in keep))
        if len(keep) >= 2:
            samples.append(keep)
    cap.release()

    print(f"采样帧 {sum(hist.values())} 帧，其中：")
    for k in sorted(hist):
        print(f"  检出 {k} 个角点：{hist[k]} 帧")
    print(f"最高置信度 {best_conf:.3f}")
    print(f"可用于标定的帧（>=2 个角点）：{len(samples)} 帧")
    if not samples:
        return

    rows = []
    for deg in np.arange(0.0, 40.01, 0.5):
        errs = []
        for keep in samples:
            p3 = [[CORNERS[k[0]][0], CORNERS[k[0]][1], 0.0] for k in keep]
            p2 = [[k[1], k[2]] for k in keep]
            C = solve(p3, p2, h, deg)
            if C is None:
                continue
            errs.append(np.hypot(C[0] - gx, C[1] - gy))
        if errs:
            rows.append((deg, float(np.median(errs)), float(np.mean(errs))))

    rows.sort(key=lambda r: r[1])
    print()
    print(f"{'俯角(度)':>8} {'位置误差中位数(cm)':>18} {'平均(cm)':>10}")
    for deg, med, mean in rows[:6]:
        print(f"{deg:8.1f} {med*100:18.1f} {mean*100:10.1f}")
    print()
    best = rows[0]
    print(f"最佳俯角 {best[0]:.1f} 度，此时位置误差中位数 {best[1]*100:.1f} 厘米")
    print(f"把 config/markers_c0.yaml 里的 camera.pitch_deg 改成 {best[0]:.1f}")

if __name__ == "__main__":
    main()
