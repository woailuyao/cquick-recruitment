
import os
import re
import sys
import time

from collections import Counter

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from fit_pitch import CONF, CORNERS, DUP, K, MODEL, pixel_to_body, solve

ROOT = os.path.normpath(os.path.join(HERE, "..", ".."))
CFG_WSL = os.path.join(ROOT, "config", "markers_c0.yaml")
CFG_WIN = "/mnt/e/CQuick/ws34/config/markers_c0.yaml"

def collect(src, model):
    cap = cv2.VideoCapture(src)
    total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    samples = []
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
            if any(r[0] == q[0] and (r[1] - q[1]) ** 2 + (r[2] - q[2]) ** 2 < DUP ** 2
                   for q in keep):
                continue
            keep.append(r)
        if len(keep) >= 2:
            samples.append(keep)
    cap.release()
    return samples, total

def fit(samples, gx, gy, h):
    # 俯角与高度耦合，扫俯角时必须固定 h
    rows = []
    for deg in np.arange(0.0, 40.01, 0.25):
        errs = []
        for keep in samples:
            p3 = [[CORNERS[k[0]][0], CORNERS[k[0]][1], 0.0] for k in keep]
            p2 = [[k[1], k[2]] for k in keep]
            C = solve(p3, p2, h, deg)
            if C is None:
                continue
            errs.append(np.hypot(C[0] - gx, C[1] - gy))
        if errs:
            rows.append((float(deg), float(np.median(errs)), len(errs)))
    rows.sort(key=lambda r: r[1])
    return rows

def apply_cfg(path, h, pitch):
    if not os.path.exists(path):
        return False
    s = open(path, encoding="utf-8").read()
    stamp = time.strftime("%Y-%m-%d %H:%M")
    s = re.sub(r"  height_m: [0-9.]+.*",
               f"  height_m: {h:.2f}        # {stamp} calibrate.py 写入", s)
    s = re.sub(r"  pitch_deg: [0-9.]+.*",
               f"  pitch_deg: {pitch:.2f}        # {stamp} calibrate.py 反解", s)
    open(path, "w", encoding="utf-8").write(s)
    return True

def infer_position(samples):
    pts = []
    for keep in samples:
        if len(keep) != 2:
            continue
        a, b = keep[0], keep[1]
        ia = a[0].endswith("_left")
        ib = b[0].endswith("_left")
        if ia == ib:
            continue
        fam_a = a[0][:-5] if ia else a[0][:-6]
        fam_b = b[0][:-5] if ib else b[0][:-6]
        if fam_a != fam_b:
            continue
        pts.append((min(a[1], b[1]), max(a[1], b[1]), fam_a))
    if not pts:
        return None, None, None
    us = np.array([[u0, u1] for u0, u1, _ in pts], float)
    fam = Counter(f for _, _, f in pts).most_common(1)[0][0]
    u_l, u_r = float(np.median(us[:, 0])), float(np.median(us[:, 1]))
    key = fam + "_left"
    half_w = CORNERS[key][1]
    fam_x = CORNERS[key][0]
    fx, cx = K[0, 0], K[0, 2]
    Z = 2.0 * half_w * fx / max(u_r - u_l, 1.0)
    y = (2.0 * cx - (u_l + u_r)) * Z / (2.0 * fx)
    x = fam_x - Z
    return x, y, (fam, u_l, u_r, Z, len(us))

def main():
    if len(sys.argv) < 4:
        print("用法: calibrate.py <视频> <站位x> <站位y> <相机高度>")
        print("      calibrate.py <视频> --auto <相机高度>")
        return
    src = sys.argv[1]
    auto = sys.argv[2] == "--auto"
    if not auto and len(sys.argv) < 5:
        print("用法: calibrate.py <视频> <站位x> <站位y> <相机高度>")
        return
    if auto:
        gx, gy, h = 0.0, 0.0, float(sys.argv[3])
    else:
        gx, gy, h = float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4])

    if not os.path.exists(src):
        print(f"找不到视频：{src}")
        return

    print(f"站位 ({gx:+.2f}, {gy:+.2f})  相机高度 {h:.2f} m")
    from ultralytics import YOLO
    model = YOLO(MODEL)

    samples, total = collect(src, model)
    print(f"视频 {total} 帧，可用于标定（>=2 个角点）{len(samples)} 帧")
    if len(samples) < 5:
        print("可用帧太少：检查画面里两个小禁区角是否都在，或相机是否面朝球门")
        return

    if auto:
        ix, iy, info = infer_position(samples)
        if ix is None:
            print("无法反解站位：没有左右成对的帧")
            return
        fam, u_l, u_r, Z, n = info
        print(f"图像反解：{fam}  左 u={u_l:.0f} 右 u={u_r:.0f}  间距 {u_r-u_l:.0f}px")
        print(f"           距离 Z={Z:.2f} m  →  站位 ({ix:+.2f}, {iy:+.2f})（{n} 帧中位）")
        gx, gy = ix, iy

    print("\n高度扫描（找让几何自洽的有效高度）：")
    print(f"{'高度(m)':>8} {'最佳俯角(度)':>14} {'误差中位数(cm)':>16}")
    h_rows = []
    for hh in np.arange(0.88, 1.29, 0.02):
        rr = fit(samples, gx, gy, float(hh))
        if rr:
            h_rows.append((float(hh), rr[0][0], rr[0][1]))
    for hh, dd, mm in h_rows:
        mark = "  <-- 当前输入" if abs(hh - h) < 0.011 else ""
        print(f"{hh:8.2f} {dd:14.2f} {mm * 100:16.1f}{mark}")
    h_best = min(h_rows, key=lambda r: r[2])
    print(f"\n自洽高度 {h_best[0]:.2f} m（俯角 {h_best[1]:.2f}°，误差 {h_best[2]*100:.1f} cm）")

    rows = fit(samples, gx, gy, h)
    print(f"\n{'俯角(度)':>8} {'误差中位数(cm)':>16}")
    for deg, med, n in rows[:6]:
        print(f"{deg:8.2f} {med * 100:16.1f}")

    best_deg, best_med, _ = rows[0]
    print(f"\n最佳俯角 {best_deg:.2f} 度，位置误差中位数 {best_med * 100:.1f} cm")

    if best_med <= 0.10:
        print("结论：可用，直接录演示")
    elif best_med <= 0.15:
        print("结论：勉强可用。先确认高度和站位是否量准（高度差 1 cm → 5 m 处偏 5 cm）")
    else:
        print("结论：不可用。优先检查 → 高度是否真的量准 / 站位是否准 / 两个角是否都在画面内")
        print("      若都确认无误仍偏高，说明该高度下几何太掠射，应抬高机位")

    ok_w = apply_cfg(CFG_WSL, h, best_deg)
    ok_e = apply_cfg(CFG_WIN, h, best_deg)
    print(f"\n配置写入：WSL {'OK' if ok_w else '失败'}，Windows {'OK' if ok_e else '失败'}")
    print("下一步：重启 localize.py（配置只在启动时读一次）")

if __name__ == "__main__":
    main()
