import csv
from pathlib import Path

import cv2
import numpy as np

from io_utils import imread, imwrite

BASE = Path(__file__).resolve().parents[1]
IMG_DIR = BASE / "data" / "images"
OUT_DIR = BASE / "outputs" / "task3"
OUT_DIR.mkdir(parents=True, exist_ok=True)

MAX_SIDE = 800
KERNEL = np.ones((5, 5), np.uint8)

CONFIG = {
    "obj1.jpg": {"lower": (98, 120, 80), "upper": (120, 255, 255), "min_area": 5000},
    "obj2.jpg": {"lower": (95, 80, 50), "upper": (125, 255, 255), "min_area": 5000},
    "obj3.jpg": {"lower": (110, 90, 110), "upper": (145, 255, 255), "min_area": 3000},
}



def detect(img, lower, upper, min_area):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    raw = cv2.inRange(hsv, np.array(lower), np.array(upper))

    # 开运算去小白点，闭运算补小黑洞
    mask = cv2.morphologyEx(raw, cv2.MORPH_OPEN, KERNEL)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    targets = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:          # 面积阈值过滤噪声
            continue
        x, y, bw, bh = cv2.boundingRect(c)
        M = cv2.moments(c)
        if M["m00"] == 0:            # 防止除零
            continue
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        targets.append((area, x, y, bw, bh, cx, cy))

    targets.sort(key=lambda t: -t[0])
    return raw, mask, targets


def main():
    rows = []
    for name, cfg in CONFIG.items():
        path = IMG_DIR / name
        img = imread(str(path))
        if img is None:
            print(f"[跳过] {name} 读取失败: {path}")
            continue

        h, w = img.shape[:2]
        scale = MAX_SIDE / max(h, w)
        if scale < 1:
            img = cv2.resize(img, None, fx=scale, fy=scale,
                             interpolation=cv2.INTER_AREA)

        raw, mask, targets = detect(img, cfg["lower"], cfg["upper"], cfg["min_area"])

        vis = img.copy()
        for i, (area, x, y, bw, bh, cx, cy) in enumerate(targets, 1):
            cv2.rectangle(vis, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
            cv2.circle(vis, (cx, cy), 6, (0, 0, 255), -1)
            # putText 只支持 ASCII，中文会变成 ???
            cv2.putText(vis, f"#{i} ({cx},{cy}) A={int(area)}",
                        (x, max(y - 8, 14)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 2, cv2.LINE_AA)
            print(f"[{name}] 目标{i}: 中心=({cx},{cy}) "
                  f"外接框=({x},{y},{bw},{bh}) 面积={int(area)}")
            rows.append([name, i, cx, cy, x, y, bw, bh, int(area),
                         round(raw.mean() / 255 * 100, 2),
                         round(mask.mean() / 255 * 100, 2)])

        stem = Path(name).stem
        imwrite(str(OUT_DIR / f"{stem}_mask_raw.png"), raw)
        imwrite(str(OUT_DIR / f"{stem}_mask.png"), mask)
        imwrite(str(OUT_DIR / f"{stem}_result.png"), vis)
        print(f"[{name}] 掩膜占比: 原始={raw.mean() / 255 * 100:.1f}% "
              f"形态学后={mask.mean() / 255 * 100:.1f}%  目标数={len(targets)}")
        if not targets:
            print(f"          [!] 没有检出目标。先用 src/check_photo.py 检查这张图是否适合颜色分割，")
            print(f"              或调整 CONFIG['{name}'] 中的 HSV 范围 / min_area")

    # 坐标与面积记录，便于写进 README
    log_path = OUT_DIR / "detect_log.csv"
    with open(log_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "target", "center_x", "center_y",
                         "box_x", "box_y", "box_w", "box_h", "area",
                         "mask_raw_pct", "mask_clean_pct"])
        writer.writerows(rows)

    print(f"\n完成，结果在: {OUT_DIR}")
    print(f"坐标记录: {log_path}")


if __name__ == "__main__":
    main()




