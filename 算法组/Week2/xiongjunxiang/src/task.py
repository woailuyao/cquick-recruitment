from pathlib import Path
import argparse
import csv
import cv2 as cv
import numpy as np

LOWER = np.array([10, 170, 150])
UPPER = np.array([20, 255, 255])
MIN_AREA = 1000
KERNEL_SIZE = 7

image_path = Path(__file__).resolve().parent.parent / "data/images"
output_path = Path(__file__).resolve().parent.parent / "outputs"


def write(img, imgname):
    ok, tmp = cv.imencode(f"{name}_{imgname}.png", img)
    if ok:
        tmp.tofile(str(output_path / f"{name}_{imgname}.png"))


rows = []
for p in image_path.glob("*.jpg"):
    name = p.stem
    tmp = np.fromfile(str(p), dtype=np.uint8)
    img_bgr = cv.imdecode(tmp, 1)
    if img_bgr is None:
        print("读取失败")
        continue

    img_hsv = cv.cvtColor(img_bgr, cv.COLOR_BGR2HSV)
    mask = cv.inRange(img_hsv, LOWER, UPPER)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (KERNEL_SIZE, KERNEL_SIZE))
    opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)         # 去散点
    clean_mask = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel, iterations=2)  # 补孔
    write(clean_mask, "mask3")

    contours, _ = cv.findContours(clean_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    cands = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area < MIN_AREA:
            continue
        moments = cv.moments(contour)
        if moments["m00"] == 0:          # m00=0 的退化轮廓不能做除法，跳过
            continue
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        cands.append((area, center_x, center_y, cv.boundingRect(contour), contour))
    cands.sort(key=lambda t: -t[0])

    result = img_bgr.copy()
    for i, (area, cx, cy, (x, y, bw, bh), contour) in enumerate(cands):
        if i == 0:
            cv.drawContours(result, [contour], -1, (0, 255, 255), 3)
            cv.rectangle(result, (x, y), (x + bw, y + bh), (0, 0, 255), 3)
            cv.circle(result, (cx, cy), 10, (0, 0, 255), -1)
            cv.putText(result, f"u={cx}, v={cy}", (x, max(50, y - 15)),
                       cv.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3)
        else:
            cv.drawContours(result, [contour], -1, (128, 128, 128), 1)   # 次要候选画细线

    if not cands:
        print("  未检测到目标")
        cv.putText(result, "NO TARGET", (30, 100), cv.FONT_HERSHEY_SIMPLEX, 2.2, (0, 0, 255), 5)
        rows.append({"image": name, "detected": 0, "u": "", "v": "", "area": "",
                     "candidates": 0, "width": w, "height": h})
    else:
        area, cx, cy, box, _ = cands[0]
        print(f"  检测到目标: u={cx}, v={cy}, 面积={area:.0f}px, 候选轮廓 {len(cands)} 个")
    write(result, "result3")
