import cv2 as cv
import numpy as np
from datetime import datetime
from pathlib import Path

from _image_io import read_image, save_image
name = "small"
experiment = "AREA_B"
print("姓名：胡钰涵 / Hu Yuhan")
print(f"运行时间：{datetime.now():%Y-%m-%d %H:%M:%S}")
BASE = Path(__file__).resolve().parents[1]
image = read_image(BASE / "images" / f"{name}.jpg")
if image is None:
    raise SystemExit("图片读取失败，请检查路径")
print(f"当前图片：{name}.jpg")
hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
def show_pixel(event, x, y, flags, param):
    if event == cv.EVENT_LBUTTONDOWN:
        print(f"坐标：x={x}, y={y}")
        print(f"BGR：{image[y, x]}")
        print(f"HSV：{hsv[y, x]}")
        print()
lower = np.array([20, 40, 50])
upper = np.array([45, 255, 255])
print(f"实验编号：{experiment}")
mask = cv.inRange(hsv, lower, upper)
res = cv.bitwise_and(image, image, mask=mask)
contours, hierarchy = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
contours = sorted(contours, key=cv.contourArea, reverse=True)
result = image.copy()
min_area = 10000
print(f"最小面积阈值：{min_area}")
detected = False
u = 0.0
v = 0.0
target_area = 0.0
print(f"外轮廓总数：{len(contours)}")
for i, cnt in enumerate(contours):
    area = cv.contourArea(cnt)
    if area < min_area:
        continue
    x, y, w, h = cv.boundingRect(cnt)
    aspect_ratio = w / h
    if aspect_ratio < 0.7 or aspect_ratio > 1.3:
        continue
    perimeter = cv.arcLength(cnt, True)
    if perimeter == 0:
        continue
    circularity = 4 * np.pi * area / (perimeter ** 2)
    print(f"轮廓{i}：圆度={circularity:.3f}")
    if circularity < 0.6:
        continue
    detected = True
    target_area = area
    u = x + w / 2
    v = y + h / 2
    print(f"轮廓{i}：面积={area:.1f}，宽={w}，高={h}")
    print(f"中心坐标：u={u:.1f}，v={v:.1f}")
    cv.drawContours(result, [cnt], -1, (0, 0, 255), 2)
    cv.putText(result, str(i), (x, y),
        cv.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
    cv.circle(result, (round(u), round(v)),
        4, (0, 255, 0), -1)
    cv.putText(result, f"u={u:.1f}, v={v:.1f}", (x, y + h + 20),
        cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    break
if not detected:
    print("未找到符合筛选条件的目标")
    cv.putText(result, "No target detected", (20, 40),cv.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
print(f"检测结果：detected={detected}，"f"u={u:.1f}，v={v:.1f}，area={target_area:.1f}")
save_image(BASE / "results" / f"{name}_{experiment}_mask.png", mask)
save_image(BASE / "results" / f"{name}_{experiment}_res.jpg", res)
save_image(BASE / "results" / f"{name}_{experiment}_detected.jpg", result)
cv.imshow("image", image)
cv.setMouseCallback("image", show_pixel)
cv.imshow("mask", mask)
cv.imshow("res", res)
cv.imshow("detected", result)
cv.waitKey(0)
cv.destroyAllWindows()