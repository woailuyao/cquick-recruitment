from pathlib import Path
import cv2 as cv
import numpy as np
from matplotlib import pyplot as plt

image_path = Path(__file__).resolve().parent.parent / "data/images"
output_path = Path(__file__).resolve().parent.parent / "outputs" / "task3"


def write(img, imgname):
    ok, tmp = cv.imencode(f"{name}_{imgname}.png", img)
    if ok:
        tmp.tofile(str(output_path / f"{name}_{imgname}.png"))


for p in image_path.glob("*.jpg"):
    # print(p)
    name = p.stem
    tmp = np.fromfile(str(p), dtype=np.uint8)
    img_bgr = cv.imdecode(tmp, 1)
    if img_bgr is None:
        print("读取失败")
        continue
    img_rgb = cv.cvtColor(img_bgr, cv.COLOR_BGR2RGB)
    img_hsv = cv.cvtColor(img_rgb, cv.COLOR_RGB2HSV)
    lower = np.array([25, 50, 50])  # 绿色
    upper = np.array([95, 255, 255])
    mask = cv.inRange(img_hsv, lower, upper)
    write(mask, f"green")
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (7, 7))
    opened = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel, iterations=1)
    clean_mask = cv.morphologyEx(opened, cv.MORPH_CLOSE, kernel, iterations=2)
    write(clean_mask, f"mask")
    contours, _ = cv.findContours(clean_mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    result = img_bgr.copy()
    records: list[dict[str, int | float | str]] = []
    for contour in contours:
        area = cv.contourArea(contour)
        if area < 1000:
            continue
        x, y, w, h = cv.boundingRect(contour)
        moments = cv.moments(contour)
        if moments["m00"] == 0:
            continue
        center_x = int(moments["m10"] / moments["m00"])
        center_y = int(moments["m01"] / moments["m00"])
        object_id = len(records) + 1
        cv.drawContours(result, [contour], -1, (0, 255, 255), 2)
        cv.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv.circle(result, (center_x, center_y), 6, (0, 255, 255), -1)
        cv.putText(result, f"#{object_id} ({center_x}, {center_y})", (x, max(25, y - 8)),
                   cv.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
        records.append({
            "image": name,
            "object_id": object_id,
            "center_x": center_x,
            "center_y": center_y,
            "area": round(area, 1),
            "box_x": x,
            "box_y": y,
            "box_width": w,
            "box_height": h,
        })
    for item in records:
        print(f"  #{item['object_id']}: 中心=({item['center_x']}, {item['center_y']}), 面积={item['area']}")
    write(result, f"result")
