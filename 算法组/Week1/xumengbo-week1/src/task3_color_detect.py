# -*- coding: utf-8 -*-
"""
任务三：颜色目标提取与轮廓定位
运行: python src/task3_color_detect.py
"""
import os
import cv2
import numpy as np


def imread_utf8(path):
    """Windows 下含生僻字/中文路径也能读图"""
    return cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)


def imwrite_utf8(path, img):
    ext = os.path.splitext(path)[1]
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(path)
    return ok

IMAGE_DIR = os.path.join("data", "images")
OUT_DIR = os.path.join("outputs", "task3")
os.makedirs(OUT_DIR, exist_ok=True)

# 每张图的处理配置：目标名、图片文件、HSV 下限、HSV 上限、最小面积
TASKS = [
    {
        "name": "test1_dark_big", "target": "orange_rect",
        "file": "test1_dark_big.png",
        "lower": (5, 100, 100), "upper": (30, 255, 255), "min_area": 500,
    },
    {
        "name": "test2_bright_mid", "target": "green_circle",
        "file": "test2_bright_mid.png",
        "lower": (50, 100, 100), "upper": (80, 255, 255), "min_area": 500,
    },
    {
        "name": "test3_mid_small", "target": "blue_circle",
        "file": "test3_mid_small.png",
        "lower": (100, 100, 100), "upper": (135, 255, 255), "min_area": 500,
    },
]


def process(cfg):
    name, target = cfg["name"], cfg["target"]
    img = imread_utf8(os.path.join(IMAGE_DIR, cfg["file"]))
    if img is None:
        print(f"[{name}] FAIL read image")
        return None

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 1) inRange 生成掩膜
    mask = cv2.inRange(hsv, np.array(cfg["lower"]), np.array(cfg["upper"]))
    mask_before = mask.copy()

    # 2) 开运算去噪 + 闭运算补洞
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # 3) 查找轮廓，按面积过滤
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    valid = [c for c in contours if cv2.contourArea(c) >= cfg["min_area"]]
    if not valid:
        print(f"[{name}] FAIL no contour with area >= {cfg['min_area']}")
        return None
    cnt = max(valid, key=cv2.contourArea)

    # 4) 外接矩形 + 中心点
    x, y, w, h = cv2.boundingRect(cnt)
    M = cv2.moments(cnt)
    cx = int(M["m10"] / M["m00"])
    cy = int(M["m01"] / M["m00"])
    area = cv2.contourArea(cnt)

    # 5) 原图标注：绿色外接矩形 + 红色中心点 + 坐标文本
    result = img.copy()
    cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
    cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
    cv2.putText(result, f"({cx},{cy})", (x, max(y - 10, 15)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 保存 mask 与 result
    imwrite_utf8(os.path.join(OUT_DIR, f"mask_{name}.png"), mask)
    imwrite_utf8(os.path.join(OUT_DIR, f"result_{name}.png"), result)

    # 记录
    line = f"{name},{target},center=({cx},{cy}),area={area},rect=({x},{y},{w},{h}),status=OK"
    with open(os.path.join(OUT_DIR, "results.txt"), "a", encoding="utf-8") as f:
        f.write(line + "\n")
    print(f"[{name}] {line}")
    return mask, result


def main():
    # 清空上次记录
    rec = os.path.join(OUT_DIR, "results.txt")
    if os.path.exists(rec):
        os.remove(rec)

    all_masks, all_results = [], []
    for cfg in TASKS:
        out = process(cfg)
        if out is None:
            continue
        m, r = out
        all_masks.append(m)
        all_results.append(r)

    # 综合对比图：3 行（原图 / mask / result）3 列
    if all_masks:
        rows = []
        for cfg, m, r in zip(TASKS, all_masks, all_results):
            img = imread_utf8(os.path.join(IMAGE_DIR, cfg["file"]))
            m_bgr = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            rows.append(np.hstack([img, m_bgr, r]))
        overview = np.vstack(rows)
        imwrite_utf8(os.path.join(OUT_DIR, "overview.png"), overview)
        print("overview saved:", os.path.join(OUT_DIR, "overview.png"))
    print("results saved:", rec)


if __name__ == "__main__":
    main()
