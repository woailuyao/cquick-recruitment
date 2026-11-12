# -*- coding: utf-8 -*-
"""任务 3：HSV 颜色目标检测（TrackBar 交互调参，自包含，可独立运行）。

为每张输入图开一组 H/S/V 滑动条，实时显示掩膜 mask 和标注结果；按 Esc 退出后，
打印每张图的检测结果（中心点 + 面积），并保存到 Outputs/task3/ 与 detections.csv。

用法（在 guojunchen/ 目录下）:
    python src/task3_color_detect.py                    # 处理默认三张图
    python src/task3_color_detect.py data/xxx.jpg ...   # 处理指定图片
"""
import argparse
import csv
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]  # guojunchen/
OUTPUT_DIR = ROOT / "Outputs" / "task3"

DEFAULT_IMAGES = ["data/shilaike.jpeg", "data/avatar.jpg", "data/chengliuxiang.webp"]


def _imread(path: str, flags: int = cv2.IMREAD_COLOR) -> Optional[np.ndarray]:
    """读图。Windows 下 cv2.imread 对含中文的路径会失败，改用 np.fromfile + imdecode。"""
    data = np.fromfile(path, dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def _save(img: np.ndarray, path: Path) -> bool:
    """存图。中文文件名用 imencode(...).tofile() 写，避免 imwrite 静默失败。"""
    ok, buf = cv2.imencode(path.suffix or ".png", img)
    if ok:
        buf.tofile(str(path))
    return ok


def _make_trackbars(win: str) -> None:
    """为一个窗口创建 6 个 HSV 滑动条（回调是空函数）。"""
    cv2.namedWindow(win)
    cv2.resizeWindow(win, 640, 240)
    cv2.createTrackbar("H Min", win, 0, 179, lambda _: None)
    cv2.createTrackbar("H Max", win, 179, 179, lambda _: None)
    cv2.createTrackbar("S Min", win, 0, 255, lambda _: None)
    cv2.createTrackbar("S Max", win, 255, 255, lambda _: None)
    cv2.createTrackbar("V Min", win, 0, 255, lambda _: None)
    cv2.createTrackbar("V Max", win, 255, 255, lambda _: None)


def _read_trackbars(win: str) -> Tuple[np.ndarray, np.ndarray]:
    """读取窗口里 6 个滑动条的值，返回 (lower, upper) HSV 数组。"""
    h_min = cv2.getTrackbarPos("H Min", win)
    s_min = cv2.getTrackbarPos("S Min", win)
    v_min = cv2.getTrackbarPos("V Min", win)
    h_max = cv2.getTrackbarPos("H Max", win)
    s_max = cv2.getTrackbarPos("S Max", win)
    v_max = cv2.getTrackbarPos("V Max", win)
    return np.array([h_min, s_min, v_min]), np.array([h_max, s_max, v_max])


def detect_frame(
    hsv: np.ndarray,
    bgr: np.ndarray,
    lower: np.ndarray,
    upper: np.ndarray,
    area_threshold: float,
    morph_ksize: int = 5,
) -> Tuple[np.ndarray, np.ndarray, List[Tuple[int, int, int]]]:
    """单帧检测：inRange -> 开运算 -> 轮廓筛选 -> 标注。

    返回 (mask, 标注图, [(cx, cy, area), ...])。
    """
    mask = cv2.inRange(hsv, lower, upper)

    # 先腐蚀再膨胀 = 开运算：去掉孤立噪点、平滑目标边缘
    kernel = np.ones((morph_ksize, morph_ksize), np.uint8)
    mask = cv2.erode(mask, kernel)
    mask = cv2.dilate(mask, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result = bgr.copy()
    records: List[Tuple[int, int, int]] = []
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area > area_threshold:
            x, y, w, h = cv2.boundingRect(cnt)
            cx, cy = x + w // 2, y + h // 2
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(result, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(result, "(%d,%d) Area:%d" % (cx, cy, int(area)),
                        (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.6, (255, 0, 0), 2)
            records.append((cx, cy, int(area)))
    return mask, result, records


def main():
    ap = argparse.ArgumentParser(description="任务3：HSV 颜色目标检测（TrackBar 调参）")
    ap.add_argument("images", nargs="*", help="输入图片路径（缺省处理 data/ 下三张）")
    ap.add_argument("--area", type=float, default=500.0, help="轮廓面积阈值")
    ap.add_argument("--morph", type=int, default=5, help="腐蚀/膨胀核大小")
    args = ap.parse_args()

    images = args.images or DEFAULT_IMAGES
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # 读图 + 建窗口：每张图一个独立的 TrackBar 窗口；HSV 只算一次（图不变，变的是滑动条）
    sessions = []  # [(路径, BGR 图, HSV 图, 窗口名)]
    for name in images:
        bgr = _imread(name, cv2.IMREAD_COLOR)
        if bgr is None:
            print("[%s] 无法读取图像" % name)
            continue
        win = "TrackBars_%s" % Path(name).stem
        _make_trackbars(win)
        sessions.append((name, bgr, cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV), win))
        print("已载入 %s，窗口 %s" % (name, win))

    if not sessions:
        return

    print("姓名拼音：guojunchen")
    print("运行时间：", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("拖动滑动条调 HSV 范围，按 Esc 退出并保存结果")
    print("=" * 60)

    records_by_img = {}  # 图名 -> [(cx, cy, area), ...]（只保留最后一帧，即最终调参结果）
    while True:
        for name, bgr, hsv, win in sessions:
            lower, upper = _read_trackbars(win)
            mask, result, records = detect_frame(hsv, bgr, lower, upper, args.area, args.morph)

            stem = Path(name).stem
            records_by_img[stem] = records
            _save(mask, OUTPUT_DIR / ("mask_%s.png" % stem))
            _save(result, OUTPUT_DIR / ("result_%s.png" % stem))
            cv2.imshow("mask_%s" % stem, mask)
            cv2.imshow("result_%s" % stem, result)

        if cv2.waitKey(1) & 0xFF == 27:  # 按 Esc 退出
            break

    cv2.destroyAllWindows()

    # 打印并保存检测结果
    csv_path = OUTPUT_DIR / "detections.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["image", "center_x", "center_y", "area"])
        for stem, records in records_by_img.items():
            for cx, cy, area in records:
                writer.writerow([stem, cx, cy, area])

    for stem, records in records_by_img.items():
        for cx, cy, area in records:
            print("%s: 中心坐标 (%d, %d)，轮廓面积 %d" % (stem, cx, cy, area))
    print("结果已保存到 %s" % csv_path)


if __name__ == "__main__":
    main()
