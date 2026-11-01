"""Run the same HSV detector on five images; review correctness by eye."""

import argparse
import csv
from datetime import datetime, timezone, timedelta
from pathlib import Path
import platform

import cv2 as cv
import numpy as np


# 与已验证的任务三静态图片检测保持一致。
LOWER = np.array([20, 40, 50], dtype=np.uint8)
UPPER = np.array([45, 255, 255], dtype=np.uint8)
MIN_AREA = 500
IMAGES = ["clear.jpg", "small.jpg", "similar_background.jpg",
          "no_target.jpg", "shadow.jpg"]


def detect(image):
    """颜色分割 -> 按面积排序 -> 面积、宽高比和圆度筛选。"""
    hsv = cv.cvtColor(image, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, LOWER, UPPER)
    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    result = image.copy()
    detected, u, v, area = False, 0.0, 0.0, 0.0

    for contour in sorted(contours, key=cv.contourArea, reverse=True):
        candidate_area = cv.contourArea(contour)
        if candidate_area < MIN_AREA:
            continue
        x, y, w, h = cv.boundingRect(contour)
        if not 0.7 <= w / h <= 1.3:
            continue
        perimeter = cv.arcLength(contour, True)
        if perimeter == 0:
            continue
        circularity = 4 * np.pi * candidate_area / perimeter**2
        if circularity < 0.6:
            continue

        detected = True
        u, v = x + w / 2, y + h / 2
        area = candidate_area
        cv.drawContours(result, [contour], -1, (0, 0, 255), 2)
        cv.circle(result, (round(u), round(v)), 4, (0, 255, 0), -1)
        break

    message = (f"detected=True  u={u:.1f} v={v:.1f} area={area:.1f}"
               if detected else "No target detected")
    # 顶部文字仅用于展示；坐标仍以原始图片左上角为原点。
    cv.putText(result, message, (10, 24), cv.FONT_HERSHEY_SIMPLEX,
               0.55, (0, 0, 0), 3, cv.LINE_AA)
    cv.putText(result, message, (10, 24), cv.FONT_HERSHEY_SIMPLEX,
               0.55, (255, 255, 255), 1, cv.LINE_AA)
    return detected, u, v, area, mask, result


def read_image(path):
    # fromfile + imdecode 也支持 Windows 中含中文的路径。
    image = cv.imdecode(np.fromfile(str(path), dtype=np.uint8), cv.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"图片无法解码：{path}")
    return image


def save_image(path, pixels):
    path = Path(path)
    params = ([cv.IMWRITE_JPEG_QUALITY, 90] if path.suffix.lower() == ".jpg"
              else [cv.IMWRITE_PNG_COMPRESSION, 9])
    ok, data = cv.imencode(path.suffix, pixels, params)
    if not ok:
        raise OSError(f"保存失败：{path}")
    data.tofile(str(path))


def main():
    base = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser(description="五张网球图片批量检测")
    parser.add_argument("--images", type=Path, default=base / "images")
    parser.add_argument("--output", type=Path,
                        default=base / "results" / "bonus_five_images")
    args = parser.parse_args()

    missing = [name for name in IMAGES if not (args.images / name).is_file()]
    if missing:
        raise FileNotFoundError(
            f"请先放齐5张图片。目录：{args.images}；缺少：{', '.join(missing)}"
        )
    args.output.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone(timedelta(hours=8))).strftime("%Y-%m-%d %H:%M:%S +08:00")
    lines = ["姓名：胡钰涵 / Hu Yuhan", f"运行时间：{now}",
             f"Python={platform.python_version()} OpenCV={cv.__version__} NumPy={np.__version__}",
             f"HSV={LOWER.tolist()} ~ {UPPER.tolist()}, min_area={MIN_AREA}",
             "宽高比=0.7~1.3，圆度>=0.6；按面积从大到小选择第一个合格轮廓。"]
    rows = []
    for name in IMAGES:
        image = read_image(args.images / name)
        detected, u, v, area, mask, result = detect(image)
        stem = Path(name).stem
        save_image(args.output / f"{stem}_mask.png", mask)
        save_image(args.output / f"{stem}_detected.jpg", result)
        height, width = image.shape[:2]
        lines.append(f"{name}: {width}x{height}, detected={detected}, "
                     f"u={u:.1f}, v={v:.1f}, area={area:.1f}")
        rows.append({"图片": name, "宽": width, "高": height,
                     "实际有球": name != "no_target.jpg", "detected": detected,
                     "u": u, "v": v, "area": area,
                     "人工判断": "待检查标注图", "原因分析": ""})

    with (args.output / "summary.csv").open("w", encoding="utf-8-sig", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    lines += ["已处理5张图，保存5张掩膜、5张标注图及summary.csv。",
              "注意：detected=True仅表示选中了轮廓，是否选中网球需要人工检查。",
              f"输出目录：{args.output}"]
    (args.output / "run_log.txt").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))


if __name__ == "__main__":
    main()
