"""任务 1：HSV 颜色分割 + 轮廓筛选 + 目标中心点。

运行：
    cd 算法组/Week2/maohaoran
    python src/task1_detect.py

输出：
    - 终端：每张图的尺寸、是否检测到、中心点 (u, v)、面积、候选数
    - outputs/task1/mask_<图名>.png  二值掩膜
    - outputs/task1/detect_report.txt  本次运行的文字记录
"""

import sys
from pathlib import Path

import cv2

from color_detector import (DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                            DEFAULT_MIN_AREA, detect)
from image_io import imread_unicode, imwrite_unicode

ROOT = Path(__file__).resolve().parent.parent
IMAGE_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task1"


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # 同时支持 jpg / png，按文件名排序，保证每次运行顺序一致
    images = sorted(p for p in IMAGE_DIR.iterdir()
                    if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    if not images:
        print(f"错误：{IMAGE_DIR} 下没有找到测试图片")
        sys.exit(1)

    print("=" * 70)
    print("任务 1：HSV 颜色目标检测")
    print(f"目标颜色：蓝色   HSV 下界 {DEFAULT_HSV_LOWER}   上界 {DEFAULT_HSV_UPPER}")
    print(f"最小轮廓面积：{DEFAULT_MIN_AREA:.0f}   多目标选择规则：面积最大")
    print(f"测试图片目录：{IMAGE_DIR}")
    print("=" * 70)

    report_lines = [
        "任务 1 检测结果",
        f"HSV 下界: {DEFAULT_HSV_LOWER}",
        f"HSV 上界: {DEFAULT_HSV_UPPER}",
        f"最小面积: {DEFAULT_MIN_AREA:.0f}",
        "",
    ]

    total = len(images)
    detected_count = 0

    for index, path in enumerate(images, start=1):
        bgr = imread_unicode(path)
        if bgr is None:
            msg = f"[{index}/{total}] {path.name}  读取失败（文件损坏或路径错误）"
            print(msg)
            report_lines.append(msg)
            continue

        height, width = bgr.shape[:2]
        print(f"\n[{index}/{total}] {path.name}   尺寸 {width}x{height} (宽x高)")

        result, mask = detect(bgr)

        # 掩膜保存下来，方便对照"图像里到底被当成蓝色的是哪些像素"
        mask_path = OUT_DIR / f"mask_{path.stem}.png"
        imwrite_unicode(mask_path, mask)

        if result.detected:
            detected_count += 1
            print(f"    检测到目标: (u={result.u:.1f}, v={result.v:.1f})"
                  f"  面积={result.area:.0f}  中心算法={result.center_method}"
                  f"  候选轮廓数={len(result.candidates)}")
            report_lines.append(
                f"{path.name}: detected=True u={result.u:.1f} v={result.v:.1f}"
                f" area={result.area:.0f} method={result.center_method}"
                f" candidates={len(result.candidates)}")
        else:
            print(f"    未检测到目标（没有任何轮廓面积 >= {DEFAULT_MIN_AREA:.0f}）")
            report_lines.append(
                f"{path.name}: detected=False u=0.0 v=0.0 area=0.0 (未检测到目标)")

    print("\n" + "=" * 70)
    print(f"汇总：{total} 张图，检测到 {detected_count} 张，未检测到 {total - detected_count} 张")
    report_lines.append("")
    report_lines.append(f"汇总: {total} 张图, 检测到 {detected_count} 张")

    report_path = OUT_DIR / "detect_report.txt"
    report_path.write_text("\n".join(report_lines) + "\n", encoding="utf-8")
    print(f"文字记录已保存: {report_path}")
    print("=" * 70)


if __name__ == "__main__":
    main()
