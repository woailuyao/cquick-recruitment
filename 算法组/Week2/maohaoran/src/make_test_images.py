"""生成第 2 周补充测试图（尺寸、颜色、噪声固定，可复现）。

为什么要自己生成：
    现有 3 张图覆盖了"目标清晰"和"强高光"，但缺少
    "目标很小"和"背景颜色接近"这两种典型失败场景。
    合成图便于精确知道目标真实位置，方便核对中心点误差。
"""

from pathlib import Path

import cv2
import numpy as np

from image_io import imwrite_unicode

ROOT = Path(__file__).resolve().parent.parent
IMAGE_DIR = ROOT / "data" / "images"


def make_small_target():
    """800x600，蓝色小圆半径 32，另有绿色/橙色干扰物。"""
    img = np.full((600, 800, 3), 235, dtype=np.uint8)
    cv2.circle(img, (620, 140), 32, (200, 90, 30), -1)      # BGR 蓝色小目标
    cv2.circle(img, (180, 430), 90, (60, 170, 60), -1)      # 绿色干扰
    cv2.circle(img, (520, 470), 70, (40, 140, 220), -1)     # 橙色干扰
    cv2.putText(img, "SMALL TARGET", (40, 70), cv2.FONT_HERSHEY_SIMPLEX,
                1.2, (60, 60, 60), 2, cv2.LINE_AA)
    imwrite_unicode(IMAGE_DIR / "04_small.jpg", img)


def make_similar_background():
    """900x600，背景是浅蓝灰色，目标蓝色圆与背景色相接近。"""
    img = np.full((600, 900, 3), (150, 145, 120), dtype=np.uint8)  # 偏蓝灰的 BGR
    # 与目标色相接近的大块背景区域，制造"背景同色"干扰
    cv2.rectangle(img, (560, 60), (860, 540), (170, 150, 125), -1)
    cv2.circle(img, (300, 300), 85, (200, 90, 30), -1)      # 蓝色目标
    cv2.circle(img, (720, 420), 45, (190, 100, 50), -1)     # 颜色相近的干扰块
    cv2.putText(img, "SIMILAR BACKGROUND", (40, 560), cv2.FONT_HERSHEY_SIMPLEX,
                1.0, (50, 50, 50), 2, cv2.LINE_AA)
    imwrite_unicode(IMAGE_DIR / "05_similar_bg.jpg", img)


def make_no_target():
    """640x480，只有橙色和绿色物体，用来验证"未检测到目标"分支。"""
    img = np.full((480, 640, 3), 240, dtype=np.uint8)
    cv2.circle(img, (200, 220), 100, (40, 140, 220), -1)    # 橙色
    cv2.rectangle(img, (380, 130), (560, 330), (60, 170, 60), -1)  # 绿色
    imwrite_unicode(IMAGE_DIR / "06_no_target.jpg", img)


def make_noisy_target():
    """800x600，蓝目标四周散布蓝色噪点，用来对比开运算的作用。"""
    rng = np.random.default_rng(42)
    img = np.full((600, 800, 3), 230, dtype=np.uint8)
    cv2.circle(img, (400, 300), 90, (200, 90, 30), -1)
    # 200 个 3~8 像素的蓝色噪点
    xs = rng.integers(0, 800, 200)
    ys = rng.integers(0, 600, 200)
    radii = rng.integers(1, 5, 200)
    for x, y, r in zip(xs, ys, radii):
        cv2.circle(img, (int(x), int(y)), int(r), (200, 90, 30), -1)
    imwrite_unicode(IMAGE_DIR / "07_noisy.jpg", img)


def main():
    IMAGE_DIR.mkdir(parents=True, exist_ok=True)
    make_small_target()
    make_similar_background()
    make_no_target()
    make_noisy_target()
    print(f"已生成补充测试图到 {IMAGE_DIR}")
    print("  04_small.jpg / 05_similar_bg.jpg / 06_no_target.jpg / 07_noisy.jpg")


if __name__ == "__main__":
    main()
