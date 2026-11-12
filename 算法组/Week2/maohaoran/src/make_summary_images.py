"""把零散的标注图和对比图拼成提交用的两张汇总图。

输出：
    outputs/summary/02_结果可视化.png  原图 vs 标注图（2x2）
    outputs/summary/02_参数对比.png    5 组对比图纵向拼接
"""

from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from image_io import imread_unicode, imwrite_unicode
from task2_visualize import panel

ROOT = Path(__file__).resolve().parent.parent
IMAGE_DIR = ROOT / "data" / "images"
TASK2_DIR = ROOT / "outputs" / "task2"
OUT_DIR = ROOT / "outputs" / "summary"


def hstack_padded(images):
    height = max(img.shape[0] for img in images)
    padded = [cv2.copyMakeBorder(img, 0, height - img.shape[0], 0, 0,
                                 cv2.BORDER_CONSTANT, value=(255, 255, 255))
              for img in images]
    return np.hstack(padded)


def vstack_with_gap(images, gap=16):
    width = max(img.shape[1] for img in images)
    fixed = [cv2.copyMakeBorder(img, 0, 0, 0, width - img.shape[1],
                                cv2.BORDER_CONSTANT, value=(255, 255, 255))
             for img in images]
    result = fixed[0]
    for img in fixed[1:]:
        spacer = np.full((gap, width, 3), 255, dtype=np.uint8)
        result = np.vstack([result, spacer, img])
    return result


def sign(img):
    """在图片底部加姓名拼音 + 生成时间戳，满足提交自查要求。"""
    text = f"maohaoran  {datetime.now():%Y-%m-%d %H:%M:%S}"
    band = np.full((44, img.shape[1], 3), 255, dtype=np.uint8)
    cv2.putText(band, text, (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                (0, 0, 0), 2, cv2.LINE_AA)
    return np.vstack([img, band])


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows = []
    for name, title in [("sample.jpg", "目标清晰"), ("sample3.jpg", "强高光")]:
        original = imread_unicode(IMAGE_DIR / name)
        annotated = imread_unicode(TASK2_DIR / f"vis_{Path(name).stem}.png")
        rows.append(hstack_padded([
            panel(original, f"原图：{name}（{title}）"),
            panel(annotated, "检测结果：轮廓/矩形/中心点/坐标"),
        ]))
    summary = vstack_with_gap(rows, gap=20)
    imwrite_unicode(OUT_DIR / "02_结果可视化.png", sign(summary))
    print("已生成 02_结果可视化.png")

    names = [
        "02_对比_A1_S下限.png",
        "02_对比_A2_H下限.png",
        "02_对比_B_最小面积.png",
        "02_对比_C_形态学.png",
        "02_对比_D_光照.png",
    ]
    images = []
    for file_name in names:
        img = imread_unicode(TASK2_DIR / file_name)
        width = 1000
        images.append(cv2.resize(
            img, (width, int(img.shape[0] * width / img.shape[1])),
            interpolation=cv2.INTER_AREA))
    comparison = vstack_with_gap(images, gap=18)
    imwrite_unicode(OUT_DIR / "02_参数对比.png", sign(comparison))
    print("已生成 02_参数对比.png")


if __name__ == "__main__":
    main()
