"""手动指定目标内部 ROI，查看 HSV 分位数；只提供调参证据，不自动认定最佳参数。"""

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import read_image


# 示例（在项目根目录运行）：
# python src/inspect_hsv.py --image data/images/01_clean.png --roi 190 190 100 70
# ROI 的四个数依次为左上角 x、y、宽度、高度，单位是像素。
# 只打印统计值，不生成图片，也不修改配置或原图。


def main():
    """读取指定图片，在目标内部取样，分别报告 H、S、V 的分布。"""
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--image', type=Path, required=True)
    p.add_argument(
        '--roi', type=int, nargs=4,
        metavar=('X', 'Y', 'W', 'H'), required=True,
    )
    a = p.parse_args()
    img = read_image(a.image)
    x, y, w, h = a.roi

    # 先检查取样框：宽高必须为正，不能超出图片范围。
    if (
        min(x, y) < 0 or min(w, h) <= 0
        or x + w > img.shape[1] or y + h > img.shape[0]
    ):
        p.error('ROI must be inside the image, with positive width/height.')

    # 数组切片先写 y 再写 x；转成 HSV 后将像素排成 N 行、3 列。
    # 每列分别是 H、S、V，便于按通道计算分位数。
    hsv = cv2.cvtColor(
        img[y:y + h, x:x + w], cv2.COLOR_BGR2HSV
    ).reshape(-1, 3)
    for q in (5, 50, 95):
        # p50 是中位数；p5 与 p95 描述主要分布，不是最终阈值上下限。
        # 边缘混色、背景、阴影都可能影响取样，需换光照和背景复查。
        print(f'p{q} HSV: {np.percentile(hsv, q, axis=0).round(1).tolist()}')
    print(
        'Sample only target interior; repeat across lighting conditions and compare background ROI.'
    )
    # 色相首尾相接，跨越 H=0/179 的红色不能直接用普通分位数概括。
    print(
        'H is circular. This percentile helper is intended for blue/green/orange, not red across 0/179.'
    )


if __name__ == '__main__':
    main()
