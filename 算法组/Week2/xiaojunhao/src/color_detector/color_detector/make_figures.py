# -*- coding: utf-8 -*-
"""生成两张对比图：结果可视化、参数对比。

图上的文字用英文，因为 cv2.putText 不支持中文（会画成问号）。

用法: python -m color_detector.make_figures
"""

import sys
from pathlib import Path

import cv2
import numpy as np

from . import core

PKG_ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = PKG_ROOT / "data" / "images"
OUT_DIR = PKG_ROOT.parents[1]      # 视觉组/Week2/xiaojunhao

CELL = (560, 420)


def imread_u(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    return None if data.size == 0 else cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_u(path, img):
    ok, buf = cv2.imencode(".png", img)
    if ok:
        buf.tofile(str(path))


def label(img, text):
    strip = np.zeros((30, img.shape[1], 3), np.uint8)
    cv2.putText(strip, text, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)
    return cv2.vconcat([strip, img])


def cell(name, lower=(5, 170, 80), upper=(22, 255, 255),
         min_area=1500, morph=True, text=None):
    """跑一次检测，返回 (拼好的单元格, Target 或 None)。"""
    img = imread_u(IMG_DIR / name)
    target, _, _ = core.detect(img, lower, upper, min_area, morph)
    out = cv2.resize(core.annotate(img, target), CELL, interpolation=cv2.INTER_AREA)
    if text is None:
        text = "%s  %s" % (name.split(".")[0],
                           ("u=%d v=%d" % (round(target.u), round(target.v)))
                           if target else "no target")
    return label(out, text), target


def figure_visualization():
    """02_结果可视化.png：四张图的检测结果。"""
    cells = [cell("01_clear.jpg")[0], cell("02_small.jpg")[0],
             cell("03_similar_bg.jpg")[0], cell("04_shadow.jpg")[0]]
    grid = cv2.vconcat([cv2.hconcat(cells[:2]), cv2.hconcat(cells[2:])])
    path = OUT_DIR / "02_结果可视化.png"
    imwrite_u(path, grid)
    print("写出 %s  %dx%d" % (path, grid.shape[1], grid.shape[0]))


def figure_compare():
    """02_参数对比.png：上排比 S 下限，下排比最小面积。"""
    # A：03 图背景和球颜色接近，S 下限放宽到 120 时背景整片被选中
    a1, t1 = cell("03_similar_bg.jpg", lower=(5, 170, 80))
    a2, t2 = cell("03_similar_bg.jpg", lower=(5, 120, 80))
    a1 = cell("03_similar_bg.jpg", lower=(5, 170, 80),
              text="A1  S>=170  area=%d" % int(t1.area if t1 else 0))[0]
    a2 = cell("03_similar_bg.jpg", lower=(5, 120, 80),
              text="A2  S>=120  area=%d" % int(t2.area if t2 else 0))[0]

    # B：02 图目标比较小，最小面积抬到 6000 就把它自己滤掉了
    b1, tb1 = cell("02_small.jpg", min_area=1500)
    b2, tb2 = cell("02_small.jpg", min_area=6000)
    b1 = cell("02_small.jpg", min_area=1500,
              text="B1  min_area=1500  area=%d" % int(tb1.area if tb1 else 0))[0]
    b2 = cell("02_small.jpg", min_area=6000,
              text="B2  min_area=6000  %s"
                   % ("no target" if tb2 is None else "area=%d" % int(tb2.area)))[0]

    grid = cv2.vconcat([cv2.hconcat([a1, a2]), cv2.hconcat([b1, b2])])
    path = OUT_DIR / "02_参数对比.png"
    imwrite_u(path, grid)
    print("写出 %s  %dx%d" % (path, grid.shape[1], grid.shape[0]))


def main():
    figure_visualization()
    figure_compare()
    return 0


if __name__ == "__main__":
    sys.exit(main())
