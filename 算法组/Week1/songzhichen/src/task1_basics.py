# -*- coding: utf-8 -*-
"""任务一：图像基础操作

对应任务书要求：
  1. 读取图片并检查是否读取成功，打印图像宽、高和通道数
  2. 分别完成缩放、指定区域裁剪和灰度转换
  3. 写入文件并检查能否重新读取；原始图片不得被覆盖

运行方式（在项目根目录下）：
    python src/task1_basics.py

输出：outputs/task1/
"""
import hashlib
import os
import sys

import cv2

from cv_io import imread_u, imwrite_u

# 项目内相对路径；三张图尺寸各不相同，正好检验脚本对不同尺寸的兼容性
IMAGES = [
    "data/images/orange_normal.jpg",     # 512 x 512
    "data/images/orange_cluttered.jpg",  # 512 x 480
    "data/images/taj.jpg",               # 236 x 146（小图）
]
OUT_DIR = "outputs/task1"


def md5(path):
    """算文件指纹，用来证明原图没被改动。"""
    h = hashlib.md5()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(65536), b""):
            h.update(chunk)
    return h.hexdigest()


def process(path):
    print("-" * 68)
    print("处理:", path)

    # ---- 1. 读取并检查 ----
    img = imread_u(path)
    if img is None:
        # imread 读不到时只返回 None，不抛异常，所以必须自己判断
        print("  [跳过] 读不到图片，路径是否存在:", os.path.exists(path))
        return False

    h, w, c = img.shape
    print("  宽=%d 高=%d 通道=%d  dtype=%s" % (w, h, c, img.dtype))

    tag = os.path.splitext(os.path.basename(path))[0]
    imwrite_u(os.path.join(OUT_DIR, "%s_00_original.png" % tag), img)

    # ---- 2. 三种操作 ----
    # 缩放：cv2.resize 的参数是 (宽, 高)，和 shape 的 (高, 宽, 通道) 顺序相反
    resized = cv2.resize(img, (w // 2, h // 2), interpolation=cv2.INTER_AREA)
    # 裁剪：切片是 img[行范围, 列范围]，即 [y1:y2, x1:x2]，取中间一半
    cropped = img[h // 4:h * 3 // 4, w // 4:w * 3 // 4]
    # 灰度
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("  缩放 -> %s   裁剪 -> %s   灰度 -> %s"
          % (resized.shape, cropped.shape, gray.shape))

    # ---- 3. 保存并回读验证 ----
    items = [("01_resized", resized), ("02_cropped", cropped), ("03_gray", gray)]
    for name, out in items:
        dst = os.path.join(OUT_DIR, "%s_%s.png" % (tag, name))
        ok_w = imwrite_u(dst, out)
        back = imread_u(dst, cv2.IMREAD_UNCHANGED)
        ok_r = back is not None and back.shape == out.shape
        print("  %-12s 写入=%-5s 回读=%-5s 尺寸一致=%s"
              % (name, ok_w, back is not None, ok_r))
    return True


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.makedirs(OUT_DIR, exist_ok=True)

    before = {}
    for p in IMAGES:
        if os.path.exists(p):
            before[p] = md5(p)

    ok = 0
    for p in IMAGES:
        if process(p):
            ok += 1

    # ---- 证明原图没被覆盖 ----
    print("-" * 68)
    for p, digest in before.items():
        same = os.path.exists(p) and md5(p) == digest
        print("原图未被修改: %-24s %s" % (os.path.basename(p), "是" if same else "否"))

    n = len([f for f in os.listdir(OUT_DIR) if f.lower().endswith(".png")])
    print("\n总结: 成功处理 %d/%d 张，outputs/task1/ 共 %d 个 png" % (ok, len(IMAGES), n))
    return 0 if ok == len(IMAGES) else 1


if __name__ == "__main__":
    sys.exit(main())
