# -*- coding: utf-8 -*-
"""
任务一：图像基础操作
运行: python src/task1_basics.py
说明: 自动处理 data/images/ 下所有 png/jpg 图片，结果保存到 outputs/task1/
"""
import os
import cv2
import numpy as np

IMAGE_DIR = os.path.join("data", "images")
OUTPUT_DIR = os.path.join("outputs", "task1")


def save_compare(img, scaled, crop, gray, name):
    """把原图/缩放/裁剪/灰度统一高度后横向拼接，便于对比前后变化。"""
    def fit(g, th=220):
        r = th / g.shape[0]
        return cv2.resize(g, (int(g.shape[1] * r), th))

    def to_bgr(g):
        return cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)

    parts = [fit(img, 220), fit(scaled, 220), fit(crop, 220), fit(to_bgr(gray), 220)]
    compare = np.hstack(parts)
    cv2.imwrite(os.path.join(OUTPUT_DIR, f"{name}_compare.png"), compare)


def process_one(path):
    name = os.path.splitext(os.path.basename(path))[0]
    img = cv2.imread(path)
    if img is None:
        print(f"[FAIL] cannot read: {path}")
        return False

    h, w, c = img.shape
    print(f"[{name}] size = {w} x {h}, channels = {c}")

    # 1) 缩放为原来的 3/4
    scaled = cv2.resize(img, (w*3 // 4, h*3 // 4), interpolation=cv2.INTER_AREA)
    # 2) 裁剪中心 50% 区域（行 0~h/2，列 0~w/2）
    crop = img[0: h // 2, 0: w // 2]
    # 3) 灰度转换
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 保存全部结果
    cv2.imwrite(os.path.join(OUTPUT_DIR, f"{name}_orig.png"), img)
    cv2.imwrite(os.path.join(OUTPUT_DIR, f"{name}_scaled.png"), scaled)
    cv2.imwrite(os.path.join(OUTPUT_DIR, f"{name}_crop.png"), crop)
    cv2.imwrite(os.path.join(OUTPUT_DIR, f"{name}_gray.png"), gray)
    save_compare(img, scaled, crop, gray, name)

    # 重新读取验证
    check = [
        cv2.imread(os.path.join(OUTPUT_DIR, f"{name}_orig.png")),
        cv2.imread(os.path.join(OUTPUT_DIR, f"{name}_scaled.png")),
        cv2.imread(os.path.join(OUTPUT_DIR, f"{name}_crop.png")),
        cv2.imread(os.path.join(OUTPUT_DIR, f"{name}_gray.png")),
    ]
    ok = all(x is not None for x in check)
    print(f"[{name}] saved & re-read: {'OK' if ok else 'FAILED'}")
    return ok


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    exts = (".png", ".jpg", ".jpeg", ".bmp")
    files = sorted(f for f in os.listdir(IMAGE_DIR) if f.lower().endswith(exts))
    if not files:
        print(f"No images in {IMAGE_DIR}. Run src/make_test_images.py first.")
        return
    results = [process_one(os.path.join(IMAGE_DIR, f)) for f in files]
    print(f"\nDone: {sum(results)}/{len(results)} images processed.")


if __name__ == "__main__":
    main()
