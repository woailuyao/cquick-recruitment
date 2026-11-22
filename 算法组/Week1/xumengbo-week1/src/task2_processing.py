# -*- coding: utf-8 -*-
"""
任务二：基础图像处理对比
运行: python src/task2_processing.py
"""
import os
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

IMAGE = os.path.join("data", "images", "real_red_apple.jpg")
OUT = os.path.join("outputs", "task2")
os.makedirs(OUT, exist_ok=True)


def white_ratio(mask):
    """二值图中白色(非零)像素占比 %"""
    return float(np.mean(mask > 0) * 100)


def main():
    img = cv2.imread(IMAGE)
    if img is None:
        print("cannot read:", IMAGE)
        return
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    cv2.imwrite(os.path.join(OUT, "01_gray.png"), gray)
    print("[01] gray saved")

    # 2) 高斯滤波：核越大越模糊（平均绝对差越大）
    g5 = cv2.GaussianBlur(gray, (5, 5), 0)
    g31 = cv2.GaussianBlur(gray, (31, 31), 0)
    cv2.imwrite(os.path.join(OUT, "02_gaussian_k5.png"), g5)
    cv2.imwrite(os.path.join(OUT, "03_gaussian_k31.png"), g31)
    d5 = np.mean(np.abs(gray.astype(int) - g5.astype(int)))
    d31 = np.mean(np.abs(gray.astype(int) - g31.astype(int)))
    print(f"[02/03] gaussian |gray-g5|={d5:.2f}, |gray-g31|={d31:.2f}  (差值越大=模糊越强)")

    # 3) 阈值分割：阈值越低保留越多白色区域
    _, t100 = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)
    _, t200 = cv2.threshold(gray, 200, 255, cv2.THRESH_BINARY)
    adp = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                cv2.THRESH_BINARY, 51, 5)
    cv2.imwrite(os.path.join(OUT, "04_thresh_100.png"), t100)
    cv2.imwrite(os.path.join(OUT, "05_thresh_200.png"), t200)
    cv2.imwrite(os.path.join(OUT, "06_thresh_adaptive.png"), adp)
    print(f"[04/05/06] white% thresh100={white_ratio(t100):.1f}%, thresh200={white_ratio(t200):.1f}%, adaptive={white_ratio(adp):.1f}%")

    # 4) Canny：双阈值越高，边缘越少、越干净
    c1 = cv2.Canny(gray, 50, 150)
    c2 = cv2.Canny(gray, 100, 200)
    cv2.imwrite(os.path.join(OUT, "07_canny_50_150.png"), c1)
    cv2.imwrite(os.path.join(OUT, "08_canny_100_200.png"), c2)
    print(f"[07/08] canny edge pixels (50,150)={np.count_nonzero(c1)}, (100,200)={np.count_nonzero(c2)}")

    # 5) 形态学：对 thresh=100 的结果做开运算(去小噪点)和闭运算(补小洞)
    kernel = np.ones((5, 5), np.uint8)
    open_ = cv2.morphologyEx(t100, cv2.MORPH_OPEN, kernel)
    close_ = cv2.morphologyEx(t100, cv2.MORPH_CLOSE, kernel)
    cv2.imwrite(os.path.join(OUT, "09_morph_open.png"), open_)
    cv2.imwrite(os.path.join(OUT, "10_morph_close.png"), close_)
    print(f"[09/10] morph white% before={white_ratio(t100):.1f}%, open={white_ratio(open_):.1f}%, close={white_ratio(close_):.1f}%")

    # 综合对比图
    titles = ["Gray", "Gaussian k5", "Gaussian k31", "Thresh 100", "Thresh 200",
              "Adaptive", "Canny 50-150", "Canny 100-200", "Morph Open", "Morph Close"]
    imgs = [gray, g5, g31, t100, t200, adp, c1, c2, open_, close_]
    fig, axes = plt.subplots(2, 5, figsize=(16, 6.6))
    for ax, im, t in zip(axes.ravel(), imgs, titles):
        ax.imshow(im, cmap="gray")
        ax.set_title(t, fontsize=11)
        ax.axis("off")
    fig.suptitle("task2: image processing comparison (input: test1_dark_big.png)", fontsize=13)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "overview.png"), dpi=120)
    print("overview saved:", os.path.join(OUT, "overview.png"))


if __name__ == "__main__":
    main()
