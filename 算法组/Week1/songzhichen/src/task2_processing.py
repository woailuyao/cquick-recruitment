# -*- coding: utf-8 -*-
"""任务二：基础图像处理对比

对应任务书要求：
  分别输出灰度图、高斯滤波结果、阈值分割结果、Canny 边缘图、形态学处理结果，
  保存到 outputs/task2/；至少更换 2 组参数进行对比。

运行方式（在项目根目录 C:\\cv 下）：
    python src/task2_processing.py

输出：outputs/task2/ 下 14 张中间结果 + 1 张总览拼图 + params.txt
"""
import os
import sys

import cv2
import numpy as np

from cv_io import imread_u, imwrite_u

IMG = "data/images/orange_normal.jpg"
OUT_DIR = "outputs/task2"

# ---------------- 参数集中放这里，方便换第二组做对比 ----------------
GAUSS_A = (5, 5)            # 第一组高斯核
GAUSS_B = (9, 9)            # 第二组高斯核
THRESH_A = 127              # 第一组固定阈值
THRESH_B = 80               # 第二组固定阈值
# 形态学演示单独用一个阈值。
# 一开始我直接拿 THRESH_A=127 的掩膜做开/闭运算，白像素只从 54.18% 变到 50.11%，
# 肉眼看不出变化，还以为形态学没生效。把连通域数打出来才明白：
# 127 那张掩膜里 489 个连通域，其中 483 个不到 50 像素，开完剩 24 个——
# 变化是有的，只是白色区域本身连成一大片，视觉上不明显。
# 换成 150（切在亮暗过渡带上）以后掩膜碎得多，723 个连通域开完剩 37 个，
# 对比一眼能看出来，所以演示图用 150 这组，127 那组也照常输出。
THRESH_MORPH = 150
CANNY_A = (50, 150)         # 第一组 Canny 低/高阈值
CANNY_B = (80, 200)         # 第二组 Canny 低/高阈值
KERNEL = np.ones((5, 5), np.uint8)   # 形态学核


def white_ratio(mask):
    """白色像素占比，用来量化阈值/形态学的影响。"""
    return 100.0 * float((mask > 0).sum()) / mask.size


def edge_count(edge):
    return int((edge > 0).sum())


def blob_count(mask):
    """连通域个数：噪点越多，个数越大，开运算后应明显减少。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    return len(contours)


def main():
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    os.makedirs(OUT_DIR, exist_ok=True)

    img = imread_u(IMG)
    if img is None:
        print("读不到图片:", IMG)
        return 1
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    print("原图:", IMG, img.shape, " 灰度均值=%.0f" % gray.mean())
    print("-" * 68)

    # ---- 滤波 ----
    gauss_a = cv2.GaussianBlur(gray, GAUSS_A, 0)
    gauss_b = cv2.GaussianBlur(gray, GAUSS_B, 0)
    blur_mean = cv2.blur(gray, GAUSS_A)
    blur_median = cv2.medianBlur(gray, 5)

    # ---- 阈值 ----
    _, th_a = cv2.threshold(gray, THRESH_A, 255, cv2.THRESH_BINARY)
    _, th_b = cv2.threshold(gray, THRESH_B, 255, cv2.THRESH_BINARY)
    th_adapt = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C,
                                     cv2.THRESH_BINARY, 11, 2)

    # ---- 边缘 ----
    canny_a = cv2.Canny(gray, CANNY_A[0], CANNY_A[1])
    canny_b = cv2.Canny(gray, CANNY_B[0], CANNY_B[1])
    canny_blur = cv2.Canny(gauss_a, CANNY_A[0], CANNY_A[1])   # 先滤波再求边缘

    # ---- 形态学：在一张有大量小噪点的掩膜上演示 ----
    _, mask_m = cv2.threshold(gray, THRESH_MORPH, 255, cv2.THRESH_BINARY)
    morph_open = cv2.morphologyEx(mask_m, cv2.MORPH_OPEN, KERNEL)
    morph_close = cv2.morphologyEx(mask_m, cv2.MORPH_CLOSE, KERNEL)

    # ---- 保存（名称带序号，方便按顺序看）----
    items = [
        ("01_gray", gray),
        ("02_gaussian_5x5", gauss_a),
        ("03_gaussian_9x9", gauss_b),
        ("04_blur_mean_5x5", blur_mean),
        ("05_blur_median_5", blur_median),
        ("06_thresh_127", th_a),
        ("07_thresh_80", th_b),
        ("08_thresh_adaptive", th_adapt),
        ("09_canny_50_150", canny_a),
        ("10_canny_80_200", canny_b),
        ("11_canny_after_gaussian", canny_blur),
        ("12_thresh_150_speckle", mask_m),
        ("13_morph_open", morph_open),
        ("14_morph_close", morph_close),
    ]
    for name, out in items:
        p = os.path.join(OUT_DIR, "%s.png" % name)
        if not imwrite_u(p, out):
            print("  [警告] 写入失败:", p)
    print("已保存 %d 张中间结果图到 %s" % (len(items), OUT_DIR))

    # ---- 对比分析：把「参数变了会怎样」量化出来 ----
    print("-" * 68)
    print("参数对比（这些数字就是 README 里要写的「影响」）")

    print("  [滤波] Canny 边缘像素数：原图 %d -> 高斯5x5 %d -> 高斯9x9 %d"
          % (edge_count(canny_a), edge_count(canny_blur),
             edge_count(cv2.Canny(gauss_b, CANNY_A[0], CANNY_A[1]))))
    print("         核越大，噪点造成的假边缘越少，但真实边缘也会变钝变少")

    print("  [阈值] 白色区域占比：阈值%d %.1f%% -> 阈值%d %.1f%%"
          % (THRESH_A, white_ratio(th_a), THRESH_B, white_ratio(th_b)))
    print("         阈值调低，更多偏暗像素被判成白，白色区域变大")

    print("  [边缘] Canny(%d,%d) 边缘 %d 个 -> Canny(%d,%d) 边缘 %d 个"
          % (CANNY_A[0], CANNY_A[1], edge_count(canny_a),
             CANNY_B[0], CANNY_B[1], edge_count(canny_b)))
    print("         阈值抬高只保留强边缘，弱边缘被丢掉，可能断线")

    print("  [形态] 用阈值%d 造一张边缘全是碎片的掩膜来演示：" % THRESH_MORPH)
    print("         开运算：白像素 %.1f%% -> %.1f%%，连通域 %d 个 -> %d 个"
          % (white_ratio(mask_m), white_ratio(morph_open),
             blob_count(mask_m), blob_count(morph_open)))
    print("         -> 开运算删掉了孤立的小碎片，剩下的是成片的亮区")
    print("         闭运算：白像素 %.1f%% -> %.1f%%，连通域 %d 个 -> %d 个"
          % (white_ratio(mask_m), white_ratio(morph_close),
             blob_count(mask_m), blob_count(morph_close)))
    print("         -> 闭运算把小暗缝填上，亮区被连成一片")

    # ---- 拼一张总览图，方便一眼对比（每格带标题，不用回头查控制台）----
    def to3(m, label):
        bgr = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(bgr, (0, 0), (bgr.shape[1], 26), (30, 30, 30), -1)
        cv2.putText(bgr, label, (8, 19), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (0, 255, 255), 1, cv2.LINE_AA)
        return bgr

    top = np.hstack([to3(gray, "gray"),
                     to3(gauss_a, "GaussianBlur 5x5"),
                     to3(th_a, "threshold %d" % THRESH_A)])
    mid = np.hstack([to3(mask_m, "threshold %d (fragments)" % THRESH_MORPH),
                     to3(morph_open, "MORPH_OPEN 5x5"),
                     to3(morph_close, "MORPH_CLOSE 5x5")])
    bot = np.hstack([to3(canny_a, "Canny %d/%d" % CANNY_A),
                     to3(canny_b, "Canny %d/%d" % CANNY_B),
                     to3(th_adapt, "adaptiveThreshold")])
    grid = np.vstack([top, mid, bot])
    imwrite_u(os.path.join(OUT_DIR, "00_side_by_side.png"), grid)
    print("-" * 68)
    print("总览拼图: outputs/task2/00_side_by_side.png")
    print("  第1行 灰度 / 高斯5x5 / 阈值127")
    print("  第2行 阈值150(多碎片) / 开运算后 / 闭运算后")
    print("  第3行 Canny(50,150) / Canny(80,200) / 自适应阈值")

    # ---- 参数清单，README 可以直接引用 ----
    lines = [
        "任务二 参数记录",
        "=" * 44,
        "输入图: %s" % IMG,
        "",
        "参数:",
        "  高斯核 A = %s    高斯核 B = %s" % (GAUSS_A, GAUSS_B),
        "  固定阈值 A = %d  固定阈值 B = %d" % (THRESH_A, THRESH_B),
        "  自适应阈值 blockSize=11, C=2",
        "  Canny A = %s    Canny B = %s" % (CANNY_A, CANNY_B),
        "  形态学核 = 5x5 全 1，演示阈值 = %d" % THRESH_MORPH,
        "",
        "量化结果:",
        "  Canny 边缘像素：原图=%d  高斯5x5后=%d" % (edge_count(canny_a), edge_count(canny_blur)),
        "  阈值%d 白占比=%.2f%%" % (THRESH_A, white_ratio(th_a)),
        "  阈值%d 白占比=%.2f%%" % (THRESH_B, white_ratio(th_b)),
        "  阈值%d 连通域=%d  开运算后=%d" % (THRESH_MORPH, blob_count(mask_m), blob_count(morph_open)),
        "  开运算后白占比=%.2f%%  闭运算后白占比=%.2f%%"
        % (white_ratio(morph_open), white_ratio(morph_close)),
    ]
    with open(os.path.join(OUT_DIR, "params.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print("参数记录: outputs/task2/params.txt")
    return 0


if __name__ == "__main__":
    sys.exit(main())
