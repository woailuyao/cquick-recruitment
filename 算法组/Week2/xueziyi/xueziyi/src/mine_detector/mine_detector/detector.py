# -*- coding: utf-8 -*-
"""
检测模块（任务 1+2 的代码）
================================================================
这个文件就是你原来写的 my_detect.py，搬进节点包里，只做了两处改动：
  1. OUT_DIR 改成可用环境变量覆盖（默认 ~/vision_ws/mine/out）
  2. 末尾的 4 行 main(...) 换成：detect() 包装函数 + __main__ 里的批量入口

其余函数（color_choose / img_read / img_write / mask_verify /
print_rect / find_center / print_center / main）全部保持原样。
"""

import os
import sys
from pathlib import Path

import cv2
import numpy as np  # 矩阵运算

# 输出目录：默认 ~/vision_ws/mine/out，可用环境变量 MINE_OUT 覆盖
OUT_DIR = Path(os.environ.get("MINE_OUT", os.path.expanduser("~/vision_ws/mine/out")))
OUT_DIR.mkdir(parents=True, exist_ok=True)


CANDIDATES = {
    "blue":   ((100, 120, 60), (130, 255, 255)),
    "green":  ((40, 80, 50),  (85, 255, 255)),
    "orange": ((8, 120, 80),  (22, 255, 255)),
    # 红色在 HSV 里跨 0 点，额外设置
    "yellow": ((22, 100, 100), (35, 255, 255)),
}


def color_choose(hsv):
    """
    输入：
        hsv
    输出：
        最适合的掩膜,
        并且已经初步去噪"""
    scores = {}
    masks = {}

    for color, (lower, upper) in CANDIDATES.items():
        mask = cv2.inRange(hsv, lower, upper)
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        scores[color] = max([cv2.contourArea(c) for c in contours], default=0.0)
        masks[color] = mask
# 红色
    lower1 = np.array([0, 100, 100])  # h o,s 100,v 100
    upper1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)
    lower2 = np.array([170, 100, 100])
    upper2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    scores["red"] = max([cv2.contourArea(c) for c in contours], default=0.0)
    masks["red"] = mask
    best = max(scores, key=scores.get)
    best_mask = masks[best]

    # 把选中的颜色打印出来，方便记录到 README
    print(f"选中颜色：{best}（各颜色最大轮廓面积："
          + ", ".join(f"{k}={v:.0f}" for k, v in sorted(scores.items(), key=lambda kv: -kv[1]))
          + "）")

    return best_mask
"""选择面积最大的颜色
    输出面积最大的颜色的掩膜"""


def img_read(path):
    if not os.path.exists(path):
        print("该文件路径不存在")
        return None
    data = np.fromfile(path, dtype=np.uint8)  # 先读取为数组
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)  # 解压缩成BGR（高，宽，3通道）（蓝，绿，红）

    if img is None:
        print(f"无法解码{path}")
    return img
# 输入图片路径，返回BGR图像


def img_write(img, name):
    """将生成图片放入文件"""
    name = name + ".png"  # 输出文件名
    out_path = os.path.join(str(OUT_DIR), name)  # 输出路径（绝对路径 + 目录已建好）
    ok, buf = cv2.imencode(".png", img)  # 得图片字节
    if ok == False:
        raise IOError(f"编码失败：{out_path}")
    buf.tofile(out_path)  # 写入outputs
# 输入图片，与原图路径。


def mask_verify(mask):
    """开运算，闭运算去噪点"""
    kernel = np.ones((5, 5), np.uint8)

    # 输入图像，结构元素，迭代次数
    # 如果一个像素元素至少有一个像素在核下是“1”，

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)  # 闭运算
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def print_rect(img, contour):
    for cnt in contour:
        area = cv2.contourArea(cnt)
        rect = cv2.minAreaRect(cnt)  # 1. 求最小外接旋转矩形
        box = cv2.boxPoints(rect)  # 2. 转成 4 个角点
        box = np.round(box).astype(np.int32)  # 3. 角点转整数
        cv2.drawContours(img, [box], 0, (0, 0, 0), 2)  # 4. 画出来
    return img


def find_center(contour):
    """找到中心点，并按面积大小排序"""

    contours_sorted = sorted(contour, key=cv2.contourArea, reverse=True)
    i = 0
    contour_center = []
    if len(contour) > 0:
        print("按面积排序，轮廓中心坐标为：")
    else:
        print("未检测到目标")
    for cnt in contours_sorted:

        i += 1
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            contour_center.append((cx, cy))
            print(f"轮廓{i}:({cx},{cy})")
        else:
            print(f"{i}轮廓面积为0，跳过")
            continue  # m00==0 时跳过，避免下面用到未定义的 cx/cy
    return contour_center


def print_center(img, contour_center):
    for center in contour_center:
        center_list = list(center)
        cx = center_list[0]
        cy = center_list[1]
        cv2.circle(img, center, 3, (255, 255, 255), -1)  # 图，圆心，半径，颜色（BGR），线宽（-1，实心）
        cv2.putText(img, f"{center}", (cx + 8, cy - 8),  # 起点：圆心右上方
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5,  # 字体、大小
                    (255, 255, 255), 1,  # 颜色(BGR)、线宽1
                    cv2.LINE_AA)
    return img


def main(path):
    img = img_read(path)
    name = os.path.splitext(os.path.basename(path))[0]
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    mask = color_choose(hsv)  # 面积最大颜色的掩膜

    #mask = mask_verify(mask)  # 开闭运算

    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)  # 得出轮廓列表

    contour = [i for i in contours if cv2.contourArea(i) >= 1600]  # 阈值去噪

    contour_center = find_center(contour)  # 得出中心点

    img = print_rect(img, contour)

    img = print_center(img, contour_center)

    img_write(img, name + "no_yunshuan")


# 新增：给 ROS 节点用的包装函数（不改动上面的任何逻辑）

def detect(path):
    """一张图 → (detected, u, v, area)

    检测不到时返回 (False, 0.0, 0.0, 0.0)；
    调用的是你自己的 color_choose / mask_verify / find_center。
    """
    img = img_read(path)
    if img is None:
        return False, 0.0, 0.0, 0.0

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = mask_verify(color_choose(hsv))

    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour = [c for c in contours if cv2.contourArea(c) >= 1600]
    if not contour:
        return False, 0.0, 0.0, 0.0

    centers = find_center(contour)
    if not centers:
        return False, 0.0, 0.0, 0.0

    u, v = centers[0]                      # 已按面积降序，第一个就是最大轮廓
    best = max(contour, key=cv2.contourArea)
    return True, float(u), float(v), float(cv2.contourArea(best))



if __name__ == "__main__":
    img_dir = sys.argv[1] if len(sys.argv) > 1 else os.path.expanduser("~/vision_ws/mine/images")
    for name in sorted(os.listdir(img_dir)):
        if not name.lower().endswith(".png"):
            continue
        path = os.path.join(img_dir, name)
        print(f"\n=== {name} ===")
        main(path)
