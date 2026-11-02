# -*- coding: utf-8 -*-
"""颜色目标检测核心模块。

任务 1 的算法流程都在这里，ROS2 发布节点也直接 import 这个模块，
保证「脚本看到的」和「发出去的话题」用的是同一套检测逻辑。

流程：
    BGR 图 -> HSV -> inRange 掩膜 -> 形态学开/闭 -> 找外轮廓
           -> 按面积筛选 -> 多个候选按面积降序 -> 取最大
           -> 用轮廓矩算中心点 (u, v)
"""

import cv2
import numpy as np

# 橙色在 OpenCV 里 H 大约 5~25（OpenCV 的 H 范围是 0~179，不是 0~360）
DEFAULT_LOWER = (5, 120, 120)
DEFAULT_UPPER = (25, 255, 255)
DEFAULT_MIN_AREA = 800

# 形态学核
KERNEL = np.ones((5, 5), np.uint8)


def detect_target(img, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER,
                  min_area=DEFAULT_MIN_AREA, use_morph=True):
    """在单张 BGR 图上检测橙色目标。

    返回 dict：
        detected   bool          是否找到目标
        u, v       float         中心点像素坐标（detected=False 时为 0.0）
        area       float         轮廓面积（detected=False 时为 0.0）
        bbox       (x, y, w, h)  外接矩形（detected=False 时为 None）
        method     str           中心点算法：'moments' 或 'bbox'
        candidates list          所有通过面积筛选的候选，按面积降序
        n_raw      int           面积筛选前的轮廓总数
        mask       ndarray       二值掩膜，方便调试和可视化
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(lower, np.uint8), np.array(upper, np.uint8))

    if use_morph:
        # 开运算去掉孤立小白点，闭运算补目标内部小洞
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, KERNEL)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, KERNEL)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    candidates = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue          # 面积太小，当噪声丢掉

        M = cv2.moments(c)
        if M["m00"] == 0:
            # 轮廓退化（面积接近 0）时矩算不出来，退回外接矩形中心
            x, y, w, h = cv2.boundingRect(c)
            u, v = x + w / 2.0, y + h / 2.0
            method = "bbox"
        else:
            u = M["m10"] / M["m00"]
            v = M["m01"] / M["m00"]
            method = "moments"

        candidates.append(dict(
            area=area, u=u, v=v, method=method,
            bbox=cv2.boundingRect(c), contour=c,
        ))

    # 多个候选时统一按面积降序，取最大的那个作为本帧目标
    candidates.sort(key=lambda d: d["area"], reverse=True)

    res = dict(detected=False, u=0.0, v=0.0, area=0.0, bbox=None,
               method="-", candidates=candidates, n_raw=len(contours), mask=mask)

    if candidates:
        best = candidates[0]
        res.update(detected=True, u=best["u"], v=best["v"],
                   area=best["area"], bbox=best["bbox"], method=best["method"],
                   contour=best["contour"])
    return res


def draw_result(img, res, lower=DEFAULT_LOWER, upper=DEFAULT_UPPER, min_area=DEFAULT_MIN_AREA):
    """把检测结果画回图上：轮廓、外接矩形、中心点、坐标文字。

    detected=False 时明确写「未检测到目标」，不要让人误以为目标在左上角。
    """
    out = img.copy()
    h, w = out.shape[:2]

    # 顶部信息条，把本次用的参数印在图上，截图时一眼能看出是哪组参数
    cv2.rectangle(out, (0, 0), (w, 34), (32, 32, 32), -1)
    cv2.putText(out, "HSV %s-%s  minArea=%d" % (lower, upper, min_area), (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2, cv2.LINE_AA)

    if not res["detected"]:
        cv2.putText(out, "NO TARGET DETECTED", (int(w * 0.12), int(h * 0.5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.4, (0, 0, 255), 3, cv2.LINE_AA)
        cv2.putText(out, "u = None, v = None", (int(w * 0.12), int(h * 0.5) + 46),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)
        return out

    # 轮廓（细线）
    cv2.drawContours(out, [res["contour"]], -1, (0, 255, 0), 2)

    # 外接矩形
    x, y, bw, bh = res["bbox"]
    cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 200, 255), 2)

    # 中心点
    cu, cv_ = int(round(res["u"])), int(round(res["v"]))
    cv2.drawMarker(out, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 26, 2)
    cv2.circle(out, (cu, cv_), 4, (0, 0, 255), -1)

    # 坐标文字（细则要求写成 u=.., v=.. 的形式）
    label = "u=%d, v=%d" % (cu, cv_)
    ty = y - 12 if y > 40 else y + bh + 28
    cv2.putText(out, label, (max(x - 10, 6), ty),
                cv2.FONT_HERSHEY_SIMPLEX, 0.95, (0, 255, 0), 2, cv2.LINE_AA)
    cv2.putText(out, "area=%.0f" % res["area"], (max(x - 10, 6), ty + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 0), 2, cv2.LINE_AA)
    return out


def format_result(name, img_shape, res):
    """把一次检测结果格式化成终端输出文本。"""
    h, w = img_shape[:2]
    lines = ["[%s] %dx%d  原始轮廓 %d 个" % (name, w, h, res["n_raw"])]
    if res["candidates"]:
        top = ", ".join("%.0f" % c["area"] for c in res["candidates"][:5])
        lines.append("  按面积降序的候选: %s" % top)
    if res["detected"]:
        lines.append("  检测到目标: u=%.1f, v=%.1f, area=%.0f, 中心算法=%s, bbox=%s"
                     % (res["u"], res["v"], res["area"], res["method"], res["bbox"]))
    else:
        lines.append("  未检测到目标（所有候选都小于最小面积阈值或被颜色范围排除）")
    return "\n".join(lines)
