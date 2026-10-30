"""颜色目标检测的核心逻辑。

这个模块只做「图 → 目标位置」这一件事，不依赖 ROS。
好处：任务 1/2 的脚本和任务 3 的 ROS 节点可以共用同一份算法，
保证「发布出去的数据」和「截图里画出来的框」是同一套逻辑算出来的。

坐标约定（重要）：
    图像左上角是原点 (0, 0)
    u 向右增大（列号，等价于 x）
    v 向下增大（行号，等价于 y）
    单位是像素，不是米，也不是机器人坐标。
"""

from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np

# ---- 默认参数：绿色目标（HSV 在 OpenCV 里的取值范围见下方说明）----
# H: 0~179（OpenCV 把 0~360 度压缩成一半，所以绿色 60° 对应 30）
# S: 0~255（饱和度，越大颜色越纯）
# V: 0~255（明度，越大越亮）
DEFAULT_LOWER = (35, 80, 60)
DEFAULT_UPPER = (85, 255, 255)
DEFAULT_MIN_AREA = 500


@dataclass
class Detection:
    """一次检测的全部结果，字段名和 TargetPosition.msg 对齐。"""

    detected: bool = False          # 本帧是否找到目标
    u: float = 0.0                  # 目标中心横坐标（像素）
    v: float = 0.0                  # 目标中心纵坐标（像素）
    area: float = 0.0               # 选中轮廓的面积（像素²）
    contour: Optional[np.ndarray] = None            # 目标外轮廓（画图用）
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)  # 外接矩形 x, y, w, h
    mask: Optional[np.ndarray] = None               # 二值掩膜（排查用）
    n_candidates: int = 0           # 通过面积筛选的候选数量（分析用）

    def __str__(self) -> str:
        if not self.detected:
            return "未检测到目标"
        return (f"u={self.u:.1f}, v={self.v:.1f}, "
                f"area={self.area:.0f}, 候选={self.n_candidates}")


def build_mask(bgr: np.ndarray,
               lower=DEFAULT_LOWER,
               upper=DEFAULT_UPPER,
               morph: str = "open",
               ksize: int = 5) -> np.ndarray:
    """BGR 图 → HSV → 阈值 → 形态学 → 二值掩膜。

    morph 取值:
        "none"        不做形态学
        "open"        开运算（先腐蚀后膨胀）：去掉小噪点
        "close"       闭运算（先膨胀后腐蚀）：补上目标内部的洞
        "open+close"  先开后闭
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv,
                       np.array(lower, dtype=np.uint8),
                       np.array(upper, dtype=np.uint8))

    if morph not in ("none", "", None) and ksize > 0:
        kernel = np.ones((ksize, ksize), np.uint8)
        if "open" in morph:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if "close" in morph:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def find_contours(mask: np.ndarray, min_area: float = DEFAULT_MIN_AREA):
    """返回按面积从大到小排序的 [(area, contour), ...]。"""
    # RETR_EXTERNAL 只取最外层轮廓，避免目标内部纹理被当成多个目标
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for c in contours:
        a = cv2.contourArea(c)
        if a >= min_area:              # 面积阈值：过滤噪声
            candidates.append((a, c))
    candidates.sort(key=lambda item: item[0], reverse=True)
    return candidates


def detect(bgr: np.ndarray,
           lower=DEFAULT_LOWER,
           upper=DEFAULT_UPPER,
           min_area: float = DEFAULT_MIN_AREA,
           morph: str = "open",
           ksize: int = 5) -> Detection:
    """完整检测流程，返回 Detection。

    选择规则：通过面积阈值后，取**面积最大**的那个轮廓。
    """
    mask = build_mask(bgr, lower, upper, morph, ksize)
    candidates = find_contours(mask, min_area)

    if not candidates:
        # 关键：没找到时必须给出明确的失败结果，不能返回未初始化的坐标
        return Detection(detected=False, mask=mask, n_candidates=0)

    area, best = candidates[0]

    # 用轮廓矩算质心：m00 是面积，m10/m00 是 x 均值，m01/m00 是 y 均值
    m = cv2.moments(best)
    if m["m00"] > 0:
        u = m["m10"] / m["m00"]
        v = m["m01"] / m["m00"]
    else:
        # 极端情况（面积算出来是 0）退化成外接矩形中心
        x, y, w, h = cv2.boundingRect(best)
        u, v = x + w / 2.0, y + h / 2.0

    x, y, w, h = cv2.boundingRect(best)
    return Detection(detected=True, u=float(u), v=float(v), area=float(area),
                     contour=best, bbox=(x, y, w, h), mask=mask,
                     n_candidates=len(candidates))


def draw_result(bgr: np.ndarray, det: Detection,
                title: str = "") -> np.ndarray:
    """把轮廓、外接矩形、中心点、坐标文字画回原图（任务 2 的要求）。

    注意：cv2.putText 只能画 ASCII，中文会变成问号，
    所以图上的文字一律用英文，中文放到终端输出里。
    """
    img = bgr.copy()

    if det.detected:
        cv2.drawContours(img, [det.contour], -1, (0, 255, 0), 2)

        x, y, w, h = det.bbox
        cv2.rectangle(img, (x, y), (x + w, y + h), (0, 255, 255), 1)

        cu, cv_ = int(round(det.u)), int(round(det.v))
        cv2.drawMarker(img, (cu, cv_), (0, 0, 255), cv2.MARKER_CROSS, 24, 2)
        cv2.circle(img, (cu, cv_), 4, (255, 0, 0), -1)

        text = f"u={cu}, v={cv_}, area={int(det.area)}"
        ty = max(24, y - 10)
        cv2.putText(img, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 3, cv2.LINE_AA)
        cv2.putText(img, text, (x, ty), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 1, cv2.LINE_AA)
    else:
        cv2.putText(img, "NO TARGET DETECTED", (18, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2, cv2.LINE_AA)

    if title:
        cv2.putText(img, title, (10, img.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1, cv2.LINE_AA)
    return img
