#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""蓝色目标检测的核心算法（任务 1 + 任务 2 的"干货"都在这里）。

设计原则
--------
1. **不依赖 ROS2**：只用 OpenCV + NumPy。先在普通 python3 里调通、调好参数，
   再让 ROS2 节点去 import 它。这样出问题时能分清是"算法问题"还是"通信问题"。
2. **失败必须显式**：没检测到目标时 detected=False，u/v/area 全部是 0.0，
   绝不留一个"上一次的坐标"或者未初始化的变量给上层用。
3. **可解释**：每个候选轮廓的面积都留着，方便在终端/日志里说清楚"为什么选了它"。

坐标约定（务必记住）
--------------------
    图像左上角是 (0, 0)
    u（横向，等价于 x）向右增大
    v（纵向，等价于 y）向下增大
    (u, v) 是**像素坐标**，不是米、不是场地坐标、不是机器人坐标。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# 默认参数（蓝色杯套 / 蓝色杯子）。H 的取值是 0~180（OpenCV 的 H 是 0~360 的一半）。
#
# 为什么 S 下界只有 20（教科书上常写 90+）：
#   实拍物体的颜色是"浅灰蓝"，饱和度远低于纯色块。
#   我在自己拍的 4 张照片上量过：蓝色区域的 S 中位数只有 31，V 中位数 148。
#   如果按 S ≥ 90 设，2 张照片直接漏检（候选数 0，mask 全黑）。
#   放宽到 S ≥ 20 后 4 张都能检出；代价是更容易被"灰蓝色背景/阴影"干扰，
#   所以面积筛选、形态学、多候选规则要一起看，不能只靠阈值。
DEFAULT_HSV_LOWER: Tuple[int, int, int] = (90, 20, 25)
DEFAULT_HSV_UPPER: Tuple[int, int, int] = (130, 255, 255)
DEFAULT_MIN_AREA: float = 800.0

# 候选目标不够时的固定返回值。0 不是"目标在左上角"，它只是一个占位值，
# 真实是否有目标必须看 detected 字段。
EMPTY_UV: float = 0.0
EMPTY_AREA: float = 0.0

_GREEN = (0, 255, 0)
_YELLOW = (0, 255, 255)
_RED = (0, 0, 255)
_WHITE = (255, 255, 255)
_BLACK = (0, 0, 0)
_ORANGE = (0, 165, 255)


@dataclass
class Detection:
    """一次检测的全部结果。"""

    detected: bool
    u: float
    v: float
    area: float
    candidates: int = 0
    center_method: str = "none"          # "moments" / "bbox_fallback" / "none"
    contour: Optional[np.ndarray] = None
    bbox: Optional[Tuple[int, int, int, int]] = None   # (x, y, w, h)
    mask: Optional[np.ndarray] = None
    all_areas: List[float] = field(default_factory=list)   # 所有通过面积筛选的候选面积
    chosen_index: int = -1               # 最终选中的是第几个候选（0 = 面积最大）

    def summary(self) -> str:
        """一行人类可读的结论，给终端打印用。"""
        if not self.detected:
            return f"未检测到目标（候选数={self.candidates}），u/v/area 置 0"
        return (f"检测到目标 u={self.u:.1f} v={self.v:.1f} area={self.area:.0f} px^2 "
                f"中心点算法={self.center_method} 候选数={self.candidates} "
                f"选中第 {self.chosen_index + 1} 大")


class BlueTargetDetector:
    """HSV 阈值分割 + 轮廓筛选 + 中心点计算。

    参数
    ----
    hsv_lower / hsv_upper : (H, S, V)，H∈[0,180]，S/V∈[0,255]
        蓝色在 OpenCV 的 HSV 里大约落在 H=100~130。
    min_area : 最小轮廓面积（像素平方）
        比它小的轮廓当噪声丢掉。
    morph : 是否做形态学开运算（去小噪点）+ 闭运算（把断裂补上）
    kernel : 形态学核大小，必须是正奇数。核越大，去噪/合并越强，但也会吃掉细节。
    min_circularity : 最小圆度 4πA/P²（1.0 是完美圆）。0 表示不启用。
        注意：带手柄的杯子圆度只有 0.5 左右，方形色块反而有 0.78，
        所以这个条件**不适合**用来区分杯子和方形色块，默认关闭。
    select : 多候选时的选择规则
        "largest"            -> 取面积最大的（默认）
        "closest_to_center"  -> 取离画面中心最近的
    """

    def __init__(
        self,
        hsv_lower: Sequence[int] = DEFAULT_HSV_LOWER,
        hsv_upper: Sequence[int] = DEFAULT_HSV_UPPER,
        min_area: float = DEFAULT_MIN_AREA,
        morph: bool = True,
        kernel: int = 5,
        min_circularity: float = 0.0,
        select: str = "largest",
    ) -> None:
        self.hsv_lower = self._check_hsv(hsv_lower, "hsv_lower")
        self.hsv_upper = self._check_hsv(hsv_upper, "hsv_upper")
        for lo, hi, name in zip(self.hsv_lower, self.hsv_upper, ("H", "S", "V")):
            if lo > hi:
                raise ValueError(f"{name} 的下界 {lo} 大于上界 {hi}，不可能有像素同时满足")
        if min_area < 0:
            raise ValueError("min_area 不能是负数")
        if select not in ("largest", "closest_to_center"):
            raise ValueError(f"未知的 select 规则：{select}")
        kernel = int(kernel)
        if kernel < 1:
            raise ValueError("kernel 至少是 1")
        if kernel % 2 == 0:
            kernel += 1        # 偶数核没有唯一中心，自动纠正成奇数
        self.min_area = float(min_area)
        self.morph = bool(morph)
        self.kernel = kernel
        self.min_circularity = float(min_circularity)
        self.select = select

    # ------------------------------------------------------------------ #
    # 参数检查
    # ------------------------------------------------------------------ #
    @staticmethod
    def _check_hsv(value: Sequence[int], name: str) -> Tuple[int, int, int]:
        if value is None or len(value) != 3:
            raise ValueError(f"{name} 必须是 3 个数 (H, S, V)，现在收到的是 {value}")
        h, s, v = (int(round(float(x))) for x in value)
        if not 0 <= h <= 180:
            raise ValueError(f"{name} 的 H 必须在 0~180 之间（OpenCV 的 H 是角度的一半），现在是 {h}")
        if not 0 <= s <= 255 or not 0 <= v <= 255:
            raise ValueError(f"{name} 的 S、V 必须在 0~255 之间，现在是 S={s} V={v}")
        return (h, s, v)

    # ------------------------------------------------------------------ #
    # 第 1 步：颜色 -> 二值 mask
    # ------------------------------------------------------------------ #
    def build_mask(self, bgr: np.ndarray) -> np.ndarray:
        """BGR 图 -> HSV -> inRange -> 形态学 -> 单通道白/黑 mask。"""
        if bgr is None or bgr.size == 0:
            raise ValueError("输入图像是空的（None 或 0 像素）")
        if bgr.ndim != 3 or bgr.shape[2] != 3:
            raise ValueError(f"输入必须是 3 通道 BGR 图，现在是 shape={bgr.shape}")

        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.hsv_lower, np.uint8),
                           np.array(self.hsv_upper, np.uint8))

        if self.morph:
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.kernel, self.kernel))
            # 开运算 = 先腐蚀再膨胀：抹掉孤立的小白点（噪点、灰尘反光）
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
            # 闭运算 = 先膨胀再腐蚀：把被反光切开的缺口补上
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
        return mask

    # ------------------------------------------------------------------ #
    # 第 2 步：找轮廓 + 筛选
    # ------------------------------------------------------------------ #
    def find_candidates(self, bgr: np.ndarray) -> List[Tuple[float, np.ndarray]]:
        """返回 [(面积, 轮廓), ...]，已按面积从大到小排序。"""
        return self.find_candidates_from_mask(self.build_mask(bgr))

    def find_candidates_from_mask(self, mask: np.ndarray) -> List[Tuple[float, np.ndarray]]:
        """同上，但直接吃一张算好的 mask（detect 里已经算过一次，别重复算）。"""
        # RETR_EXTERNAL：只要最外层轮廓，杯子内壁的洞不重复计算
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        cands: List[Tuple[float, np.ndarray]] = []
        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < self.min_area:
                continue                      # 太小，当噪声丢掉
            if self.min_circularity > 0.0:
                perim = float(cv2.arcLength(cnt, True))
                if perim <= 0.0:
                    continue
                circularity = 4.0 * math.pi * area / (perim * perim)
                if circularity < self.min_circularity:
                    continue
            cands.append((area, cnt))
        cands.sort(key=lambda x: x[0], reverse=True)
        return cands

    # ------------------------------------------------------------------ #
    # 第 3 步：选一个 + 算中心点
    # ------------------------------------------------------------------ #
    def detect(self, bgr: np.ndarray) -> Detection:
        if bgr is None or bgr.size == 0:
            raise ValueError("输入图像是空的（None 或 0 像素）")

        mask = self.build_mask(bgr)
        cands = self.find_candidates_from_mask(mask)
        if not cands:
            return Detection(detected=False, u=EMPTY_UV, v=EMPTY_UV,
                             area=EMPTY_AREA, candidates=0, center_method="none", mask=mask)

        all_areas = [a for a, _ in cands]

        if self.select == "largest" or len(cands) == 1:
            idx = 0
        else:
            h, w = bgr.shape[:2]
            cx, cy = w / 2.0, h / 2.0
            best = None
            for i, (_, cnt) in enumerate(cands):
                u, v, _ = self._centroid(cnt)
                dist = (u - cx) ** 2 + (v - cy) ** 2
                if best is None or dist < best[0]:
                    best = (dist, i)
            idx = best[1]

        area, cnt = cands[idx]
        u, v, method = self._centroid(cnt)
        x, y, bw, bh = cv2.boundingRect(cnt)

        if not (math.isfinite(u) and math.isfinite(v)):
            # 理论上走不到，但万一算出 NaN，也不能让它流到 ROS2 消息里
            return Detection(detected=False, u=0.0, v=0.0, area=0.0,
                             candidates=len(cands), center_method="none", mask=mask)

        return Detection(
            detected=True, u=float(u), v=float(v), area=float(area),
            candidates=len(cands), center_method=method, contour=cnt,
            bbox=(int(x), int(y), int(bw), int(bh)), mask=mask,
            all_areas=all_areas, chosen_index=idx,
        )

    def _centroid(self, cnt: np.ndarray) -> Tuple[float, float, str]:
        """轮廓矩算质心；m00 为 0 时退化成外接矩形中心，并如实报告用了哪种算法。"""
        m = cv2.moments(cnt)
        m00 = float(m.get("m00", 0.0))
        if m00 > 1e-6:
            return float(m["m10"] / m00), float(m["m01"] / m00), "moments"
        # m00 为 0：轮廓退化成一条线或一个点，矩不能除。
        x, y, w, h = cv2.boundingRect(cnt)
        return x + w / 2.0, y + h / 2.0, "bbox_fallback"

    # ------------------------------------------------------------------ #
    # 可视化
    # ------------------------------------------------------------------ #
    def annotate(self, bgr: np.ndarray, det: Detection, draw_grid: bool = True) -> np.ndarray:
        """在图上画轮廓、外接矩形、中心点、坐标文字、坐标轴和网格。返回新图，不改原图。"""
        out = bgr.copy()
        h, w = out.shape[:2]
        scale = max(0.5, min(1.4, w / 1000.0))
        thick = 2 if w < 1600 else 3

        if draw_grid:
            self._draw_grid(out, w, h, scale)

        if det.detected and det.contour is not None:
            cv2.drawContours(out, [det.contour], -1, _GREEN, thick)
            if det.bbox:
                x, y, bw, bh = det.bbox
                cv2.rectangle(out, (x, y), (x + bw, y + bh), _YELLOW, thick)
            cx, cy = int(round(det.u)), int(round(det.v))
            cv2.circle(out, (cx, cy), 9, _RED, -1, cv2.LINE_AA)
            cv2.circle(out, (cx, cy), 12, _WHITE, thick, cv2.LINE_AA)
            cv2.line(out, (cx - 22, cy), (cx + 22, cy), _WHITE, 1, cv2.LINE_AA)
            cv2.line(out, (cx, cy - 22), (cx, cy + 22), _WHITE, 1, cv2.LINE_AA)

            lines = [
                f"u={det.u:.1f}  v={det.v:.1f}",
                f"area={det.area:.0f} px^2",
                f"center={det.center_method}",
                f"candidates={det.candidates}  pick=#{det.chosen_index + 1}",
            ]
            self._panel_bl(out, lines, scale, _GREEN)
        else:
            banner = "NO TARGET DETECTED  (detected=false)"
            self._banner(out, banner, scale, _RED)
            lines = ["u = v = area = 0", "these zeros are INVALID,", "judge by detected flag"]
            self._panel_bl(out, lines, scale, _RED)

        self._draw_axes(out, w, h, scale)
        cv2.putText(out, f"image {w}x{h}", (w - int(190 * scale), h - int(14 * scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55 * scale, _WHITE, 1, cv2.LINE_AA)
        return out

    # -- 画图小工具 ----------------------------------------------------- #
    @staticmethod
    def _draw_grid(img: np.ndarray, w: int, h: int, scale: float) -> None:
        """每 100 像素一条细线（文字每 200 标一次），让"坐标从左上角开始"一眼可见。"""
        overlay = img.copy()
        for x in range(100, w, 100):
            cv2.line(overlay, (x, 0), (x, h), (255, 255, 255), 1)
            if x % 200 == 0:
                cv2.putText(overlay, f"u={x}", (x + 3, 15), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40 * scale, (255, 255, 255), 1, cv2.LINE_AA)
        for y in range(100, h, 100):
            cv2.line(overlay, (0, y), (w, y), (255, 255, 255), 1)
            if y % 200 == 0:
                cv2.putText(overlay, f"v={y}", (3, y - 4), cv2.FONT_HERSHEY_SIMPLEX,
                            0.40 * scale, (255, 255, 255), 1, cv2.LINE_AA)
        # 只混 18%，够看见又不糊住目标
        cv2.addWeighted(overlay, 0.18, img, 0.82, 0, dst=img)

    @staticmethod
    def _draw_axes(img: np.ndarray, w: int, h: int, scale: float) -> None:
        """在真正的原点（左上角）画 u 向右、v 向下的箭头。"""
        ox, oy = int(26 * scale), int(26 * scale)
        L = int(70 * scale)
        cv2.arrowedLine(img, (ox, oy), (ox + L, oy), _ORANGE, 2, cv2.LINE_AA, tipLength=0.22)
        cv2.arrowedLine(img, (ox, oy), (ox, oy + L), _ORANGE, 2, cv2.LINE_AA, tipLength=0.22)
        cv2.putText(img, "u", (ox + L + 5, oy + 7), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62 * scale, _ORANGE, 2, cv2.LINE_AA)
        cv2.putText(img, "v", (ox - 6, oy + L + 20), cv2.FONT_HERSHEY_SIMPLEX,
                    0.62 * scale, _ORANGE, 2, cv2.LINE_AA)
        cv2.putText(img, "(0,0)", (ox + 6, oy + 24), cv2.FONT_HERSHEY_SIMPLEX,
                    0.42 * scale, _ORANGE, 1, cv2.LINE_AA)

    @staticmethod
    def _panel(img: np.ndarray, lines: List[str], org: Tuple[int, int],
               scale: float, color: Tuple[int, int, int]) -> None:
        """半透明黑底 + 彩字，保证文字在任何背景上都读得清。"""
        fs = 0.62 * scale
        th = 1 if scale < 1.1 else 2
        pad = int(10 * scale)
        heights = int(30 * scale)
        widths = [cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, fs, th)[0][0] for s in lines]
        box_w = max(widths) + pad * 2
        box_h = int(26 * scale) * len(lines) + pad
        x0, y0 = org
        overlay = img.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h), _BLACK, -1)
        cv2.addWeighted(overlay, 0.55, img, 0.45, 0, dst=img)
        for i, s in enumerate(lines):
            y = y0 + int(20 * scale) + i * heights
            cv2.putText(img, s, (x0 + pad, y), cv2.FONT_HERSHEY_SIMPLEX, fs, color, th, cv2.LINE_AA)

    def _panel_bl(self, img: np.ndarray, lines: List[str], scale: float,
                  color: Tuple[int, int, int]) -> None:
        """信息框放左下角，避免压住左上角的坐标轴。"""
        h = img.shape[0]
        box_h = int(26 * scale) * len(lines) + int(10 * scale)
        self._panel(img, lines, (int(12 * scale), h - box_h - int(12 * scale)), scale, color)

    @staticmethod
    def _banner(img: np.ndarray, text: str, scale: float, color: Tuple[int, int, int]) -> None:
        """顶部居中的红色警示条（用于"未检测到目标"）。"""
        fs = 0.8 * scale
        th = 2 if scale < 1.1 else 3
        (tw, tht), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, fs, th)
        w = img.shape[1]
        x = max(4, (w - tw) // 2)
        overlay = img.copy()
        cv2.rectangle(overlay, (x - 10, 8), (x + tw + 10, 8 + tht + 16), _BLACK, -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, dst=img)
        cv2.putText(img, text, (x, 8 + tht + 8), cv2.FONT_HERSHEY_SIMPLEX, fs, color, th, cv2.LINE_AA)
