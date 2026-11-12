# -*- coding: utf-8 -*-
"""蓝色目标检测核心模块（纯 OpenCV，不 import rclpy）。

整条链路（任务 1 的算法流程，逐级把上一步输出当作下一步输入）：

    读取 BGR 图像
        -> 转换到 HSV（把“颜色”从和亮度纠缠的 RGB 里解耦出来）
        -> 用 HSV 上下界生成二值 mask（这一步的输出是一张只有 0/255 的单通道图）
        -> 形态学开/闭运算（清掉 mask 上的孤立噪点、补上目标内部的小洞）
        -> 提取外轮廓（mask 的边界 -> 一串点坐标）
        -> 按面积阈值筛选（丢掉明显是噪声的候选）
        -> 对选中的轮廓求中心点 (u, v)

任务 1 的脚本、任务 2 的对比脚本、任务 3 的 ROS2 发布节点都 import 本模块，
保证「脚本里打印的坐标」和「话题里发布的坐标」是同一套逻辑，不是两套代码。
"""

from __future__ import annotations

import os
from dataclasses import dataclass, field
from datetime import datetime
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# 默认参数（README 里记录的“统一验收规则”就是这几个值）
# ---------------------------------------------------------------------------
DEFAULT_HSV_LOWER: Tuple[int, int, int] = (95, 120, 60)
DEFAULT_HSV_UPPER: Tuple[int, int, int] = (130, 255, 255)
DEFAULT_MIN_AREA: float = 500.0      # 像素平方
DEFAULT_MORPH_KERNEL: int = 5        # 5x5 椭圆核

# OpenCV 里 H 的取值范围是 0~179（不是 0~360），S/V 是 0~255。
# 蓝色大致落在 H≈90~130：深蓝偏 105~115，天蓝/青蓝偏 95~105。
# S 下界取 120 而不是 60，是为了把「蓝天」（S≈90~100，饱和度低）挡在外面——
# 蓝天是蓝球检测最主要的干扰源，这条阈值就是专门为它调的。


@dataclass
class DetectorConfig:
    """一次检测用到的全部参数。改参数就是改这个对象。"""

    hsv_lower: Tuple[int, int, int] = DEFAULT_HSV_LOWER
    hsv_upper: Tuple[int, int, int] = DEFAULT_HSV_UPPER
    min_area: float = DEFAULT_MIN_AREA
    morph_kernel: int = DEFAULT_MORPH_KERNEL
    use_open: bool = True     # 开运算：先腐蚀后膨胀，去掉小的白色噪点
    use_close: bool = True    # 闭运算：先膨胀后腐蚀，补掉目标内部的黑洞

    def __post_init__(self) -> None:
        self.hsv_lower = tuple(int(v) for v in self.hsv_lower)  # type: ignore[assignment]
        self.hsv_upper = tuple(int(v) for v in self.hsv_upper)  # type: ignore[assignment]
        self.min_area = float(self.min_area)
        if len(self.hsv_lower) != 3 or len(self.hsv_upper) != 3:
            raise ValueError("HSV 上下界必须是 3 个整数 (H, S, V)")

        # 6 个分量都要查 0~255：只查 H 的话，S/V 传 300 会在 np.array(..., uint8)
        # 那一步被静默截断成 44（numpy 1.x 回绕，numpy>=2 直接 OverflowError），
        # 于是 upper 反而小于 lower -> inRange 恒空 -> 永远 detected=false，很难查。
        for name, value in (("H", self.hsv_lower[0]), ("H", self.hsv_upper[0]),
                            ("S", self.hsv_lower[1]), ("S", self.hsv_upper[1]),
                            ("V", self.hsv_lower[2]), ("V", self.hsv_upper[2])):
            if not (0 <= value <= 255):
                raise ValueError("%s 分量必须在 0~255 之间，收到 %d" % (name, value))
        if self.hsv_upper[0] > 179:
            raise ValueError("H 上界必须在 0~179 之间（OpenCV 的 H 是 8 位存储）")

        for lo, hi in zip(self.hsv_lower, self.hsv_upper):
            if lo > hi:
                raise ValueError(
                    "HSV 下界不能大于上界：lower=%s upper=%s"
                    % (self.hsv_lower, self.hsv_upper)
                )
        if self.min_area < 0:
            raise ValueError("min_area 不能是负数")

        # 形态学核尺寸归一化：偶数核在 OpenCV 里锚点有半像素偏移，太小的核等于没做。
        k = int(self.morph_kernel)
        if k < 3:
            k = 0            # 等价于关闭形态学（build_mask 里 morph_kernel >= 3 才生效）
        elif k % 2 == 0:
            k += 1           # 2/4/6 -> 3/5/7
        self.morph_kernel = k

    def describe(self) -> str:
        morph = []
        if self.use_open:
            morph.append("open")
        if self.use_close:
            morph.append("close")
        if self.morph_kernel < 3:
            morph_s = "none(k<3,off)"
        else:
            morph_s = "+".join(morph) if morph else "none"
        return (
            "HSV lower=%s upper=%s min_area=%.1f morph=%s(k=%d)"
            % (self.hsv_lower, self.hsv_upper, self.min_area, morph_s, self.morph_kernel)
        )


@dataclass
class Detection:
    """一帧图像的检测结果。detected=False 时 u/v/area 固定为 0，不允许输出未初始化坐标。"""

    detected: bool
    u: float = 0.0
    v: float = 0.0
    area: float = 0.0
    center_method: str = "none"        # moments / bbox / none
    contour: Optional[np.ndarray] = None
    bbox: Tuple[int, int, int, int] = (0, 0, 0, 0)   # x, y, w, h
    candidates: List[float] = field(default_factory=list)  # 通过面积筛选的候选面积（降序）
    mask: Optional[np.ndarray] = None

    @property
    def center_int(self) -> Tuple[int, int]:
        """画图用的整数坐标。"""
        return int(round(self.u)), int(round(self.v))


def build_mask(image_bgr: np.ndarray, cfg: DetectorConfig) -> np.ndarray:
    """BGR -> HSV -> 阈值 -> 形态学，返回单通道 0/255 的 mask。"""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(cfg.hsv_lower, dtype=np.uint8),
                       np.array(cfg.hsv_upper, dtype=np.uint8))

    if cfg.morph_kernel >= 3 and (cfg.use_open or cfg.use_close):
        k = cfg.morph_kernel
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        if cfg.use_open:
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        if cfg.use_close:
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def detect(image_bgr: np.ndarray, cfg: Optional[DetectorConfig] = None) -> Detection:
    """在一张 BGR 图像上找蓝色目标，返回中心点 (u, v) 与面积。

    多个候选择优规则：**面积最大者胜**（面积相同则保留 findContours 返回顺序靠前的）。
    理由：本周目标只有一个，面积越大越可能是真目标而不是反光点/背景碎片。
    """
    if cfg is None:
        cfg = DetectorConfig()
    if image_bgr is None or image_bgr.size == 0:
        raise ValueError("输入图像为空，检查图片路径是否读到了文件")

    mask = build_mask(image_bgr, cfg)

    # OpenCV 4 的 findContours 返回 (contours, hierarchy) 两个值；
    # RETR_EXTERNAL 只取最外层轮廓，CHAIN_APPROX_SIMPLE 只存拐点，省内存。
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    accepted: List[Tuple[float, np.ndarray]] = []
    for c in contours:
        area = float(cv2.contourArea(c))
        if area >= cfg.min_area:      # 面积阈值：小于它的当噪声丢掉
            accepted.append((area, c))
    accepted.sort(key=lambda item: item[0], reverse=True)

    det = Detection(detected=False, mask=mask,
                    candidates=[a for a, _ in accepted])

    if not accepted:
        # 没有候选：明确返回“未检测到”，u/v/area 保持 0，绝不返回未初始化坐标
        return det

    area, contour = accepted[0]

    # 中心点用轮廓矩：m00 = 轮廓面积，m10/m00 = 重心的 u，m01/m00 = 重心的 v
    m = cv2.moments(contour)
    if m["m00"] > 1e-6:
        u = float(m["m10"] / m["m00"])
        v = float(m["m01"] / m["m00"])
        method = "moments"
    else:
        # 退化情况（面积 0 或轮廓是单点/直线）：退回外接矩形中心
        x, y, w, h = cv2.boundingRect(contour)
        u = float(x + w / 2.0)
        v = float(y + h / 2.0)
        method = "bbox"

    det.detected = True
    det.u, det.v, det.area = u, v, area
    det.center_method = method
    det.contour = contour
    det.bbox = tuple(int(v_) for v_ in cv2.boundingRect(contour))  # type: ignore[assignment]
    return det


def annotate(image_bgr: np.ndarray, det: Detection,
             cfg: Optional[DetectorConfig] = None) -> np.ndarray:
    """把轮廓 / 外接矩形 / 中心点 / 坐标文字画回原图。

    注意：cv2.putText 只支持 ASCII，所以图上的文字用英文，避免出现一堆问号。
    """
    out = image_bgr.copy()

    if det.detected and det.contour is not None:
        # 2) 选中的目标：粗轮廓（画成绿色，只是为了和蓝球本身区分开）
        cv2.drawContours(out, [det.contour], -1, (0, 255, 0), 2)

        # 3) 外接矩形：品红色，便于对比“矩形中心”和“重心”是否一致
        x, y, w, h = det.bbox
        cv2.rectangle(out, (x, y), (x + w, y + h), (255, 0, 255), 1)

        # 4) 中心点：红色实心圆 + 十字准星
        cu, cv_ = det.center_int
        cv2.circle(out, (cu, cv_), 5, (0, 0, 255), -1)
        cv2.line(out, (cu - 14, cv_), (cu + 14, cv_), (0, 0, 255), 1)
        cv2.line(out, (cu, cv_ - 14), (cu, cv_ + 14), (0, 0, 255), 1)

        lines = [
            ("detected=True", (0, 255, 0)),
            ("u=%d, v=%d" % (cu, cv_), (0, 0, 255)),
            ("area=%.0f px^2" % det.area, (0, 255, 255)),
            ("center=%s" % det.center_method, (255, 255, 255)),
            ("cands=%d (picked largest)" % len(det.candidates), (255, 255, 255)),
        ]
    else:
        # 5) 没检测到：画面中央给一个明确的红字提示
        text = "NO TARGET DETECTED"
        (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)
        ox = max(10, (out.shape[1] - tw) // 2)
        oy = max(40, out.shape[0] // 2)
        cv2.putText(out, text, (ox, oy), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        lines = [
            ("detected=False", (0, 0, 255)),
            ("u=0, v=0 (default)", (255, 255, 255)),
            ("area=0", (255, 255, 255)),
        ]

    if cfg is not None:
        lines.append(("min_area=%.0f" % cfg.min_area, (200, 200, 200)))
        lines.append(("H %d-%d S %d-%d V %d-%d"
                      % (cfg.hsv_lower[0], cfg.hsv_upper[0],
                         cfg.hsv_lower[1], cfg.hsv_upper[1],
                         cfg.hsv_lower[2], cfg.hsv_upper[2]), (200, 200, 200)))

    # 6) 左上角半透明信息面板
    pad, lh = 8, 22
    box_w = 300
    box_h = pad * 2 + lh * len(lines)
    overlay = out.copy()
    cv2.rectangle(overlay, (0, 0), (box_w, box_h), (0, 0, 0), -1)
    out = cv2.addWeighted(overlay, 0.55, out, 0.45, 0)
    for i, (text, color) in enumerate(lines):
        cv2.putText(out, text, (pad, pad + lh * (i + 1) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
    return out


def mask_to_bgr(mask: np.ndarray) -> np.ndarray:
    """把单通道 mask 转成三通道，方便和彩色图拼在一起保存。"""
    return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)


# ---------------------------------------------------------------------------
# 小工具：读写带中文路径的文件、终端抬头
# ---------------------------------------------------------------------------
def imread_unicode(path: str) -> Optional[np.ndarray]:
    """cv2.imread 在 Windows 上遇到中文路径会返回 None，用 imdecode 兜一层。"""
    if not os.path.isfile(path):
        return None
    data = np.fromfile(path, dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_unicode(path: str, image: np.ndarray) -> bool:
    """同样地，cv2.imwrite 写不了中文路径，改用 imencode + tofile。"""
    ext = os.path.splitext(path)[1] or ".png"
    ok, buf = cv2.imencode(ext, image)
    if not ok:
        return False
    folder = os.path.dirname(os.path.abspath(path))
    if folder:
        os.makedirs(folder, exist_ok=True)
    buf.tofile(path)
    return True


def print_banner(name: str, title: str) -> None:
    """终端抬头：姓名 + 实时时间戳（防抄袭要求，截图时能一起被截进去）。"""
    print("=" * 62)
    print("姓名: %s" % name)
    print("时间: %s" % datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("任务: %s" % title)
    print("=" * 62)


def format_result(det: Detection) -> str:
    """统一的结果行格式，脚本和 README 里都用这一行。"""
    if not det.detected:
        return "未检测到目标（detected=false），u/v/area 使用默认值 0"
    return ("detected=true  (u, v)=(%.1f, %.1f)  area=%.1f px^2  方法=%s"
            % (det.u, det.v, det.area, det.center_method))
