#!/usr/bin/env python3
from __future__ import annotations   # 兼容 3.10 的 list[str] / tuple[...] / X | None 写法

import argparse       # 命令行参数：让你不改代码就能换参数做对比实验
import csv            # 结果写 CSV，方便贴进 README
from dataclasses import dataclass   # 用它定义"检测结果"这个数据结构，比字典清爽
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


STUDENT_NAME = "皮董彦"
STUDENT_ID = "pidongyan"


# ═══════════════════════════════════════════════════════════════════════════
# STEP 1 / 7   读取 BGR 图像
#
#   本步用到的全部东西都在这里：
#     IMAGE_SUFFIX         哪些后缀算图片
#     SCRIPT_DIR           本文件所在目录（用来推断默认图片目录）
#     _default_image_dir() 决定默认去哪个目录找图
#     DEFAULT_IMAGE_DIR    默认图片目录（--images 的默认值）
#     read_image()         真正把文件读成 BGR 数组
# ═══════════════════════════════════════════════════════════════════════════
IMAGE_SUFFIX = {".jpg", ".jpeg", ".png", ".bmp"}

# Path.home() 在 Ubuntu 是 /home/pdy，在 Windows 是 C:\Users\pdy。
# 下面这个函数让脚本在两个系统里都能直接跑：优先找 WSL 的目录，
# 找不到就退回脚本所在目录下的 photos / test_images（方便在 PyCharm 里点运行）
SCRIPT_DIR = Path(__file__).resolve().parent


def _default_image_dir() -> Path:
    """按优先级挑一个存在的图片目录；全都不存在就返回 WSL 的默认路径（后面会报错提示）。"""
    for cand in (Path.home() / "ros2_ws" / "test_images",
                 SCRIPT_DIR / "photos",
                 SCRIPT_DIR / "test_images"):
        if cand.is_dir():
            return cand
    return Path.home() / "ros2_ws" / "test_images"


DEFAULT_IMAGE_DIR = _default_image_dir()


def read_image(path: str | Path) -> np.ndarray | None:
    """读取一张图片，返回 BGR 数组；读失败返回 None。

    cv2.imread(路径) 返回一个 numpy 数组，形状 (高, 宽, 3)，通道顺序 BGR。
    两个要点：
      1) 读失败不抛异常，而是返回 None（路径写错、文件损坏、格式不支持都这样），
         所以调用方必须判空 —— 判空在 detect_target() 的第一行。
      2) 读到的不是 RGB 而是 BGR，这就是 STEP 2 必须用 COLOR_BGR2HSV 的原因。

    注意：这个函数只负责"把磁盘文件变成数组"。
    ROS2 节点里图像来自相机话题，不经过这一步，直接拿数组喂 detect_target()。
    """
    return cv2.imread(str(path))


# ═══════════════════════════════════════════════════════════════════════════
# STEP 2 / 7   转换到 HSV
#
#   本步只有一件事：把 BGR 换成 HSV。转换用的常量（COLOR_BGR2HSV）直接在
#   函数里写死，因为它是 OpenCV 的枚举，没有可调参数。
# ═══════════════════════════════════════════════════════════════════════════
def to_hsv(bgr: np.ndarray) -> np.ndarray:
    """BGR 图 -> HSV 图（同尺寸、同通道数）。

    cv2.cvtColor(src, code) 逐像素做色彩空间转换。
    code 必须是 COLOR_BGR2HSV：
      - OpenCV 的 imread 读进来是 BGR 顺序，写成 RGB2HSV 就是红蓝互换，阈值全错
        （新手第一大坑，现象是"红色怎么也检不出来"）；
      - 转完 H∈[0,179]、S∈[0,255]、V∈[0,255]。注意 OpenCV 的 H 是 0-179
        而不是常说的 0-360（它把角度除以 2 塞进一个字节），所以红色上界写 179。

    为什么不在 RGB 上做阈值：RGB 三个通道都同时混了"颜色"和"亮度"，
    灯一亮三个值全变；HSV 把色相/饱和度 和 亮度 拆开，光照变化时 H 更稳。
    """
    return cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)


# ═══════════════════════════════════════════════════════════════════════════
# STEP 3 / 7   根据颜色范围生成二值掩膜（mask）
#
#   本步用到的全部东西都在这里：
#     HSV_PRESETS   阈值表（任务最关键的一组数字，就放在这一步里）
#     build_mask()  用阈值表生成 mask
# ═══════════════════════════════════════════════════════════════════════════
#
# HSV 阈值预设：
#   h_ranges : 色相区间列表。红色跨 0 度（0 度两侧都是红），所以必须写两段
#              [0,10] 和 [170,179]，只写一段会漏掉另一半红 —— 新手最常踩的坑
#   s_range  : 饱和度区间。S 低 = 发白/发灰，提高下限可以滤掉反光和浅色背景
#   v_range  : 明度区间。V 低 = 暗，提高下限可以滤掉暗部噪声和暗色背景
#
# 现在的 S>=100 / V>=100 不是拍脑袋定的，是用 tune_hsv.py 采样实拍图后得出的：
#   红球       H 0-10 / 170-179，S 中位 130-160，V 中位 130-180
#   木柜(干扰) H 5-8（和球重叠！），但 V 只有 60-90  -> 靠 V>=100 切开
#   暗地板     H 12-13，V 约 83                      -> H 上界 10 + V>=100 双保险
HSV_PRESETS: dict[str, dict] = {
    "red": {
        "h_ranges": [(0, 10), (170, 179)],
        "s_range": (100, 255),
        "v_range": (100, 255),
    },
    "green": {
        "h_ranges": [(40, 80)],        # 绿色不跨 0 度，一段就够
        "s_range": (80, 255),
        "v_range": (50, 255),
    },
    "blue": {
        "h_ranges": [(100, 130)],
        "s_range": (80, 255),
        "v_range": (50, 255),
    },
}

DEFAULT_COLOR = "red"      # --color 的默认值


def build_mask(hsv: np.ndarray, color: str = DEFAULT_COLOR,
               h_shift: int = 0, s_shift: int = 0, v_shift: int = 0) -> np.ndarray:
    """按 HSV 阈值生成二值掩膜（mask）：白 = 命中颜色，黑 = 没命中。

    mask 是后续所有步骤的基础 —— 调试颜色阈值时，
    「看 mask 图」比「看最终标框图」有用十倍，因为 mask 直接反映阈值切得对不对。

    h_shift / s_shift / v_shift 是给任务 2 做参数对比用的偏移量，
    默认 0 表示用预设值，传正数/负数就是临时放宽或收紧阈值。

    ── inRange 到底在干什么 ────────────────────────────────────────────
    cv2.inRange(hsv, lower, upper) 对每个像素的三个通道分别比较，
    三个通道都落在 [lower, upper] 区间内才输出 255，否则输出 0。
    所以 lower / upper 必须是长度为 3 的数组，顺序是 [H, S, V]，
    dtype 必须是 uint8（写成 int 会报"参数类型不支持"）。
    返回值是一张单通道图，尺寸和原图一样 —— 这就是 mask。
    """
    preset = HSV_PRESETS[color]
    s_lo, s_hi = preset["s_range"]
    v_lo, v_hi = preset["v_range"]

    # np.clip 保证偏移后不会越界（H 上限 179，S/V 上限 255）
    s_lo = int(np.clip(s_lo + s_shift, 0, 255))
    v_lo = int(np.clip(v_lo + v_shift, 0, 255))

    mask = None
    # 为什么是循环而不是一次 inRange：红色在 H 上跨了 0 度，
    # 表现为 [0,10] 和 [170,179] 两段互不相连的区间，
    # inRange 只能取一个连续区间，所以两段各生成一张 mask，最后按位或合并。
    for h_lo, h_hi in preset["h_ranges"]:
        # inRange 会逐像素判断：lower <= 像素 <= upper 就置 255，否则置 0
        lower = np.array([int(np.clip(h_lo - h_shift, 0, 179)), s_lo, v_lo], dtype=np.uint8)
        upper = np.array([int(np.clip(h_hi + h_shift, 0, 179)), s_hi, v_hi], dtype=np.uint8)
        part = cv2.inRange(hsv, lower, upper)

        # 红色有两端区间，两个 mask 用「或」合并（255 或 0 = 255，白色区域取并集）
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    return mask


# ═══════════════════════════════════════════════════════════════════════════
# STEP 4 / 7   形态学处理
#
#   本步用到的全部东西都在这里：
#     MORPH_KSIZE            核大小（21）
#     MORPH_CLOSE_ITERATIONS 闭运算迭代次数（2）
#     apply_morphology()     执行开/闭运算
# ═══════════════════════════════════════════════════════════════════════════
MORPH_KSIZE = 21              # 结构元尺寸，越大越能跨越宽缝隙（7→21 是实测结论）
MORPH_CLOSE_ITERATIONS = 2    # 闭运算做两遍，才跨得过球面上较宽的白色条纹


def apply_morphology(mask: np.ndarray, do_open: bool = False,
                     do_close: bool = True, ksize: int = MORPH_KSIZE) -> np.ndarray:
    """形态学处理（用来修 mask 的瑕疵）。

    open  开运算 = 先腐蚀再膨胀：吃掉孤立的小白点（噪声），但目标也会瘦一圈
    close 闭运算 = 先膨胀再腐蚀：填掉目标内部的小洞和缝隙，把碎块粘起来

    红白相间的足球 mask 一定是碎的（白色贴片把红块切断），
    所以这里默认只做闭运算、核开到 21、迭代 2 次，碎块才会连成一片，质心才准。

    ── 两个参数为什么这么定 ────────────────────────────────────────────
    ksize=21：核越大，能跨越的缝隙越宽。7 的时候 p1 的红块还连不上，
              面积只有 16 万；改到 21 后连成一整块，面积 29 万，质心靠近真实球心。
    iterations=2：单次膨胀跨不过球面上较宽的白色条纹，做两遍才合得上。

    ── 代价（答辩要能说）────────────────────────────────────────────────
    闭运算会让轮廓"变胖"：填掉白条的同时，边缘也往外扩了半个核，
    所以 area 是偏大的近似值，不是球的真实投影面积。
    """
    out = mask
    if do_open or do_close:
        # 椭圆形结构元比方形更自然，不会把圆形目标磨出直角
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (ksize, ksize))
        if do_open:
            out = cv2.morphologyEx(out, cv2.MORPH_OPEN, kernel, iterations=1)
        if do_close:
            # iterations=2 是为了把红块之间较宽的白色缝隙也合上
            out = cv2.morphologyEx(out, cv2.MORPH_CLOSE, kernel,
                                   iterations=MORPH_CLOSE_ITERATIONS)
    return out


# ═══════════════════════════════════════════════════════════════════════════
# STEP 5 / 7   提取外轮廓
#
#   本步用到的全部东西都在这里：
#     CONTOUR_RETRIEVAL_MODE  检索模式（只取最外层）
#     CONTOUR_APPROX_METHOD   近似方法（只留拐点）
#     extract_contours()      调 findContours
# ═══════════════════════════════════════════════════════════════════════════
CONTOUR_RETRIEVAL_MODE = cv2.RETR_EXTERNAL          # 只取最外层，忽略内部孔洞
CONTOUR_APPROX_METHOD = cv2.CHAIN_APPROX_SIMPLE     # 只留拐点，共线点被压缩


def extract_contours(mask_clean: np.ndarray) -> list:
    """从清理后的 mask 里提取外轮廓，返回轮廓列表。

    cv2.findContours(二值图, 检索模式, 近似方法) -> (轮廓列表, 层级信息)
      RETR_EXTERNAL      : 只取最外层轮廓，忽略目标内部的孔洞（我们只要外边界）
      CHAIN_APPROX_SIMPLE: 只保留拐点，共线的点会被压缩掉，省内存
    注意 OpenCV 3 返回 3 个值、OpenCV 4/5 返回 2 个值，这里按 4/5 的写法解包
    （所以下面用 `_` 接住第二个返回值）。
    每个 contour 是 N×1×2 的 numpy 数组，元素是轮廓上各点的 (x, y)。

    ⚠ 只能传单通道二值图，传彩色图会直接报错。
    """
    contours, _ = cv2.findContours(mask_clean, CONTOUR_RETRIEVAL_MODE,
                                   CONTOUR_APPROX_METHOD)
    return contours


# ═══════════════════════════════════════════════════════════════════════════
# STEP 6 / 7   按面积等条件筛选
#
#   本步用到的全部东西都在这里：
#     DEFAULT_MIN_AREA  最小轮廓面积（--min-area 的默认值）
#     filter_by_area()  按面积筛掉噪声
# ═══════════════════════════════════════════════════════════════════════════
DEFAULT_MIN_AREA = 800.0      # 像素平方。低于它的轮廓当噪声丢弃


def filter_by_area(contours: list, min_area: float) -> list:
    """丢掉面积太小的轮廓，返回剩下的候选列表。

    cv2.contourArea(cnt) 用格林公式算轮廓围出的面积，单位是"像素平方"。
    小于 min_area 的一律当噪声丢掉：远端小球、mask 碎块、地面反光点都靠它滤。

    这里是"筛选"，排序在 STEP 7 做（按面积取最大）。两者作用不同，别混。
    代价：min_area 调大必然漏掉小目标（任务 2 实验 2 的 p2/p3 就是这样漏检的）。
    """
    return [c for c in contours if cv2.contourArea(c) >= min_area]


# ═══════════════════════════════════════════════════════════════════════════
# STEP 7 / 7   计算目标中心点 (u, v)
#
#   本步用到的全部东西都在这里：
#     DEFAULT_CENTER_METHOD  中心点算法（--center 的默认值）
#     contour_center()       单个轮廓算中心点（两种方法）
#     locate_target()        多候选中选一个，再算它的面积和中心点
# ═══════════════════════════════════════════════════════════════════════════
DEFAULT_CENTER_METHOD = "moments"      # 可选 "moments"（质心）或 "rect"（矩形中心）
CENTER_METHODS = ["moments", "rect"]   # 给 argparse 的 choices 用


def contour_center(cnt: np.ndarray, method: str = DEFAULT_CENTER_METHOD
                   ) -> tuple[float, float, str]:
    """算一个轮廓的中心点，返回 (u, v, 方法名)。

    moments 质心：按面积加权，抗边缘毛刺，形状不规则时也合理 —— 本项目选用
    rect   矩形： 外接矩形中心，简单直观，但会被少数突出点把框拉大，从而拉偏中心

    ── 矩（moments）到底是什么 ──────────────────────────────────────────
    cv2.moments(cnt) 返回一个字典，里面是轮廓的几何矩：
        m00 = 轮廓围出的面积（零阶矩）
        m10 = Σ(x · 权重)，m01 = Σ(y · 权重)（一阶矩）
    质心就是"面积加权平均位置"：
        u = m10 / m00        v = m01 / m00
    直觉理解：把轮廓当成一块密度均匀的薄板，质心就是它的重心。
    好处是每个像素按面积投票，边缘几个毛刺点拉不动它；
    而外接矩形只看最左/最右/最上/最下的点，一两个离群点就能把框撑大、把中心拉偏。

    ── 两个坑 ──────────────────────────────────────────────────────────
    1) m00 可能为 0（轮廓退化成一条线或一个点），必须防除零，
       否则 u/v 变成 nan，一路污染到 ROS2 消息里，排查起来极痛苦。
    2) 这里算出来的是"像素坐标"：原点在左上，u 向右、v 向下，
       不是米，也不是机器人坐标。
    """
    if method == "rect":
        x, y, w, h = cv2.boundingRect(cnt)
        return x + w / 2.0, y + h / 2.0, "rect"

    # cv2.moments 算图像的矩：m00 就是面积，m10/m00 是 x 方向质心，m01/m00 是 y 方向质心
    m = cv2.moments(cnt)
    if m["m00"] == 0:
        # 除零保护！细则明确要求处理 m00=0。
        # 不处理的话 u/v 会变成 nan，一路污染到 ROS2 消息里，而且极难排查
        x, y, w, h = cv2.boundingRect(cnt)
        return x + w / 2.0, y + h / 2.0, "rect(m00=0 回退)"
    return m["m10"] / m["m00"], m["m01"] / m["m00"], "moments"


def locate_target(candidates: list, center_method: str = DEFAULT_CENTER_METHOD
                  ) -> tuple[float, float, float, str, np.ndarray]:
    """多个候选中选出最终目标，并算它的面积和中心点。

    【细则 1.2 第 6 项】多候选的规则：max(candidates, key=contourArea)
    —— 取轮廓面积最大的那个。这条规则必须写进 README（p3 有 10 个候选，
    正是靠它选中了拜仁球而不是旁边的小白球；但 V>=50 时误选木柜也是因为它更大，
    所以"面积最大"这条规则本身有风险，要靠阈值先把干扰物切掉）。

    返回 (u, v, area, 方法名, 选中的轮廓)：
      u/v   像素坐标，直接填 DetectResult / ROS2 消息
      area  轮廓面积，任务是要求发出去的字段之一
      方法名 写进结果，方便自证"用了 moments 还是 rect"
      轮廓  留给任务 2 画图用（画粗绿线就画它）
    """
    cnt = max(candidates, key=cv2.contourArea)          # 面积最大的轮廓 = 最终目标
    area = float(cv2.contourArea(cnt))                  # 这个目标的面积，要发给 ROS2
    u, v, method = contour_center(cnt, center_method)   # 质心或矩形中心
    return u, v, area, method, cnt


# ═══════════════════════════════════════════════════════════════════════════
# 组装件一：DetectResult —— 七步跑完后要交出去的那份"结论"
# ═══════════════════════════════════════════════════════════════════════════
@dataclass
class DetectResult:
    """一次检测的结论（所有 STEP 跑完后由 detect_target() 组装出来）。

    重点：detected=False 时 u/v/area 一律填 0，意思是「这三个数没有意义」。
    绝不能让下游把 (0, 0) 理解成「目标在图像左上角」—— 细则里专门强调了这点，
    所以接收方必须先看 detected，再决定要不要用 u/v。
    """

    detected: bool = False        # 本帧有没有检测到目标
    u: float = 0.0                # 中心点横坐标（像素，向右增大）
    v: float = 0.0                # 中心点纵坐标（像素，向下增大）
    area: float = 0.0             # 选中轮廓的面积（像素平方）
    n_contours: int = 0           # mask 里全部外轮廓的个数
    n_candidates: int = 0         # 通过面积筛选后剩下的候选个数
    method: str = "moments"       # 中心点用的哪种算法（写进 README 要说明）
    message: str = ""             # 人话说明，直接打印给终端看

    def as_tuple(self) -> tuple[bool, float, float, float]:
        """任务 3 填 ROS2 消息时直接用这个，四个值顺序和消息字段一一对应。"""
        return self.detected, self.u, self.v, self.area


# ═══════════════════════════════════════════════════════════════════════════
# 组装件二：detect_target —— 把 STEP 2 ~ STEP 7 串成一条链
# ═══════════════════════════════════════════════════════════════════════════
def detect_target(bgr: np.ndarray | None, color: str = DEFAULT_COLOR,
                  min_area: float = 500.0,
                  do_open: bool = False, do_close: bool = True, ksize: int = MORPH_KSIZE,
                  center_method: str = DEFAULT_CENTER_METHOD,
                  h_shift: int = 0, s_shift: int = 0, v_shift: int = 0
                  ) -> tuple[DetectResult, dict]:
    """核心检测函数：输入 BGR 图像，返回 (检测结果, 调试信息)。

    输入是"一张图的数据"，不是文件路径 —— 这样任务 3 的 ROS2 节点
    可以直接把相机帧传进来，不必落盘。STEP 1（读文件）在外面由 main() 做。

    函数体就是按顺序调用 STEP 2 ~ STEP 7，没有任何自己的算法：
        STEP 2  to_hsv()           转 HSV
        STEP 3  build_mask()       生成 mask
        STEP 4  apply_morphology() 形态学清理
        STEP 5  extract_contours() 提外轮廓
        STEP 6  filter_by_area()   面积筛选
        STEP 7  locate_target()    取最大候选 + 算中心点

    ⚠ min_area 这里默认 500，比命令行默认 800 宽松：这是给"模块调用者"的兜底值，
      参数漏传时宁可多检一点也别漏检；命令行跑任务 1 请用 DEFAULT_MIN_AREA（800）。

    调试信息 debug 里有 hsv / mask / mask_clean / contours / candidates，
    任务 2 画图和任务 3 复用时直接拿，不要重新算一遍（省时间也避免参数不一致）。
    """
    # 第一件事永远是判空：imread 失败返回 None 而不是抛异常
    # 【细则 1.2 第 1 项】读取图片并检查是否成功 —— 判空就在这一句
    if bgr is None:
        return DetectResult(message="未检测到目标：图像读取失败（imread 返回 None）"), {}

    hsv = to_hsv(bgr)                                          # STEP 2 转 HSV
    mask = build_mask(hsv, color, h_shift, s_shift, v_shift)   # STEP 3 生成 mask
    mask_clean = apply_morphology(mask, do_open, do_close, ksize)   # STEP 4 清理
    contours = extract_contours(mask_clean)                    # STEP 5 提轮廓
    candidates = filter_by_area(contours, min_area)            # STEP 6 面积筛选

    debug = {
        "hsv": hsv,
        "mask": mask,             # 原始 mask（未做形态学）
        "mask_clean": mask_clean, # 形态学之后的 mask
        "contours": contours,     # 所有轮廓
        "candidates": candidates, # 通过面积筛选的候选
    }

    # ──────────────────────────────────────────────────────────────────────
    # 【细则 1.2 第 5 项】没找到目标时，必须输出明确提示，且不能返回未初始化的坐标
    #   下面两个分支返回的 DetectResult 里 detected=False、u=v=area=0.0，
    #   0 只是"无意义"的占位值，绝不能理解成"目标在图像左上角"。
    #   分支 1 和分支 2 分开写，是为了让你一眼看出该调阈值还是该调 min_area。
    # ──────────────────────────────────────────────────────────────────────
    # 失败分支 1：一个轮廓都没有 = mask 全黑 = 画面里压根没有这个颜色
    if not contours:
        return DetectResult(message="未检测到目标：掩膜全黑，没有符合阈值范围的像素"), debug

    # 失败分支 2：有轮廓但都被面积阈值挡掉了 = 目标太小或 mask 太碎
    # 这里把最大的三个面积打出来，方便判断该调阈值还是调 min_area
    if not candidates:
        biggest = sorted((round(cv2.contourArea(c), 1) for c in contours), reverse=True)[:3]
        return DetectResult(
            n_contours=len(contours),
            message=(f"未检测到目标：{len(contours)} 个轮廓都小于最小面积 {min_area:g}"
                     f"（最大的三个：{biggest}）"),
        ), debug

    u, v, area, method, _ = locate_target(candidates, center_method)   # STEP 7

    result = DetectResult(
        detected=True,
        u=float(u),
        v=float(v),
        area=area,
        n_contours=len(contours),
        n_candidates=len(candidates),
        method=method,
        message=f"检测到目标：{len(candidates)} 个候选中取面积最大者",
    )
    return result, debug


# ═══════════════════════════════════════════════════════════════════════════
# 批处理外壳：输出的目录、扫目录、逐张跑、打表、写 CSV
#   （这部分不属于算法的任何一步，只负责"批量"和"汇报"，可整块替换）
# ═══════════════════════════════════════════════════════════════════════════
def _default_out_dir() -> Path:
    """输出目录：优先 ~/ros2_ws/outputs/task1，找不到工作空间就放脚本旁边。"""
    base = Path.home() / "ros2_ws"
    return (base if base.is_dir() else SCRIPT_DIR) / "outputs" / "task1"


DEFAULT_OUT_DIR = _default_out_dir()


def load_ground_truth(path: Path) -> dict[str, tuple[int, int, int]]:
    """读 ground_truth.csv（人工目测的球心），用来算检测误差。

    没有这个文件就返回空字典 —— 有没有误差列都不影响检测本身。
    """
    if not path.exists():
        return {}
    gt: dict[str, tuple[int, int, int]] = {}
    with path.open(encoding="utf-8") as f:
        for row in csv.DictReader(f):      # DictReader 按表头把每行变成字典
            gt[row["image"]] = (int(row["has_target"]), float(row["u"]), float(row["v"]))
    return gt


def main() -> int:
    """命令行入口：遍历目录下的所有图片，逐张检测并打印表格。"""
    print("=" * 78)
    print(f"姓名: {STUDENT_NAME} ({STUDENT_ID})    "
          f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")

    parser = argparse.ArgumentParser(description="HSV 颜色目标检测（第 2 周任务 1）")
    # 各参数的默认值都取自对应 STEP 区块里的常量，改参数只需改那一个地方
    parser.add_argument("--images", type=Path, default=DEFAULT_IMAGE_DIR, help="图片目录")
    parser.add_argument("--color", default=DEFAULT_COLOR, choices=sorted(HSV_PRESETS))
    parser.add_argument("--min-area", type=float, default=DEFAULT_MIN_AREA,
                        help="最小轮廓面积（像素平方）")
    parser.add_argument("--open", action="store_true", help="加开运算（去小白点）")
    parser.add_argument("--no-close", action="store_true", help="关闭闭运算")
    parser.add_argument("--ksize", type=int, default=MORPH_KSIZE, help="形态学核大小")
    parser.add_argument("--center", default=DEFAULT_CENTER_METHOD, choices=CENTER_METHODS)
    parser.add_argument("--h-shift", type=int, default=0, help="色相区间两端各放宽多少")
    parser.add_argument("--s-shift", type=int, default=0, help="饱和度下限偏移")
    parser.add_argument("--v-shift", type=int, default=0, help="明度下限偏移")
    parser.add_argument("--save-mask", action="store_true", help="保存 mask 图片")
    parser.add_argument("--out", type=Path, default=DEFAULT_OUT_DIR, help="输出目录")
    args = parser.parse_args()

    if not args.images.is_dir():
        print(f"图片目录不存在：{args.images}")
        return 1

    # 只挑图片后缀，并且排序，保证每次输出顺序一致（截图对比时才不会乱）
    images = sorted(p for p in args.images.iterdir() if p.suffix.lower() in IMAGE_SUFFIX)
    if not images:
        print(f"{args.images} 下没有图片")
        return 1

    gt = load_ground_truth(args.images / "ground_truth.csv")
    if args.save_mask:
        args.out.mkdir(parents=True, exist_ok=True)

    # 先把本次运行的参数打印出来 —— 截图里必须能看到用的什么参数，否则无法复现
    preset = HSV_PRESETS[args.color]
    print("=" * 78)
    print(f"目标颜色 : {args.color}   HSV 色相 {preset['h_ranges']}  "
          f"S>={preset['s_range'][0] + args.s_shift}  V>={preset['v_range'][0] + args.v_shift}"
          + (f"  (色相放宽 ±{args.h_shift})" if args.h_shift else ""))
    print(f"最小面积 : {args.min_area:g}   形态学 : "
          f"{'开 ' if args.open else ''}{'关' if args.no_close else '闭'}(ksize={args.ksize})"
          f"   中心点 : {args.center}")
    print("=" * 78)
    print(f"{'图片':<18}{'检出':<6}{'u':>8}{'v':>8}{'面积':>10}{'候选':>6}{'误差px':>9}  说明")
    print("-" * 78)

    rows, n_ok = [], 0
    # 五张实拍照片（p1/p3/p4/p5/p6）就是在这里被逐个处理的
    for path in images:
        bgr = read_image(path)          # STEP 1 读图（读失败返回 None，由 detect_target 判空）
        # 这里一次调用就串起了 STEP 2 ~ STEP 7，返回值里的 u/v 就是最终中心点
        res, debug = detect_target(
            bgr, color=args.color, min_area=args.min_area,
            do_open=args.open, do_close=not args.no_close, ksize=args.ksize,
            center_method=args.center,
            h_shift=args.h_shift, s_shift=args.s_shift, v_shift=args.v_shift,
        )

        # 有真值时算欧氏距离当误差；顺便标出漏检/误检
        err = ""
        if path.name in gt:
            has, gu, gv = gt[path.name]
            if has and res.detected:
                err = f"{((res.u - gu) ** 2 + (res.v - gv) ** 2) ** 0.5:.1f}"
            elif has and not res.detected:
                err = "漏检"          # 有目标却没检出来
            elif not has and res.detected:
                err = "误检"          # 没目标却检出了东西

        n_ok += int(res.detected)
        print(f"{path.name:<18}{'是' if res.detected else '否':<6}"
              f"{res.u:>8.1f}{res.v:>8.1f}{res.area:>10.0f}{res.n_candidates:>6}{err:>9}  {res.message}")
        rows.append([path.name, int(res.detected), round(res.u, 2), round(res.v, 2),
                     round(res.area, 1), res.n_candidates, err, res.method])

        # 存 mask：调参时对比两张 mask 图，比盯数字直观得多
        if args.save_mask and debug:
            cv2.imwrite(str(args.out / f"mask_{path.stem}.png"), debug["mask"])
            cv2.imwrite(str(args.out / f"mask_clean_{path.stem}.png"), debug["mask_clean"])

    print("-" * 78)
    print(f"共 {len(images)} 张，检出 {n_ok} 张，未检出 {len(images) - n_ok} 张")

    # 结果存 CSV，README 里的表格可以直接从这里抄
    args.out.mkdir(parents=True, exist_ok=True)
    csv_path = args.out / "result_task1.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["image", "detected", "u", "v", "area", "candidates", "gt_error", "center_method"])
        w.writerows(rows)
    print(f"结果已写入 {csv_path}")
    if args.save_mask:
        print(f"掩膜已写入 {args.out}")
    return 0


if __name__ == "__main__":
    # raise SystemExit(main()) 把 main 的返回值当成进程退出码（0=正常）
    raise SystemExit(main())
