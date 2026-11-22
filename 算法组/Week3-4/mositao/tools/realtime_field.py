# -*- coding: utf-8 -*-
"""任务二实时定位：单目相机 -> 场地标志 -> 地面点 -> 地图匹配 -> 二维位姿。

窗口分左右两半（细则要求「可视化处理流程图」）：
    左半  相机原始画面 + 标志检测框 + 类别 + 置信度 + 几何细化后的交点
    右半  场地俯视图 + 全部已知标志 + 本帧位姿（相机位置与朝向）

每帧按细则 2.7 的链路走，一步都不绕：
    像素 (u,v) → 去畸变 → K⁻¹ 得相机系射线 d_c → 外参 (R,t) 转载体 → 与地面 Z=0 求交
    → 载体系地面点 (x_b,y_b) → 与 field.yaml 匹配 → (x,yaw)

外参是「相机相对底座的固定安装」，必须先用 tools/solve_extrinsics_auto.py（或
tools/estimate_extrinsics.py）解出 calibration/camera_extrinsics.yaml，本程序只读它。

用法（项目根目录）：
    run.bat tools\\realtime_field.py                       # 真相机 index 1
    run.bat tools\\realtime_field.py --video data\\videos\\marker_s03.mp4 --no-show
    run.bat tools\\realtime_field.py --record outputs\\realtime\\field_demo.mp4

按键：q 退出 / s 存图 / 空格 暂停
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml                                # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                     # noqa: E402
from src.loc.camera_model import (                                          # noqa: E402
    CameraModel, Extrinsics, pixels_to_ground,
)
from src.loc.field_map import FieldMap, FieldMarker, to_field_xy            # noqa: E402
from src.loc.matcher import (                                               # noqa: E402
    MatchParams, ObservedMarker, locate, wrap_pi,
)
from src.loc.pnp_locator import PnpParams, locate_pnp, _planarity_ratio        # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                           # noqa: BLE001
    pass

BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
W, H = 1280, 720

TYPE_COLOR = {                       # BGR
    "LCross": (0, 220, 0),
    "TCross": (230, 160, 0),
    "XCross": (0, 0, 235),
    "PenaltyPoint": (200, 0, 200),
}
SHAPE_NAME = {"LCross": "L", "TCross": "T", "XCross": "X", "PenaltyPoint": "P"}
MAX_RANGE_M = 12.0     # 地面落点距离上限（米）。场地对角约 9.7 m，取 12 m 留余量：
                       # 早先写 7 m，会把相机斜看的远端交点（8~10 m）判成「反投影失败」
                       # 而直接丢掉，实测这是「明明检出却定位不了」的来源之一。
                       # 球门网等空中目标远于 12 m 仍会被滤掉。


# ---------------------------------------------------------------------------
# 地图
# ---------------------------------------------------------------------------
def _sym_fill(pts: list[FieldMarker], axis: str, tol: float = 0.30) -> list[FieldMarker]:
    """按某一轴对称补点：axis='y' 关于 y=0，axis='x' 关于 x=0。

    足球场对长边中线（y=0）和中线（x=0）都对称，所以两类对称都要补。
    官方 14 点在 +x 半场里本身就漏了罚球区前沿的 (+4, +3) 角点，
    只按 x 镜像会把这个空缺原样复制成两处缺口——右屏看起来就不再中心对称。
    """
    out = list(pts)
    for m in pts:
        ref = FieldMarker(type=m.type, x=m.x, y=-m.y) if axis == "y" \
            else FieldMarker(type=m.type, x=-m.x, y=m.y)
        if any(o.type == ref.type and math.hypot(o.x - ref.x, o.y - ref.y) <= tol
               for o in out):
            continue
        ref.name = (m.name or "m") + f"_{axis}sym"
        out.append(ref)
    return out


def _site_geometry(site_path):
    """读出 config/field_site.yaml 里的现场几何（没有就返回 None）。

    为什么需要它：现场 7.60×6.03 m 与官方 14×9 m 的各条线比例并不一致
    （0.50~1.14），整体缩放对齐不了；但标志点永远是「白线交点」，
    只要知道线在哪，交点就能算出来，所以按尺寸参数化重建，不做缩放。
    """
    p = Path(site_path) if site_path else None
    if p is None or not p.exists():
        return None
    ov = load_yaml(p).get("field_site_overrides", {}) or {}
    sg = ov.get("site_geometry")
    if isinstance(sg, dict) and sg.get("field_length_m"):
        return sg
    return None


def build_site_markers(sg: dict, mirror: bool = True) -> list[FieldMarker]:
    """按现场实测尺寸算白线交点（mirror=True 时补上 x<0 的另一半场）。

    与 tools/site_field_map.py 同一套公式；改尺寸后坐标自动跟着变：
        T1/T4 球门线 × 罚球区两竖线     T2/T3 球门线 × 球门区两竖线
        T5/T6 中线 × 两边线            L1/L2 球门线 × 边线（角点）
        L3/L4 球门区前沿 × 球门区竖线    L5/L6 罚球区前沿 × 罚球区竖线
        X1/X2 中线 × 中圈
    """
    hl = float(sg["field_length_m"]) / 2.0
    hw = float(sg["field_width_m"]) / 2.0
    pen_w = float(sg["penalty_area_width_m"]) / 2.0
    pen_d = float(sg["penalty_area_length_m"])
    goal_w = float(sg["goal_area_width_m"]) / 2.0
    goal_d = float(sg["goal_area_length_m"])
    circ = float(sg["circle_diameter_m"]) / 2.0

    right = [
        ("T1", "TCross", hl, -pen_w), ("T2", "TCross", hl, -goal_w),
        ("T3", "TCross", hl, +goal_w), ("T4", "TCross", hl, +pen_w),
        ("L1", "LCross", hl, -hw), ("L2", "LCross", hl, +hw),
        ("L3", "LCross", hl - goal_d, -goal_w), ("L4", "LCross", hl - goal_d, +goal_w),
        ("L5", "LCross", hl - pen_d, -pen_w), ("L6", "LCross", hl - pen_d, +pen_w),
    ]
    center = [
        ("T5", "TCross", 0.0, -hw), ("T6", "TCross", 0.0, +hw),
        ("X1", "XCross", 0.0, -circ), ("X2", "XCross", 0.0, +circ),
    ]
    pts = [FieldMarker(type=t, x=float(x), y=float(y), name=f"{c}@site")
           for c, t, x, y in right + center]
    if sg.get("penalty_mark_visible"):
        pts.append(FieldMarker(type="PenaltyPoint", x=hl - 2.1, y=0.0, name="P@site"))
    # 镜像到另一半场（球场对 x=0 对称）
    if mirror:
        for c, t, x, y in right:
            if abs(x) > 1e-9:
                pts.append(FieldMarker(type=t, x=-float(x), y=float(y), name=f"{c}m@site"))
    return pts


def _site_geometry_dict(sg: dict, geom: dict) -> dict:
    """把现场尺寸覆盖进几何字典，绘图与覆盖范围都跟着变成现场尺度。"""
    g = dict(geom)
    g.update({
        "field_length_m": float(sg["field_length_m"]),
        "field_width_m": float(sg["field_width_m"]),
        "goal_width_m": float(sg["goal_width_m"]),
        "circle_radius_m": float(sg["circle_diameter_m"]) / 2.0,
        "penalty_area_length_m": float(sg["penalty_area_length_m"]),
        "penalty_area_width_m": float(sg["penalty_area_width_m"]),
        "goal_area_length_m": float(sg["goal_area_length_m"]),
        "goal_area_width_m": float(sg["goal_area_width_m"]),
        "site_verified": True,
    })
    return g


def apply_site_overrides(pts: list[FieldMarker], site_path,
                         mirror: bool = True) -> tuple[list[FieldMarker], list[str]]:
    """叠加「现场实测」覆盖，官方 field.yaml 一个字都不改。

    现场与官方地图有出入时（例如点球点实际在场地中心，而官方反推在 (4.9, 0)），
    按细则不能擅自篡改官方文件，但程序必须按现场事实工作。
    于是把差异单独记在 config/field_site.yaml，由这一层叠加：官方是哪几个点、现场在哪，
    两边都留痕，评审一眼能看出哪部分是我们加的。

    若本文件带了 site_geometry（现场整场实测尺寸），则直接按尺寸参数化重建全部
    标志点，不再使用官方坐标 —— 因为现场与官方的差异不是平移/旋转/缩放能表达的。
    """
    applied: list[str] = []
    p = Path(site_path) if site_path else None
    if p is None or not p.exists():
        return pts, applied
    sg = _site_geometry(site_path)
    if sg is not None:
        rebuilt = build_site_markers(sg, mirror=mirror)
        applied.append(
            "现场地图：L %.2f × W %.2f m（官方 14×9 m），按白线交点重建 %d 个标志；"
            "官方 field.yaml 未改动" % (sg["field_length_m"], sg["field_width_m"], len(rebuilt))
        )
        return rebuilt, applied
    ov = load_yaml(p).get("field_site_overrides", {}) or {}
    pm = ov.get("point_mark")
    if isinstance(pm, dict) and pm.get("site_xy"):
        t = pm.get("type", "PenaltyPoint")
        sx, sy = (float(v) for v in pm["site_xy"])
        n_before = sum(1 for m in pts if m.type == t)
        pts = [m for m in pts if m.type != t]
        pts.append(FieldMarker(type=t, x=sx, y=sy, name=f"{t}@site"))
        applied.append(f"{t} 官方 {pm.get('official_xy')} → 现场 ({sx:.2f}, {sy:.2f})"
                       f"（替换掉 {n_before} 个官方点）")
    return pts, applied


def build_field_map(field_path: Path, geometry_path: Path, mirror: bool = True,
                    site_path=None) -> FieldMap:
    """读官方 field.yaml，换算到场地中心系，按对称性补全整场，再叠加现场覆盖。

    官方 field.yaml 的坐标不是场地中心系（相对中心转了 90° 并平移），
    换算公式由 config/field_geometry.yaml 给出。官方只给了 x∈[0,7] 半场，且半场内
    还漏了一个罚球区前沿角点；这里按「球场对 y=0 与 x=0 都对称」补成完整 26 点，
    再按 config/field_site.yaml 把与现场不符的点改到实测位置。
    这是几何推理与现场登记，不改动官方给出的任何数值。
    """
    fm = FieldMap.from_files(field_path, geometry_path)
    geom = fm.geometry
    pts: list[FieldMarker] = []
    for m in fm.markers:
        fx, fy = to_field_xy(m.x, m.y, geom)
        pts.append(FieldMarker(type=m.type, x=float(fx), y=float(fy), name=m.name))
    if mirror:
        pts = _sym_fill(pts, "y")      # 补 (+4, +3) 这类同侧缺口
        pts = _sym_fill(pts, "x")      # 镜像到另一半场
    pts, applied = apply_site_overrides(pts, site_path, mirror=mirror)
    sg = _site_geometry(site_path)
    geom_out = _site_geometry_dict(sg, geom) if sg is not None else geom
    return FieldMap(markers=pts, geometry=geom_out, field_path=fm.field_path,
                    geometry_path=fm.geometry_path, field_sha256=fm.field_sha256,
                    warnings=list(fm.warnings) + [f"site: {a}" for a in applied])


def field_extent(fm: FieldMap) -> tuple[float, float, float, float]:
    """地图覆盖范围；取几何文件里的场地尺寸，缺省用 14×9。"""
    lo = fm.geometry.get("field_length_m") or 14.0
    wi = fm.geometry.get("field_width_m") or 9.0
    return -float(lo) / 2.0, float(lo) / 2.0, -float(wi) / 2.0, float(wi) / 2.0


# ---------------------------------------------------------------------------
# 框内几何细化
# ---------------------------------------------------------------------------
def white_mask(bgr: np.ndarray, ksize: int = 31, thr: int = 45) -> np.ndarray:
    """白色场地线掩膜：用**亮度顶帽**取「比邻域更亮」的细结构。

    为什么不再用「低饱和 + 高亮度」：
        这块场地的草皮在灯光下几乎是灰的 —— 实测样本 BGR (128,137,135)、(113,117,116)，
        饱和度 S 中位只有 9。旧判据 S<95 且 V>135 会把草皮大片判成白线（整帧命中 86.8%），
        于是交点细化、白线距离对齐、几何预标全部失去意义。
        顶帽（原图减去开运算）只保留「比周围亮一截的细条」，与绝对亮度、饱和度无关，
        白线在这种灰色草皮上依然比邻域亮 30~80 个灰阶。

    参数：ksize 顶帽结构元尺寸（像素，≥ 线宽即可）；thr 亮出邻域多少灰阶算线。
    返回：uint8 掩膜，线为 255。
    """
    v = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[:, :, 2]
    k = cv2.getStructuringElement(cv2.MORPH_RECT, (ksize, ksize))
    tophat = cv2.morphologyEx(v, cv2.MORPH_TOPHAT, k)
    return (tophat > thr).astype(np.uint8) * 255


def junction_ok(bgr: np.ndarray, box, mode: str, bins: int = 18,
                min_dir_frac: float = 0.20, min_perp_frac: float = 0.20,
                min_len_frac: float = 0.30, tol_deg: float = 35.0) -> bool:
    """几何验证：真正的 L / T / X 在框内只有**两个互相接近垂直**的主方向。

    圆弧、球、拖把、道具之类会给出一堆方向分散的短线段，主方向不成对——
    低机位下模型虚警多，用它把这些框滤掉（代价是极少数退化拐角会被误杀）。
    判据：最强方向占比 ≥ min_dir_frac，且与之接近垂直的方向占比 ≥ min_perp_frac。
    """
    if mode != "cross":
        return False
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    gw, gh = bgr.shape[1], bgr.shape[0]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(gw - 1, x2), min(gh - 1, y2)
    if x2 - x1 < 8 or y2 - y1 < 8:
        return False
    roi = bgr[y1:y2 + 1, x1:x2 + 1]
    mask = white_mask(roi)
    rw, rh = mask.shape[1], mask.shape[0]
    min_len = max(8, int(min_len_frac * min(rw, rh)))
    segs = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=max(10, min_len // 2),
                           minLineLength=min_len, maxLineGap=4)
    if segs is None:
        return False
    hist = np.zeros(bins, float)
    for s in np.asarray(segs).reshape(-1, 4):
        dx, dy = float(s[2] - s[0]), float(s[3] - s[1])
        L = float(np.hypot(dx, dy))
        if L < min_len:
            continue
        ang = math.degrees(math.atan2(dy, dx)) % 180.0
        hist[int(ang / (180.0 / bins)) % bins] += L
    total = hist.sum()
    if total <= 0:
        return False
    hist /= total
    i1 = int(np.argmax(hist))
    v1 = float(hist[i1])
    target = (i1 + bins // 2) % bins          # 与主方向垂直的方向
    off = max(1, int(round(tol_deg / (180.0 / bins))))
    idx = [(target + k) % bins for k in range(-off, off + 1)]
    v2 = float(hist[idx].max())
    return v1 >= min_dir_frac and v2 >= min_perp_frac


def dedup_dets(dets, radius: float = 35.0):
    """同一交点被重复检出时只留置信度最高的那个。

    为什么需要：低机位（0.73 m）掠射视角下模型给不出高分（0.1~0.2），同一个交点常常
    同时被判成 T、L、X 三个框，位置几乎一样。若照单全收，matcher 会把三个「不同类别」
    的观测喂进同一个地图点，关联必然错乱。这里按像素距离合并，保留最高分并把合并数
    记在 merged 上，便于界面显示「3 检测 / 2 点」这类信息。
    """
    kept = []
    for d in sorted(dets, key=lambda z: -z["conf"]):
        near = None
        for k in kept:
            if math.hypot(d["u"] - k["u"], d["v"] - k["v"]) <= radius:
                near = k
                break
        if near is None:
            d = dict(d)
            d["merged"] = 1
            kept.append(d)
        else:
            near["merged"] = near.get("merged", 1) + 1
    return kept


MAX_ROI_PX = 240      # rectify_junction 处理的最大边长（像素），超过先降采样


def refine_junction(bgr: np.ndarray, box, max_shift: float = 0.42):
    """把检测框中心细化成「线段交汇点」。

    细则 2.3 明确：不能直接把 bbox 中心当作 L/T/X 的几何交点，要在框内做几何细化。
    做法：
        1) 框内取白色线掩膜，HoughLinesP 找直线段；
        2) 两两求交，取落在框内、且离框中心最近的交点；
        3) 退路一：白像素质心；退路二：框中心。
    返回 (u, v, mode, n_lines)，mode ∈ {"cross","centroid","bbox"}。
    """
    x1, y1, x2, y2 = [int(round(v)) for v in box]
    gw, gh = bgr.shape[1], bgr.shape[0]
    x1, y1 = max(0, x1), max(0, y1)
    x2, y2 = min(gw - 1, x2), min(gh - 1, y2)
    cu, cv_ = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    if x2 - x1 < 6 or y2 - y1 < 6:
        return cu, cv_, "bbox", 0

    roi = bgr[y1:y2 + 1, x1:x2 + 1]
    # 大框先降采样：Hough 的代价与像素数成正比，十几框时这一项能把 FPS 从 9 拉到 20+
    k_down = 1.0
    if max(roi.shape[:2]) > MAX_ROI_PX:
        k_down = MAX_ROI_PX / float(max(roi.shape[:2]))
        roi = cv2.resize(roi, None, fx=k_down, fy=k_down, interpolation=cv2.INTER_AREA)
    mask = white_mask(roi, ksize=max(9, int(round(31 * k_down))))
    n_white = int((mask > 0).sum())

    if n_white >= 18:
        # 1) 直线段
        rw, rh = mask.shape[1], mask.shape[0]
        min_len = max(8, int(0.45 * min(rw, rh)))
        segs = cv2.HoughLinesP(mask, 1, np.pi / 180, threshold=max(10, min_len // 2),
                               minLineLength=min_len, maxLineGap=4)
        lines = []
        if segs is not None:
            for s in np.asarray(segs).reshape(-1, 4):
                x3, y3, x4, y4 = (float(v) for v in s)
                dx, dy = x4 - x3, y4 - y3
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    continue
                lines.append((dx, dy, x3, y3))
        # 2) 两两求交（齐次叉乘）
        best, best_d = None, 1e9
        lim = max_shift * max(rw, rh)
        for i in range(len(lines)):
            for j in range(i + 1, len(lines)):
                d1x, d1y, p1x, p1y = lines[i]
                d2x, d2y, p2x, p2y = lines[j]
                den = d1x * d2y - d1y * d2x
                if abs(den) < 1e-6:
                    continue                      # 平行
                # 夹角太小说明是同一根线的两段，交点会跑到无穷远
                n1, n2 = np.hypot(d1x, d1y), np.hypot(d2x, d2y)
                if abs((d1x * d2x + d1y * d2y) / max(n1 * n2, 1e-9)) > 0.97:
                    continue
                t = ((p2x - p1x) * d2y - (p2y - p1y) * d2x) / den
                ix, iy = p1x + t * d1x, p1y + t * d1y
                if not (0 <= ix <= rw - 1 and 0 <= iy <= rh - 1):
                    continue
                d = float(np.hypot(ix - rw / 2.0, iy - rh / 2.0))
                if d < best_d and d <= lim:
                    best_d, best = d, (ix, iy)
        if best is not None:
            return x1 + best[0] / k_down, y1 + best[1] / k_down, "cross", len(lines)
        # 3) 白像素质心
        ys, xs = np.nonzero(mask)
        return (float(x1 + xs.mean() / k_down), float(y1 + ys.mean() / k_down),
                "centroid", len(lines))
    return cu, cv_, "bbox", 0


# ---------------------------------------------------------------------------
# 绘制
# ---------------------------------------------------------------------------
class MapView:
    """右半屏：场地俯视图。世界 (x,y) -> 画布。"""

    def __init__(self, x0, y0, x1, y1, extent):
        self.box = (x0, y0, x1, y1)
        ex0, ex1, ey0, ey1 = extent
        pad = 0.9
        self.ex0, self.ex1 = ex0 - pad, ex1 + pad      # X：画面上为纵向（+X 朝上）
        self.ey0, self.ey1 = ey0 - pad, ey1 + pad      # Y：画面上为横向（+Y 朝左）
        # 与组长《场地坐标系示意图·俯视图》同向
        self.scale = min((y1 - y0) / (self.ex1 - self.ex0),
                         (x1 - x0) / (self.ey1 - self.ey0))
        self.ox = x0 + ((x1 - x0) - self.scale * (self.ey1 - self.ey0)) / 2.0
        self.oy = y0 + ((y1 - y0) - self.scale * (self.ex1 - self.ex0)) / 2.0

    def to_px(self, x, y):
        # +X 指向球门（画面朝上）、+Y 为面朝 +X 时的左侧（画面朝左）
        return (int(self.ox + (self.ey1 - y) * self.scale),
                int(self.oy + (self.ex1 - x) * self.scale))

    @staticmethod
    def _dir_px(th):
        """世界方向 (cos th, sin th) 在本俯视图上的像素方向。"""
        return -np.sin(th), -np.cos(th)

    def draw(self, canvas, fm: FieldMap, extent, result, det_dbg, n_obs, matched=None):
        x0, y0, x1, y1 = self.box
        cv2.rectangle(canvas, (x0, y0), (x1, y1), (40, 40, 40), -1)

        fx0, fx1, fy0, fy1 = self.extent_field(extent)
        p_tl, p_br = self.to_px(fx0, fy1), self.to_px(fx1, fy0)
        cv2.rectangle(canvas, p_tl, p_br, (90, 90, 90), 1)
        # 中线
        cv2.line(canvas, self.to_px(0, fy0), self.to_px(0, fy1), (90, 90, 90), 1)
        # 中圈
        r = float(fm.geometry.get("circle_radius_m") or 1.5)
        cv2.circle(canvas, self.to_px(0, 0), int(r * self.scale), (90, 90, 90), 1)

        for m in fm.markers:
            c = self.to_px(m.x, m.y)
            col = TYPE_COLOR.get(m.type, (200, 200, 200))
            if m.type == "LCross":
                cv2.drawMarker(canvas, c, col, cv2.MARKER_TRIANGLE_UP, 9, 2)
            elif m.type == "TCross":
                cv2.drawMarker(canvas, c, col, cv2.MARKER_SQUARE, 8, 2)
            elif m.type == "XCross":
                cv2.drawMarker(canvas, c, col, cv2.MARKER_TILTED_CROSS, 9, 2)
            else:
                # 点球点在场地中心（中圈圆心），但现场没有可见标记 —— 用空心圈表示「几何点、不可检测」
                cv2.circle(canvas, c, 5, col, 1)

        # 观测（载体系）投影回地图：用本帧位姿把它摆到地图上，能一眼看出匹配好不好
        # 灰点 = 检测到了但没配上地图；绿色实心 + 序号 = 真正参与定位的内点
        if result.x == result.x:      # 有坐标就画（ambiguous 被采用时也算）
            th = np.radians(result.yaw_deg)
            c_, s_ = np.cos(th), np.sin(th)
            hit = {int(m[0]) for m in (matched or [])}
            for i, dbg in enumerate(det_dbg or []):
                if dbg is None or i in hit:
                    continue
                _t, _u, _v, ox, oy = dbg
                if not (math.isfinite(ox) and math.isfinite(oy)):
                    continue
                mx = result.x + c_ * ox - s_ * oy
                my = result.y + s_ * ox + c_ * oy
                cv2.circle(canvas, self.to_px(mx, my), 3, (110, 110, 110), 1)

            for k, (oi, mt, mx, my, d) in enumerate(matched or []):
                c = self.to_px(mx, my)
                cv2.circle(canvas, c, 6, (80, 255, 120), -1, cv2.LINE_AA)
                cv2.putText(canvas, str(k + 1), (c[0] - 4, c[1] + 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 40, 0), 1)

            c = self.to_px(result.x, result.y)
            L = int(0.55 * self.scale)
            ux, uy = self._dir_px(th)
            tip = (int(c[0] + L * ux), int(c[1] + L * uy))
            lx, ly = self._dir_px(th + 2.5)
            left = (int(c[0] + 0.30 * L * lx), int(c[1] + 0.30 * L * ly))
            rx, ry = self._dir_px(th - 2.5)
            right = (int(c[0] + 0.30 * L * rx), int(c[1] + 0.30 * L * ry))
            cv2.fillConvexPoly(canvas, np.array([tip, left, c, right]), (0, 220, 255))
            cv2.putText(canvas, "camera", (c[0] + 8, c[1] + 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 220, 255), 1)

        cv2.putText(canvas, f"field view  obs={n_obs}", (x0 + 10, y0 + 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        # 坐标轴定义（细则视频证据第 2 条）：原点、+X 指向球门（画面上）、+Y 朝左
        ox, oy = self.to_px(0.0, 0.0)
        ax = int(1.9 * self.scale)      # 长于中圈半径 1.5 m，避免箭头与中圈重叠
        cv2.arrowedLine(canvas, (ox, oy), (ox, oy - ax), (80, 160, 255), 2, tipLength=0.18)
        cv2.putText(canvas, "+X goal", (ox + 12, oy - ax - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (80, 160, 255), 1)
        cv2.arrowedLine(canvas, (ox, oy), (ox - ax, oy), (255, 160, 80), 2, tipLength=0.18)
        cv2.putText(canvas, "+Y left", (ox - ax - 8, oy - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 160, 80), 1)
        cv2.circle(canvas, (ox, oy), 3, (255, 255, 255), -1)
        cv2.putText(canvas, "O", (ox + 5, oy + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1)

    def extent_field(self, extent):
        return extent


def open_source(args):
    if args.video:
        vp = Path(args.video)
        if not vp.is_absolute():
            vp = ROOT / vp
        if not vp.exists():
            print(f"没找到视频 {rel(vp)}")
            return None, "", 0, 0
        cap = cv2.VideoCapture(str(vp))
        if not cap.isOpened():
            print(f"打不开视频 {rel(vp)}")
            return None, "", 0, 0
        return (cap, "FILE", int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(args.index, api)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return (cap, name, int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    return None, "", 0, 0


def main() -> int:
    ap = argparse.ArgumentParser(description="实时场地标志定位（任务二）")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--video", default="", help="用视频文件代替相机（离线自检）")
    ap.add_argument("--weights", default="weights/best_marker_v11.pt",
                    help="默认 v11：现场实拍召回最高（同一帧 8–12 个框）。"
                         "v7 在低机位召回与速度最平衡但只出 2–4 个；"
                         "v9 对 0.73 m 掠射几乎零召回；v12 在低姿测试集指标最好但现场反而更差")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--site", default="config/field_site.yaml",
                    help="现场实测覆盖（官方文件不动，差异记这里）")
    ap.add_argument("--no-site", action="store_true", help="不叠加现场覆盖，只用官方数值")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=str(CALIB_DIR / "camera_extrinsics.yaml"))
    ap.add_argument("--conf", type=float, default=0.35,
                    help="检测阈值，默认 0.35。实测：样片上 0.20~0.50 区间有效定位率不变"
                         "（77%%），只是框多框少的差别；而现场实拍里交点置信度低，"
                         "0.45 时整帧只剩 3 个框、22/30 帧点数不足，0.30~0.35 才稳定出解。"
                         "框太多就往上调（别超 0.50），点数不够就往下调")
    ap.add_argument("--dedup", type=float, default=35.0,
                    help="同点去重半径（像素）：低阈值下同一交点会被判成 T/L/X 多个框，只留最高分")
    ap.add_argument("--require-cross", action="store_true",
                    help="只保留框内真的检出线段交汇（mode=cross）的检测，滤掉草地误检")
    ap.add_argument("--angle-filter", action="store_true",
                    help="实验：主方向验证（低机位下 Hough 出不了稳定主方向，实测会把真点也滤掉，暂不建议开）")
    ap.add_argument("--device", default="0")
    ap.add_argument("--imgsz", type=int, default=1280,
                    help="推理分辨率，默认 1280（= 相机原生）。实测现场单帧：640 只出 1 个框"
                         "（最高 0.50）、960 出 3 个（0.67）、1280 出 4 个（0.74）、1600 反而"
                         "回落到 3 个。远处小标志靠的是像素密度，不是亮度")
    ap.add_argument("--no-mirror", action="store_true", help="不镜像补全另半场（只用法官方 14 点）")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--record", default="")
    ap.add_argument("--print-every", type=int, default=10)
    ap.add_argument("--far-crop", action="store_true",
                    help="对画面远端带做放大重检，提升远处小标志的召回（切片推理）")
    ap.add_argument("--far-ratio", type=float, default=0.34,
                    help="远端带占画面高度比例（默认 0.34）")
    ap.add_argument("--far-scale", type=float, default=2.0, help="远端带放大倍数")
    ap.add_argument("--far-conf", type=float, default=0.20, help="远端带检测置信度门槛")
    ap.add_argument("--init-x", type=float, default=None, help="首帧先验：载体 x（米）")
    ap.add_argument("--init-y", type=float, default=None, help="首帧先验：载体 y（米）")
    ap.add_argument("--init-yaw", type=float, default=None, help="首帧先验：载体 yaw（度）")
    ap.add_argument("--init-win", type=float, default=2.0, help="先验位置窗口（米）")
    ap.add_argument("--init-yaw-win", type=float, default=18.0, help="先验航向窗口（度）")
    ap.add_argument("--match-tol", type=float, default=0.35,
                    help="地图匹配内点距离（米），默认 0.35")
    ap.add_argument("--min-markers", type=int, default=4,
                    help="最少匹配点数，默认 4（4 点即共面 PnP 的最小定解配置）")
    ap.add_argument("--pose-mode", choices=["pnp", "matcher"], default="pnp",
                    help="位姿解算路线：pnp=4 点最小解集 + 重投影迭代（不依赖外参，默认）；"
                         "matcher=旧的反投影 + SE(2) 匹配（依赖外参高度/俯角，留作对比）")
    ap.add_argument("--reproj-tol", type=float, default=12.0,
                    help="PnP 内点的重投影门限（像素），默认 12")
    ap.add_argument("--start-half", choices=["auto", "neg", "pos"], default="auto",
                    help="随机位置冷启动时指定相机在场地哪一端：neg=x<0 一侧，pos=x>0 一侧，"
                         "auto=不限制（会因场地中心对称而判 ambiguous）")
    ap.add_argument("--start-side", choices=["auto", "neg", "pos"], default="auto",
                    help="按 y 的正负指定相机在哪条边一侧：neg=y<0 一侧，pos=y>0 一侧。"
                         "站在中线附近时 x≈0、--start-half 分不开，但 180° 翻转会让 y 变号，"
                         "这时用本参数定一个 bit 即可消歧")
    ap.add_argument("--use-ambiguous", action="store_true",
                    help="ambiguous（存在等价解）时也采用最优解并标警告；"
                         "只看得见 3 个交点的方位靠它才能出坐标")
    ap.add_argument("--no-three-point", dest="allow_three_point", action="store_false",
                    help="禁用 3 点回退。默认允许：4 点定不出时用 3 点解并标黄色警告，"
                         "因为现场有些方位就是只看得见 3 个角；3 点无冗余、无法自证")
    args = ap.parse_args()

    wpath = Path(args.weights)
    if not wpath.is_absolute():
        wpath = ROOT / wpath
    epath = Path(args.extrinsics)
    if not epath.is_absolute():
        epath = ROOT / epath
    if not epath.exists():
        print(f"缺外参 {rel(epath)}")
        print("先跑：run.bat tools\\solve_extrinsics_auto.py --video data\\extr\\ext_02.mp4")
        return 1

    cam = CameraModel.from_yaml(ROOT / args.intrinsics)
    ext = Extrinsics.from_yaml(epath)
    fm = build_field_map(ROOT / args.field, ROOT / args.geometry,
                         mirror=not args.no_mirror,
                         site_path=None if args.no_site else (ROOT / args.site))
    extent = field_extent(fm)
    params = MatchParams(inlier_tol_m=args.match_tol, min_markers=args.min_markers)
    pnp_params = PnpParams(min_markers=args.min_markers, reproj_tol_px=args.reproj_tol)
    x_sign = {"auto": 0, "neg": -1, "pos": 1}.get(args.start_half, 0)
    y_sign = {"auto": 0, "neg": -1, "pos": 1}.get(args.start_side, 0)

    # 地图点索引，顺序必须与 matcher._build_index 一致（按 fm.markers 分组），
    # 这样 result.matches 里的 (type, 下标) 才能反查到坐标
    map_by_type: dict = {}
    for _m in fm.markers:
        map_by_type.setdefault(_m.type, []).append((_m.x, _m.y))

    # ---- 标定与地图的元信息（细则三.视频证据第 2、3 条要求展示）----
    geom = load_yaml(ROOT / args.geometry).get("field_geometry", {})
    geom_ver = f"{geom.get('version', '?')} ({geom.get('status', '?')})"
    site_txt = "（未启用现场覆盖）"
    if not args.no_site:
        s = load_yaml(ROOT / args.site).get("field_site_overrides", {})
        site_txt = f"+ site v{s.get('version', '?')}（{s.get('status', '?')}）"
    intr_yaml = load_yaml(ROOT / args.intrinsics).get("camera_intrinsics", {})
    dist_str = ", ".join(f"{d:.4f}" for d in np.asarray(cam.dist, float).ravel())
    optic_v = ext.R @ np.array([0.0, 0.0, 1.0])
    camx_v = ext.R @ np.array([1.0, 0.0, 0.0])
    pitch_deg = math.degrees(math.atan2(-optic_v[2], math.hypot(optic_v[0], optic_v[1])))
    roll_deg = math.degrees(math.asin(max(-1.0, min(1.0, float(camx_v[2])))))
    # 展示用：优先读外参文件里的标量字段，避免与 from_params 的 roll 符号约定混淆
    ext_yaml = load_yaml(epath).get("camera_extrinsics", {})
    pitch_show = float(ext_yaml.get("pitch_deg", pitch_deg))
    roll_show = float(ext_yaml.get("roll_deg", roll_deg))

    from ultralytics import YOLO
    model = YOLO(str(wpath))

    cap, backend, got_w, got_h = open_source(args)
    if cap is None:
        return 1

    print("=" * 84)
    print(f"任务二 实时定位｜权重 {rel(wpath)}｜conf≥{args.conf} imgsz={args.imgsz}｜{backend} idx={args.index}｜{got_w}x{got_h}")
    print("-" * 84)
    print("【地图】")
    print(f"  官方文件   {rel(ROOT / args.field)}（14 个标志，组织方发布，未改动）")
    print(f"  几何文件   {rel(ROOT / args.geometry)}  {geom_ver}")
    print(f"  现场登记   {rel(ROOT / args.site) if not args.no_site else '—'}  {site_txt}")
    print(f"  参与定位   {len(fm.markers)} 个标志（镜像补全 {'关' if args.no_mirror else '开'}）")
    print(f"  场地范围   x[{extent[0]:.2f}, {extent[1]:.2f}]  y[{extent[2]:.2f}, {extent[3]:.2f}] m")
    print("  坐标约定   原点=场地中心｜+X→唯一球门｜+Y=面朝+X时的左侧｜Z↑地面Z=0｜yaw 从 +X 逆时针为正")
    print("【相机】")
    print(f"  内参       fx={cam.K[0,0]:.2f}  fy={cam.K[1,1]:.2f}  cx={cam.K[0,2]:.2f}  cy={cam.K[1,2]:.2f}"
          f"  （{intr_yaml.get('calibration_images', '?')} 张，RMS {intr_yaml.get('reprojection_error_px', float('nan')):.3f} px）")
    print(f"  畸变       [{dist_str}]  模型 {intr_yaml.get('distortion_model', '?')}")
    print(f"  外参       {rel(epath)}")
    print(f"             高度 h={ext.t[2]:.3f} m｜俯角 pitch={pitch_show:+.2f}°｜滚转 roll={roll_show:+.2f}°"
          f"｜t=({ext.t[0]:+.3f}, {ext.t[1]:+.3f}, {ext.t[2]:+.3f})")
    print("=" * 84)
    for w in fm.warnings:
        if str(w).startswith("site:"):
            print(f"现场覆盖 {str(w)[5:].strip()}")
    print("按键：q 退出 / s 存图 / 空格 暂停")
    print("=" * 84)

    writer = None
    if args.record:
        rec = Path(args.record)
        if not rec.is_absolute():
            rec = ROOT / rec
        rec.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(rec), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (W, H))
        print(f"录制到 {rel(rec)}")

    out_dir = OUTPUT_DIR / "realtime"
    out_dir.mkdir(parents=True, exist_ok=True)

    fps_win, infer_win = deque(maxlen=30), deque(maxlen=30)
    frame_idx = 0
    paused = False
    # 首帧先验：现场地图关于场地两轴对称，没有先验时「相机在场地另一头、朝反方向」
    # 这一支同样能关联上同样多的点，于是判成 ambiguous。给一个来自标定的粗略位姿即可破除。
    prior = None
    if args.init_x is not None and args.init_y is not None and args.init_yaw is not None:
        prior = (args.init_x, args.init_y, np.radians(args.init_yaw),
                 args.init_win, args.init_yaw_win)
        print(f"首帧先验：x={args.init_x:+.2f} y={args.init_y:+.2f} "
              f"yaw={args.init_yaw:+.2f}°（窗口 ±{args.init_win} m / ±{args.init_yaw_win}°）")
    stats = {"frames": 0, "valid": 0, "three_point": 0, "ambiguous": 0, "insufficient": 0,
             "det_sum": 0, "matched_sum": 0}
    view = MapView(W // 2, 0, W, H, extent)

    print("预热中…", end="", flush=True)
    for _ in range(5):
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        model.predict(frame, conf=args.conf, imgsz=args.imgsz, device=args.device, verbose=False)
    print("完成")
    t_prev = time.perf_counter()

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("取帧结束")
                break
            frame_idx += 1
            stats["frames"] += 1

            t0 = time.perf_counter()
            res = model.predict(frame, conf=args.conf, imgsz=args.imgsz, device=args.device,
                                verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000
            infer_win.append(infer_ms)
            now = time.perf_counter()
            fps_win.append(1.0 / max(now - t_prev, 1e-9))
            t_prev = now
            fps_now = float(np.mean(fps_win)) if fps_win else 0.0

            # ---- 1) 检测 + 框内几何细化 ----
            dets = []
            if res.boxes is not None and len(res.boxes) > 0:
                for xyxy, cf, cl in zip(res.boxes.xyxy.cpu().numpy(),
                                        res.boxes.conf.cpu().numpy(),
                                        res.boxes.cls.cpu().numpy()):
                    tname = model.names[int(cl)]
                    u, v, mode, nline = refine_junction(frame, xyxy)
                    dets.append({"type": tname, "conf": float(cf), "box": xyxy,
                                 "u": float(u), "v": float(v), "mode": mode, "nline": nline})

            # 远端切片重检：远处的标志在整幅图里只有 1~2 px 线宽，检测器召回很低；
            # 把画面上部那一条带裁出来放大再推一次，能显著提高远处角点的置信度。
            if args.far_crop:
                h_f = frame.shape[0]
                y2f = int(h_f * args.far_ratio)
                crop = frame[:y2f, :]
                big = cv2.resize(crop, None, fx=args.far_scale, fy=args.far_scale,
                                 interpolation=cv2.INTER_CUBIC)
                res2 = model.predict(big, conf=args.far_conf, imgsz=args.imgsz,
                                     device=args.device, verbose=False)[0]
                if res2.boxes is not None and len(res2.boxes) > 0:
                    for xyxy, cf, cl in zip(res2.boxes.xyxy.cpu().numpy(),
                                            res2.boxes.conf.cpu().numpy(),
                                            res2.boxes.cls.cpu().numpy()):
                        tname = model.names[int(cl)]
                        box = xyxy / args.far_scale
                        try:
                            u, v, mode, nline = refine_junction(frame, box)
                        except Exception:                                  # noqa: BLE001
                            continue
                        dets.append({"type": tname, "conf": float(cf), "box": box,
                                     "u": float(u), "v": float(v), "mode": mode,
                                     "nline": nline, "src": "far"})
            dets = dedup_dets(dets, radius=args.dedup)
            stats["det_sum"] += len(dets)
            if args.require_cross:
                # 低机位置信度低，草地纹理常被判成标志；只保留框内真的检出「线段交汇」的框。
                dets = [d for d in dets if d.get("mode") == "cross"]
            if args.angle_filter:
                # 再进一步：主方向必须成对且近似垂直，滤掉圆弧/球/杂物
                dets = [d for d in dets
                        if junction_ok(frame, d["box"], d.get("mode", ""))]

            # ---- 2) 像素 -> 载体系地面点（显示用；PnP 直接用像素）----
            uv_all = np.array([[d["u"], d["v"]] for d in dets], float) if dets \
                else np.zeros((0, 2), float)
            types_all = [d["type"] for d in dets]
            obs = []
            det_dbg = [None] * len(dets)   # 按 det 下标： (type, u, v, x_base, y_base)
            det_to_obs = {}                # dets 下标 -> obs 下标（跳过反投影失败的）
            if dets:
                pts, okm = pixels_to_ground(cam, ext, uv_all, max_range=MAX_RANGE_M)
                for i, d in enumerate(dets):
                    bx, by = float(pts[i, 0]), float(pts[i, 1])
                    det_dbg[i] = (d["type"], d["u"], d["v"], bx, by)
                    if not okm[i]:
                        continue
                    det_to_obs[i] = len(obs)
                    obs.append(ObservedMarker(type=d["type"], x=bx, y=by,
                                              confidence=d["conf"]))

            # ---- 3) 位姿解算 ----
            # 先用旧路线求出「哪个检测配哪个地图点」的关联初值（它只需大体正确），
            # 再由 PnP 在像素域以 4 点为最小解集精解位姿 —— 这一步不吃外参误差。
            obs_to_det = {v: k for k, v in det_to_obs.items()}
            rA = locate(obs, fm, params, prior=prior, x_sign=x_sign, y_sign=y_sign)
            used_pnp = False
            if args.pose_mode == "pnp":
                seed = []
                for mt_ in (rA.matches or []):
                    di = obs_to_det.get(int(mt_["obs_index"]))
                    if di is None:
                        continue
                    seed.append({"obs_index": di, "type": mt_["type"],
                                 "map_x": mt_["map_x"], "map_y": mt_["map_y"]})
                result = locate_pnp(uv_all, types_all, fm, cam, seed_matches=seed,
                                    params=pnp_params, x_sign=x_sign, y_sign=y_sign)
                used_pnp = True
                # 4 点定不出时退回 3 点：共面 3 点在「外参已知 + 落地平面」下
                # 其实可解（SE2 少一个冗余），只是没有正交校验、无法自证。
                # 现场有些方位就是只看得见 3 个角（实测球门区远端那角常出画面），
                # 与其给红字「认不出来」，不如把坐标交出来并明确标注可信度。
                if (not result.valid and args.allow_three_point
                        and rA.marker_count >= 3 and rA.x == rA.x):
                    # 3 点回退也必须放弃共线解：球门线 x=±3.80 上并排 6 个标志，
                    # 若三个匹配点全落在同一条线上，位姿沿该线可任意滑动，
                    # 实测同一相机不动却解出 y=+0.12 与 y=+3.08 两套完全不同的位姿。
                    _mp = np.array([[m["map_x"], m["map_y"]] for m in (rA.matches or [])], float)
                    if _mp.shape[0] >= 3 and _planarity_ratio(_mp) >= 0.18:
                        result = rA
                        result.status = "three_point"
                        used_pnp = False
            # 旧路线在「只看得见 3 个交点」的方位会给出等价解（ambiguous）；
            # --use-ambiguous 时带警告地采用它，否则这类方位永远没有坐标。
            usable = (result.valid
                      or result.status == "three_point"
                      or (args.pose_mode == "matcher"
                          and getattr(args, "use_ambiguous", False)
                          and result.status == "ambiguous"
                          and result.marker_count >= args.min_markers
                          and result.x == result.x))
            if result.valid:
                stats["valid"] += 1
            elif result.status == "three_point":
                stats["three_point"] += 1
            elif result.status == "ambiguous":
                stats["ambiguous"] += 1
            elif result.status == "insufficient_markers":
                stats["insufficient"] += 1
            if usable:
                prior = (result.x, result.y, np.radians(result.yaw_deg), 0.9, 28.0)

            # 内点：真正参与定位的检测 -> 对应的地图点坐标。索引一律用 dets 下标，
            # 编号与左下角列表、右侧俯视图上的绿色圆点一一对应。
            matched = []
            for mt_ in (result.matches or []):
                oi = int(mt_["obs_index"])
                if used_pnp:
                    di = oi
                    ot = str(mt_.get("obs_type", mt_["type"]))
                    dd = float(mt_.get("residual_m", mt_.get("residual_px", 0.0)))
                else:
                    di = obs_to_det.get(oi)
                    if di is None:
                        continue
                    ot = str(mt_["type"])
                    dd = float(mt_["residual_m"])
                matched.append((di, ot, float(mt_["map_x"]), float(mt_["map_y"]), dd))

            # ---- 4) 左半屏：检测结果 ----
            canvas = np.zeros((H, W, 3), np.uint8)
            lv = cv2.resize(frame, (W // 2, H // 2), interpolation=cv2.INTER_AREA)
            canvas[0:H // 2, 0:W // 2] = lv
            sc = (W // 2) / float(got_w or W)
            hit_det = {int(di) for di, *_ in matched}
            stats["matched_sum"] += len(hit_det)
            n_drawn = 0
            for i, d in enumerate(dets):
                used = i in hit_det
                if used:
                    n_drawn += 1
                x1, y1, x2, y2 = d["box"]
                p1 = (int(x1 * sc), int(y1 * sc))
                p2 = (int(x2 * sc), int(y2 * sc))
                if used:
                    col = TYPE_COLOR.get(d["type"], (200, 200, 200))
                    cv2.rectangle(canvas, p1, p2, col, 2)
                    k = [j for j, m in enumerate(matched) if m[0] == i]
                    tag = f"#{k[0] + 1} {SHAPE_NAME.get(d['type'], '?')}" if k else SHAPE_NAME.get(d['type'], '?')
                    cv2.putText(canvas, tag, (p1[0], max(p1[1] - 4, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
                    cv2.circle(canvas, (int(d["u"] * sc), int(d["v"] * sc)), 3, (0, 0, 255), -1)
                else:
                    # 检测到了但没配上地图：照样画出来，用暗灰细框区分
                    cv2.rectangle(canvas, p1, p2, (120, 120, 120), 1)
                    cv2.putText(canvas, SHAPE_NAME.get(d["type"], "?") + "?",
                                (p1[0], max(p1[1] - 3, 11)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (130, 130, 130), 1)
            cv2.putText(canvas, f"{backend} idx={args.index} {got_w}x{got_h}  det={len(dets)}  "
                                f"used={n_drawn}",
                        (10, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            # ---- 5) 右半屏：地图 + 位姿 ----
            view.draw(canvas, fm, extent, result, det_dbg, len(obs), matched)

            # ---- 6) 下半屏：数值面板 ----
            panel_y = H // 2 + 8
            cv2.putText(canvas, f"FPS {fps_now:5.1f}   infer {float(np.mean(infer_win)):5.1f} ms"
                                f"   frame {frame_idx}",
                        (10, panel_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1)
            if usable:
                residual_m = float(getattr(result, "residual_m",
                                           getattr(result, "residual", float("nan"))))
                txt = (f"x_camera_map {result.x:+.3f} m   y_camera_map {result.y:+.3f} m   "
                       f"yaw_camera_map {result.yaw_deg:+.2f} deg")
                cv2.putText(canvas, txt, (10, panel_y + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (0, 255, 0) if result.valid else (0, 200, 255), 1)
                txt2 = (f"carrier 同值   marker_count {result.marker_count}   "
                        f"residual {residual_m:.3f} m   status {result.status}")
                cv2.putText(canvas, txt2, (10, panel_y + 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 220, 255), 1)
                if used_pnp:
                    cv2.putText(canvas,
                                f"PnP 4-point   h {result.height_m:.3f} m   "
                                f"pitch {result.pitch_deg:+.2f} deg   "
                                f"rms {result.rms_px:.2f} px",
                                (10, panel_y + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (150, 255, 200), 1)
                elif result.status == "three_point":
                    cv2.putText(canvas,
                                f"WARNING 3-point pose (no redundancy, cannot self-check)   "
                                f"markers {result.marker_count}",
                                (10, panel_y + 84), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 140, 255), 1)
                if not result.valid and result.status != "three_point":
                    cv2.putText(canvas, "WARNING ambiguous pose used (2 equivalent solutions)",
                                (10, panel_y + 106), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 120, 255), 1)
            else:
                hint = {
                    "degenerate_geometry":
                        "可见交点几乎共线 -> 左右转动相机，让视野含不在同一线上的标志",
                    "not_converged":
                        "4 点无几何合理解 -> 让视野含更多标志（或向后退一点）",
                    "insufficient_markers":
                        f"可用交点不足 {params.min_markers} 个 -> 让视野含更多标志（或向后退一点）",
                }.get(result.status, "让视野含至少 4 个不共线的交点")
                cv2.putText(canvas,
                            f"status {result.status}   det {len(dets)}  可反投影 {len(obs)}"
                            f"  需 >= {params.min_markers} 个不共线交点",
                            (10, panel_y + 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 1)
                cv2.putText(canvas, hint, (10, panel_y + 54),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 140, 255), 1)
            # 左下角：只列「确认匹配到地图」的内点，编号与右侧俯视图上的绿点一致
            base_y = panel_y + 114
            if matched:
                cv2.putText(canvas,
                            f"matched markers ({len(matched)})   px  ->  map(m)",
                            (10, base_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (120, 255, 150), 1)
                for k, (di, mt, mx, my, dd) in enumerate(matched[:9]):
                    dbg = det_dbg[di] if 0 <= di < len(det_dbg) else None
                    t, u, v = (dbg[0], dbg[1], dbg[2]) if dbg else (mt, 0, 0)
                    label = SHAPE_NAME.get(t, "?")
                    if t != mt:            # 跨类别配对（T 观测配上了 X 地图点）
                        label += "->" + SHAPE_NAME.get(mt, "?")
                    cv2.putText(canvas,
                                f"  #{k + 1} {label}  "
                                f"px({int(u):4d},{int(v):3d})  map({mx:+.2f},{my:+.2f})  "
                                f"d={dd:.2f}",
                                (10, base_y + 21 * (k + 1)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 255, 170), 1)
            else:
                cv2.putText(canvas,
                            f"matched markers (0)   observed {len(obs)} but none matched "
                            f"| status {result.status}",
                            (10, base_y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 160, 255), 1)
            cv2.putText(canvas, f"map {Path(args.field).name}{' + site' if not args.no_site else ''}"
                                f" | +X=goal +Y=left | K {cam.K[0,0]:.0f}/{cam.K[1,1]:.0f}"
                                f" | ext h={ext.t[2]:.2f} p={pitch_show:+.1f} r={roll_show:+.1f}",
                        (10, H - 34), cv2.FONT_HERSHEY_SIMPLEX, 0.42, (190, 190, 190), 1)
            cv2.putText(canvas, "L=triangle T=square X=cross P=hollow(site:no visible mark) "
                                "red dot=refined junction",
                        (10, H - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (170, 170, 170), 1)

            # 终端结构化输出（细则要求的字段都在）
            if args.print_every > 0 and frame_idx % args.print_every == 0:
                rec = {"frame": frame_idx, "ts": datetime.now().strftime("%H:%M:%S.%f")[:-3],
                       "fps": round(fps_now, 1), "infer_ms": round(infer_ms, 1),
                       "det": len(dets), "ground": len(obs),
                       **result.as_dict(force=usable)}
                if result.status == "three_point":
                    # 3 点无冗余、无法自证，即使给了坐标也不能标成 valid
                    rec["valid"] = False
                print(json.dumps(rec, ensure_ascii=False))

            if paused:
                cv2.putText(canvas, "PAUSED", (W - 150, panel_y + 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            if writer is not None:
                writer.write(canvas)
            if not args.no_show:
                cv2.imshow("task2 realtime localization", canvas)

        if args.no_show:
            if args.max_frames and frame_idx >= args.max_frames:
                break
            continue

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            paused = not paused
        if key == ord("s"):
            p = out_dir / f"field_{datetime.now():%H%M%S}.png"
            cv2.imwrite(str(p), canvas)
            print(f"已存图：{rel(p)}")
        if args.max_frames and frame_idx >= args.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
        print("录制已保存")
    if not args.no_show:
        cv2.destroyAllWindows()

    n = max(stats["frames"], 1)
    print("-" * 84)
    print(f"共 {stats['frames']} 帧｜有效定位 {stats['valid']}（{stats['valid'] / n:.1%}）"
          f"｜3点 {stats['three_point']}｜ambiguous {stats['ambiguous']}｜标志不足 {stats['insufficient']}")
    print(f"平均每帧检出 {stats['det_sum'] / n:.1f} 个框，其中参与定位 {stats['matched_sum'] / n:.1f} 个")
    print("=" * 84)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


