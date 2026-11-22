#!/usr/bin/env python3
"""实时演示窗口（只有壳子，不含任何定位算法）。

界面
    左栏：相机 / 视频画面
    右栏：场地俯视图（边线、禁区、中圈、标志点、原点与 X/Y 轴）
    顶栏：地图版本 / 内参 / 输入源
    底栏：位姿与状态（算法接进来之前显示占位符）

按键
    q 退出   空格 暂停   s 截图   r 录制   t 轨迹开关

接入算法的位置
    只改 analyze() 一个函数。它应当返回：
        {"dets": [ {"class_name": "Lcross", "confidence": 0.9,
                    "pixel": (u, v), "box": (x1, y1, x2, y2)} ... ],
         "pose": {"x": 1.2, "y": -0.8, "yaw": 15.0, "pitch": -13.0,
                  "height": 0.75, "status": "TRACKING", "rms": 1.2,
                  "n_pairs": 4} 或 None,
         "trail": [(x, y), ...] 或 None}
    现在的实现直接返回空，界面照常显示。

用法
    python3 tools/demo_realtime.py --camera tcp:5599
    python3 tools/demo_realtime.py --video "/mnt/c/.../3053.avi" --height 0.90
    python3 tools/demo_realtime.py --video 後.avi --headless --max-frames 30
"""

import argparse
import datetime
import itertools
import math
import os
import sys
import time

import cv2
import numpy as np
import yaml


# 单应初始化（检测 ↔ 地图对应）
from homography_init import init_by_homography, pose_from_homography  # noqa: E402

# 视觉定位包（场地模型）
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from localization.field_map import FieldMap  # noqa: E402

# 标志类别配色（左栏检测框与右栏标志点保持一致，BGR）
MARKER_COLORS = {"L": (80, 200, 255),   # 橙黄
                 "T": (120, 255, 120),  # 绿
                 "X": (255, 160, 120),  # 蓝
                 "P": (200, 200, 200)}  # 灰

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))   # xjx/
FIELD_YAML = os.path.join(ROOT, "config", "field_geometry.yaml")
CFG_YAML = os.path.join(ROOT, "config", "localization.yaml")
WIN = "demo_realtime"

# 左栏画面宽度，右栏宽度固定 700（与原版一致，保证画布尺寸不变）
LEFT_W = 1280
RIGHT_W = 700
FOOT_LINES = 6
LINE_H = 22


# ----------------------------------------------------------------- 场地俯视图
class FieldPanel:
    def __init__(self, field, margin=40):
        self.field = field
        self.margin = margin
        L, W = field["length"], field["width"]
        avail_w = RIGHT_W - 2 * margin
        avail_h = LEFT_W * 720 // LEFT_W - 2 * margin - 40
        self.scale = min(avail_w / L, avail_h / W)
        self.ox = margin + avail_w / 2.0
        self.oy = margin + 30 + avail_h / 2.0

    def w2p(self, x, y):
        return (int(round(self.ox + x * self.scale)),
                int(round(self.oy - y * self.scale)))

    def p2w(self, px, py):
        """面板像素 → 场地坐标。"""
        return ((px - self.ox) / self.scale, (self.oy - py) / self.scale)

    def nearest_marker(self, px, py, radius_px=28.0, exclude=()):
        """面板上鼠标位置最近的标志点。"""
        best = None
        for m in self.field["markers"]:
            if m["type"] == "P" or m.get("id") in exclude:
                continue
            mx, my = self.w2p(m["x"], m["y"])
            d = math.hypot(mx - px, my - py)
            if d <= radius_px and (best is None or d < best[3]):
                best = (m["type"], float(m["x"]), float(m["y"]), d,
                        m.get("id", ""))
        return best

    def draw(self, h, pose=None, trail=None, trail_on=True):
        img = np.full((h, RIGHT_W, 3), 18, np.uint8)
        cv2.putText(img, "top view  (m)", (10, 22), cv2.FONT_HERSHEY_SIMPLEX,
                    0.55, (200, 200, 200), 1)
        f = self.field
        L, W = f["length"], f["width"]
        pal, paw = f["penalty_area_length"], f["penalty_area_width"] / 2.0
        gal, gaw = f["goal_area_length"], f["goal_area_width"] / 2.0
        col = (200, 200, 200)
        segs = [((-L / 2, -W / 2), (L / 2, -W / 2)), ((-L / 2, W / 2), (L / 2, W / 2)),
                ((-L / 2, -W / 2), (-L / 2, W / 2)), ((L / 2, -W / 2), (L / 2, W / 2)),
                ((0, -W / 2), (0, W / 2))]
        for s in (1, -1):
            xp, xg = s * (L / 2 - pal), s * (L / 2 - gal)
            segs += [((xp, -paw), (xp, paw)), ((xp, -paw), (s * L / 2, -paw)),
                     ((xp, paw), (s * L / 2, paw)),
                     ((xg, -gaw), (xg, gaw)), ((xg, -gaw), (s * L / 2, -gaw)),
                     ((xg, gaw), (s * L / 2, gaw))]
        for a, b in segs:
            cv2.line(img, self.w2p(*a), self.w2p(*b), col, 1, cv2.LINE_AA)
        r = f.get("circle_radius", 0.75)
        pts = [self.w2p(r * np.cos(t), r * np.sin(t))
               for t in np.linspace(0, 2 * np.pi, 48)]
        cv2.polylines(img, [np.array(pts, np.int32)], False, col, 1, cv2.LINE_AA)

        # 原点与坐标轴
        o = self.w2p(0, 0)
        cv2.circle(img, o, 3, (255, 255, 255), -1)
        cv2.putText(img, "origin", (o[0] - 18, o[1] + 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (180, 180, 180), 1)
        cv2.arrowedLine(img, o, self.w2p(0.8, 0), (80, 80, 255), 1, tipLength=0.15)
        cv2.putText(img, "+X", (o[0] + int(0.9 * self.scale), o[1] + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 255), 1)
        cv2.arrowedLine(img, o, self.w2p(0, 0.8), (120, 255, 120), 1, tipLength=0.15)
        cv2.putText(img, "+Y", (o[0] - 6, o[1] - int(0.9 * self.scale)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (120, 255, 120), 1)

        # 标志点
        for m in self.field["markers"]:
            if m["type"] == "P":
                continue
            p = self.w2p(m["x"], m["y"])
            c = {"L": (80, 200, 255), "T": (120, 255, 120), "X": (255, 160, 120)}.get(m["type"],
                                                                                   (200, 200, 200))
            cv2.circle(img, p, 3, c, -1)
            cv2.putText(img, m["id"], (p[0] + 4, p[1] - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.32, c, 1)

        # 轨迹
        if trail_on and trail:
            pts = [self.w2p(x, y) for x, y in trail[-600:]]
            if len(pts) > 1:
                cv2.polylines(img, [np.array(pts, np.int32)], False,
                              (60, 180, 60), 1, cv2.LINE_AA)

        # 相机
        if pose is not None:
            p = self.w2p(pose["x"], pose["y"])
            th = np.radians(pose["yaw"])
            d = np.array([np.cos(th), -np.sin(th)]) * 34
            for sgn in (-1, 1):
                a = th + sgn * np.radians(140)
                q = p + np.array([np.cos(a), -np.sin(a)]) * 14
                cv2.line(img, p, (int(q[0]), int(q[1])), (0, 255, 0), 2, cv2.LINE_AA)
            cv2.circle(img, p, 4, (0, 255, 0), -1)
            cv2.arrowedLine(img, p, (int(p[0] + d[0]), int(p[1] + d[1])),
                            (0, 255, 0), 2, tipLength=0.25)
        return img


# ----------------------------------------------------------------- 文字栏
def banner(width, lines, color=(0, 255, 0)):
    img = np.full((len(lines) * LINE_H + 10, width, 3), 30, np.uint8)
    for i, s in enumerate(lines):
        cv2.putText(img, s, (8, 20 + i * LINE_H), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, color, 1)
    return img


# ------------------------------------------------------------ 算法：检测
_CFG = {}
_MODELS = {}


def cfg():
    """读 config/localization.yaml（带缓存）。"""
    if not _CFG:
        try:
            _CFG.update(yaml.safe_load(open(CFG_YAML, encoding="utf-8")) or {})
        except Exception as e:
            print("读配置失败：", e)
    return _CFG


def load_models():
    """懒加载标志检测模型（支持 marker_models 多模型）。"""
    if "list" in _MODELS:
        return _MODELS["list"]
    from ultralytics import YOLO
    paths = cfg().get("paths", {})
    lst = paths.get("marker_models") or []
    if not lst:
        lst = [paths.get("marker_model")]
    if _MODELS.get("override"):
        lst = [_MODELS["override"]]
    models = []
    for p in lst:
        if not p:
            continue
        full = p if os.path.isabs(p) else os.path.join(ROOT, p)
        if not os.path.isfile(full):
            print("找不到模型：", full)
            print("  权重未随仓库提交，请按 README「权重获取方式」下载后：")
            print("    python src/interface/demo_realtime.py --video X --model /path/best.pt")
            continue
        models.append(YOLO(full))
        print("已加载模型：", os.path.relpath(full, ROOT))
    _MODELS["list"] = models
    return models


def refine_point(frame, box, g):
    """检测框 → 交点像素。

    框短边 < min_refine_size 时直接用框中心（小框里白带宽 8~15 px，
    细化反而更差）；否则用白度掩膜 + 距离变换脊线 + Hough 求交，
    并要求交点靠近框中心。
    """
    x1, y1, x2, y2 = box
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    bw, bh = x2 - x1, y2 - y1
    if min(bw, bh) < g.get("min_refine_size", 80):
        return (cx, cy), "center"

    h, w = frame.shape[:2]
    ex = g.get("expand", 1.5)
    gx1 = int(max(0, cx - bw * ex / 2)); gx2 = int(min(w - 1, cx + bw * ex / 2))
    gy1 = int(max(0, cy - bh * ex / 2)); gy2 = int(min(h - 1, cy + bh * ex / 2))
    if gx2 - gx1 < 20 or gy2 - gy1 < 20:
        return (cx, cy), "center"

    crop = frame[gy1:gy2, gx1:gx2]
    whit = crop.min(axis=2)
    thr = max(180, int(np.percentile(whit, g.get("ridge_percentile", 85))))
    mask = np.where(whit >= thr, 1, 0).astype(np.uint8)
    if mask.sum() < 30:
        return (cx, cy), "center"
    dt = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    md = float(dt.max())
    if md < 1.5:
        return (cx, cy), "center"
    ridge = np.where(dt >= max(1.0, g.get("ridge_ratio", 0.55) * md), 255, 0).astype(np.uint8)
    ridge = cv2.morphologyEx(ridge, cv2.MORPH_CLOSE,
                             cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))
    ch, cw = mask.shape
    lines = cv2.HoughLinesP(ridge, 1, np.pi / 180, 12,
                            max(10, int(0.22 * min(ch, cw))), 6)
    if lines is None or len(lines) < 2:
        return (cx, cy), "center"

    tol_x, tol_y = 0.45 * cw, 0.45 * ch
    best = None
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            xa, ya, xb, yb = lines[i][0]
            xc, yc, xd, yd = lines[j][0]
            da = math.atan2(yb - ya, xb - xa)
            db = math.atan2(yd - yc, xd - xc)
            diff = abs((da - db + math.pi / 2) % math.pi - math.pi / 2)
            if math.degrees(diff) < g.get("intersection_min_angle", 25.0):
                continue
            A = np.array([[xb - xa, -(xd - xc)], [yb - ya, -(yd - yc)]], float)
            if abs(np.linalg.det(A)) < 1e-6:
                continue
            t = np.linalg.solve(A, np.array([xc - xa, yc - ya], float))
            px, py = xa + t[0] * (xb - xa), ya + t[0] * (yb - ya)
            if abs(px - cw / 2.0) > tol_x or abs(py - ch / 2.0) > tol_y:
                continue
            score = -math.hypot(px - cw / 2.0, py - ch / 2.0)
            if best is None or score > best[0]:
                best = (score, px + gx1, py + gy1)
    if best is None:
        return (cx, cy), "center"
    return (best[1], best[2]), "intersection"


def merge_dets(dets, radius):
    """同类且中心距离小于 radius 的重复检测合并（多模型融合用）。"""
    if radius <= 0:
        return dets
    out = []
    for d in sorted(dets, key=lambda x: -x["confidence"]):
        dup = None
        for o in out:
            if o["class_name"] != d["class_name"]:
                continue
            if math.hypot(o["pixel"][0] - d["pixel"][0],
                          o["pixel"][1] - d["pixel"][1]) < radius:
                dup = o
                break
        if dup is None:
            out.append(d)
    return out


# ------------------------------------------------------------ 算法：位姿
def _load_intr():
    cal = yaml.safe_load(open(os.path.join(ROOT, "calibration",
                                           "camera_intrinsics.yaml"), encoding="utf-8"))
    K = np.array(cal["camera_matrix"], float)
    dist = np.array(cal["dist_coeffs"], float).reshape(-1, 1)
    return K, dist


def rot_from_ypr(yaw, pitch, roll):
    """(yaw, pitch, roll) 度 → R（相机→场地）。z 轴为光轴，y 轴朝图像下方。"""
    y, p, r = np.radians([yaw, pitch, roll])
    z = np.array([math.cos(p) * math.cos(y), math.cos(p) * math.sin(y), math.sin(p)])
    x0 = np.cross(z, [0.0, 0.0, 1.0])
    n = np.linalg.norm(x0)
    x0 = x0 / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
    R0 = np.column_stack([x0, np.cross(z, x0), z])
    c, s = math.cos(r), math.sin(r)
    return R0 @ np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def ypr_from_R(R):
    z = R[:, 2]
    yaw = math.degrees(math.atan2(z[1], z[0]))
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, z[2]))))
    R0 = rot_from_ypr(yaw, pitch, 0.0)
    xa = R[:, 0]
    roll = math.degrees(math.atan2(float(xa @ R0[:, 1]), float(xa @ R0[:, 0])))
    return yaw, pitch, roll


def algo_get(state, field):
    """算法状态：初始化一次，之后每帧复用。"""
    a = state.get("algo")
    if a is not None:
        return a
    K, dist = _load_intr()
    c = cfg()
    a = {
        "K": K, "dist": dist,
        "K_inv": np.linalg.inv(K),
        "dims": {"L": float(field["length"]), "W": float(field["width"])},
        "markers": [(m["type"], float(m["x"]), float(m["y"]), m.get("id", ""))
                    for m in field["markers"] if m["type"] != "P"],
        "R": None, "C": None, "ypr": None,
        "prev": None, "fail": 0, "status": "INIT",
        "rms": None, "n_pairs": 0, "trail": [],
        "picks": [], "msg": "click 4 map markers on the right to init",
        "cfg": c,
        "half": None,      # 学会的半场限制（点选成功后自动记住）
    }
    state["algo"] = a
    return a


def _pnp(a, obj, img, check=True):
    """解一组对应关系，返回物理成立的候选（含两解）。"""
    pcfg = a["cfg"].get("pose", {})
    hmin = pcfg.get("height_min", 0.55)
    hmax = pcfg.get("height_max", 1.35)
    rmax = pcfg.get("rms_max", 12.0)
    K, dist = a["K"], a["dist"]
    try:
        cnt, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    except cv2.error:
        return []
    half_l = a["dims"]["L"] / 2.0 + 0.3
    half_w = a["dims"]["W"] / 2.0 + 0.3
    out = []
    for i in range(cnt):
        R, _ = cv2.Rodrigues(rvecs[i])
        C = -R.T @ tvecs[i].reshape(3)
        yaw, pitch, roll = ypr_from_R(R.T)
        if check:
            if not (hmin <= C[2] <= hmax):
                continue
            if abs(C[0]) > half_l or abs(C[1]) > half_w:
                continue
            if abs(roll) > 30.0 or not (-35.0 <= pitch <= 0.0):
                continue
        proj, _ = cv2.projectPoints(obj, rvecs[i], tvecs[i], K, dist)
        rms = float(np.sqrt(np.mean(np.sum((proj.reshape(-1, 2) - img) ** 2, axis=1))))
        if rms > rmax:
            continue
        out.append({"R": R.T.copy(), "C": C.copy(), "rms": rms,
                    "yaw": yaw, "pitch": pitch, "roll": roll})
    return out


def _learn_half(a, picks):
    """从人工点选的标志推断半场限制，供后续自动枚举使用。"""
    if not picks:
        return None
    xs = [p[1] for p in picks]
    ys = [p[2] for p in picks]
    if all(v < 0 for v in xs):
        return "x<0"
    if all(v > 0 for v in xs):
        return "x>0"
    if all(v < 0 for v in ys):
        return "y<0"
    if all(v > 0 for v in ys):
        return "y>0"
    return None


def algo_init_from_picks(a, dets):
    """人在右栏点了地图标志；把当前检测框按类别分配过去，解 PnP。

    返回 True 表示初始化成功。
    """
    picks = a["picks"]
    if len(picks) < 4:
        a["msg"] = "need %d more picks" % (4 - len(picks))
        return False
    by_type = {}
    for t, x, y, _ in a["markers"]:
        by_type.setdefault(t, []).append((x, y))
    d_by = {}
    for d in dets:
        if d.get("class_name"):
            d_by.setdefault(d["class_name"][0].upper(), []).append(d)
    p_by = {}
    for pk in picks:
        p_by.setdefault(pk[0], []).append((pk[1], pk[2]))
    for t in p_by:
        if len(d_by.get(t, [])) < len(p_by[t]):
            a["msg"] = "not enough %s detections (need %d)" % (t, len(p_by[t]))
            return False
    keys = list(p_by)
    best = None
    for assign in itertools.product(*[itertools.permutations(range(len(d_by[t])), len(p_by[t]))
                                      for t in keys]):
        obj, img = [], []
        for t, idxs in zip(keys, assign):
            for (mx, my), di in zip(p_by[t], idxs):
                px = d_by[t][di]["pixel"]
                obj.append([mx, my, 0.0])
                img.append([px[0], px[1]])
        for cand in _pnp(a, np.array(obj, float), np.array(img, float)):
            if best is None or cand["rms"] < best["rms"]:
                best = cand
    if best is None:
        a["msg"] = "init failed, try other markers (c clear)"
        a["picks"] = []
        return False
    a["R"], a["C"] = best["R"], best["C"]
    a["ypr"] = [best["yaw"], best["pitch"], best["roll"]]
    a["rms"] = best["rms"]
    a["status"] = "TRACKING"
    a["fail"] = 0
    a["trail"] = [(float(best["C"][0]), float(best["C"][1]))]
    a["msg"] = "INIT OK  rms %.2f px" % best["rms"]
    h = _learn_half(a, a["picks"])
    if h:
        a["half"] = h
        print("记住半场限制：%s（后续自动枚举只在此半场搜索）" % h)
    print("初始化成功：C=(%.2f, %.2f, %.2f) yaw %.2f pitch %.2f RMS %.2f px"
          % (best["C"][0], best["C"][1], best["C"][2],
             best["yaw"], best["pitch"], best["rms"]))
    return True


def _ground(a, u, v):
    """像素 → 地面点（相机系用当前 R、C）。"""
    d = a["K_inv"] @ np.array([u, v, 1.0])
    df = a["R"] @ d
    if df[2] > -1e-6:
        return None
    lam = -a["C"][2] / df[2]
    if not (0.5 < lam < 15.0):
        return None
    return a["C"] + lam * df


def _project(a, P, R):
    rvec, _ = cv2.Rodrigues(R.T)
    uv, _ = cv2.projectPoints(P.reshape(-1, 3), rvec, -R.T @ a["C"],
                              a["K"], a["dist"])
    return uv.reshape(-1, 2)


def algo_auto_match(a, dets, max_combos=8000, time_budget=1.5):
    """没有人工点选时，自动枚举求初始位姿。

    规则（按实际需求）
        · X 类不参与（检测和地图都不用），只用 L / T
        · 检测点超过 4 个时，枚举「任选 4 个」的所有子集
        · 每个子集内按类别分配地图标志，解 PnP
        · 物理约束过滤，取残差最小的解；解出来就用它

    三重限制避免卡界面
        · geometry.half_restrict 限定半场（"x<0" 等）
        · 组合数上限 max_combos
        · 时间预算 time_budget 秒
    """
    g = a["cfg"].get("geometry", {})
    half = (g.get("half_restrict") or a.get("half") or "").strip()
    ai = a["cfg"].get("auto_init", {})
    min_points = int(ai.get("min_dets", 4))

    # X 类无效：地图标志里也不用
    marks = [m for m in a["markers"] if m[0] in ("L", "T")]
    if half in ("x<0", "x>0", "y<0", "y>0"):
        axis = 1 if half[0] == "x" else 2      # markers 是 (type, x, y, id)
        want_neg = half[1] == "<"
        marks = [m for m in marks if (m[axis] < 0 if want_neg else m[axis] > 0)]

    m_by = {}
    for t, x, y, mid in marks:
        m_by.setdefault(t, []).append((x, y, mid))

    use_dets = [d for d in dets if d.get("class_name")
                and d["class_name"][0].upper() in ("L", "T")]
    if len(use_dets) < min_points:
        a["msg"] = "need >=%d L/T detections (%d now)" % (min_points, len(use_dets))
        return False

    d_by_all = {}
    for d in use_dets:
        d_by_all.setdefault(d["class_name"][0].upper(), []).append(d)

    t0 = time.perf_counter()
    best = None
    tried = 0
    subsets = list(itertools.combinations(range(len(use_dets)), 4))
    for sub in subsets:
        group = [use_dets[i] for i in sub]
        d_by = {}
        for d in group:
            d_by.setdefault(d["class_name"][0].upper(), []).append(d)
        if any(t not in m_by or len(d_by[t]) > len(m_by[t]) for t in d_by):
            continue
        keys = list(d_by)
        ranges = [itertools.permutations(range(len(m_by[t])), len(d_by[t]))
                  for t in keys]
        for assign in itertools.product(*ranges):
            obj, img = [], []
            for t, idxs in zip(keys, assign):
                for d, mi in zip(d_by[t], idxs):
                    mx, my, _ = m_by[t][mi]
                    px = d["pixel"]
                    obj.append([mx, my, 0.0])
                    img.append([px[0], px[1]])
            tried += 1
            for cand in _pnp(a, np.array(obj, float), np.array(img, float)):
                if best is None or cand["rms"] < best["rms"]:
                    best = cand
            if tried >= max_combos or (time.perf_counter() - t0) > time_budget:
                break
        if tried >= max_combos or (time.perf_counter() - t0) > time_budget:
            break

    dt = time.perf_counter() - t0
    if best is None:
        a["msg"] = "auto failed (%d combos, %.1fs)" % (tried, dt)
        return False
    a["R"], a["C"] = best["R"], best["C"]
    a["ypr"] = [best["yaw"], best["pitch"], best["roll"]]
    a["rms"] = best["rms"]
    a["status"] = "TRACKING"
    a["fail"] = 0
    a["n_pairs"] = 4
    a["trail"] = [(float(best["C"][0]), float(best["C"][1]))]
    a["msg"] = "AUTO OK rms %.2f (%d combos)" % (best["rms"], tried)
    print("自动初始化：C=(%.2f, %.2f, %.2f) yaw %.2f pitch %.2f RMS %.2f px"
          "（%d 组 / %.2fs）"
          % (best["C"][0], best["C"][1], best["C"][2],
             best["yaw"], best["pitch"], best["rms"], tried, dt))
    return True

def algo_track(a, dets):
    """增量跟踪：帧间配对 → 地面点 → 只解旋转（C 固定）。"""
    from scipy.optimize import least_squares
    tcfg = a["cfg"].get("tracking", {})
    radius = tcfg.get("match_radius", 90.0)
    min_pairs = int(tcfg.get("min_pairs", 2))
    max_yaw = tcfg.get("max_yaw_step", 12.0)
    max_pitch = tcfg.get("max_pitch_step", 6.0)
    max_lost = int(tcfg.get("max_lost", 5))

    prev = a["prev"]
    if not prev:
        a["prev"] = dets
        return
    cand = []
    for i, pa in enumerate(prev):
        for j, pb in enumerate(dets):
            if pa.get("class_name") != pb.get("class_name"):
                continue
            e = math.hypot(pa["pixel"][0] - pb["pixel"][0],
                           pa["pixel"][1] - pb["pixel"][1])
            if e <= radius:
                cand.append((e, i, j))
    cand.sort()
    used_a, used_b, pairs = set(), set(), []
    for e, i, j in cand:
        if i in used_a or j in used_b:
            continue
        used_a.add(i); used_b.add(j)
        pairs.append((prev[i], dets[j]))

    G, P1 = [], []
    for pa, pb in pairs:
        g = _ground(a, *pa["pixel"])
        if g is None:
            continue
        G.append(g)
        P1.append(pb["pixel"])
    if len(G) < min_pairs:
        a["fail"] += 1
        a["status"] = "PREDICTING" if a["fail"] <= max_lost else "LOST"
        a["prev"] = dets
        return

    G = np.array(G, float)
    P1 = np.array(P1, float)

    def resid(p):
        return (_project(a, G, rot_from_ypr(*p)) - P1).ravel()

    sol = least_squares(resid, np.array(a["ypr"], float), method="lm", max_nfev=200)
    yaw, pitch, roll = sol.x
    dyaw = (yaw - a["ypr"][0] + 180) % 360 - 180
    dpitch = pitch - a["ypr"][1]
    if abs(dyaw) > max_yaw or abs(dpitch) > max_pitch:
        a["fail"] += 1
        a["status"] = "PREDICTING" if a["fail"] <= max_lost else "LOST"
        a["prev"] = dets
        return
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    a["ypr"] = [float(yaw), float(pitch), float(roll)]
    a["R"] = rot_from_ypr(*a["ypr"])
    a["rms"] = rms
    a["n_pairs"] = len(G)
    a["fail"] = 0
    a["status"] = "TRACKING"
    a["trail"].append((float(a["C"][0]), float(a["C"][1])))
    a["prev"] = dets


def algo_pose(a):
    if a["R"] is None or a["C"] is None:
        return None
    return {"x": float(a["C"][0]), "y": float(a["C"][1]),
            "yaw": float(a["ypr"][0]), "pitch": float(a["ypr"][1]),
            "height": float(a["C"][2]), "status": a["status"],
            "rms": float(a["rms"] or 0.0), "n_pairs": int(a["n_pairs"]),
            "msg": a["msg"]}


def algo_reset(a, keep_msg=True):
    a["R"] = None
    a["C"] = None
    a["ypr"] = None
    a["prev"] = None
    a["fail"] = 0
    a["status"] = "INIT"
    a["rms"] = None
    a["n_pairs"] = 0
    a["trail"] = []
    a["picks"] = []
    a["msg"] = "click 4 map markers on the right to init"


# ------------------------------------------------------------ 算法接口
def analyze(frame, state):
    """检测 L/T/X → 输出检测框与交点；有位姿时一并给出。

    返回契约（界面依赖）：
        {"dets": [{class_name, confidence, pixel, box}, ...],
         "pose": {x,y,yaw,pitch,height,status,rms,n_pairs} 或 None,
         "trail": [(x,y), ...] 或 None}
    """
    g = cfg().get("geometry", {})
    conf = g.get("marker_confidence", 0.25)
    imgsz = g.get("marker_imgsz", 512)
    dets = []
    for m in load_models():
        r = m.predict(frame, imgsz=imgsz, conf=conf, verbose=False)[0]
        if r.boxes is None:
            continue
        names = m.names if isinstance(m.names, dict) else dict(enumerate(m.names))
        for box, cid, cf in zip(r.boxes.xyxy.cpu().numpy(),
                                r.boxes.cls.cpu().numpy().astype(int),
                                r.boxes.conf.cpu().numpy()):
            b = [float(v) for v in box]
            px, src = refine_point(frame, b, g)
            dets.append({"class_name": str(names[int(cid)]),
                         "confidence": float(cf),
                         "box": b,
                         "pixel": (float(px[0]), float(px[1])),
                         "point_source": src})
    dets = merge_dets(dets, g.get("merge_radius", 25.0))
    state["dets"] = dets
    # ---- 球检测（每 ball_every_n 帧跑一次，结果单独返回，不进 dets）----
    balls = []
    dm = cfg().get("demo", {})
    bp = cfg().get("paths", {}).get("ball_model")
    state["ball_n"] = state.get("ball_n", 0) + 1
    if bp and (state["ball_n"] % max(1, int(dm.get("ball_every_n", 2))) == 0):
        bfull = bp if os.path.isabs(bp) else os.path.join(ROOT, bp)
        if os.path.isfile(bfull):
            if "ball" not in _MODELS:
                from ultralytics import YOLO
                _MODELS["ball"] = YOLO(bfull)
                print("已加载球模型：", os.path.relpath(bfull, ROOT))
            rb = _MODELS["ball"].predict(
                frame, imgsz=int(dm.get("ball_imgsz", 640)),
                conf=float(dm.get("ball_confidence", 0.30)), verbose=False)[0]
            if rb.boxes is not None:
                for bb, cf in zip(rb.boxes.xyxy.cpu().numpy(),
                                  rb.boxes.conf.cpu().numpy()):
                    bx = [float(v) for v in bb]
                    balls.append({"box": bx, "confidence": float(cf),
                                  "center": ((bx[0] + bx[2]) / 2.0,
                                             (bx[1] + bx[3]) / 2.0)})
            state["balls"] = balls
        else:
            balls = state.get("balls", [])
    else:
        balls = state.get("balls", [])       # 中间帧复用上次结果

    field = state.get("field")
    if field is None:
        return {"dets": dets, "balls": balls, "pose": None, "trail": None}

    a = algo_get(state, field)
    if a["R"] is None:
        # ① 有人点了地图标志 → 按点选解算
        if a.get("picks"):
            algo_init_from_picks(a, dets)
        else:
            # ② 没人点 → 自动枚举（节流：每 auto_init.cooldown 帧试一次，避免卡界面）
            a["tries"] = a.get("tries", 0) + 1
            cooldown = int(a["cfg"].get("auto_init", {}).get("cooldown", 10))
            if len(dets) >= 4 and (a["tries"] - a.get("last_try", -10 ** 6)) >= cooldown:
                a["last_try"] = a["tries"]
                a["msg"] = "auto search..."
                algo_auto_match(a, dets)
    else:
        algo_track(a, dets)
        # 跟踪彻底丢失 → 自动回到搜索状态（不必按 n）
        if a["status"] == "LOST":
            print("跟踪丢失，重新自动搜索")
            a["R"] = None
            a["C"] = None
            a["ypr"] = None
            a["prev"] = None
            a["fail"] = 0
            a["msg"] = "lost -> auto search"
    pose = algo_pose(a)
    return {"dets": dets, "balls": balls, "pose": pose, "trail": a["trail"]}


def load_anchor_pairs(name=None):
    """从 config/marker_anchors.yaml 取一组「像素 ↔ 场地坐标」对应。"""
    p = os.path.join(ROOT, "config", "marker_anchors.yaml")
    if not os.path.isfile(p):
        return None
    d = yaml.safe_load(open(p, encoding="utf-8")) or {}
    items = d.get("anchors") or []
    if not items:
        return None
    item = items[0]
    if name:
        for it in items:
            if it.get("name") == name:
                item = it
                break
    return [{"pixel": tuple(q["pixel"]), "world": tuple(q["world"])}
            for q in item.get("pairs", [])]

# ----------------------------------------------------------------- 输入
def open_input(a):
    if a.video:
        path = a.video
        if not os.path.isabs(path) and not os.path.exists(path):
            path = os.path.join(HERE, path)
        cap = cv2.VideoCapture(path)
        return cap, os.path.basename(path)
    if a.camera.startswith("tcp:"):
        port = int(a.camera.split(":", 1)[1])
        try:
            from net_camera import NetCamera  # 可选模块，没有就直接失败
            return NetCamera(port), a.camera
        except Exception:
            pass
    cap = cv2.VideoCapture(a.camera)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return cap, a.camera


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", default="/dev/video0")
    ap.add_argument("--video", default=None)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--height-cam", type=float, default=0.75,
                    help="相机安装高度（m），将来算法用")
    ap.add_argument("--model", default=None,
                    help="标志模型权重（权重不入库，从网盘下载后指定）")
    ap.add_argument("--ball-model", default=None,
                    help="足球模型权重")
    ap.add_argument("--out", default=os.path.join(ROOT, "demo_out"))
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()

    if a.model:
        _MODELS["override"] = a.model
    if a.ball_model:
        cfg().setdefault("paths", {})["ball_model"] = a.ball_model
    _fm = FieldMap(FIELD_YAML)
    field = dict(_fm.dims)
    field["markers"] = ([{"type": t, "x": x, "y": y, "id": mid}
                         for t, x, y, mid in _fm.markers]
                        + ([{"type": "P", "x": _fm.penalty_point[0],
                            "y": _fm.penalty_point[1],
                            "id": _fm.penalty_point[2]}]
                           if _fm.penalty_point else []))
    print("地图：", _fm.summary())
    intr = {}
    try:
        cal = yaml.safe_load(open(os.path.join(ROOT, "calibration",
                                              "camera_intrinsics.yaml"), encoding="utf-8"))
        intr = {"fx": cal["camera_matrix"][0][0], "fy": cal["camera_matrix"][1][1],
                "cx": cal["camera_matrix"][0][2], "cy": cal["camera_matrix"][1][2]}
    except Exception:
        pass

    cap, src = open_input(a)
    if not cap.isOpened():
        print("打不开输入：", src)
        return 1
    print("输入：", src)

    panel = FieldPanel(field)

    def on_mouse(event, x, y, flags, param):
        """左键点右栏的地图标志 → 攒够 4 个做初始化。"""
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        try:
            a_ = state.get("algo")
            if a_ is not None and a_["R"] is not None:
                return          # 已定位，点选无效（按 n 重新标定）
            yy = y - state.get("head_h", 0)
            if x < state.get("left_w", LEFT_W) or yy < 0:
                return
            used = {p[3] for p in (a_ or {}).get("picks", [])}
            hit = panel.nearest_marker(x - state.get("left_w", LEFT_W), yy, exclude=used)
            if hit is None:
                return
            a2 = a_ or algo_get(state, field)
            a2["picks"].append((hit[0], hit[1], hit[2], hit[4]))
            a2["msg"] = "picked %d/4: %s"
            a2["msg"] = a2["msg"] % (len(a2["picks"]), hit[4])
            print("选点：%s (%.2f, %.2f)" % (hit[4], hit[1], hit[2]))
        except Exception as e:
            print("点选出错：", e)

    if not a.headless:
        cv2.namedWindow(WIN, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(WIN, on_mouse)
    try:
        K = np.array(cal["camera_matrix"], float)
        dist = np.array(cal["dist_coeffs"], float).reshape(-1, 1)
    except Exception:
        K, dist = np.eye(3), np.zeros((5, 1))
    try:
        K = np.array(cal["camera_matrix"], float)
        dist = np.array(cal["dist_coeffs"], float).reshape(-1, 1)
    except Exception:
        K, dist = np.eye(3), np.zeros((5, 1))
    state = {"height": a.height_cam, "field": field, "head_h": 0,
             "trail": [], "path": [], "pose": None, "dets": [],
             "markers": field.get("markers", []),
             "K": K, "dist": dist, "homography": None, "homo_tried": False}
    writer = None
    paused = False
    trail_on = True
    n = 0
    fps = 0.0
    t_prev = time.perf_counter()

    hdr = [
        "map field_c0.yaml | intr fx %.0f fy %.0f cx %.0f cy %.0f | h %.2f"
        % (intr.get("fx", 0), intr.get("fy", 0), intr.get("cx", 0), intr.get("cy", 0),
           a.height_cam),
        "src %s   |   algo: (none)" % src,
    ]

    os.makedirs(a.out, exist_ok=True)

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("取帧结束")
                break
            n += 1
            out = analyze(frame, state)
            pose = out.get("pose")

            now = time.perf_counter()
            dt = now - t_prev
            t_prev = now
            if dt > 1e-6:
                fps = 1.0 / dt if fps == 0 else 0.9 * fps + 0.1 / dt

            left = frame.copy()
            for d in out.get("dets", []):
                x1, y1, x2, y2 = [int(round(v)) for v in d.get("box", (0, 0, 0, 0))]
                cls = d.get("class_name") or ""
                # 颜色和右栏一致：L 橙黄 / T 绿 / X 蓝
                c = MARKER_COLORS.get(cls[:1].upper(), (200, 200, 200))
                cv2.rectangle(left, (x1, y1), (x2, y2), c, 2)
                u, v = [int(round(t)) for t in d.get("pixel", (0, 0))]
                cv2.drawMarker(left, (u, v), c, cv2.MARKER_CROSS, 12, 2)
                if cls:
                    cv2.putText(left, "%s %.2f" % (cls, d.get("confidence", 0)),
                                (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                                c, 1)
            # 球：红框 + 标签 + 圆心点（球不参与定位）
            for bb in out.get("balls", []):
                x1, y1, x2, y2 = [int(round(v)) for v in bb.get("box", (0, 0, 0, 0))]
                cv2.rectangle(left, (x1, y1), (x2, y2), (60, 60, 255), 2)
                cx_, cy_ = bb.get("center", ((x1 + x2) / 2.0, (y1 + y2) / 2.0))
                cv2.circle(left, (int(cx_), int(cy_)), 4, (60, 60, 255), -1)
                cv2.putText(left, "ball %.2f" % bb.get("confidence", 0.0),
                            (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                            (60, 60, 255), 1)
            cv2.putText(left, "%dx%d  %.1f fps" % (left.shape[1], left.shape[0], fps),
                        (10, left.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (220, 220, 220), 1)

            tr = out.get("trail")
            if tr:
                state["trail"] = tr
            elif pose:
                state["trail"].append((pose["x"], pose["y"]))
            right = panel.draw(left.shape[0], pose=pose, trail=state["trail"],
                               trail_on=trail_on)

            # 右栏选点反馈：已选标志画圆 + 序号，底部提示当前状态
            a_now = state.get("algo")
            if a_now is not None and pose is None:
                for idx, pk in enumerate(a_now.get("picks", [])):
                    c = MARKER_COLORS.get(pk[0], (200, 200, 200))
                    pp = panel.w2p(pk[1], pk[2])
                    cv2.circle(right, pp, 8, c, 2, cv2.LINE_AA)
                    cv2.putText(right, str(idx + 1), (pp[0] + 9, pp[1] + 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, c, 1)
                msg = a_now.get("msg", "")
                if msg:
                    cv2.putText(right, msg, (10, right.shape[0] - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                                (0, 255, 255), 1)

            if pose:
                lines = [
                    "x_camera_map = %+7.3f m" % pose["x"],
                    "y_camera_map = %+7.3f m" % pose["y"],
                    "yaw_camera_map = %+7.2f deg" % pose["yaw"],
                    "marker_count = %d      residual = %.2f px"
                    % (pose.get("n_pairs", 0), pose.get("rms", 0.0)),
                    "valid = %-5s  status = %s" % (True, pose.get("status", "")),
                    "update = %.1f Hz" % fps,
                ]
                color = (0, 255, 0)
            else:
                lines = [
                    "x_camera_map = -", "y_camera_map = -", "yaw_camera_map = -",
                    "marker_count = 0      residual = - px",
                    "valid = False  status = IDLE (no algorithm)",
                    "update = %.1f Hz" % fps,
                ]
                color = (0, 200, 255)
        else:
            lines = ["paused"] + [""] * (FOOT_LINES - 1)
            color = (200, 200, 120)

        # 画布按实际帧宽拼接（相机可能是 640x480）
        lw = left.shape[1]
        rw = right.shape[1]
        state["left_w"] = lw
        head = banner(lw + rw, hdr, (200, 200, 120))
        state["head_h"] = head.shape[0]
        foot = banner(lw + rw, lines, color)
        canvas = np.zeros((head.shape[0] + left.shape[0] + foot.shape[0],
                           lw + rw, 3), np.uint8)
        canvas[:head.shape[0]] = head
        canvas[head.shape[0]:head.shape[0] + left.shape[0], :lw] = left
        canvas[head.shape[0]:head.shape[0] + right.shape[0], lw:] = right
        canvas[head.shape[0] + left.shape[0]:] = foot

        if writer is not None:
            writer.write(canvas)

        if a.headless:
            if n % 10 == 0:
                cv2.imwrite(os.path.join(a.out, "rt_%04d.jpg" % n), canvas)
                print("frame %d  %s" % (n, lines[4] if len(lines) > 4 else ""))
            limit = a.max_frames if a.max_frames > 0 else 20
            if n >= limit:
                break
            continue

        cv2.imshow(WIN, canvas)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord(" "):
            paused = not paused
        elif k == ord("s"):
            p = os.path.join(a.out, "shot_%s.jpg"
                             % datetime.datetime.now().strftime("%H%M%S"))
            cv2.imwrite(p, canvas)
            print("截图", p)
        elif k == ord("n"):
            if state.get("algo"):
                algo_reset(state["algo"])
            print("已清空位姿，进入重新标定")
        elif k == ord("c"):
            if state.get("algo"):
                state["algo"]["picks"] = []
                state["algo"]["msg"] = "click 4 map markers on the right to init"
        elif k == ord("t"):
            trail_on = not trail_on
        elif k == ord("r"):
            if writer is None:
                p = os.path.join(a.out, "rec_%s.avi"
                                 % datetime.datetime.now().strftime("%H%M%S"))
                writer = cv2.VideoWriter(p, cv2.VideoWriter_fourcc(*"MJPG"), 20.0,
                                         (canvas.shape[1], canvas.shape[0]))
                print("录制 ->", p)
            else:
                writer.release()
                writer = None
                print("停止录制")

        if a.max_frames and n >= a.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
    if not a.headless:
        cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
