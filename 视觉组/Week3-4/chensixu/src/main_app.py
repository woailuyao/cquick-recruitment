#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
main_app.py —— CQuick 视觉考核：足球检测 + 场地标志单目自定位（实时主程序）
==========================================================================

运行
----
    python main_app.py                  # 用默认摄像头（脚本里 DEFAULT_CAMERA）
    python main_app.py --camera 0        # 指定摄像头序号
    python main_app.py --video test.avi  # 用视频文件调试（不用相机）
    python main_app.py --save demo.avi   # 同时把左右分屏结果写成视频

键盘
----
    q / ESC : 退出
    c       : 多解时循环切换候选位姿（对称歧义人工消歧）
    l       : 锁定当前最优候选为跟踪先验
    x       : 清空先验，重新做全局搜索（也用于演示"重新定位"）
    r       : 开关标志几何细化（白线求交）
    o       : 用当前匹配结果反推相机高度/俯仰角（结果打印到终端）
    w       : 把当前高度/俯仰角写回 config/camera_extrinsics.yaml
    s       : 保存一张当前画面 PNG
    + / -   : 调整标志检测置信度阈值
    [ / ]   : 调整相机高度（-/+ 2 cm）
    , / .   : 调整相机俯仰角（-/+ 1 度）

坐标约定（与 match_map.py 一致，README 必须写清楚）
---------------------------------------------------
    * 载体参考原点 = 相机光心在地面 Z=0 的投影点（相机装在移动测试架上，
      因此 carrier_pose 与 camera_pose 相同）。
    * 输出 x_camera_map / y_camera_map 单位为米；yaw_camera_map 为相机水平
      前向相对地图 +X 轴的角度，逆时针为正，单位为度。
    * 地面 Z=0；相机高度 h、俯仰角 pitch、滚转角 roll 属于安装外参（现场可微调）。

关于对称场地的多解（很重要）
----------------------------
    field.yaml 的标志点阵是轴对称/中心对称的，单独一帧经常存在多个同样好的位姿
    （例如整体转 180° 或镜像后仍然吻合）。程序不会硬编一个"看起来连续"的位姿，
    而是如实输出 status=ambiguous / valid=False，并在右侧地图上画出候选
    （cand0? 为最优，altN 为其它）。此时：
        * 按 c 在候选之间切换，选定后程序用该位姿作为连续帧先验继续跟踪；
        * 或启动时用 --init X Y YAW 直接给先验；
        * 按 x 清空先验重新全局搜索（演示"重新定位"/丢失恢复）。
    这正是细则要求的"处理同类型标志的多候选匹配和场地对称歧义"。

链路
----
    相机帧 -> YOLO(ball) 检测框/置信度/像素中心
           -> YOLO(marker) 检测框 -> 白线求交细化出几何特征点
           -> 去畸变 + 内参 -> 相机射线 -> 与地面 Z=0 求交 -> 载体地面坐标
           -> field.yaml 多特征匹配（两两组合 + 内点评估 + 最小二乘精化
              + 对称歧义判定 + 连续帧先验）-> 地图位姿
           -> 显示 / 平滑 / 有效性判定
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

import match_map as mm

# ============================================================================
# 可调参数（现场调试基本只需要动这一块）
# ============================================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))


def first_existing(candidates):
    """按顺序返回第一个存在的文件；都不存在时返回第一个（便于报错时给出预期路径）。"""
    for p in candidates:
        if p and os.path.isfile(p):
            return p
    return candidates[0]


# 兼容两种目录布局：
#   1) 开发目录：main_app.py、camera_intrinsics.yaml、config/ 都在同一层
#   2) 提交目录：src/main_app.py + ../calibration/ + ../config/
INTRINSICS_CANDIDATES = [
    os.path.join(BASE_DIR, "camera_intrinsics.yaml"),
    os.path.join(BASE_DIR, "config", "camera_intrinsics.yaml"),
    os.path.join(BASE_DIR, os.pardir, "calibration", "camera_intrinsics.yaml"),
    os.path.join(os.getcwd(), "camera_intrinsics.yaml"),
]
EXTRINSICS_CANDIDATES = [
    os.path.join(BASE_DIR, "config", "camera_extrinsics.yaml"),
    os.path.join(BASE_DIR, os.pardir, "calibration", "camera_extrinsics.yaml"),
    os.path.join(os.getcwd(), "config", "camera_extrinsics.yaml"),
]


def resolve_model_path(path):
    """权重默认写在当前目录；找不到时再到脚本所在目录找，保证从任意目录启动都能跑。"""
    if not path:
        return path
    if os.path.isfile(path):
        return path
    alt = os.path.join(BASE_DIR, path)
    return alt if os.path.isfile(alt) else path

PROC_W, PROC_H = 640, 480        # 处理分辨率（与标定分辨率一致，或按比例换算内参）
DEFAULT_CAMERA = 1               # 摄像头序号；打不开时自动尝试 0/1/2

BALL_CONF = 0.30                 # 足球置信度阈值（原来代码 >0.3，调高会漏检）
MARKER_CONF = 0.30               # 场地标志置信度阈值（原来代码 >0.3，调高会漏检 L/T）

CAMERA_HEIGHT = 0.82             # 相机光心离地高度 (m)，外参，建议实测
PITCH_DEGREE = 8.0               # 相机俯仰角（低头为正，度），外参，建议实测
ROLL_DEGREE = 0.0                # 滚转角（本次假设为 0，不参与计算）

# 地面点合理范围：太近（贴地/噪声大）、太远（接近地平线）都丢掉。
# 这两个上限不要收太紧：外参不准时会把本来有效的标志判成"no ground"，
# 表现出来就像"某些 L/T 点识别不出来"。
GROUND_X_MIN, GROUND_X_MAX = 0.15, 20.00
GROUND_Y_ABS_MAX = 15.00

REFINE_ENABLED = True            # 是否用白线求交细化 L/T/X 特征点
WHITE_V_MIN = 120                # 白线亮度阈值（HSV 的 V）
WHITE_S_MAX = 95                 # 白线饱和度上限（HSV 的 S）

MATCH_CFG = dict(mm.DEFAULT_CFG)
MATCH_CFG.update({
    "min_inliers": 2,             # 2 个标志能定姿，但要有上一帧先验才判有效
    "inlier_tol": 0.35,
    "pair_dist_tol": 0.60,
    "max_residual": 0.40,
    "prior_gate": 1.50,
})

# 位姿平滑：2 个内点时噪声大，用更强平滑；大跳变要连续确认才接受（见 PoseTracker）
ALPHA_SINGLE = 0.22              # 只有 2 个内点时的指数平滑系数（越小越稳）
ALPHA_MULTI = 0.45               # 3 个以上内点时的指数平滑系数
MAX_JUMP_M = 0.80                # 单帧超过该跳变视为可疑，需连续 3 帧一致才切换
MAX_YAW_JUMP_DEG = 20.0
PRIOR_TTL_FRAMES = 15            # 连续无效超过这么多帧就丢弃先验（不无限沿用旧位姿）
AUTO_LOCK_FRAMES = 5             # 多解时累计这么多帧都只有"同样好的多个解"，自动锁定最优候选


# ============================================================================
# 小工具
# ============================================================================
def wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


def put_text(img, text, org, scale=0.5, color=(255, 255, 255), thick=1, bg=True):
    """带黑色底衬的文字，避免叠在画面里看不清。"""
    font = cv2.FONT_HERSHEY_SIMPLEX
    (tw, th), base = cv2.getTextSize(text, font, scale, thick)
    x, y = int(org[0]), int(org[1])
    if bg:
        cv2.rectangle(img, (x - 3, y - th - 3), (x + tw + 3, y + base + 3), (0, 0, 0), -1)
    cv2.putText(img, text, (x, y), font, scale, color, thick, cv2.LINE_AA)
    return tw + 6, th + base + 6


def log(msg):
    print(msg, flush=True)


def compat_classes(c):
    """T 与 X 都是线交点标志，模型容易互相认错，匹配/诊断时按同类处理。"""
    return (1, 2) if c in (1, 2) else (c,)


def nearest_map_pair_distance(markers, cls_a, cls_b, d_obs):
    """地图里与观测间距 d_obs 最接近的同类标志对间距（只用于尺度自检）。"""
    ca, cb = compat_classes(cls_a), compat_classes(cls_b)
    best = None
    for i in range(len(markers)):
        if markers[i]["cls"] not in ca:
            continue
        for j in range(i + 1, len(markers)):
            if markers[j]["cls"] not in cb:
                continue
            dm = math.hypot(markers[j]["x"] - markers[i]["x"],
                            markers[j]["y"] - markers[i]["y"])
            if best is None or abs(dm - d_obs) < abs(best - d_obs):
                best = dm
    return best


def pick_obs_pair(obs, min_dist=0.3):
    """挑一对间距 >= min_dist 的观测，避免同一标志被重复检测时输出 0 间距。"""
    for i in range(len(obs)):
        for j in range(i + 1, len(obs)):
            d = math.hypot(obs[j]["x"] - obs[i]["x"], obs[j]["y"] - obs[i]["y"])
            if d >= min_dist:
                return i, j, d
    return None


class DetTracker:
    """
    逐帧检测 -> 短轨迹，用来压制"特征点闪烁 / L/T/X 跳变 / 草地褶皱误检"。

    做法：
      * 检测框按 IoU 关联到轨迹上（不看类别，允许类别抖动时仍能关联）；
      * 只有连续出现 >= min_hits 帧的轨迹才参与定位，单帧误检直接被丢掉；
      * 类别按最近 vote_len 帧的置信度加权投票，压掉 L/T/X 之间跳；
      * 特征点像素做指数平滑，之后用平滑后的像素重新投影地面，减小抖动。

    注意：如果模型对某个标志**稳定地**判错（比如始终把 X 判成 T），跟踪是纠正不了的，
    那属于训练数据问题（见 README 的"问题与解决"）。
    """

    def __init__(self, iou_thr=0.30, max_age=2, min_hits=2, vote_len=6,
                 smooth=0.5, min_conf=0.25):
        self.iou_thr = iou_thr
        self.max_age = max_age
        self.min_hits = min_hits
        self.vote_len = vote_len
        self.smooth = smooth
        self.min_conf = min_conf
        self.tracks = []
        self._next_id = 0

    @staticmethod
    def _iou(a, b):
        ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
        ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
        iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
        inter = iw * ih
        ua = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
        return inter / ua if ua > 1e-6 else 0.0

    def update(self, dets):
        for t in self.tracks:
            t["matched"] = False

        pairs = []
        for di, d in enumerate(dets):
            for ti, t in enumerate(self.tracks):
                v = self._iou(d["xyxy"], t["box"])
                if v >= self.iou_thr:
                    pairs.append((v, di, ti))
        pairs.sort(key=lambda x: -x[0])

        used_d, used_t = set(), set()
        for _v, di, ti in pairs:
            if di in used_d or ti in used_t:
                continue
            used_d.add(di)
            used_t.add(ti)
            d, t = dets[di], self.tracks[ti]
            t["matched"] = True
            t["age"] = 0
            t["box"] = d["xyxy"]
            t["hits"] += 1
            t["votes"].append((d["cls"], d["conf"]))
            t["votes"] = t["votes"][-self.vote_len:]
            a = self.smooth
            t["uv"] = (t["uv"][0] + a * (d["u"] - t["uv"][0]),
                       t["uv"][1] + a * (d["v"] - t["uv"][1]))
            d["_trk"] = t

        for di, d in enumerate(dets):
            if di in used_d:
                continue
            t = {"box": d["xyxy"], "votes": [(d["cls"], d["conf"])],
                 "uv": (d["u"], d["v"]), "hits": 1, "age": 0,
                 "matched": True, "id": self._next_id}
            self._next_id += 1
            d["_trk"] = t
            self.tracks.append(t)

        for t in self.tracks:
            if not t["matched"]:
                t["age"] += 1
        self.tracks = [t for t in self.tracks if t["age"] <= self.max_age]

        for d in dets:
            t = d["_trk"]
            votes = {}
            for c, cf in t["votes"]:
                votes[c] = votes.get(c, 0.0) + cf
            # 平票时用最近一帧的类别（避免 max() 随便挑第一个）
            best = t["votes"][-1][0]
            for c, v in votes.items():
                if v > votes[best]:
                    best = c
            total = sum(votes.values()) or 1.0
            d["track_hits"] = t["hits"]
            d["voted_cls"] = best
            d["vote_margin"] = votes[best] / total
            d["smoothed_uv"] = t["uv"]
            d["confirmed"] = t["hits"] >= self.min_hits
            # 不再因为"类别投票不够集中"就整帧丢掉该点（模型类别本来就在跳），
            # 只在类别明显分裂(L 与 T/X 各占一半)且置信度很低时才不参与。
            split = len(votes) > 1 and d["vote_margin"] < 0.40
            d["trusted"] = (d["confirmed"] and d["conf"] >= self.min_conf
                            and not (split and d["conf"] < 0.45))


def save_extrinsics(path, height, pitch_deg, roll_deg):
    """把当前外参写回 camera_extrinsics.yaml（按 w 触发），下次启动直接生效。"""
    text = (
        "# 相机安装外参（由 main_app.py 按 w 写入，也可手改）\n"
        "# height/pitch 必须是录视频时实际安装状态下的测量值\n"
        f"camera_height_m: {height:.3f}\n"
        f"pitch_deg: {pitch_deg:.2f}\n"
        f"roll_deg: {roll_deg:.2f}\n"
    )
    return write_text_file(path, text, {})


def pose_yaml_text(pose, result, frame_idx, map_src, projector, fresh, miss):
    """
    按任务书 2.1 / 2.11 的字段定义生成 camera_pose_map 文本（YAML 片段）。
    字段名、单位、坐标约定与任务书一致：x/y 米，yaw 度，逆时针相对地图 +X。
    """
    res = result.get("residual", float("inf"))
    res_txt = "null" if not math.isfinite(res) else f"{res:.3f}"
    valid_txt = "true" if result.get("valid") else "false"
    status = str(result.get("status", "lost"))
    if result.get("tie_resolved"):
        status += "(prior)"
    # 只在"本帧有效"时才写坐标：无效/过期一律写 null，
    # 避免把上一帧的旧位姿当成结果交给下游（任务书要求无法定位时输出无效）。
    if pose is None or not fresh:
        xs = ys = yaws = "null"
    else:
        xs, ys, yaws = f"{pose[0]:.3f}", f"{pose[1]:.3f}", f"{pose[2]:.2f}"

    lines = [
        f"# frame {frame_idx}  {time.strftime('%Y-%m-%d %H:%M:%S')}  "
        f"field={os.path.basename(str(map_src))}  "
        f"h={projector.height:.3f}m pitch={projector.pitch_deg:.2f}deg",
        "camera_pose_map:",
        f"  x_camera_map: {xs}          # m，相机光心在地图中的 X",
        f"  y_camera_map: {ys}          # m，相机光心在地图中的 Y",
        f"  yaw_camera_map: {yaws}      # deg，相机水平朝向相对地图 +X，逆时针为正",
        f"  marker_count: {int(result.get('marker_count', 0))}",
        f"  residual: {res_txt}         # m，内点 RMS",
        f"  valid: {valid_txt}",
        f'  status: "{status}"',
        "carrier_pose_map:            # 相机光心地面投影即载体参考原点，两者相同",
        f"  x_carrier_map: {xs}",
        f"  y_carrier_map: {ys}",
        f"  yaw_carrier_map: {yaws}",
        "camera_extrinsics:",
        f"  height_m: {projector.height:.3f}",
        f"  pitch_deg: {projector.pitch_deg:.2f}",
        f"  roll_deg: {ROLL_DEGREE:.2f}",
        f"  frame_is_fresh: {'true' if fresh else 'false'}",
        f"  stale_frames: {int(miss)}",
    ]
    return "\n".join(lines) + "\n"


def write_text_file(path, text, warn_state):
    try:
        with open(path, "w", encoding="utf-8") as f:
            f.write(text)
        return True
    except OSError as exc:
        if not warn_state.get("warned"):
            warn_state["warned"] = True
            log(f"[out] 写入 {path} 失败：{exc}（后续不再提示）")
        return False


# ============================================================================
# 1. 内参 / 外参
# ============================================================================
def scale_intrinsics(K, from_size, to_size):
    """把在 from_size 下标定的内参换算到 to_size（等比缩放）。"""
    fw, fh = from_size
    tw, th = to_size
    if not fw or not fh or (fw == tw and fh == th):
        return K.astype(float).copy()
    sx, sy = tw / float(fw), th / float(fh)
    K2 = K.astype(float).copy()
    K2[0, 0] *= sx
    K2[0, 2] *= sx
    K2[1, 1] *= sy
    K2[1, 2] *= sy
    if abs(sx - sy) > 1e-3:
        log(f"[warn] 标定分辨率 {from_size} 与处理分辨率 {to_size} 宽高比不一致，"
            f"内参换算可能不准（sx={sx:.3f}, sy={sy:.3f}）")
    return K2


class GroundProjector:
    """
    像素 -> 地面（载体坐标系）投影。

    相机坐标系：X_c 向右、Y_c 向下、Z_c 向前（光轴）。
    载体地面坐标系：X_b 向前（光轴的水平投影）、Y_b 向左、Z_b 向上。
    相机低头 pitch 后，相机坐标 -> 载体坐标的旋转为

        R_bc = [[ 0,      -sin(p),  cos(p)],
                [-1,        0,      0     ],
                [ 0,      -cos(p), -sin(p)]]

    射线 d_c = (x_n, y_n, 1)，d_b = R_bc d_c，相机光心 O_b=(0,0,h)，
    地面 Z=0 => lambda = h / (cos(p)*y_n + sin(p))，交点 P_b = lambda*(d_bx, d_by)。

    注意：这里必须用 cv2.undistortPoints 的**归一化**输出（不传 P=K）。
    之前代码传了 P=K，拿到的是"去畸变后的像素坐标"（几百的量级），
    却当成归一化坐标用，X/Y 会被放大几百倍 —— 这是原来误差巨大的主因之一。
    """

    def __init__(self, K, dist, height, pitch_deg,
                 x_min=GROUND_X_MIN, x_max=GROUND_X_MAX, y_abs=GROUND_Y_ABS_MAX):
        self.K = np.asarray(K, dtype=np.float64)
        self.dist = np.asarray(dist, dtype=np.float64).reshape(-1)
        self.height = float(height)
        self.pitch_deg = float(pitch_deg)
        self.pitch_rad = math.radians(self.pitch_deg)
        self.x_min, self.x_max, self.y_abs = x_min, x_max, y_abs

    def clone(self, height=None, pitch_deg=None):
        return GroundProjector(self.K, self.dist,
                               self.height if height is None else height,
                               self.pitch_deg if pitch_deg is None else pitch_deg,
                               self.x_min, self.x_max, self.y_abs)

    def set_params(self, height=None, pitch_deg=None):
        if height is not None:
            self.height = float(height)
        if pitch_deg is not None:
            self.pitch_deg = float(pitch_deg)
            self.pitch_rad = math.radians(self.pitch_deg)

    def project_many(self, uvs, filter_range=True):
        """uvs: [(u,v), ...] -> [(x_b, y_b) 或 None, ...]（顺序一一对应）。"""
        if not len(uvs):
            return []
        pts = np.asarray(uvs, dtype=np.float64).reshape(-1, 1, 2)
        norm = cv2.undistortPoints(pts, self.K, self.dist).reshape(-1, 2)
        st, ct = math.sin(self.pitch_rad), math.cos(self.pitch_rad)
        out = []
        for xn, yn in norm:
            denom = st + ct * yn
            if denom <= 1e-3:                 # 射线朝上 / 接近地平线
                out.append(None)
                continue
            lam = self.height / denom
            xb = lam * (ct - st * yn)
            yb = lam * (-xn)
            if filter_range and (not (self.x_min <= xb <= self.x_max)
                                 or abs(yb) > self.y_abs):
                out.append(None)              # 落在合理场地之外
                continue
            out.append((float(xb), float(yb)))
        return out


# ============================================================================
# 2. 标志几何细化：L/T/X 用白线求交，点球点用白色斑块质心
# ============================================================================
def _intersect_lines(p1, d1, p2, d2):
    den = d1[0] * d2[1] - d1[1] * d2[0]
    if abs(den) < 1e-6:
        return None
    rx, ry = p2[0] - p1[0], p2[1] - p1[1]
    t = (rx * d2[1] - ry * d2[0]) / den
    return np.array([p1[0] + t * d1[0], p1[1] + t * d1[1]], dtype=float)


def refine_marker_point(frame, box, cls_id, enabled=True, debug=False):
    """
    YOLO 检测框 -> 几何特征点像素坐标。

    * LCross / TCross / XCross：在框内（稍外扩）提取白色场地线，按方向聚类出
      两条主线并求交点 —— 这才是真正的线交点；bbox 中心对 L/T 会沿某条线
      偏出半条边，远处会带来几十厘米的系统误差。
    * PenaltyPoint：取白色斑块质心。
    任何一步失败都退回 bbox 中心（debug=True 时会打印原因），保证不会崩。
    """
    x1, y1, x2, y2 = [float(v) for v in box]
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
    if not enabled:
        return cx, cy, "bbox"

    bw, bh = max(2.0, x2 - x1), max(2.0, y2 - y1)
    # 检测框往往只框住标志的一部分（例如 T 的横线），外扩 50% 才能把另一条线也包进来
    pad_x, pad_y = max(5, int(0.50 * bw)), max(5, int(0.50 * bh))
    H, W = frame.shape[:2]
    rx1, rx2 = int(max(0, x1 - pad_x)), int(min(W - 1, x2 + pad_x))
    ry1, ry2 = int(max(0, y1 - pad_y)), int(min(H - 1, y2 + pad_y))
    if rx2 - rx1 < 6 or ry2 - ry1 < 6:
        return cx, cy, "bbox"

    roi = frame[ry1:ry2 + 1, rx1:rx2 + 1]
    if roi.size == 0:
        return cx, cy, "bbox"

    def _fail(reason):
        if debug:
            print(f"[refine] 退回 bbox：{reason}")
        return cx, cy, "bbox"

    try:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, WHITE_V_MIN), (180, WHITE_S_MAX, 255))
    except cv2.error as exc:
        return _fail(f"cvtColor/inRange 异常 {exc}")

    n_white = cv2.countNonZero(mask)
    if n_white < 35:
        return _fail(f"白色像素太少({n_white})")

    if cls_id == 3:  # 点球点：白色斑块质心
        mo = cv2.moments(mask)
        if mo["m00"] > 30:
            return rx1 + mo["m10"] / mo["m00"], ry1 + mo["m01"] / mo["m00"], "blob"
        return _fail("点球点斑块太小")

    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    # 用边缘图而不是实心 mask 做霍夫：实心白色区域会让霍夫只在粗线内部乱找方向，
    # 而 Canny 边缘能同时拿到横线和竖线两条细边，两个方向的线段才能相交。
    edges = cv2.Canny(mask, 50, 150)
    min_len = max(8, int(0.25 * max(bw, bh)))
    segs = cv2.HoughLinesP(edges, 1, np.pi / 180.0,
                           threshold=max(10, min_len // 2),
                           minLineLength=min_len, maxLineGap=5)
    if segs is None or len(segs) < 2:
        return _fail(f"霍夫线段不足({0 if segs is None else len(segs)})")

    # 按方向聚类（方向对 180° 取模，同一条线的线段归为一类）
    clusters = []  # [{'ang', 'pts', 'len'}]
    for s in np.asarray(segs).reshape(-1, 4):
        p1 = np.array([s[0], s[1]], dtype=float)
        p2 = np.array([s[2], s[3]], dtype=float)
        d = p2 - p1
        length = float(np.hypot(d[0], d[1]))
        if length < 4.0:
            continue
        ang = math.atan2(d[1], d[0])
        placed = False
        for c in clusters:
            diff = abs(((ang - c["ang"] + math.pi / 2) % math.pi) - math.pi / 2)
            if diff < math.radians(15.0):
                c["pts"].extend([p1, p2])
                c["len"] += length
                placed = True
                break
        if not placed:
            clusters.append({"ang": ang, "pts": [p1, p2], "len": length})

    if debug:
        print("[refine] 方向聚类：" + ", ".join(
            f"{math.degrees(c['ang']):+.0f}°/{c['len']:.0f}px" for c in clusters))
    if len(clusters) < 2:
        return _fail("只找到 1 个直线方向（标志另一条线不在框内？）")

    clusters.sort(key=lambda c: -c["len"])
    best_pair, best_score = None, -1.0
    for i in range(min(3, len(clusters))):
        for j in range(i + 1, min(3, len(clusters))):
            ang_diff = abs(((clusters[i]["ang"] - clusters[j]["ang"] + math.pi / 2)
                            % math.pi) - math.pi / 2)
            if ang_diff < math.radians(20.0):
                continue
            score = min(clusters[i]["len"], clusters[j]["len"])
            if score > best_score:
                best_score, best_pair = score, (i, j)
    if best_pair is None:
        return _fail("两个主方向夹角太小")

    lines = []
    for idx in best_pair:
        pts = np.array(clusters[idx]["pts"], dtype=np.float32)
        if len(pts) < 2:
            return _fail("拟合直线的点太少")
        vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1)
        lines.append((np.array([float(x0), float(y0)]), np.array([float(vx), float(vy)])))

    ip = _intersect_lines(lines[0][0], lines[0][1], lines[1][0], lines[1][1])
    if ip is None or not np.all(np.isfinite(ip)):
        return _fail("两条线近乎平行，求交失败")

    # 交点换算回整幅图像坐标，再检查它是否落在"外扩后的检测框"里（误拟合保护）
    gx, gy = rx1 + float(ip[0]), ry1 + float(ip[1])
    tol_x, tol_y = 0.10 * bw, 0.10 * bh
    if not (x1 - tol_x <= gx <= x2 + tol_x and y1 - tol_y <= gy <= y2 + tol_y):
        return _fail(f"交点({gx:.0f},{gy:.0f})离检测框太远")
    if not (0 <= gx < W and 0 <= gy < H):
        return _fail("交点超出画面")

    # 交点附近必须是白色场地线，否则说明拟合到了别的边（例如草地纹理）
    ix, iy = int(round(gx - rx1)), int(round(gy - ry1))
    x_lo, x_hi = max(0, ix - 3), min(mask.shape[1], ix + 4)
    y_lo, y_hi = max(0, iy - 3), min(mask.shape[0], iy + 4)
    if cv2.countNonZero(mask[y_lo:y_hi, x_lo:x_hi]) < 3:
        return _fail("交点不在白色场地线上")

    return gx, gy, "line"


# ============================================================================
# 3. 位姿平滑 + 先验管理
# ============================================================================
class PoseTracker:
    """
    连续帧位姿平滑 + 跳变确认。

    只有 2 个内点时位姿对像素噪声非常敏感（两个标志距离越近越严重），测出来的
    "另一个解"往往和上一帧只差一点点，所以这里不再"一跳就重置"，而是：
      * 小变化（<= max_jump / max_yaw_jump）：指数平滑（2 个内点用更强的 alpha）；
      * 大跳变：必须连续 confirm 帧彼此一致，才认定为真实重定位并切换；
        否则保持原值，避免把错误匹配直接画出来；
      * 无效帧：只保留 PRIOR_TTL_FRAMES 帧先验，超时清空。
    """

    def __init__(self, alpha_single=0.22, alpha_multi=0.45,
                 max_jump=0.8, max_yaw_jump=20.0,
                 ttl=PRIOR_TTL_FRAMES, confirm=3,
                 settle_n=15, settle_m=0.30, settle_yaw=6.0):
        self.alpha_single = alpha_single   # 2 个内点：强平滑，压噪声
        self.alpha_multi = alpha_multi     # 3 个以上内点：正常平滑
        self.max_jump = max_jump
        self.max_yaw_jump = max_yaw_jump
        self.ttl = ttl
        self.confirm = confirm
        # 静止稳定：最近 settle_n 帧测量足够集中时，用中位数代替 EMA，
        # 这样相机不动时读数不会一直在最后一位上跳（移动时自动失效）。
        self.settle_n = settle_n
        self.settle_m = settle_m
        self.settle_yaw = settle_yaw
        self.pose = None      # 平滑后的 (x, y, yaw)
        self.prior = None     # 给匹配用的先验 (x, y, yaw)
        self.miss = 0
        self.just_reset = False
        self._pending = None  # 待确认的大跳变 (meas, count)
        self._mhist = []      # 最近的原始测量（做中位数用）
        self.last_jump = 0.0

    def reset(self, pose=None):
        self.pose = tuple(pose) if pose else None
        self.prior = tuple(pose) if pose else None
        self.miss = 0
        self.just_reset = True
        self._pending = None
        self._mhist = []
        self.last_jump = 0.0

    def clear_prior(self):
        self.prior = None
        self.miss = self.ttl + 1
        self._pending = None

    def update(self, result):
        """返回 (显示位姿, 连续无效帧数, 是否刚重置)。"""
        self.just_reset = False
        if not result["valid"]:
            self.miss += 1
            if self.miss > self.ttl:
                # 超过宽限期就彻底清空：不把旧位姿当成"当前位姿"继续显示/写文件，
                # 界面回到 --，pose_output.yaml 里对应字段输出 null（valid: false）。
                self.prior = None
                self.pose = None
                self._pending = None
            return self.pose, self.miss, self.just_reset

        meas = (float(result["x"]), float(result["y"]), float(result["yaw_deg"]))
        n_inl = int(result.get("marker_count", 0))
        self.miss = 0

        if self.pose is None:
            self.pose = meas
            self.prior = meas
            self.just_reset = True
            self._pending = None
            self.last_jump = 0.0
            return self.pose, self.miss, self.just_reset

        jump = math.hypot(meas[0] - self.pose[0], meas[1] - self.pose[1])
        dyaw = abs(wrap_deg(meas[2] - self.pose[2]))
        self.last_jump = jump

        if jump <= self.max_jump and dyaw <= self.max_yaw_jump:
            a = self.alpha_single if n_inl <= 2 else self.alpha_multi
            self.pose = (
                self.pose[0] + a * (meas[0] - self.pose[0]),
                self.pose[1] + a * (meas[1] - self.pose[1]),
                wrap_deg(self.pose[2] + a * wrap_deg(meas[2] - self.pose[2])),
            )
            self._pending = None
        else:
            # 大跳变：连续 confirm 帧彼此一致才切换（不要被单帧错误匹配带跑）
            if (self._pending is not None
                    and math.hypot(meas[0] - self._pending[0][0],
                                   meas[1] - self._pending[0][1]) <= 0.6
                    and abs(wrap_deg(meas[2] - self._pending[0][2])) <= 15.0):
                cnt = self._pending[1] + 1
            else:
                cnt = 1
            self._pending = (meas, cnt)
            if cnt >= self.confirm:
                self.pose = meas
                self._pending = None
                self.just_reset = True
                self._mhist = []     # 重定位后历史作废

        # ---- 静止稳定：最近若干帧测量足够集中时用中位数（相机不动时读数不再抖）----
        self._mhist.append(meas)
        if len(self._mhist) > self.settle_n:
            self._mhist = self._mhist[-self.settle_n:]
        if len(self._mhist) >= 8:
            y0 = self._mhist[-1][2]
            xs = sorted(m[0] for m in self._mhist)
            ys = sorted(m[1] for m in self._mhist)
            yaws = sorted(y0 + wrap_deg(m[2] - y0) for m in self._mhist)
            k = len(xs) // 2
            mx, my, myaw = xs[k], ys[k], yaws[k]
            spread = max(math.hypot(x - mx, y - my) for x, y, _ in self._mhist)
            yaw_spread = max(abs(wrap_deg(y - myaw)) for _, _, y in self._mhist)
            if spread <= self.settle_m and yaw_spread <= self.settle_yaw:
                self.pose = (mx, my, wrap_deg(myaw))

        self.prior = self.pose
        return self.pose, self.miss, self.just_reset


# ============================================================================
# 4. 右侧地图视图
# ============================================================================
class MapView:
    """
    field.yaml 全局俯视图。

    显示方向与场地坐标系示意图一致：
        +X 指向球门（画面上方）、+Y 为面向 +X 时的左侧（画面左方）、+Z 垂直地面向上。
    即屏幕像素与场地坐标的关系为
        px_x = 中心 - (y - cy) * scale
        px_y = 中心 - (x - cx) * scale
    （若以后要改成 +X 向右、+Y 向上，只改 to_px 和 _camera 里的方向向量即可。）
    """

    def __init__(self, markers, width=640, height=480, side_w=200, pad=30, source=None):
        self.w, self.h = width, height
        self.side_w = side_w
        self.map_w = width - side_w
        self.map_h = height
        self.pad = pad
        self.source = os.path.basename(str(source)) if source else "field.yaml"
        xs = [m["x"] for m in markers] + [0.0]
        ys = [m["y"] for m in markers] + [0.0]
        margin = 0.9
        self.minx, self.maxx = min(xs) - margin, max(xs) + margin
        self.miny, self.maxy = min(ys) - margin, max(ys) + margin
        spanx = max(0.1, self.maxx - self.minx)
        spany = max(0.1, self.maxy - self.miny)
        # 注意：X 对应屏幕纵向、Y 对应屏幕横向
        self.scale = min((self.map_h - 2 * pad) / spanx,
                         (self.map_w - 2 * pad) / spany)
        self.cx = (self.minx + self.maxx) / 2.0
        self.cy = (self.miny + self.maxy) / 2.0
        self.markers = markers
        # 场地边界 = 标志点的外包矩形（T 在边线/球门线上）。
        # 相机（载体）可以在场地外（桌子放在场边），所以位姿超出这个矩形是正常的。
        self.f_minx = min(m["x"] for m in markers)
        self.f_maxx = max(m["x"] for m in markers)
        self.f_miny = min(m["y"] for m in markers)
        self.f_maxy = max(m["y"] for m in markers)

    def to_px(self, x, y):
        """场地坐标 (m) -> 屏幕像素：+X 向上、+Y 向左。"""
        return (int(round(self.map_w / 2.0 - (y - self.cy) * self.scale)),
                int(round(self.map_h / 2.0 - (x - self.cx) * self.scale)))

    @staticmethod
    def dir_to_screen(yaw_rad):
        """场地坐标里的方向角 -> 屏幕方向单位向量（配合 to_px 的旋转）。"""
        return -math.sin(yaw_rad), -math.cos(yaw_rad)

    # ---- 底层绘制 ----
    def _grid(self, img):
        # 场地等距网格：X 坐标标在画面左缘，Y 坐标标在画面底边
        for gx in range(int(math.floor(self.minx)), int(math.ceil(self.maxx)) + 1):
            p1 = self.to_px(gx, self.miny)
            p2 = self.to_px(gx, self.maxy)
            cv2.line(img, p1, p2, (55, 75, 55), 1, cv2.LINE_AA)
            if gx % 2 == 0:
                put_text(img, f"{gx}", (6, p1[1] + 4), 0.33,
                         (130, 150, 130), 1, False)
        for gy in range(int(math.floor(self.miny)), int(math.ceil(self.maxy)) + 1):
            p1 = self.to_px(self.minx, gy)
            p2 = self.to_px(self.maxx, gy)
            cv2.line(img, p1, p2, (55, 75, 55), 1, cv2.LINE_AA)
            if gy % 2 == 0:
                put_text(img, f"{gy}", (p1[0] - 4, self.h - 10), 0.33,
                         (130, 150, 130), 1, False)

    def _field_rect(self, img):
        """画出场地边界（标志点外包矩形），让"相机在场地外"一目了然。"""
        x0, x1 = self.f_minx, self.f_maxx
        y0, y1 = self.f_miny, self.f_maxy
        col = (110, 140, 110)
        edges = (((x0, y0), (x0, y1)), ((x1, y0), (x1, y1)),
                 ((x0, y0), (x1, y0)), ((x0, y1), (x1, y1)))
        for a, b in edges:
            p1 = np.array(self.to_px(*a), float)
            p2 = np.array(self.to_px(*b), float)
            seg = float(np.linalg.norm(p2 - p1))
            if seg < 1.0:
                continue
            d = (p2 - p1) / seg
            t = 0.0
            while t < seg:
                q1 = p1 + d * t
                q2 = p1 + d * min(t + 6.0, seg)
                cv2.line(img, tuple(np.round(q1).astype(int)),
                         tuple(np.round(q2).astype(int)), col, 1, cv2.LINE_AA)
                t += 12.0
        px = self.to_px(x1, y0)
        put_text(img, "field", (px[0] - 36, px[1] - 8), 0.32, col, 1, False)

    def _axes(self, img):
        o = self.to_px(0.0, 0.0)
        ax = self.to_px(1.0, 0.0)      # +X 指向球门（画面上方）
        ay = self.to_px(0.0, 1.0)      # +Y 面向 +X 时的左侧（画面左方）
        cv2.arrowedLine(img, o, ax, (60, 60, 255), 2, cv2.LINE_AA, tipLength=0.25)
        cv2.arrowedLine(img, o, ay, (80, 220, 80), 2, cv2.LINE_AA, tipLength=0.25)
        put_text(img, "+X", (ax[0] + 6, ax[1] + 4), 0.42, (60, 60, 255), 1, False)
        put_text(img, "+Y", (ay[0] - 26, ay[1] - 6), 0.42, (80, 220, 80), 1, False)
        put_text(img, "O", (o[0] - 16, o[1] + 16), 0.4, (200, 200, 200), 1, False)

    def _markers(self, img):
        for m in self.markers:
            p = self.to_px(m["x"], m["y"])
            color = mm.CLASS_COLORS.get(m["cls"], (200, 200, 200))
            short = mm.CLASS_SHORT.get(m["cls"], "?")
            cv2.circle(img, p, 3, color, -1, cv2.LINE_AA)
            if m["cls"] != 3:
                cv2.putText(img, short, (p[0] + 4, p[1] - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1, cv2.LINE_AA)

    def _hypotheses(self, img, groups, chosen_xy):
        for i, g in enumerate(groups):
            # 和已选中的解重合的候选不用再画一个圈（会糊在相机图标上）
            if chosen_xy is not None and math.hypot(g["x"] - chosen_xy[0],
                                                    g["y"] - chosen_xy[1]) < 0.35:
                continue
            p = self.to_px(g["x"], g["y"])
            cv2.circle(img, p, 7, (170, 170, 170), 1, cv2.LINE_AA)
            cv2.putText(img, f"alt{i}", (p[0] + 7, p[1] - 7),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.36, (170, 170, 170), 1, cv2.LINE_AA)

    def _observations(self, img, result, markers):
        obs_map = result.get("obs_map") or []
        if not obs_map:
            return
        for m in result.get("matches", []):
            idx = m["map"]
            if not (0 <= idx < len(markers)) or not (0 <= m["obs"] < len(obs_map)):
                continue
            mid = self.to_px(markers[idx]["x"], markers[idx]["y"])
            op = self.to_px(obs_map[m["obs"]][0], obs_map[m["obs"]][1])
            cv2.line(img, op, mid, (120, 220, 255), 1, cv2.LINE_AA)
            cv2.circle(img, op, 3, (255, 255, 255), -1, cv2.LINE_AA)
            put_text(img, f"{m['dist']:.2f}", (op[0] + 4, op[1] + 12),
                     0.32, (200, 230, 255), 1, False)

    def _camera(self, img, pose, fresh, valid, half_fov_deg):
        if pose is None:
            return
        p = self.to_px(pose[0], pose[1])
        if valid and fresh:
            color = (0, 255, 0)
        elif fresh:
            color = (0, 0, 255)
        else:
            color = (0, 165, 255)
        yaw = math.radians(pose[2])
        # 场地方向 -> 屏幕方向（+X 向上、+Y 向左）
        ux, uy = self.dir_to_screen(yaw)
        tip = (int(p[0] + 34 * ux), int(p[1] + 34 * uy))
        for s in (-1.0, 1.0):                      # 水平视场角示意
            a = yaw + s * math.radians(half_fov_deg)
            ex, ey = self.dir_to_screen(a)
            e = (int(p[0] + 58 * ex), int(p[1] + 58 * ey))
            cv2.line(img, p, e, color, 1, cv2.LINE_AA)
        cv2.circle(img, p, 7, color, -1, cv2.LINE_AA)
        cv2.circle(img, p, 10, color, 1, cv2.LINE_AA)
        cv2.arrowedLine(img, p, tip, color, 3, cv2.LINE_AA, tipLength=0.35)
        if not fresh:
            cv2.circle(img, p, 16, color, 1, cv2.LINE_AA)

    # ---- 主绘制 ----
    def render(self, result, pose, fresh, stale_frames, half_fov_deg,
               fps, loc_hz, extra_hint=None, raw_pose=None, best_cand=None):
        img = np.zeros((self.h, self.w, 3), dtype=np.uint8)
        img[:] = (18, 22, 18)
        cv2.rectangle(img, (0, 0), (self.map_w - 1, self.h - 1), (32, 48, 32), -1)
        self._grid(img)
        self._field_rect(img)
        self._axes(img)
        self._markers(img)
        put_text(img, f"map: {self.source}   {len(self.markers)} markers   unit: m",
                 (8, 16), 0.36, (170, 205, 170), 1, False)

        groups = result.get("groups") or []
        chosen_xy = None
        if pose is not None:
            chosen_xy = (pose[0], pose[1])
        elif groups:
            chosen_xy = (groups[0]["x"], groups[0]["y"])
        if groups:
            self._hypotheses(img, groups, chosen_xy)
        self._observations(img, result, self.markers)
        self._camera(img, pose, fresh, result.get("valid", False), half_fov_deg)
        # 原始（未滤波）估计位姿：空心白圈，便于检查滤波有没有引入延迟
        if raw_pose is not None and pose is not None and fresh:
            rp = self.to_px(raw_pose[0], raw_pose[1])
            fp = self.to_px(pose[0], pose[1])
            cv2.circle(img, rp, 5, (255, 255, 255), 1, cv2.LINE_AA)
            if math.hypot(rp[0] - fp[0], rp[1] - fp[1]) > 3:
                cv2.line(img, rp, fp, (200, 200, 200), 1, cv2.LINE_AA)
                put_text(img, "raw", (rp[0] + 6, rp[1] + 13), 0.30, (210, 210, 210), 1, False)
        if pose is None and groups:
            # 还没锁定先验时，把最优候选画成青色"问号箭头"，提示可以按 c 选择
            g = groups[0]
            p = self.to_px(g["x"], g["y"])
            ux, uy = self.dir_to_screen(math.radians(g["yaw_deg"]))
            tip = (int(p[0] + 30 * ux), int(p[1] + 30 * uy))
            cv2.circle(img, p, 6, (0, 220, 255), 1, cv2.LINE_AA)
            cv2.arrowedLine(img, p, tip, (0, 220, 255), 2, cv2.LINE_AA, tipLength=0.4)
            put_text(img, "cand0?", (p[0] + 8, p[1] + 18), 0.34, (0, 220, 255), 1, False)

        self._sidebar(img, result, pose, fresh, stale_frames, fps, loc_hz,
                      extra_hint, best_cand)
        return img

    def _sidebar(self, img, result, pose, fresh, stale_frames, fps, loc_hz, hint,
                 best_cand=None):
        """右侧信息栏：位姿、匹配质量、有效状态、图例、按键提示。"""
        x0 = self.map_w
        cv2.rectangle(img, (x0, 0), (self.w - 1, self.h - 1), (24, 28, 24), -1)
        cv2.line(img, (x0, 0), (x0, self.h - 1), (90, 90, 90), 1)

        ok = bool(result.get("valid"))
        color = (0, 255, 0) if ok else (0, 0, 255)
        status = str(result.get("status", "-"))
        if result.get("tie_resolved"):
            status += "(prior)"
        residual = result.get("residual", float("inf"))
        res_txt = "--" if not math.isfinite(residual) else f"{residual:.2f} m"

        # 没有可信位姿时，把最优候选也显示出来（青色，明确标注未确认），
        # 避免"一直是 --"却看不到程序到底算出了什么。
        if pose is not None:
            src, vcolor = pose, (255, 255, 255)
        elif best_cand is not None:
            src = (best_cand["x"], best_cand["y"], best_cand["yaw_deg"])
            vcolor = (0, 220, 255)
        else:
            src, vcolor = None, (255, 255, 255)

        tx = x0 + 12
        y = 24
        put_text(img, "CAMERA POSE", (tx, y), 0.42, (150, 200, 255), 1, False)
        y += 15
        if pose is None and best_cand is not None:
            put_text(img, "cand (unverified) - press c", (tx, y), 0.29,
                     (0, 220, 255), 1, False)
        else:
            put_text(img, "camera_frame = rig_frame", (tx, y), 0.30,
                     (140, 140, 140), 1, False)
        y += 22

        vals = (("x_camera_map", "-- m" if src is None else f"{src[0]:.3f} m"),
                ("y_camera_map", "-- m" if src is None else f"{src[1]:.3f} m"),
                ("yaw_camera_map", "-- deg" if src is None else f"{src[2]:.2f} deg"))
        for key, val in vals:
            put_text(img, key, (tx, y), 0.34, (165, 165, 165), 1, False)
            y += 15
            put_text(img, val, (tx + 8, y), 0.50, vcolor, 1, False)
            y += 25

        y += 2
        put_text(img, f"marker_count {result.get('marker_count', 0)}", (tx, y),
                 0.38, (220, 220, 220), 1, False)
        y += 19
        put_text(img, f"residual  {res_txt}", (tx, y), 0.38, (220, 220, 220), 1, False)
        y += 19
        put_text(img, f"valid  {ok}", (tx, y), 0.42, color, 1, False)
        y += 19
        put_text(img, f"status {status}", (tx, y), 0.34, color, 1, False)
        y += 19
        if not fresh and pose is not None:
            put_text(img, f"STALE {stale_frames} frames", (tx, y), 0.34,
                     (0, 165, 255), 1, False)
            y += 17
        put_text(img, f"fps {fps:.1f}   loc {loc_hz:.1f} Hz", (tx, y), 0.34,
                 (195, 195, 195), 1, False)
        y += 26

        put_text(img, "MARKERS", (tx, y), 0.34, (150, 200, 255), 1, False)
        y += 16
        for cid in sorted(mm.CLASS_COLORS):
            cv2.circle(img, (tx + 4, y - 3), 4, mm.CLASS_COLORS[cid], -1, cv2.LINE_AA)
            put_text(img, mm.CLASS_NAMES[cid], (tx + 14, y), 0.34, (215, 215, 215), 1, False)
            y += 16

        if hint:
            parts = [p.strip() for p in str(hint).split("|") if p.strip()]
            y = self.h - 16 - 14 * (len(parts) + 1)
            put_text(img, "KEYS", (tx, y), 0.32, (150, 150, 150), 1, False)
            y += 14
            for part in parts:
                put_text(img, part, (tx, y), 0.30, (0, 210, 255), 1, False)
                y += 14


# ============================================================================
# 5. 外参反推：用当前匹配结果估计相机高度 / 俯仰角
# ============================================================================
def fit_height_pitch(projector, dets, markers, prior, cfg):
    """
    在当前观测上网格搜索 (height, pitch)，取"内点数最多、残差最小"的一组。

    只在按键触发时调用（耗时 1~3 秒可接受），结果打印到终端，方便写进 README。
    原理：height 主要决定整体尺度，pitch 主要决定透视压缩；两者都错了，
    观测点之间的间距就没法和地图点间距对上，因此可用匹配残差反推。
    """
    if len(dets) < 3:
        log("[fit] 至少需要 3 个有效标志才能反推外参："
            "只有 2 个点时 height/pitch 不可辨识（任何间距都能被某组外参凑出来）。")
        return None
    if prior is None:
        log("[fit] 提示：当前没有先验，多解场地下反推结果可能不可靠，建议先按 c 锁定一个候选。")
    uvs = [(d["u"], d["v"]) for d in dets]

    def evaluate(h, p):
        proj = projector.clone(height=h, pitch_deg=p)
        # 这里不过滤地面范围，保证不同 (h,p) 之间的评分口径一致
        grounds = proj.project_many(uvs, filter_range=False)
        obs = [{"cls": d["cls"], "x": g[0], "y": g[1], "conf": d["conf"]}
               for d, g in zip(dets, grounds) if g is not None]
        if len(obs) < 3:
            return None
        res = mm.estimate_pose(obs, markers, last_pose=prior,
                               cfg=dict(cfg, min_inliers=3))
        if res["marker_count"] < 3:
            return None
        return (res["marker_count"], res["residual"], res)

    def search(h_values, p_values, best):
        for h in h_values:
            for p in p_values:
                if h <= 0.1:
                    continue
                sc = evaluate(float(h), float(p))
                if sc is None:
                    continue
                key = (sc[0], -sc[1])
                if best is None or key > best[0]:
                    best = (key, float(h), float(p), sc)
        return best

    best = search(np.arange(0.40, 1.61, 0.10), np.arange(5.0, 45.1, 5.0), None)
    if best is None:
        log("[fit] 反推失败：没有找到能自洽匹配的外参组合。")
        return None
    _, h0, p0, _ = best
    best = search(np.arange(h0 - 0.08, h0 + 0.081, 0.02),
                  np.arange(p0 - 3.0, p0 + 3.01, 1.0), best)

    _, h, p, sc = best
    res = sc[2]
    log(f"[fit] 外参反推：CAMERA_HEIGHT={h:.3f} m  PITCH_DEGREE={p:.2f} deg  "
        f"(内点 {res['marker_count']}  残差 {res['residual']:.3f} m  status={res['status']})")
    return h, p, res["marker_count"], res["residual"]


# ============================================================================
# 6. 相机打开 / 模型加载
# ============================================================================
def open_capture(args):
    if args.video:
        cap = cv2.VideoCapture(args.video)
        if not cap.isOpened():
            log(f"[camera] 无法打开视频文件：{args.video}")
            return None
        log(f"[camera] 已打开视频文件：{args.video}")
        return cap

    indices = [args.camera] if args.camera is not None else [DEFAULT_CAMERA, 0, 2, 1]
    backends = [cv2.CAP_DSHOW, cv2.CAP_ANY] if os.name == "nt" else [cv2.CAP_ANY]

    for idx in indices:
        for backend in backends:
            cap = cv2.VideoCapture(idx, backend)
            if cap is not None and cap.isOpened():
                ok, frame = cap.read()
                if ok and frame is not None:
                    log(f"[camera] 已打开摄像头 index={idx} backend={backend}  "
                        f"分辨率 {frame.shape[1]}x{frame.shape[0]}  "
                        f"CAP_PROP_FPS={cap.get(cv2.CAP_PROP_FPS):.1f}")
                    return cap
            if cap is not None:
                cap.release()

    log("[camera] 打开失败：请确认设备没被其他程序占用（关掉相机预览/其他脚本），"
        "或用 --camera 0 / --camera 1 指定序号。")
    return None


def load_models(args):
    from ultralytics import YOLO

    models = {}
    for name, path, enabled in (("ball", args.ball_model, not args.no_ball),
                                ("marker", args.marker_model, True)):
        if not enabled:
            continue
        path = resolve_model_path(path)
        if not os.path.isfile(path):
            log(f"[model] 找不到权重文件 {path}（当前目录 {os.getcwd()}）")
            if name == "marker":
                return None
            continue
        log(f"[model] 加载 {name}: {path}")
        models[name] = YOLO(path)
        names = getattr(models[name], "names", {})
        log(f"[model] {name} 类别: " + ", ".join(f"{k}={v}" for k, v in sorted(names.items())))
    if "marker" not in models:
        return None
    return models


# ============================================================================
# 7. 命令行 / 主循环
# ============================================================================
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description="CQuick 视觉考核：足球检测 + 场地单目自定位")
    ap.add_argument("--camera", type=int, default=None, help="摄像头序号（默认自动尝试）")
    ap.add_argument("--video", default=None, help="用视频文件代替相机（调试用）")
    ap.add_argument("--loop", action="store_true", help="视频文件循环播放")
    ap.add_argument("--save", default=None, help="把左右分屏结果写成视频文件")
    ap.add_argument("--headless", action="store_true", help="不显示窗口（配合 --save 离线跑）")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧后退出（0=不限）")
    ap.add_argument("--no-ball", action="store_true", help="不加载足球模型")
    ap.add_argument("--no-refine", action="store_true", help="关闭标志几何细化（用 bbox 中心）")
    ap.add_argument("--no-track", action="store_true",
                    help="关闭标志短轨迹确认（默认开启：压闪烁/瞬时误检/类别跳变）")
    ap.add_argument("--map-scale", type=float, default=1.0,
                    help="地图坐标整体缩放系数（外参/地图尺度不一致时临时修正，默认 1.0）")
    ap.add_argument("--min-hits", type=int, default=1,
                    help="标志连续出现几帧才参与定位（默认 1 = 出现即用；想压误检可设 2）")
    ap.add_argument("--no-auto-lock", action="store_true",
                    help="关闭多解自动锁定（默认开启：多帧多解时自动选最优候选，按 c 可改）")
    ap.add_argument("--ball-model", default="ball_best.pt")
    ap.add_argument("--marker-model", default="marker_best.pt")
    ap.add_argument("--intrinsics", default=first_existing(INTRINSICS_CANDIDATES))
    ap.add_argument("--field", default=None, help="field.yaml 路径（默认 config/field.yaml）")
    ap.add_argument("--extrinsics", default=first_existing(EXTRINSICS_CANDIDATES),
                    help="相机安装外参 yaml（默认 config/camera_extrinsics.yaml）")
    ap.add_argument("--height", type=float, default=None,
                    help="相机高度 m（覆盖 yaml；默认读 camera_extrinsics.yaml）")
    ap.add_argument("--pitch", type=float, default=None,
                    help="相机俯仰角 deg（覆盖 yaml；默认读 camera_extrinsics.yaml）")
    ap.add_argument("--init", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                    help="初始先验位姿（米/米/度），用于多解消歧；不给则按 c 选择")
    ap.add_argument("--conf-ball", type=float, default=BALL_CONF)
    ap.add_argument("--conf-marker", type=float, default=MARKER_CONF)
    ap.add_argument("--min-inliers", type=int, default=MATCH_CFG["min_inliers"])
    ap.add_argument("--max-residual", type=float, default=MATCH_CFG["max_residual"])
    ap.add_argument("--yaml-out", default="pose_output.yaml",
                    help="把 camera_pose_map 结果按任务书字段写成 YAML（默认 pose_output.yaml）")
    ap.add_argument("--no-yaml-out", action="store_true", help="不写 YAML 输出文件")
    return ap.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    # ---- 内参 ----
    intr = mm.load_intrinsics(args.intrinsics)
    if intr is None:
        log(f"[calib] 读不到内参文件 {args.intrinsics}，退出。")
        return 1
    ref_w = intr["image_width"] or PROC_W
    ref_h = intr["image_height"] or PROC_H
    K = scale_intrinsics(intr["K"], (ref_w, ref_h), (PROC_W, PROC_H))
    dist = intr["dist"]
    log(f"[calib] {intr['source']} 标定分辨率 {ref_w}x{ref_h} -> 处理 {PROC_W}x{PROC_H}")
    log(f"[calib] fx={K[0,0]:.2f} fy={K[1,1]:.2f} cx={K[0,2]:.2f} cy={K[1,2]:.2f}")
    log(f"[calib] dist=[{', '.join(f'{v:.5f}' for v in dist)}]  (k1,k2,p1,p2,k3)")
    if len(dist) > 4 and abs(float(dist[4])) > 2.0:
        log("[warn] |k3|>2，标定很可能不稳定：建议补更多棋盘格图片重标，"
            "并使用 cv2.CALIB_FIX_K3 / cv2.CALIB_ZERO_TANGENT_DIST。")

    # ---- 外参：命令行 > config/camera_extrinsics.yaml > 内置默认值 ----
    # 注意：训练集图片是手持随手拍的，不能用它们反推 height/pitch；
    # 这里只接受"录视频时实际安装状态"下的测量值。
    extr = mm.load_extrinsics(args.extrinsics)
    height, pitch = args.height, args.pitch
    height_src = "命令行" if height is not None else ""
    pitch_src = "命令行" if pitch is not None else ""
    if extr is not None:
        if height is None and extr.get("height") is not None:
            height, height_src = extr["height"], extr["source"]
        if pitch is None and extr.get("pitch_deg") is not None:
            pitch, pitch_src = extr["pitch_deg"], extr["source"]
        if args.height is None or args.pitch is None:
            log(f"[extr] 读取外参文件 {extr['source']}")
    elif not os.path.isfile(args.extrinsics):
        log(f"[extr] 未找到外参文件 {args.extrinsics}，使用内置默认值；"
            f"录视频前请按实际安装测量 height/pitch（可用 o 反推、w 保存）")
    if height is None:
        height, height_src = CAMERA_HEIGHT, "内置默认"
    if pitch is None:
        pitch, pitch_src = PITCH_DEGREE, "内置默认"
    log(f"[extr] CAMERA_HEIGHT={height:.3f} m (来源:{height_src})  "
        f"PITCH_DEGREE={pitch:.2f} deg (来源:{pitch_src})  ROLL=0 deg")
    log("[extr] 相机光心地面投影 = 载体参考原点，故 carrier_pose == camera_pose；"
        "外参须在录视频时的实际安装状态下测量")
    projector = GroundProjector(K, dist, height, pitch)
    # 用当前外参算一下画面顶部/中心/底部落在多远的地面上，方便一眼判断 height/pitch 是否合理：
    # 顶部没有交点 = 地平线在画面内，说明 pitch 太小、或 height 与 pitch 组合不对。
    for label, vv in (("top", 0), ("center", PROC_H // 2), ("bottom", PROC_H - 1)):
        gg = projector.project_many([(K[0, 2], vv)], filter_range=False)[0]
        if gg is None:
            log(f"[extr] 画面{label:6s} -> 地面：无交点（在地平线以上）")
        else:
            log(f"[extr] 画面{label:6s} -> 地面：前方 {gg[0]:5.2f} m，左向 {gg[1]:+5.2f} m")

    # ---- 地图 ----
    if args.field:
        markers, map_src = mm.load_field([args.field])
    else:
        markers, map_src = mm.load_field()
    if not markers:
        log("[map] 地图为空，退出。")
        return 1
    # 地图整体缩放：现场实测发现"观测间距 / 地图间距"不是 1.00 时，
    # 先用 --map-scale 临时对齐，确认是地图尺度问题后再固化进 field.yaml。
    if abs(float(args.map_scale) - 1.0) > 1e-9:
        k = float(args.map_scale)
        markers = [dict(m, x=m["x"] * k, y=m["y"] * k) for m in markers]
        log(f"[map] 坐标整体缩放 x{k:.3f}（仅本次运行，不改文件）")

    # ---- 模型 / 相机 ----
    models = load_models(args)
    if models is None:
        return 1
    cap = open_capture(args)
    if cap is None:
        return 1

    writer = None
    if args.save:
        fourcc = cv2.VideoWriter_fourcc(*"XVID")
        writer = cv2.VideoWriter(args.save, fourcc, 15.0, (PROC_W * 2, PROC_H))
        if not writer.isOpened():
            log(f"[save] 无法创建视频文件 {args.save}")
            writer = None

    # ---- 预热：第一次推理要 1 s 左右（加载 kernel / 分配内存），
    #      先跑一次空帧，避免录屏第一帧卡住、FPS 从 1 开始 ----
    t_warm = time.perf_counter()
    warm_frame = np.zeros((PROC_H, PROC_W, 3), dtype=np.uint8)
    for name, mdl in models.items():
        try:
            mdl.predict(warm_frame, verbose=False, imgsz=640)
        except Exception as exc:  # 预热失败不影响正式运行
            log(f"[model] {name} 预热失败（忽略）：{exc}")
    log(f"[model] 预热完成，耗时 {(time.perf_counter() - t_warm) * 1000.0:.0f} ms")

    # ---- 状态 ----
    tracker = PoseTracker(alpha_single=ALPHA_SINGLE, alpha_multi=ALPHA_MULTI,
                          max_jump=MAX_JUMP_M, max_yaw_jump=MAX_YAW_JUMP_DEG,
                          ttl=PRIOR_TTL_FRAMES)
    det_tracker = None if args.no_track else DetTracker(min_hits=max(1, int(args.min_hits)))
    log(f"[track] 标志短轨迹确认：{'OFF' if det_tracker is None else 'ON'}"
        f"（连续 2 帧出现且类别投票 >=60% 才参与定位）")
    if args.init:
        tracker.reset(tuple(args.init))
        log(f"[init] 初始先验 x={args.init[0]:.2f} y={args.init[1]:.2f} "
            f"yaw={args.init[2]:.2f} deg（仅用于多解消歧）")

    refine_on = REFINE_ENABLED and not args.no_refine
    conf_marker = float(args.conf_marker)
    half_fov = math.degrees(math.atan((PROC_W / 2.0) / K[0, 0]))

    cfg = dict(MATCH_CFG)
    cfg["min_inliers"] = int(args.min_inliers)
    cfg["max_residual"] = float(args.max_residual)

    class_remap = {}
    if "marker" in models:
        for cid, name in getattr(models["marker"], "names", {}).items():
            canon = mm.class_id_from_name(name)
            if canon is None and int(cid) in mm.CLASS_NAMES:
                canon = int(cid)   # 名字认不出就按模型原始类别号，避免整类被丢掉
            if canon is not None:
                class_remap[int(cid)] = canon
    if not class_remap:
        class_remap = {0: 0, 1: 1, 2: 2, 3: 3}
    log(f"[model] 模型类别 -> 地图类别映射: {class_remap}")

    map_view = MapView(markers, PROC_W, PROC_H, source=map_src)

    frame_idx = 0
    t_prev = time.time()
    fps_ema = 0.0
    loc_events = []
    pose_hist = []
    loc_hz = 0.0
    last_print = 0.0
    last_yaml = 0.0
    last_yaml_print = 0.0
    fail_reads = 0
    ambiguous_frames = 0
    auto_lock = not args.no_auto_lock
    hint = "c cycle | l lock | x reset | o fit h/pitch"
    hint_until = 0.0
    unknown_marker_names = set()
    refine_error_shown = False
    match_error_shown = False
    yaml_path = None if args.no_yaml_out else args.yaml_out
    yaml_warn = {}
    result = mm.estimate_pose([], markers)
    yaml_txt = pose_yaml_text(None, result, 0, map_src, projector, False, 0)
    if yaml_path:
        log(f"[out] camera_pose_map 结果将实时写入 {os.path.abspath(yaml_path)}")

    log("[run] 开始运行。q 退出；c 切换候选；l 锁定；x 重置先验；o 反推外参。")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            fail_reads += 1
            if args.video and args.loop:
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            if args.video or fail_reads > 30:
                log("[camera] 取帧失败，结束。")
                break
            time.sleep(0.01)
            continue
        fail_reads = 0
        frame_idx += 1

        if frame.shape[1] != PROC_W or frame.shape[0] != PROC_H:
            frame = cv2.resize(frame, (PROC_W, PROC_H), interpolation=cv2.INTER_LINEAR)

        t_now = time.time()
        dt = max(1e-6, t_now - t_prev)
        t_prev = t_now
        fps_ema = (1.0 / dt) if fps_ema <= 0 else (0.9 * fps_ema + 0.1 * (1.0 / dt))

        # ---------------- 足球检测 ----------------
        ball_dets = []
        infer_ms = 0.0
        t0 = time.perf_counter()
        if "ball" in models:
            res_ball = models["ball"].predict(frame, conf=float(args.conf_ball),
                                              verbose=False, imgsz=640)[0]
            if res_ball is not None and res_ball.boxes is not None:
                for box in res_ball.boxes:
                    conf = float(box.conf[0])
                    xyxy = [float(v) for v in box.xyxy[0].tolist()]
                    ball_dets.append({
                        "conf": conf,
                        "xyxy": xyxy,
                        "center": ((xyxy[0] + xyxy[2]) / 2.0, (xyxy[1] + xyxy[3]) / 2.0),
                    })
        infer_ms += (time.perf_counter() - t0) * 1000.0

        # ---------------- 场地标志检测 ----------------
        t0 = time.perf_counter()
        marker_boxes = []
        marker_names = getattr(models["marker"], "names", {}) or {}
        res_marker = models["marker"].predict(frame, conf=conf_marker,
                                               verbose=False, imgsz=640)[0]
        if res_marker is not None and res_marker.boxes is not None:
            for box in res_marker.boxes:
                raw_cls = int(box.cls[0])
                canon = class_remap.get(raw_cls)
                if canon is None and raw_cls in mm.CLASS_NAMES:
                    # 模型类别名没认出来时，退回按模型原始类别号使用，绝不静默丢掉检测
                    canon = raw_cls
                label = str(marker_names.get(raw_cls, f"cls{raw_cls}"))
                if canon is None:
                    if label not in unknown_marker_names:
                        unknown_marker_names.add(label)
                        log(f"[model] 类别 '{label}' 不在标准 L/T/X/P 里，已跳过（"
                            f"检查模型 names 是否与 marker_data.yaml 一致）")
                    continue
                marker_boxes.append({
                    "cls": canon,
                    "raw_cls": raw_cls,
                    "label": label,
                    "conf": float(box.conf[0]),
                    "xyxy": [float(v) for v in box.xyxy[0].tolist()],
                })
        infer_ms += (time.perf_counter() - t0) * 1000.0

        # ---------------- 几何细化 + 像素->地面 ----------------
        dets = []
        for det in marker_boxes:
            try:
                u, v, how = refine_marker_point(frame, det["xyxy"], det["cls"], refine_on)
            except Exception as exc:   # 细化绝不能把整帧搞崩
                if not refine_error_shown:
                    refine_error_shown = True
                    log(f"[refine] 几何细化异常，改用 bbox 中心：{exc}")
                x1, y1, x2, y2 = det["xyxy"]
                u, v, how = (x1 + x2) / 2.0, (y1 + y2) / 2.0, "bbox"
            det["u"], det["v"], det["how"] = u, v, how
            dets.append(det)

        # ---------------- 短轨迹确认：压闪烁 / 瞬时误检 / 类别跳变 ----------------
        if det_tracker is not None:
            det_tracker.update(dets)
        else:
            for det in dets:
                det.update(confirmed=True, trusted=True, voted_cls=det["cls"],
                           vote_margin=1.0, smoothed_uv=(det["u"], det["v"]),
                           track_hits=1)

        # 用平滑后的特征点重新投影地面；只有"连续出现 + 类别投票明确"的检测参与定位
        grounds = projector.project_many([det["smoothed_uv"] for det in dets])
        obs_pairs = []
        obs_for_match = []
        for det, g in zip(dets, grounds):
            det["ground"] = g
            det["match_cls"] = det["voted_cls"]
            det["obs_idx"] = None
            if g is None:
                continue
            obs_pairs.append((det, g))
            if det["trusted"]:
                det["obs_idx"] = len(obs_for_match)
                obs_for_match.append({"cls": det["match_cls"],
                                      "x": g[0], "y": g[1], "conf": det["conf"]})

        # 兜底：如果轨迹确认把点筛掉了、导致不足 2 个可用观测，
        # 但仍有 >=2 个检测投影到了地面，就直接用它们 —— 宁可噪声大，也不能一个坐标都不给。
        if len(obs_for_match) < 2 and len(obs_pairs) >= 2:
            obs_for_match = []
            for k, (d, g) in enumerate(obs_pairs):
                d["obs_idx"] = k
                obs_for_match.append({"cls": d["match_cls"], "x": g[0], "y": g[1],
                                      "conf": d["conf"]})

        # ---------------- 地图匹配 ----------------
        # 只有 1~2 个观测时，把先验搜索范围收紧：错误匹配对通常落在远处，
        # 门限放宽会让"另一个解"被当成真值，表现为数值跳变。
        cfg_frame = cfg
        if len(obs_for_match) <= 2:
            cfg_frame = dict(cfg, prior_gate=min(float(cfg["prior_gate"]), 0.80))
        t0 = time.perf_counter()
        try:
            result = mm.estimate_pose(obs_for_match, markers,
                                      last_pose=tracker.prior, cfg=cfg_frame)
        except Exception as exc:   # 匹配模块异常时降级为"无效"，不要让程序退出
            if not match_error_shown:
                match_error_shown = True
                log(f"[match] 位姿估计异常（本帧按无效处理）：{exc}")
            result = mm.estimate_pose([], markers)
        match_ms = (time.perf_counter() - t0) * 1000.0

        pose, miss, just_reset = tracker.update(result)
        fresh = bool(result["valid"])

        # 多解自动锁定：相机固定时，若多帧都是"同样好的多个解"且没有先验，
        # 自动选最优候选作为先验（否则会一直只显示候选、不出坐标）。
        # 注意：现场经常出现"1 个标志 / 2 个标志"交替，所以这里只累计 ambiguous 帧，
        #      不因为中间夹了几帧 insufficient 就清零，否则永远等不到锁定。
        if result.get("status") == "ambiguous" and tracker.prior is None:
            ambiguous_frames += 1
        elif fresh:
            ambiguous_frames = 0
        if (auto_lock and ambiguous_frames >= AUTO_LOCK_FRAMES
                and result.get("groups") and not result.get("degraded")):
            g = result["groups"][0]
            # 只允许锁到"离当前位姿不远"的候选，否则可能是镜像解，宁可不锁
            near_ok = (tracker.pose is None or
                       math.hypot(g["x"] - tracker.pose[0],
                                  g["y"] - tracker.pose[1]) <= MAX_JUMP_M)
            if near_ok:
                tracker.reset((g["x"], g["y"], g["yaw_deg"]))
                pose = tracker.pose
                ambiguous_frames = 0
                hint = (f"auto-lock ({g['x']:.2f},{g['y']:.2f}) - c to change")
                log(f"[auto] 多解自动锁定候选 x={g['x']:.2f} y={g['y']:.2f} "
                    f"yaw={g['yaw_deg']:.1f}；不对就按 c 切换")
            else:
                ambiguous_frames = 0   # 候选离太远（疑似镜像），本轮不锁

        # 无效原因：区分"检测都没确认"和"匹配失败"，方便现场判断
        why_txt = str(result.get("reason") or "")
        if len(obs_for_match) < 2 and len(obs_pairs) >= 2:
            why_txt = f"{len(obs_pairs)} markers detected, waiting track confirm"
        if fresh:
            loc_events.append(t_now)
            pose_hist.append((t_now, pose[0], pose[1], pose[2]))
        loc_events = [t for t in loc_events if t_now - t < 2.0]
        pose_hist = [h for h in pose_hist if t_now - h[0] <= 2.0][-60:]
        loc_hz = len(loc_events) / 2.0

        # ---------------- 左侧：相机画面 ----------------
        left = frame.copy()
        # 足球：类别 / 置信度 / bbox / 像素中心（任务书 1.7 要求的全部字段）
        for b in ball_dets:
            x1, y1, x2, y2 = [int(round(v)) for v in b["xyxy"]]
            cu, cvv = b["center"]
            cv2.rectangle(left, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(left, (int(cu), int(cvv)), 3, (0, 255, 0), -1)
            put_text(left, f"ball {b['conf']:.2f}", (x1, max(14, y1 - 8)), 0.48,
                     (0, 255, 0), 1)
            put_text(left, f"box({x1},{y1},{x2},{y2})", (x1, min(PROC_H - 6, y2 + 15)),
                     0.36, (0, 255, 0), 1)
            put_text(left, f"center({cu:.0f},{cvv:.0f})", (x1, min(PROC_H - 6, y2 + 30)),
                     0.36, (0, 255, 0), 1)

        # 观测索引 -> 匹配到的地图点（位姿无效时匹配关系只是候选，加 ? 前缀提示）
        obs_mapname = {}
        for m in result.get("matches", []):
            if 0 <= m["map"] < len(markers):
                obs_mapname[m["obs"]] = (markers[m["map"]]["name"], m["dist"])
        pose_trusted = bool(result.get("valid"))
        prefix = "" if pose_trusted else "?"

        # 每一个检测都必须画出框：地面投影失败（太近/太远/朝地平线）也要画，
        # 只是换个颜色并写明原因，避免"明明检测到了却像没识别出来"。
        for det in dets:
            x1, y1, x2, y2 = [int(round(v)) for v in det["xyxy"]]
            g = det.get("ground")
            color = mm.CLASS_COLORS.get(det["cls"], (255, 0, 0))
            box_color = color if g is not None else (0, 140, 255)
            cv2.rectangle(left, (x1, y1), (x2, y2), box_color, 2)

            # bbox 中心（白点，供对比）与几何细化后的特征点（彩色十字）
            bcx, bcy = int(round((x1 + x2) / 2.0)), int(round((y1 + y2) / 2.0))
            cv2.circle(left, (bcx, bcy), 2, (200, 200, 200), -1)
            su, sv = det.get("smoothed_uv", (det["u"], det["v"]))
            cu, cvv = int(round(su)), int(round(sv))
            shift = math.hypot(det["u"] - bcx, det["v"] - bcy)
            cv2.drawMarker(left, (cu, cvv), color, cv2.MARKER_CROSS, 12, 1)
            cv2.circle(left, (cu, cvv), 2, (255, 255, 255), -1)

            # 轨迹状态：new=只出现 1 帧（不参与定位）；?cls=类别投票不明确；
            # 若投票类别与模型给出不同，显示 ->X/T/L
            tag = ""
            if not det.get("confirmed", True):
                tag = " new"
            elif not det.get("trusted", True):
                tag = " ?cls"
            elif det.get("match_cls") != det["cls"]:
                tag = f" ->{mm.CLASS_SHORT.get(det['match_cls'], '?')}"
            put_text(left, f"{det['label']} {det['conf']:.2f} "
                           f"[{det['how']} {shift:.0f}px]{tag}",
                     (x1, max(14, y1 - 8)), 0.42, box_color, 1)

            line2 = f"pix({su:.0f},{sv:.0f})"
            if g is None:
                line2 += "  no ground"
            else:
                line2 += f"  g({g[0]:.2f},{g[1]:.2f})m"
                if det["obs_idx"] is not None and det["obs_idx"] in obs_mapname:
                    name, dres = obs_mapname[det["obs_idx"]]
                    line2 += f" -> {prefix}{name} {dres:.2f}m"
            put_text(left, line2, (x1, min(PROC_H - 6, y2 + 16)), 0.34, box_color, 1)

        # 左侧 HUD
        counts = {c: 0 for c in sorted(set(class_remap.values()))}
        for det in dets:
            counts[det["cls"]] = counts.get(det["cls"], 0) + 1
        count_txt = " ".join(f"{mm.CLASS_SHORT.get(c, '?')}:{n}" for c, n in counts.items())
        hud_y = 24
        hud = [
            (f"FPS {fps_ema:5.1f}   infer {infer_ms:5.1f} ms   match {match_ms:4.1f} ms",
             (220, 220, 220)),
            (f"res {PROC_W}x{PROC_H}  frame {frame_idx}  {time.strftime('%Y-%m-%d %H:%M:%S')}",
             (200, 200, 200)),
            (f"ball {len(ball_dets)}   det {len(dets)}  gnd {len(obs_pairs)}  "
             f"used {len(obs_for_match)}  cand {len(result.get('groups', []))}  "
             f"inl {result.get('marker_count', 0)}", (200, 200, 200)),
            (f"L/T/X detected: {count_txt}   conf>{conf_marker:.2f}   "
             f"refine {'ON' if refine_on else 'OFF'}   track "
             f"{'ON' if det_tracker is not None else 'OFF'}", (200, 200, 200)),
        ]
        # 尺度诊断：第一对观测的间距 vs 地图里最近的同类标志对间距。
        # 外参已实测，所以这里明显不是 1.00 就说明地图坐标（或安装）与现场不符。
        pair_txt, pair_col = "-", (200, 220, 200)
        pk = pick_obs_pair(obs_for_match)
        if pk is not None:
            oi, oj, d_obs = pk
            d_map = nearest_map_pair_distance(markers, obs_for_match[oi]["cls"],
                                              obs_for_match[oj]["cls"], d_obs)
            if d_map:
                ratio = d_obs / d_map
                pair_txt = f"obs {d_obs:.2f}m / map {d_map:.2f}m = x{ratio:.2f}"
                pair_col = (120, 255, 120) if abs(ratio - 1.0) <= 0.10 else (0, 200, 255)
        hud.append((f"pair1-2: {pair_txt}", pair_col))
        # 位姿稳定度：最近 2 s 平滑后位姿的标准差（数值"稳不稳"看这一行）
        stab_txt, stab_col = "-", (200, 220, 200)
        if len(pose_hist) >= 5:
            px = [h[1] for h in pose_hist]
            py = [h[2] for h in pose_hist]
            pyaw = [h[3] for h in pose_hist]
            y0 = pyaw[0]
            pyaw = [y0 + wrap_deg(v - y0) for v in pyaw]
            sx, sy = float(np.std(px)), float(np.std(py))
            syaw = float(np.std(pyaw))
            stab_txt = f"x+-{sx:.2f} y+-{sy:.2f} yaw+-{syaw:.1f}"
            stab_col = ((120, 255, 120) if (sx < 0.05 and sy < 0.05 and syaw < 2.0)
                        else (0, 200, 255))
        hud.append((f"pose std 2s: {stab_txt}", stab_col))
        c = (0, 255, 0) if result.get("valid") else (0, 0, 255)
        res = result.get("residual", float("inf"))
        res_txt = "--" if not math.isfinite(res) else f"{res:.2f} m"
        st_txt = str(result.get("status", "-"))
        if result.get("degraded"):
            st_txt += " / best-effort"
        hud.append((f"status {st_txt}   residual {res_txt}", c))
        for text, col in hud:
            put_text(left, text, (10, hud_y), 0.46, col, 1)
            hud_y += 22
        # 无效原因（为什么没有位姿）：直接给一句话，省得猜
        why = why_txt
        if why and not result.get("valid"):
            put_text(left, f"why: {why[:78]}", (10, hud_y), 0.38, (0, 165, 255), 1)
            hud_y += 20

        # ---------------- 右侧：地图 ----------------
        # 状态提示优先于按键提示（按键提示保留 2.5 s）
        if t_now > hint_until:
            st = result.get("status")
            if st == "ambiguous":
                hint = "AMBIGUOUS - c cycle | l lock"
            elif st == "baseline_too_short":
                hint = "markers too close (<0.6 m)"
            elif st == "insufficient_markers":
                hint = "need >= 2 markers in view"
            elif dets and not obs_pairs:
                hint = "no ground: adjust pitch with , ."
            elif result.get("valid") and result.get("marker_count", 0) <= 2:
                hint = "2 markers: yaw sensitive, need 3+"
            elif result.get("valid"):
                hint = "c cycle | l lock | x reset | o fit"
            elif len(obs_for_match) >= 2:
                hint = "no map pair matches - check field.yaml vs pair1-2"
            else:
                hint = "not converged - move / more markers"
        raw_pose = (result["x"], result["y"]) if result.get("valid") else None
        best_cand = result["groups"][0] if (pose is None and result.get("groups")) else None
        right = map_view.render(result, pose, fresh, miss, half_fov, fps_ema, loc_hz,
                                hint, raw_pose=raw_pose, best_cand=best_cand)

        final_show = np.hstack((left, right))
        if writer is not None:
            writer.write(final_show)
        if not args.headless:
            try:
                cv2.imshow("CQuick Vision System  (left: camera | right: field map)",
                           final_show)
                key = cv2.waitKey(1) & 0xFF
            except cv2.error:
                log("[gui] 窗口已关闭，退出。")
                break
        else:
            key = 0xFF

        # ---------------- 结果输出：任务书 2.1 字段（YAML 文件 + 终端） ----------------
        yaml_txt = pose_yaml_text(pose, result, frame_idx, map_src, projector, fresh, miss)
        if yaml_path and (t_now - last_yaml > 0.5):
            last_yaml = t_now
            write_text_file(yaml_path, yaml_txt, yaml_warn)

        # ---------------- 终端输出（约 1 Hz） ----------------
        if t_now - last_print > 1.0:
            last_print = t_now
            head = f"[{frame_idx:5d}] fps={fps_ema:4.1f} infer={infer_ms:4.1f}ms " \
                   f"match={match_ms:4.1f}ms"
            if ball_dets:
                for i, b in enumerate(ball_dets[:3]):
                    head += (f" | ball{i + 1} conf={b['conf']:.2f} "
                             f"box=({b['xyxy'][0]:.0f},{b['xyxy'][1]:.0f},"
                             f"{b['xyxy'][2]:.0f},{b['xyxy'][3]:.0f}) "
                             f"center=({b['center'][0]:.0f},{b['center'][1]:.0f})")
            else:
                head += " | ball 0"
            log(head)
            for det in dets:
                g = det.get("ground")
                if g is None:
                    gtxt = "no_ground"
                else:
                    gtxt = f"g=({g[0]:.2f},{g[1]:.2f})m"
                    if det["obs_idx"] is not None and det["obs_idx"] in obs_mapname:
                        nm, dd = obs_mapname[det["obs_idx"]]
                        gtxt += f" -> {prefix}{nm} d={dd:.2f}m"
                ttxt = (f"trk={det.get('track_hits', 1)} "
                        f"vote={det.get('vote_margin', 1.0):.2f}")
                if not det.get("confirmed", True):
                    ttxt += " NEW"
                elif not det.get("trusted", True):
                    ttxt += " UNTRUSTED"
                elif det.get("match_cls") != det["cls"]:
                    ttxt += f" ->{mm.CLASS_SHORT.get(det['match_cls'], '?')}"
                if not det["trusted"]:
                    ttxt += " (skipped)"
                log(f"        {det['label']:<13s} conf={det['conf']:.2f} "
                    f"box=({det['xyxy'][0]:.0f},{det['xyxy'][1]:.0f},"
                    f"{det['xyxy'][2]:.0f},{det['xyxy'][3]:.0f}) "
                    f"pix=({det['u']:.0f},{det['v']:.0f})[{det['how']}] {ttxt} {gtxt}")
            res = result.get("residual", float("inf"))
            res_txt = "--" if not math.isfinite(res) else f"{res:.2f}m"
            pose_txt = ("pose=--" if pose is None else
                        f"x={pose[0]:6.2f} y={pose[1]:6.2f} yaw={pose[2]:7.1f}")
            stale_txt = "" if fresh else f" stale={miss}"
            log(f"        {pose_txt} markers={result.get('marker_count', 0)} "
                f"residual={res_txt} valid={result.get('valid')} "
                f"status={result.get('status')}{stale_txt} loc={loc_hz:.1f}Hz")
            if why_txt and not result.get("valid"):
                log(f"        why: {why_txt}")

            # 尺度自检：只有在相机按外参固定安装时才有意义。
            # 训练集图片是手持随手拍的，用它们看这行没有意义。
            pair = None
            matches = result.get("matches", [])
            if result.get("valid") and len(matches) >= 2:
                # 已有可信位姿：直接用匹配上的地图点对，结论最可靠
                for ia in range(len(matches)):
                    for ib in range(ia + 1, len(matches)):
                        oa = obs_for_match[matches[ia]["obs"]]
                        ob = obs_for_match[matches[ib]["obs"]]
                        d_obs = math.hypot(ob["x"] - oa["x"], ob["y"] - oa["y"])
                        ma_ = markers[matches[ia]["map"]]
                        mb_ = markers[matches[ib]["map"]]
                        d_map = math.hypot(mb_["x"] - ma_["x"], mb_["y"] - ma_["y"])
                        if d_obs > 0.3:
                            pair = (d_obs, d_map, "matched pair")
                            break
                    if pair:
                        break
            elif len(obs_for_match) >= 2:
                # 位姿还不可信：只能猜"间距最接近的同类点对"，仅供参考
                pk = pick_obs_pair(obs_for_match)
                if pk is not None:
                    oi, oj, d_obs = pk
                    best = nearest_map_pair_distance(markers, obs_for_match[oi]["cls"],
                                                     obs_for_match[oj]["cls"], d_obs)
                    if best:
                        pair = (d_obs, best, "guessed pair")

            if pair:
                d_obs, d_map, how = pair
                ratio = d_obs / d_map if d_map > 1e-6 else float("inf")
                implied_h = projector.height * d_map / d_obs if d_obs > 1e-6 else 0.0
                if abs(ratio - 1.0) <= 0.10:
                    verdict = "尺度一致(±10%)"
                else:
                    verdict = (f"尺度不一致：若 height={projector.height:.2f}m 正确，"
                               f"地图里这对标志的间距应约 {d_obs:.2f} m（当前 {d_map:.2f} m）；"
                               f"若地图正确，height 应约 {implied_h:.2f} m")
                log(f"        [scale] {how}: obs={d_obs:.2f}m map={d_map:.2f}m "
                    f"ratio={ratio:.2f} -> {verdict}（仅在相机按外参固定安装时有效）")

            # 每 5 s 打一次任务书示例格式的 camera_pose_map 块
            if t_now - last_yaml_print > 5.0:
                last_yaml_print = t_now
                log(yaml_txt.rstrip("\n"))

        # ---------------- 按键 ----------------
        hint_before = hint
        if key in (ord("q"), 27):
            break
        elif key == ord("c"):
            groups = result.get("groups") or []
            if not groups:
                log("[key] 当前没有候选位姿，无法切换。")
            else:
                if tracker.prior is None:
                    cur = -1
                else:
                    ds = [math.hypot(g["x"] - tracker.prior[0], g["y"] - tracker.prior[1])
                          for g in groups]
                    cur = int(np.argmin(ds))
                nxt = (cur + 1) % len(groups)
                g = groups[nxt]
                tracker.reset((g["x"], g["y"], g["yaw_deg"]))
                hint = (f"candidate {nxt + 1}/{len(groups)} locked: "
                        f"({g['x']:.2f},{g['y']:.2f},{g['yaw_deg']:.1f})")
                log("[key] " + hint)
        elif key == ord("l"):
            if result.get("groups"):
                g = result["groups"][0]
                tracker.reset((g["x"], g["y"], g["yaw_deg"]))
                hint = "best candidate locked as prior"
                log("[key] 已锁定当前最优候选为先验。")
        elif key == ord("x"):
            tracker.clear_prior()
            hint = "prior cleared - global re-search (press c to pick)"
            log("[key] 清空先验，重新全局搜索（多解时按 c 选择）。")
        elif key == ord("r"):
            refine_on = not refine_on
            hint = f"geometric refine {'ON' if refine_on else 'OFF'}"
            log(f"[key] 几何细化：{'开' if refine_on else '关'}")
        elif key == ord("o"):
            log("[fit] 正在网格搜索相机高度/俯仰角 ...")
            dets_ground = [d for d in dets if d.get("ground") is not None]
            fit = fit_height_pitch(projector, dets_ground, markers, tracker.prior, cfg)
            if fit is not None:
                h_best, p_best, cnt, res_best = fit
                cur_key = (result.get("marker_count", 0), -result.get("residual", 1e9))
                if (cnt, -res_best) >= cur_key:
                    projector.set_params(height=h_best, pitch_deg=p_best)
                    hint = f"extrinsics set h={h_best:.2f} p={p_best:.1f} (w save)"
                    log(f"[key] 已应用新外参：h={h_best:.3f} m p={p_best:.2f} deg，"
                        f"按 w 写回 {args.extrinsics}")
                else:
                    log("[key] 反推结果没有优于当前外参，保持原值。")
        elif key == ord("w"):
            if save_extrinsics(args.extrinsics, projector.height,
                               projector.pitch_deg, ROLL_DEGREE):
                hint = f"saved h={projector.height:.2f} p={projector.pitch_deg:.1f}"
                log(f"[key] 外参已写入 {os.path.abspath(args.extrinsics)}"
                    f"（下次启动自动读取）")
            else:
                hint = "save extrinsics failed"
                log(f"[key] 写外参失败：{args.extrinsics}")
        elif key == ord("s"):
            fn = f"shot_{time.strftime('%Y%m%d_%H%M%S')}.png"
            cv2.imwrite(fn, final_show)
            log(f"[key] 已保存画面：{fn}")
        elif key in (ord("+"), ord("=")):
            conf_marker = min(0.95, conf_marker + 0.05)
            hint = f"marker conf {conf_marker:.2f}"
        elif key == ord("-"):
            conf_marker = max(0.05, conf_marker - 0.05)
            hint = f"marker conf {conf_marker:.2f}"
        elif key == ord("["):
            projector.set_params(height=max(0.1, projector.height - 0.02))
            hint = f"height {projector.height:.2f} m"
        elif key == ord("]"):
            projector.set_params(height=projector.height + 0.02)
            hint = f"height {projector.height:.2f} m"
        elif key == ord(","):
            projector.set_params(pitch_deg=projector.pitch_deg - 1.0)
            hint = f"pitch {projector.pitch_deg:.1f} deg"
        elif key == ord("."):
            projector.set_params(pitch_deg=projector.pitch_deg + 1.0)
            hint = f"pitch {projector.pitch_deg:.1f} deg"

        if hint != hint_before:
            hint_until = t_now + 2.5

        if args.max_frames and frame_idx >= args.max_frames:
            log(f"[run] 已处理 {frame_idx} 帧，退出。")
            break

    cap.release()
    if writer is not None:
        writer.release()
    if not args.headless:
        cv2.destroyAllWindows()
    if yaml_path:
        write_text_file(yaml_path, yaml_txt, yaml_warn)
        log(f"[out] 最终 camera_pose_map 已写入 {os.path.abspath(yaml_path)}")
    log(f"[run] 结束：共 {frame_idx} 帧。最终外参 "
        f"CAMERA_HEIGHT={projector.height:.3f}  PITCH_DEGREE={projector.pitch_deg:.2f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
