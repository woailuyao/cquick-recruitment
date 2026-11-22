# -*- coding: utf-8 -*-
"""
单目场地定位：像素 -> 地面坐标 -> 与 field.yaml 匹配 -> 相机在地图中的位姿
（CQuick 招新考核 · 任务二 的最后一环）

坐标系
    世界/相机系：X 向前（水平）、Y 向左、Z 向上；地面 Z=0；相机光心 (0,0,h)
    相机系：x 右、y 下、z 前（OpenCV / ROS optical frame）
    pitch：相机相对水平面低头角度，向下为正（装机后固定）
    注意：这里没有 yaw 参数。机器人扭头是正常的，朝向由 solve_pose() 解出来。

本版新增
    1) field.yaml 兜底解析：没有 PyYAML 也能读，自动去掉行尾注释和引号
    2) 自动配对 solve_pose_auto()：field.yaml 里没有 id、同类标志有多个时，
       枚举"两个观测 × 两个地图点"提位姿，用内点数/残差投票，不需要人工指定编号
    3) --demo --yaml config/field.yaml：用真实场地坐标仿真，先看能不能解出位姿

用法
    python task2_localization.py --check
    python task2_localization.py --pitch 378,366,1.60,0.25 --res 640x480
        # 相机正对 +X 时，点一个已用卷尺量过的地面点，反解俯仰角
    python task2_localization.py --demo
    python task2_localization.py --demo --yaml config/field.yaml
    python task2_localization.py --yaml config/field.yaml            # 接相机，左键点标志
    python task2_localization.py --yaml config/field.yaml --res 640x480

实时按键（不带 --yaml 时只做像素->地面读数）
    左键   点标志与地面的接触点（不是框中心），点完自动尝试解位姿
    c      清空观测
    q      退出
"""
import math
import re
import sys
import time
from itertools import combinations

import cv2
import numpy as np

# ========================= 1. 相机参数 =========================
DESIGN_W, DESIGN_H = 1280, 720      # 内参对应的分辨率
FX, FY = 1600.13, 1611.72
CX, CY = 684.35, 333.26
DIST = np.array([0.1870, -0.9677, -0.0073, 0.0041, 0.4217], dtype=float)  # k1,k2,p1,p2,k3

STUDENT_NAME = "刘勰渊 20263312"   # 视频水印：姓名+学号（改这里）

CAM_HEIGHT = 0.73   # 相机光心离地高度 (m)
PITCH_DEG = 19.0    # 俯仰角，向下为正 (deg)。用 --pitch 标定，标定后把这里改成实测值

_K_BASE = np.array([[FX, 0.0, CX],
                    [0.0, FY, CY],
                    [0.0, 0.0, 1.0]], dtype=float)
_ACTIVE_K = _K_BASE.copy()


def set_frame_size(width, height):
    """实际分辨率与内参标定分辨率不同时，按比例缩放内参（DIST 与分辨率无关）。"""
    global _ACTIVE_K
    K = _K_BASE.copy()
    sx, sy = width / float(DESIGN_W), height / float(DESIGN_H)
    K[0, 0] *= sx
    K[0, 2] *= sx
    K[1, 1] *= sy
    K[1, 2] *= sy
    _ACTIVE_K = K
    return K


# ========================= 2. 像素 <-> 地面 =========================
def R_wc(pitch_deg=PITCH_DEG):
    """相机系 -> 相机水平系（X 前、Y 左、Z 上）的旋转矩阵，只含俯仰。"""
    t = math.radians(pitch_deg)
    st, ct = math.sin(t), math.cos(t)
    return np.array([[0.0, -st, ct],
                     [-1.0, 0.0, 0.0],
                     [0.0, -ct, -st]])


def undistort_norm(u, v):
    """像素 -> 去畸变归一化坐标 (x_n, y_n)，y 向下为正。"""
    pts = np.array([[[float(u), float(v)]]], dtype=np.float32)
    n = cv2.undistortPoints(pts, _ACTIVE_K, DIST)
    return float(n[0, 0, 0]), float(n[0, 0, 1])


def pixel_to_ground(u, v, height=None, pitch_deg=None):
    """像素 -> 相机系地面点 (X 前, Y 左)，m；地平线以上返回 (None, None)。"""
    height = CAM_HEIGHT if height is None else height
    pitch_deg = PITCH_DEG if pitch_deg is None else pitch_deg
    x_n, y_n = undistort_norm(u, v)
    d = R_wc(pitch_deg) @ np.array([x_n, y_n, 1.0])
    if d[2] > -1e-6:                      # 射线朝天，打不到地面
        return None, None
    s = -height / d[2]
    return float(s * d[0]), float(s * d[1])


def ground_to_pixel(X, Y, height=None, pitch_deg=None):
    """相机系地面点 -> 像素；点在相机后方返回 None。"""
    height = CAM_HEIGHT if height is None else height
    pitch_deg = PITCH_DEG if pitch_deg is None else pitch_deg
    R_cw = R_wc(pitch_deg).T
    origin = np.array([0.0, 0.0, height])
    if (R_cw @ (np.array([X, Y, 0.0]) - origin))[2] <= 1e-6:
        return None
    rvec, _ = cv2.Rodrigues(R_cw)
    tvec = (-R_cw @ origin).reshape(3, 1)
    uv, _ = cv2.projectPoints(np.array([[[X, Y, 0.0]]]), rvec, tvec, _ACTIVE_K, DIST)
    return float(uv[0, 0, 0]), float(uv[0, 0, 1])


# ========================= 3. 位姿解算 =========================
def fit_pitch(u, v, X_true, Y_true, height=CAM_HEIGHT, lo=3.0, hi=70.0):
    """已知地面点(相机系)反解俯仰角，假设相机正对 +X（yaw=0）。

    做法：相机正对场地 +X，在正前方量一个点（X 前、Y 左），点它的像素，跑：
        python task2_localization.py --pitch 378,366,1.60,0.25 --res 640x480
    返回 (pitch_deg, 残差_m)。
    """
    best = None
    p = lo
    while p <= hi:
        X, Y = pixel_to_ground(u, v, height=height, pitch_deg=p)
        if X is not None:
            e = math.hypot(X - X_true, Y - Y_true)
            if best is None or e < best[1]:
                best = (p, e)
        p += 0.05
    return best


def _fit_rt(P, Q, scale=1.0):
    """最小二乘求 R,t（可带固定比例 scale）使 scale*R@P+t ≈ Q。"""
    P = np.asarray(P, dtype=float)
    Q = np.asarray(Q, dtype=float)
    pc, qc = P.mean(0) * scale, Q.mean(0)
    U, _, Vt = np.linalg.svd(((P - P.mean(0)) * scale).T @ (Q - Q.mean(0)))
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, d]) @ U.T
    return R, qc - R @ pc


def solve_pose(pairs, thresh=0.20, iters=300, seed=0):
    """已知配对时解位姿。pairs = [(obs_x, obs_y, map_x, map_y), ...]。

    返回 (x, y, yaw_deg, 内点数, 残差m)；点数不足返回 None。
    """
    if pairs is None or len(pairs) < 2:
        return None
    P = np.array([[p[0], p[1]] for p in pairs], float)
    Q = np.array([[p[2], p[3]] for p in pairs], float)
    n = len(P)

    def residual(R, t, idx):
        return np.linalg.norm(P[idx] @ R.T + t - Q[idx], axis=1)

    if n >= 3:
        rng = np.random.default_rng(seed)
        best = None
        for _ in range(iters):
            i, j = rng.choice(n, 2, replace=False)
            if np.linalg.norm(P[i] - P[j]) < 0.05:
                continue
            R, t = _fit_rt(P[[i, j]], Q[[i, j]])
            inl = np.where(residual(R, t, np.arange(n)) < thresh)[0]
            if len(inl) >= 2 and (best is None or len(inl) > len(best[2])):
                best = (R, t, inl)
        if best is None:
            return None
        R, t, inl = best
        R, t = _fit_rt(P[inl], Q[inl])
        inl = np.where(residual(R, t, np.arange(n)) < thresh)[0]
    else:
        R, t = _fit_rt(P, Q)
        inl = np.arange(n)

    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    rms = float(np.sqrt(np.mean(residual(R, t, inl) ** 2)))
    return float(t[0]), float(t[1]), yaw, int(len(inl)), rms


def solve_pose_auto(obs, markers, thresh=0.25, verbose=False):
    """不知道"每个观测对应哪个地图点"时自动配对并解位姿。

    obs     = [(X, Y), ...] 或 [(X, Y, type), ...]   相机系观测；type 可为 None
    markers = [(name, type, x, y), ...]               field.yaml 里的点
    返回 (x, y, yaw_deg, 内点数, 残差m, assign)
         assign = [(obs_index, marker_index), ...]  成功配上的那些
    思路：枚举"两个观测 × 两个地图点"直接算出位姿，再用全部观测投票，
          内点多、残差小的就是正确答案（同类标志多个也能自动区分）。
    """
    if len(obs) < 2 or len(markers) < 2:
        return None
    P = np.array([[o[0], o[1]] for o in obs], float)
    otype = [(o[2] if len(o) > 2 else None) for o in obs]
    M = np.array([[m[2], m[3]] for m in markers], float)
    mtype = [str(m[1]).strip("'\"").lower() for m in markers]

    def type_ok(i, k):
        t = otype[i]
        return t is None or str(t).strip("'\"").lower() == mtype[k]

    n, m = len(obs), len(markers)
    # 位姿合理性门限：相机应站在场地附近且大致朝着标志，
    # 否则会出现"数学上成立、物理上荒谬"的解（例如站到场外、朝反方向）
    x_lo, x_hi = float(M[:, 0].min()) - 1.2, float(M[:, 0].max()) + 1.2
    y_lo, y_hi = float(M[:, 1].min()) - 1.2, float(M[:, 1].max()) + 1.2

    def plausible(R_, t_):
        yaw_ = math.degrees(math.atan2(R_[1, 0], R_[0, 0]))
        return (x_lo <= t_[0] <= x_hi and y_lo <= t_[1] <= y_hi and abs(yaw_) <= 75.0)

    best = None
    for i, j in combinations(range(n), 2):
        d_obs = np.linalg.norm(P[i] - P[j])
        if d_obs < 0.1:
            continue
        for a, b in combinations(range(m), 2):
            if not (type_ok(i, a) and type_ok(j, b)):
                continue
            d_map = np.linalg.norm(M[a] - M[b])
            # 先按刚性（scale=1）试；不行再试"距离比"给出的缩放（应对 pitch/场图尺寸偏差）
            for s in (1.0,):
                if not (0.80 <= s <= 1.30):
                    continue
                if s == 1.0 and abs(d_obs - d_map) > max(0.35, 0.40 * d_map):
                    continue
                R, t = _fit_rt(P[[i, j]], M[[a, b]], s)
                PR = (P @ R.T) * s + t
                resid = []
                used = set()
                for k in range(n):
                    d = np.linalg.norm(PR[k] - M, axis=1)
                    order = np.argsort(d)
                    pick = None
                    for c in order:
                        if not type_ok(k, c) or c in used:
                            continue
                        pick = c
                        break
                    if pick is None:
                        resid.append(9.9)
                        continue
                    used.add(pick)
                    resid.append(float(np.linalg.norm(PR[k] - M[pick])))
                resid = np.array(resid)
                inl = int(np.sum(resid < thresh))
                score = (inl, -float(np.mean(resid[resid < thresh])) if inl else -9.9)
                if inl >= 2 and (best is None or score > best[0]) and plausible(R, t):
                    best = (score, R, t, s, resid)

    if best is None:
        return None
    _, R, t, s, resid = best
    inl = np.where(resid < thresh)[0]
    # 用内点重解一次
    pairs = []
    used = set()
    PR = (P @ R.T) * s + t
    for k in inl:
        d = np.linalg.norm(PR[k] - M, axis=1)
        for c in np.argsort(d):
            if type_ok(k, c) and c not in used:
                used.add(c)
                pairs.append((k, int(c)))
                break
    if len(pairs) < 2:
        return None
    R, t = _fit_rt(P[[p[0] for p in pairs]], M[[p[1] for p in pairs]], s)
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    res = np.array([np.linalg.norm((P[k] @ R.T) * s + t - M[c]) for k, c in pairs])
    if abs(s - 1.0) > 0.08:
        print(f"[warn] 尺度偏差 x{s:.2f}：观测距离与场图不成 1:1，"
              f"位姿会跟着缩水，先把 pitch 或场图尺寸校准")
    if verbose:
        for k, c in pairs:
            print(f"    观测#{k} ({P[k][0]:6.2f},{P[k][1]:6.2f}) <-> {markers[c][0]}"
                  f"({markers[c][1]}) ({M[c][0]:5.2f},{M[c][1]:5.2f})  残差 {res[list(pairs).index((k, c))]*100:.0f}cm")
    return float(t[0]), float(t[1]), yaw, len(pairs), float(np.sqrt(np.mean(res ** 2))), pairs


# ========================= 4. field.yaml 读取 =========================
def _clean(text):
    """去掉行尾 # 注释，避免注释文字被当成字段值。"""
    out = []
    for line in text.splitlines():
        cut = line.split("#", 1)[0]
        out.append(cut)
    return "\n".join(out)


def _parse_simple(text):
    """不装 PyYAML 时的兜底解析：按 '-' 分块，抓 type/id/x/y。"""
    out = []
    for block in re.split(r"^\s*-\s+", _clean(text), flags=re.M)[1:]:
        kv = dict(re.findall(r"([A-Za-z_]\w*)\s*:\s*([^,\n\}\u3000]+)", block))
        if "x" in kv and "y" in kv:
            try:
                typ = kv.get("type", kv.get("class", "?")).strip().strip("'\"")
                name = kv.get("id", kv.get("name", "")).strip().strip("'\"")
                out.append((name or f"{typ}#{len(out)}", typ,
                            float(kv["x"].strip().strip("'\"")),
                            float(kv["y"].strip().strip("'\""))))
            except ValueError:
                pass
    return out


def load_field(path):
    """读 field.yaml -> [(name, type, x, y), ...]。优先 PyYAML，失败用兜底解析。"""
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()

    pts = []
    try:
        import yaml
        data = yaml.safe_load(text)
        if isinstance(data, dict):
            for key in ("markers", "points", "features", "landmarks"):
                if isinstance(data.get(key), list):
                    data = data[key]
                    break
        if isinstance(data, list):
            for i, it in enumerate(data):
                if not isinstance(it, dict):
                    continue
                x = it.get("x", it.get("X"))
                y = it.get("y", it.get("Y"))
                if x is None or y is None:
                    continue
                typ = str(it.get("type", it.get("class", "?")))
                name = str(it.get("id", it.get("name", f"{typ}#{i}")))
                pts.append((name, typ, float(x), float(y)))
    except Exception:
        pts = []

    if not pts:
        pts = _parse_simple(text)
    if not pts:
        raise SystemExit(f"{path} 里没解析出任何特征点，请确认格式（需要 type/x/y）")
    return pts


# ========================= 5. 自检 / 仿真 =========================
def self_check():
    expect = CAM_HEIGHT / math.tan(math.radians(PITCH_DEG))
    cx_a, cy_a = float(_ACTIVE_K[0, 2]), float(_ACTIVE_K[1, 2])
    X, Y = pixel_to_ground(cx_a, cy_a)
    print(f"[check] 内参 cx={cx_a:.2f} cy={cy_a:.2f}（设计 {DESIGN_W}x{DESIGN_H}）")
    print(f"[check] 主点落地点距相机 {math.hypot(X, Y):.4f} m "
          f"(期望 h/tan(pitch)={expect:.4f} m)")

    rng = np.random.default_rng(3)
    lm = [(0.0, 0.0), (9.0, 0.0), (9.0, 6.0), (0.0, 6.0), (4.5, 3.0)]
    tx, ty, tyaw = 3.2, 1.7, 47.0
    c, s = math.cos(math.radians(tyaw)), math.sin(math.radians(tyaw))
    R = np.array([[c, -s], [s, c]])
    obs = (np.array(lm) - np.array([tx, ty])) @ R + rng.normal(0, 0.03, (5, 2))
    obs[2] += np.array([0.9, -0.6])
    pairs = [(o[0], o[1], m[0], m[1]) for o, m in zip(obs, lm)]
    r = solve_pose(pairs)
    ok = r is not None and abs(r[0] - tx) < 0.05 and abs(r[1] - ty) < 0.05 and abs(r[2] - tyaw) < 1
    print(f"[check] 已知配对解位姿: x={r[0]:.3f}/{tx} y={r[1]:.3f}/{ty} yaw={r[2]:.1f}/{tyaw} "
          f"内点={r[3]}/5 rms={r[4]*100:.1f}cm -> {'OK' if ok else 'FAIL'}")


def demo(markers=None):
    """地图坐标 -> 像素 -> 地面 -> 自动配对 -> 位姿，整条链跑一遍。"""
    set_frame_size(640, 480)
    rng = np.random.default_rng(7)
    if not markers:
        markers = [("T1", "T", 1.20, 0.00), ("L2", "L", 6.00, 1.50),
                   ("X3", "X", 6.00, 5.20), ("T4", "T", 1.20, 5.20),
                   ("P5", "penalty", 4.50, 2.80)]
    true = (1.10, 0.25, 33.0) if markers[0][2] < 5 else (3.10, 1.90, 41.0)

    print(f"机器人真值 x={true[0]} y={true[1]} yaw={true[2]}°   （共 {len(markers)} 个地图点）")
    obs = []
    for (name, typ, mx, my) in markers:
        c, s = math.cos(math.radians(-true[2])), math.sin(math.radians(-true[2]))
        dx, dy = mx - true[0], my - true[1]
        ox, oy = c * dx - s * dy, s * dx + c * dy      # 地图 -> 相机系
        if ox <= 0.5:
            continue
        uv = ground_to_pixel(ox, oy)
        if uv is None or not (0 <= uv[0] < 640 and 0 <= uv[1] < 480):
            continue
        uv = (uv[0] + rng.normal(0, 0.8), uv[1] + rng.normal(0, 0.8))   # 点击误差 ~1px
        gx, gy = pixel_to_ground(*uv)
        if gx is None:
            continue
        print(f"  {name:10s} 地图({mx:5.2f},{my:5.2f}) 像素({uv[0]:6.1f},{uv[1]:5.1f}) "
              f"反解({gx:5.2f},{gy:5.2f}) 误差 {math.hypot(gx-ox, gy-oy)*100:4.1f}cm")
        obs.append((gx, gy))

    if len(obs) < 2:
        print("可见标志不足 2 个：把相机朝向球门方向、或让 pitch 更小一点再试")
        return
    r = solve_pose_auto(obs, markers, verbose=True)
    if r is None:
        print("自动配对失败：观测之间的距离和地图上任何两点都对不上")
        return
    print(f"\n解出 x={r[0]:.3f} y={r[1]:.3f} yaw={r[2]:.2f}°  内点={r[3]}/{len(obs)}  "
          f"rms={r[4]*100:.1f} cm")
    print(f"误差 dx={r[0]-true[0]:+.3f} dy={r[1]-true[1]:+.3f} dyaw={r[2]-true[2]:+.2f}°")


# ========================= 6. 实时点选 + 匹配 =========================
def _render_map(pose, markers, size=(420, 480)):
    """右侧俯视小地图：场地线框 + 标志点 + 相机位置与朝向（对应考核要求的"右侧地图"）。"""
    W, Hh = size
    img = np.full((Hh, W, 3), 28, np.uint8)
    pad, field_h = 16, 392
    x_max, y_half = 3.9, 2.2
    s = min((W - 2 * pad) / x_max, (field_h - 2 * pad) / (2 * y_half))

    def to_px(x, y):
        return int(pad + x * s), int(field_h / 2 - y * s)

    corners = [(0, -2.0), (3.8, -2.0), (3.8, 2.0), (0, 2.0)]
    cv2.polylines(img, [np.array([to_px(x, y) for x, y in corners])], True, (0, 190, 0), 2)
    cv2.line(img, to_px(0, -2.0), to_px(0, 2.0), (0, 120, 0), 1)
    cv2.line(img, to_px(3.8, -2.0), to_px(3.8, 2.0), (0, 120, 0), 1)
    cv2.putText(img, "+X -> goal", to_px(2.2, 2.45), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                (0, 200, 0), 1, cv2.LINE_AA)

    colors = {"TCross": (255, 200, 0), "LCross": (0, 220, 255),
              "CenterPoint": (200, 200, 200), "PenaltyPoint": (200, 200, 200)}
    for (name, typ, mx, my) in (markers or [])[:40]:
        p = to_px(mx, my)
        if -50 < p[0] < W + 50 and -50 < p[1] < field_h + 50:
            cv2.circle(img, p, 3, colors.get(typ, (180, 180, 180)), -1)

    if pose is not None:
        c = to_px(pose[0], pose[1])
        yawr = math.radians(pose[2])
        tip = (int(c[0] + 26 * math.cos(yawr)), int(c[1] - 26 * math.sin(yawr)))
        left = (int(c[0] + 10 * math.cos(yawr + 2.5)), int(c[1] - 10 * math.sin(yawr + 2.5)))
        right = (int(c[0] + 10 * math.cos(yawr - 2.5)), int(c[1] - 10 * math.sin(yawr - 2.5)))
        cv2.fillPoly(img, [np.array([tip, left, right])], (60, 60, 255))
        cv2.circle(img, c, 3, (255, 255, 255), -1)

    txt = ["MAP VIEW (top-down)", "field 3.8 x 4.0 m",
           "pose: --" if pose is None else "x=%.2f y=%.2f" % (pose[0], pose[1]),
           "" if pose is None else "yaw=%.1f deg" % pose[2]]
    for i, t in enumerate(txt):
        cv2.putText(img, t, (12, field_h + 26 + 20 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (0, 255, 255), 1, cv2.LINE_AA)
    return img


def _write_pitch_to_file(path, pitch):
    """把 PITCH_DEG 直接写回本文件，省得手动改。成功返回 True。"""
    try:
        with open(path, "r", encoding="utf-8") as f:
            text = f.read()
        new, n = re.subn(r"^PITCH_DEG\s*=\s*[0-9.]+", f"PITCH_DEG = {pitch:.1f}",
                         text, count=1, flags=re.M)
        if not n:
            return False
        with open(path, "w", encoding="utf-8") as f:
            f.write(new)
        return True
    except Exception as e:
        print(f"[calib] 写文件失败: {e}")
        return False


def ros_setup():
    """可选：ROS2 发布。装了 ros2 且 source 过环境才生效，否则返回 None。"""
    try:
        import rclpy
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import String
        rclpy.init()
        node = rclpy.create_node("task2_localization")
        return (rclpy, node,
                node.create_publisher(Pose2D, "/task2/pose", 10),
                node.create_publisher(String, "/task2/status", 10))
    except Exception as e:                      # 没装 ROS2 就安静跳过
        print(f"[ros] 未启用（{type(e).__name__}）：{e}")
        return None


def ros_publish(ros, pose, n_feat):
    """发布 Pose2D(x, y, theta[rad]) + String(status)。"""
    if ros is None:
        return
    rclpy, node, pub_pose, pub_stat = ros
    try:
        from geometry_msgs.msg import Pose2D
        from std_msgs.msg import String
        if pose is None:
            pub_stat.publish(String(data="valid=false marker_count=%d" % n_feat))
        else:
            m = Pose2D()
            m.x = float(pose[0])
            m.y = float(pose[1])
            m.theta = float(math.radians(pose[2]))
            pub_pose.publish(m)
            pub_stat.publish(String(data="valid=true marker_count=%d residual=%.3f"
                                         % (pose[3], pose[4])))
        rclpy.spin_once(node, timeout_sec=0.0)
    except Exception as e:
        print(f"[ros] 发布失败: {e}")


def run_camera(markers, source=0, calib=False, use_ros=False):
    global PITCH_DEG
    ros = ros_setup() if use_ros else None
    if isinstance(source, int) or (isinstance(source, str) and source.isdigit()):
        idx = int(source)
        cap = cv2.VideoCapture(idx, getattr(cv2, "CAP_DSHOW", 0))
        if not cap.isOpened():
            cap = cv2.VideoCapture(idx)
    else:
        cap = cv2.VideoCapture(str(source))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频源: {source!r}")

    ok, first = cap.read()
    if not ok:
        raise SystemExit("读不到画面")
    fh, fw = first.shape[:2]
    if (fw, fh) != (DESIGN_W, DESIGN_H):
        set_frame_size(fw, fh)
        print(f"实际分辨率 {fw}x{fh}，已按比例缩放内参")

    win = "Field Localization"
    state = {"obs": [], "pose": None, "pending": None, "sel": None}

    if calib:
        print("=== 俯仰角标定模式 ===")
        print("1) 相机正对场地 +X（沿中线朝球门）")
        print("2) 在画面上左键点一个已用卷尺量过的地面点")
        print("3) 回到这个窗口，输入该点的  X,Y（米，例：1.60,0.25），回车")
        print("4) 程序自动算出 pitch 并写回文件，重启本程序生效")
        print()

    def solve_now():
        if len(state["obs"]) < 2:
            state["pose"] = None
            return
        if all(len(o) > 2 for o in state["obs"]):
            pairs = []
            for (x, y, mi) in state["obs"]:
                X, Y = pixel_to_ground(x, y)
                if X is not None:
                    pairs.append((X, Y, markers[mi][2], markers[mi][3]))
            r = solve_pose(pairs)
        else:
            pairs = []
            for o in state["obs"]:
                X, Y = pixel_to_ground(o[0], o[1])
                if X is not None:
                    pairs.append((X, Y))
            r = solve_pose_auto(pairs, markers)
        state["pose"] = r
        if ros is not None:
            ros_publish(ros, r, len(pairs))
        if r is None:
            print(f"[pose] valid=FALSE  marker_count={len(pairs)}  reason=匹配失败  "
                  f"pitch={PITCH_DEG:.1f}")
            return
        print(f"[pose] x_camera_map={r[0]:+.3f} m  y_camera_map={r[1]:+.3f} m  "
              f"yaw_camera_map={r[2]:+.2f} deg  marker_count={r[3]}  "
              f"residual={r[4]*100:.1f} cm  valid=TRUE  pitch={PITCH_DEG:.1f}")

    def on_mouse(event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if calib:
            state["pending"] = (x, y)
            print(f"[calib] 已选像素 ({x},{y})，回到终端输入实测 X,Y")
            return
        X, Y = pixel_to_ground(x, y)
        if X is None:
            print(f"[skip] ({x},{y}) 在地平线以上")
            return
        if state["sel"] is not None:
            mi = state["sel"]
            state["obs"].append((x, y, mi))
            print(f"[obs] {markers[mi][0]}({markers[mi][1]}) 像素({x},{y}) -> "
                  f"相机系({X:.2f},{Y:.2f})，共 {len(state['obs'])} 个")
        else:
            state["obs"].append((x, y))
            print(f"[obs] 像素({x},{y}) -> 相机系({X:.2f},{Y:.2f})，共 {len(state['obs'])} 个")
        if markers:
            solve_now()

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    if markers:
        print(f"已加载 {len(markers)} 个地图点。两种用法：")
        print("  ① 直接左键点（自动配对）")
        print("  ② 先按数字键选中标志，再点（配对确定，最稳）")
        for i, m in enumerate(markers[:9]):
            print(f"     [{i+1}] {m[0]}({m[1]}) 地图({m[2]:.2f}, {m[3]:.2f})")
    else:
        print("没有 field.yaml，只做像素->地面读数（建议加 --yaml config/field.yaml）")
    print("左键点标志底部，c 清空，q 退出")

    while True:
        if source and not isinstance(source, int) and not str(source).isdigit():
            frame = first.copy()
        else:
            ok, frame = cap.read()
            if not ok:
                break
        view = frame.copy()

        r = state["pose"]
        names = {i: markers[c][0] for i, c in r[5]} if r is not None else {}
        for k, (u, v) in enumerate(state["obs"]):
            col = (0, 0, 255) if r is not None else (0, 200, 0)
            cv2.drawMarker(view, (u, v), col, cv2.MARKER_CROSS, 16, 2)
            if k < len(names):
                pass
            if k in names:
                cv2.putText(view, names[k], (u + 8, v - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)

        lines = [f"obs={len(state['obs'])}  pitch={PITCH_DEG:.1f} deg",
                 "x_camera_map = -- m" if r is None else
                 f"x_camera_map = {r[0]:+.3f} m",
                 "y_camera_map = -- m" if r is None else
                 f"y_camera_map = {r[1]:+.3f} m",
                 "yaw_camera_map = -- deg" if r is None else
                 f"yaw_camera_map = {r[2]:+.2f} deg",
                 "valid = FALSE (need >=2 features)" if r is None else
                 f"valid = TRUE   marker_count = {r[3]}   residual = {r[4]*100:.1f} cm"]
        _bg = view.copy()
        cv2.rectangle(_bg, (2, 2), (560, 20 + 22 * len(lines) + 6), (0, 0, 0), -1)
        view = cv2.addWeighted(_bg, 0.62, view, 0.38, 0)
        for i, s in enumerate(lines):
            col = (0, 0, 255) if (i >= 4 and r is None) else (0, 255, 255)
            cv2.putText(view, s, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(view, s, (10, 24 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        col, 1, cv2.LINE_AA)

        mview = _render_map(r, markers)
        if mview.shape[0] != view.shape[0]:
            mview = cv2.resize(mview, (int(mview.shape[1] * view.shape[0] / mview.shape[0]),
                                       view.shape[0]))
        view = np.hstack([view, mview])

        # 防伪水印：姓名 + 实时时间（每秒刷新，能证明是实时拍摄）
        stamp = f"{STUDENT_NAME}   {time.strftime('%Y-%m-%d %H:%M:%S')}"
        cv2.putText(view, stamp, (12, view.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(view, stamp, (12, view.shape[0] - 14), cv2.FONT_HERSHEY_SIMPLEX,
                    0.65, (255, 255, 255), 1, cv2.LINE_AA)

        cv2.imshow(win, view)
        k = cv2.waitKey(1) & 0xFF

        if k in (ord("+"), ord("="), ord("-"), ord("_")):
            PITCH_DEG += 0.5 if k in (ord("+"), ord("=")) else -0.5
            print(f"[pitch] {PITCH_DEG:.1f}°  重新解算 {len(state['obs'])} 个观测")
            if markers and state["obs"]:
                solve_now()

        if calib and state["pending"] is not None:
            u, v = state["pending"]
            state["pending"] = None
            try:
                s = input(f"[calib] 像素({u},{v}) 的实测地面坐标 X,Y（米，如 1.60,0.25）: ")
                vals = [t for t in re.split(r"[,，、;；\s]+", s.strip()) if t]
                if len(vals) < 2:
                    raise ValueError("需要两个数，用逗号或空格分开，例如 2.82,0.64")
                Xt, Yt = float(vals[0]), float(vals[1])
                p, err = fit_pitch(u, v, Xt, Yt)
                print(f"[calib] 拟合 pitch = {p:.2f}°   残差 = {err*100:.1f} cm")
                X1, Y1 = pixel_to_ground(u, v, pitch_deg=p)
                print(f"[calib] 该像素 -> X={X1:.3f} Y={Y1:.3f}（目标 {Xt}, {Yt}）")
                if err > 0.15:
                    print("[calib] 残差偏大：相机没正对 +X 或有 yaw，或这一点量错了，换个点再来")
                elif _write_pitch_to_file(__file__, p):
                    print(f"[calib] 已写回 {__file__} 的 PITCH_DEG，关掉重开生效")
                else:
                    print(f"[calib] 自动写文件失败，手动把第 47 行改成 PITCH_DEG = {p:.1f}")
            except Exception as e:
                print(f"[calib] 输入有误（要两个数 X,Y）：{e}")

        if k == ord("q"):
            break
        elif k == ord("c"):
            state["obs"].clear()
            state["pose"] = None
        elif ord("1") <= k <= ord("9"):
            state["sel"] = k - ord("1")
            if state["sel"] < len(markers):
                m = markers[state["sel"]]
                print(f"[sel] 下一个点 = #{state['sel']+1} {m[0]}({m[1]}) 地图({m[2]:.2f},{m[3]:.2f})")
            else:
                print("[sel] 编号超出地图点数")
                state["sel"] = None

    cap.release()
    cv2.destroyAllWindows()


# ========================= 7. 入口 =========================
def main():
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(encoding="utf-8", errors="replace")
        except Exception:
            pass

    args = sys.argv[1:]
    yaml_path = None
    if "--yaml" in args:
        i = args.index("--yaml")
        yaml_path = args[i + 1]
        del args[i:i + 2]

    res = None
    if "--res" in args:
        i = args.index("--res")
        res = args[i + 1]
        del args[i:i + 2]

    calib_flag = "--calib" in args
    if calib_flag:
        args.remove("--calib")
    use_ros_flag = "--ros2" in args
    if use_ros_flag:
        args.remove("--ros2")

    if res:
        w, h = (int(t) for t in res.lower().split("x"))
        set_frame_size(w, h)
        print(f"内参按 {w}x{h} 缩放（设计分辨率 {DESIGN_W}x{DESIGN_H}）")

    if args and args[0] == "--pitch":
        # --pitch 像素u,像素v,实测X,实测Y  （相机需正对 +X）
        u, v, Xt, Yt = (float(t) for t in args[1].split(","))
        p, err = fit_pitch(u, v, Xt, Yt)
        print(f"拟合 pitch = {p:.2f}°   残差 = {err*100:.1f} cm")
        X1, Y1 = pixel_to_ground(u, v, pitch_deg=p)
        print(f"  该像素 -> X={X1:.3f} Y={Y1:.3f}  (目标 {Xt}, {Yt})")
        if err > 0.15:
            print("  残差偏大：多半是相机没正对 +X（有 yaw），或这一点量错了。"
                  "先把相机对准中线再量一个点。")
        else:
            print(f"  把文件顶部 PITCH_DEG 改成 {p:.1f} 即可")
        return

    markers = None
    if yaml_path:
        markers = load_field(yaml_path)
        xs = [m[2] for m in markers]
        ys = [m[3] for m in markers]
        print(f"已加载 {len(markers)} 个地图点：{yaml_path}")
        print(f"  地图范围 X {min(xs):.2f}~{max(xs):.2f} m  Y {min(ys):.2f}~{max(ys):.2f} m"
              f"（对照实测场地尺寸看一眼是否合理）")

    if "--check" in args:
        self_check()
        return
    if "--demo" in args:
        demo(markers)
        return

    src = args[0] if args else 0
    src = int(src) if isinstance(src, str) and src.isdigit() else src
    if calib_flag:
        run_camera(markers or [], src, calib=True)
        return
    run_camera(markers or [], src, use_ros=use_ros_flag)


if __name__ == "__main__":
    main()

















