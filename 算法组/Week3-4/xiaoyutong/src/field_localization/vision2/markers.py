"""场地标志检测：从白线里找 L / T / X 交点，再找罚球点。

思路（不训练模型也能先跑通整条链路）：
    1. 白色像素分割：草坪是绿的（饱和度高），白线饱和度低、亮度高
    2. 霍夫变换找线段，只留够长的
    3. 用线段的"端点关系"判交点，这正好对应三种标志的几何定义：
         X = 两条线段在各自中间交叉
         T = 一条线段的端点，落在另一条线段的中间
         L = 两条线段端点靠在一起，夹角接近 90°
    4. 罚球点 = 小而圆、实心的白斑
    5. 最后用"沿四条胳膊探测白色是否延续"复核一遍，去掉误检

已知局限（写进 README）：
    * 大角度斜视、线很脏、强光过曝时会漏检或误检；
    * 像素精度受线宽影响（粗线 1~2 像素）；
    * 遮挡严重时不如 YOLO —— 所以最终还是要换成训练好的 YOLO 权重。
"""

import math

import cv2
import numpy as np

PERP_TOL = 4      # 侧向容忍（像素）
ARM_MIN = 0.5     # 胳膊延续比例阈值


def white_mask(bgr, v_min=135, s_max=80):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    s, v = hsv[:, :, 1], hsv[:, :, 2]
    mask = ((v >= v_min) & (s <= s_max)).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return mask


def _segments(mask, min_len=45, max_gap=10, thresh=45):
    edges = cv2.Canny(mask, 60, 160)
    segs = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=thresh,
                           minLineLength=min_len, maxLineGap=max_gap)
    if segs is None:
        return []
    out = []
    for x1, y1, x2, y2 in segs.reshape(-1, 4):
        p1 = np.array([x1, y1], float)
        p2 = np.array([x2, y2], float)
        if np.linalg.norm(p2 - p1) < min_len:
            continue
        out.append((p1, p2))
    return out


def _angle_between(d1, d2):
    c = abs(np.dot(d1, d2) / (np.linalg.norm(d1) * np.linalg.norm(d2) + 1e-9))
    return math.degrees(math.acos(min(1.0, c)))


def _point_seg_dist(p, a, b):
    """点 p 到线段 ab 的距离，以及投影参数 t（0=在 a，1=在 b）。"""
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < 1e-9:
        return float(np.linalg.norm(p - a)), 0.0
    t = float((p - a) @ ab) / L2
    t_cl = min(1.0, max(0.0, t))
    proj = a + t_cl * ab
    return float(np.linalg.norm(p - proj)), t


def _ray_alive(mask, p, d, length=40, tol=PERP_TOL):
    """从 p 沿方向 d 探测白色覆盖率。"""
    h, w = mask.shape[:2]
    n = np.linalg.norm(d)
    if n < 1e-9:
        return 0.0
    d = d / n
    perp = np.array([-d[1], d[0]])
    hit = total = 0
    for t in range(5, length + 1, 3):
        q = np.asarray(p, float) + d * t
        x, y = int(round(q[0])), int(round(q[1]))
        if not (0 <= x < w and 0 <= y < h):
            continue
        total += 1
        ok = False
        for o in range(-tol, tol + 1, 2):
            xo, yo = int(round(q[0] + perp[0] * o)), int(round(q[1] + perp[1] * o))
            if 0 <= xo < w and 0 <= yo < h and mask[yo, xo] > 0:
                ok = True
                break
        hit += 1 if ok else 0
    return hit / max(total, 1)


def _verify(mask, p, dirs, want_arms):
    """复核：沿这些方向白色是否延续，返回真正的胳膊数。"""
    arms = 0
    for d in dirs:
        if _ray_alive(mask, p, d) > ARM_MIN:
            arms += 1
    return arms


def find_junctions(bgr, mask, min_len=45):
    segs = _segments(mask, min_len=min_len)
    if len(segs) < 2:
        return []

    h, w = mask.shape[:2]
    cands = []

    # --- X：两段在各自中间交叉
    for i in range(len(segs)):
        a1, b1 = segs[i]
        for j in range(i + 1, len(segs)):
            a2, b2 = segs[j]
            if _angle_between(b1 - a1, b2 - a2) < 30:
                continue
            d1, d2 = b1 - a1, b2 - a2
            A = np.array([[d1[0], -d2[0]], [d1[1], -d2[1]]])
            if abs(np.linalg.det(A)) < 1e-9:
                continue
            t, u = np.linalg.solve(A, a2 - a1)
            if 0.12 < t < 0.88 and 0.12 < u < 0.88:
                p = a1 + t * d1
                if 6 <= p[0] < w - 6 and 6 <= p[1] < h - 6:
                    cands.append((p, "X"))

    # --- T：一段的端点落在另一段中间
    for i in range(len(segs)):
        a1, b1 = segs[i]
        for j in range(len(segs)):
            if i == j:
                continue
            a2, b2 = segs[j]
            if _angle_between(b1 - a1, b2 - a2) < 30:
                continue
            for p_end in (a1, b1):
                d, t = _point_seg_dist(p_end, a2, b2)
                if d < 10 and 0.15 < t < 0.85:
                    cands.append((p_end, "T"))

    # --- L：两段端点靠在一起，夹角接近直角
    ends = []
    for a, b in segs:
        ends.append((a, -1, b - a))
        ends.append((b, +1, a - b))
    for i in range(len(ends)):
        p1, _, d1 = ends[i]
        for j in range(i + 1, len(ends)):
            p2, _, d2 = ends[j]
            if np.linalg.norm(p1 - p2) > 12:
                continue
            ang = _angle_between(d1, d2)
            if 60 <= ang <= 120:
                cands.append(((p1 + p2) / 2.0, "L"))

    # --- 复核 + 去重
    out = []
    for p, typ in cands:
        dirs = []
        for a, b in segs:
            d = b - a
            dist, _ = _point_seg_dist(p, a, b)
            if dist < 12:
                dirs += [d / (np.linalg.norm(d) + 1e-9), -d / (np.linalg.norm(d) + 1e-9)]
        if not dirs:
            continue
        arms = _verify(mask, p, dirs, {"L": 2, "T": 3, "X": 4}[typ])
        need = {"L": 2, "T": 3, "X": 4}[typ]
        if arms < need:
            continue
        conf = min(1.0, 0.45 + 0.14 * arms)
        out.append({"type": typ, "u": float(p[0]), "v": float(p[1]), "conf": conf, "arms": arms})

    # 近邻去重：同一位置保留置信度最高、类别优先级 X>T>L
    prio = {"X": 3, "T": 2, "L": 1}
    out.sort(key=lambda c: (-prio[c["type"]], -c["conf"]))
    kept = []
    for c in out:
        if all(math.hypot(c["u"] - k["u"], c["v"] - k["v"]) > 20 for k in kept):
            kept.append(c)
    return kept


def find_penalty_point(bgr, mask):
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        area = cv2.contourArea(c)
        if not (80 <= area <= 4000):
            continue
        per = cv2.arcLength(c, True)
        if per <= 0:
            continue
        circ = 4 * math.pi * area / (per * per)
        (x, y), r = cv2.minEnclosingCircle(c)
        fill = area / (math.pi * r * r + 1e-9)
        if circ > 0.75 and fill > 0.65:
            out.append({"type": "P", "u": float(x), "v": float(y),
                        "conf": float(min(1.0, 0.5 + circ * 0.4)), "arms": 0})
    return out


def detect(bgr, want=("L", "T", "X", "P"), min_conf=0.4):
    """检测一张图里的所有标志，返回 [{'type','u','v','conf','arms'}]。"""
    mask = white_mask(bgr)
    dets = []
    if any(t in want for t in ("L", "T", "X")):
        dets += find_junctions(bgr, mask)
    if "P" in want:
        dets += find_penalty_point(bgr, mask)
    return [d for d in dets if d["conf"] >= min_conf]


def draw(bgr, dets):
    vis = bgr.copy()
    palette = {"L": (0, 200, 255), "T": (80, 220, 80), "X": (255, 140, 60), "P": (0, 0, 255)}
    for d in dets:
        c = palette.get(d["type"], (255, 255, 255))
        p = (int(round(d["u"])), int(round(d["v"])))
        cv2.circle(vis, p, 8, c, 2, cv2.LINE_AA)
        cv2.drawMarker(vis, p, c, cv2.MARKER_CROSS, 14, 2)
        cv2.putText(vis, "%s %.2f" % (d["type"], d["conf"]), (p[0] + 11, p[1] - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1, cv2.LINE_AA)
    return vis
