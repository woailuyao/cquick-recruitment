# -*- coding: utf-8 -*-
"""手动点选 + 白线中心线细化，得到 L/T/X 标志的"真实交点"像素坐标。

考核细则 2.3 要求：不能直接把检测框中心当交点，要做几何细化。

做法（按胶带"中心线"而不是边缘求交）：
    你在交点附近点一下 -> 取 120x120 邻域
    -> HSV 白色掩膜（胶带的整条像素，不是边缘）
    -> 在掩膜像素上做 RANSAC，提取 2 条直线（掩膜的两侧边缘都在内点里，
       所以拟合出来天然就是胶带中心线）
    -> 两中心线求交 = 交点（亚像素）

只采纳同时满足下面 4 条的细化结果，否则原样返回点击位置：
    1) 找到两条直线，各自内点足够多
    2) 两条线夹角 >= 25°
    3) 交点在图像内
    4) 交点离点击位置 <= 35 px（太远说明找错了线）

用法：
    python pick_marker.py 图片.png 输出.json
"""
import json
import math
import sys

import cv2
import numpy as np

from imgio import imread

TYPE_KEYS = {"1": "L", "2": "T", "3": "X"}
RNG = np.random.default_rng(12345)


def _fit_line(pts):
    """PCA 拟合直线，返回 (a, b, c)：a*x + b*y + c = 0。"""
    P = np.asarray(pts, dtype=np.float64)
    if len(P) < 2:
        return None
    c = P.mean(axis=0)
    u, s, vt = np.linalg.svd(P - c, full_matrices=False)
    d = vt[0]
    n = np.array([-d[1], d[0]])
    nn = np.linalg.norm(n)
    if nn < 1e-9:
        return None
    n = n / nn
    return float(n[0]), float(n[1]), float(-(n @ c))


def _intersect(l1, l2):
    a1, b1, c1 = l1
    a2, b2, c2 = l2
    det = a1 * b2 - a2 * b1
    if abs(det) < 1e-9:
        return None
    x = (b1 * c2 - b2 * c1) / det
    y = (c1 * a2 - c2 * a1) / det
    return float(x), float(y)


def _angle_between(l1, l2):
    """两条直线的夹角（度）。平行=0，垂直=90。"""
    a1, b1 = l1[0], l1[1]
    a2, b2 = l2[0], l2[1]
    dot = abs(a1 * a2 + b1 * b2)          # 单位法向点积：1=平行，0=垂直
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(math.acos(dot))


def _ransac_line(pts, tol=3.5, iters=400, min_inliers=80):
    """在点集上找一条支持度最高的直线，返回 (line, inlier_mask)。"""
    if len(pts) > 2200:                 # 点太多时降采样，速度提升数倍而精度几乎不变
        keep = np.arange(0, len(pts), 2)
        pts = pts[keep]
        sub = keep
    else:
        sub = None
    n = len(pts)
    if n < min_inliers:
        return None, None
    best = (-1, None, None)
    for _ in range(iters):
        i, j = RNG.integers(0, n, 2)
        p1, p2 = pts[i], pts[j]
        d = p2 - p1
        L = float(np.hypot(*d))
        if L < 30:                        # 两点太近，方向不可靠
            continue
        nv = np.array([-d[1], d[0]]) / L
        c = float(nv @ p1)
        inl = np.abs(pts @ nv - c) < tol
        cnt = int(inl.sum())
        if cnt > best[0]:
            best = (cnt, nv, c, inl)
    if best[0] < min_inliers:
        return None, None
    line = _fit_line(pts[best[3]])
    if line is None:
        return None, None
    inl = np.abs(pts @ np.array(line[:2]) + line[2]) < tol
    return line, inl


def refine_intersection(gray_bgr, u, v, win=60, max_shift=35.0, min_angle_deg=25.0, iters=800):
    """在 (u,v) 附近用白线中心线求交点。

    gray_bgr: 彩色图（用 HSV 做白色掩膜）或灰度图（退化用灰度阈值）
    返回 (x, y, how)，how ∈ {"center", "raw"}
    """
    h, w = gray_bgr.shape[:2]
    x0, y0 = max(0, int(u) - win), max(0, int(v) - win)
    x1, y1 = min(w, int(u) + win), min(h, int(v) + win)
    roi = gray_bgr[y0:y1, x0:x1]
    if roi.size == 0:
        return u, v, "raw"

    if roi.ndim == 3:
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        s, val = hsv[..., 1].astype(np.int32), hsv[..., 2].astype(np.int32)
        mask = ((s < 70) & (val > 150)).astype(np.uint8) * 255
    else:
        mask = (roi > 165).astype(np.uint8) * 255
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

    ys, xs = np.where(mask > 0)
    if len(xs) < 150:
        return u, v, "raw"
    pts = np.column_stack([xs, ys]).astype(np.float64)

    # 在掩膜像素上做 RANSAC：掩膜包含胶带两侧，拟合出来就是中心线
    l1, inl1 = _ransac_line(pts, iters=iters, min_inliers=120)
    if l1 is None:
        return u, v, "raw"
    # 关键：按"带宽"剔除整条胶带（不能只剔除 RANSAC 的窄内点，
    # 否则第二次找到的是同一条胶带的另一条边，两条线平行，求不出交点）
    n1 = np.array(l1[:2])
    band = np.abs(pts @ n1 + l1[2]) > 12.0
    rest = pts[band]
    if len(rest) < 60:
        return u, v, "raw"
    l2, _ = _ransac_line(rest, iters=max(120, iters // 2), min_inliers=50)
    if l2 is None:
        return u, v, "raw"

    if _angle_between(l1, l2) < min_angle_deg:
        return u, v, "raw"

    xy = _intersect(l1, l2)
    if xy is None:
        return u, v, "raw"
    xi, yi = xy[0] + x0, xy[1] + y0
    if not (0 <= xi < w and 0 <= yi < h):
        return u, v, "raw"
    if math.hypot(xi - u, yi - v) > max_shift:
        return u, v, "raw"
    return float(xi), float(yi), "center"


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        return
    img_path, out_json = sys.argv[1], sys.argv[2]

    img = imread(img_path)
    if img is None:
        print(f"[ERROR] 读不到 {img_path}")
        return

    state = {"type": "L", "points": []}
    disp = img.copy()

    def redraw():
        nonlocal disp
        disp = img.copy()
        for p in state["points"]:
            cv2.circle(disp, (int(p["u"]), int(p["v"])), 24, (0, 0, 255), 2)
            cv2.putText(disp, p["type"], (int(p["u"]) + 28, int(p["v"]) - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(disp, f"current type: {state['type']}  (1=L 2=T 3=X, s=save, q=quit)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("pick marker", disp)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            xi, yi, how = refine_intersection(img, x, y)
            state["points"].append({"type": state["type"], "u": xi, "v": yi, "how": how})
            print(f"  + {state['type']}  click=({x},{y}) -> refined=({xi:.2f},{yi:.2f}) [{how}]")
            redraw()

    cv2.namedWindow("pick marker", cv2.WINDOW_NORMAL)
    cv2.setMouseCallback("pick marker", on_mouse)
    redraw()

    while True:
        k = cv2.waitKey(30) & 0xFF
        if k in (ord(q) for q in TYPE_KEYS):
            state["type"] = TYPE_KEYS[chr(k)]
            redraw()
        elif k == ord("u") and state["points"]:
            state["points"].pop()
            redraw()
        elif k == ord("s"):
            with open(out_json, "w", encoding="utf-8") as f:
                json.dump({"image": img_path, "points": state["points"]}, f,
                          ensure_ascii=False, indent=2)
            print(f"[SAVE] {out_json}")
            break
        elif k == ord("q"):
            break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
