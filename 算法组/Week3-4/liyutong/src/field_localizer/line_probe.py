# -*- coding: utf-8 -*-
"""
线特征探针：检测场地白线 -> 聚类成无限直线 -> 编号可视化

用途
    用「场地白线的已知真实位置」独立反推相机高度，判定 0.390 与 0.450 谁对。
    这条路完全不依赖棋盘格，用的是场地自身的 8x6 m 实测尺寸。

用法
    python line_probe.py _probe/gt2_1_mid.jpg
    python line_probe.py _probe/gt2_1_mid.jpg --thr 170 --minlen 50

输出
    _probe/<name>_lines.jpg   每簇一种颜色 + 编号
    终端打印 每簇的 (id, 倾角, 距原点, 段数, 总长, 端点)
"""
import argparse
import os

import cv2
import numpy as np

OUT_DIR = "_probe"


def white_mask(bgr, v_thr=165, s_thr=60):
    """白线掩膜：亮且不彩。足球场是绿的（饱和度高），白漆线饱和度低。"""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    m = ((v >= v_thr) & (s <= s_thr)).astype(np.uint8) * 255
    # 闭运算把断线接起来，再细一点便于 Hough
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    return m


def seg_to_line(p1, p2):
    """线段两端点 -> 归一化齐次直线 (a, b, c)，a*x + b*y + c = 0，且 a^2+b^2=1"""
    x1, y1 = p1
    x2, y2 = p2
    a = y2 - y1
    b = x1 - x2
    c = x2 * y1 - x1 * y2
    n = np.hypot(a, b)
    if n < 1e-9:
        return None
    return np.array([a / n, b / n, c / n])


def line_angle(l):
    """直线方向角（度，0~180）。法向量 (a,b) 的角度 +90 就是方向角。"""
    ang = np.degrees(np.arctan2(l[1], l[0])) + 90.0
    return ang % 180.0


def same_line(l1, l2, dtheta=4.0, drho=14.0):
    """两条直线是否属于同一条场线：方向差小 且 一条线到另一条线的距离小"""
    a1 = line_angle(l1)
    a2 = line_angle(l2)
    d = abs(a1 - a2)
    d = min(d, 180.0 - d)
    if d > dtheta:
        return False
    # 检查两条线上各取两点互相到对方线的距离
    mids = []
    for l in (l1, l2):
        # 直线上的点：取法向量的垂足附近的前后两点，这里简化用 (a,b,c) 直接算
        # 选点 P = -c*(a,b) 为直线上离原点最近的点
        p = -l[2] * np.array([l[0], l[1]])
        mids.append(p)
    # 点到线距离
    d1 = abs(l1[0] * mids[1][0] + l1[1] * mids[1][1] + l1[2])
    d2 = abs(l2[0] * mids[0][0] + l2[1] * mids[0][1] + l2[2])
    return max(d1, d2) < drho


def cluster_lines(segs, dtheta=4.0, drho=14.0):
    """贪心聚类：每簇用总长度加权的直线参数，反复迭代 2 轮让参数收敛"""
    lines = []
    for p1, p2 in segs:
        l = seg_to_line(p1, p2)
        if l is not None:
            ln = float(np.hypot(p2[0] - p1[0], p2[1] - p1[1]))
            lines.append({"l": l, "p1": p1, "p2": p2, "len": ln})

    lines.sort(key=lambda d: -d["len"])          # 长段先当种子
    groups = []
    for item in lines:
        placed = False
        for g in groups:
            if same_line(g["l"], item["l"], dtheta, drho):
                g["items"].append(item)
                g["len"] += item["len"]
                placed = True
                break
        if not placed:
            groups.append({"l": item["l"].copy(), "items": [item],
                           "len": item["len"]})

    # 用簇内所有端点做总体最小二乘，重估直线
    for g in groups:
        pts = []
        for it in g["items"]:
            pts.append(it["p1"])
            pts.append(it["p2"])
        pts = np.array(pts, float)
        c = pts.mean(axis=0)
        u, s, vt = np.linalg.svd(pts - c)
        d = vt[0]                                   # 主方向
        nrm = np.array([-d[1], d[0]])
        a, b = nrm
        cc = -(a * c[0] + b * c[1])
        g["l"] = np.array([a, b, cc])
        g["points"] = pts
        g["range"] = (float(pts[:, 0].min()), float(pts[:, 0].max()),
                      float(pts[:, 1].min()), float(pts[:, 1].max()))

    groups.sort(key=lambda g: -g["len"])
    return groups


def draw(img, groups, path):
    pal = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (0, 255, 255), (255, 0, 255),
           (255, 255, 0), (0, 128, 255), (128, 0, 255), (0, 255, 128), (255, 128, 0),
           (128, 255, 0), (255, 0, 128), (0, 0, 128), (128, 128, 0), (0, 128, 128)]
    vis = img.copy()
    h, w = img.shape[:2]
    for i, g in enumerate(groups):
        col = pal[i % len(pal)]
        a, b, c = g["l"]
        # 取图像内两个远端点
        x0, x1 = 0, w - 1
        pts = []
        for x in (x0, x1):
            if abs(b) > 1e-6:
                y = -(a * x + c) / b
                if 0 <= y <= h:
                    pts.append((x, int(y)))
        for y in (0, h - 1):
            if abs(a) > 1e-6:
                x = -(b * y + c) / a
                if 0 <= x <= w:
                    pts.append((int(x), int(y)))
        if len(pts) >= 2:
            cv2.line(vis, pts[0], pts[-1], col, 1, cv2.LINE_AA)
        for it in g["items"]:
            cv2.line(vis, tuple(map(int, it["p1"])), tuple(map(int, it["p2"])), col, 2)
        # 编号标在簇重心处
        cp = g["points"].mean(axis=0)
        cv2.putText(vis, str(i), (int(cp[0]) + 4, int(cp[1]) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(vis, str(i), (int(cp[0]) + 4, int(cp[1]) - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, col, 2, cv2.LINE_AA)
    cv2.imwrite(path, vis)
    return path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("img")
    ap.add_argument("--thr", type=int, default=45, help="Hough 累加器阈值")
    ap.add_argument("--minlen", type=int, default=40, help="最短线段长度(px)")
    ap.add_argument("--maxgap", type=int, default=16, help="线段之间允许的最大断裂(px)")
    ap.add_argument("--v", type=int, default=165, help="白线亮度下限")
    ap.add_argument("--s", type=int, default=60, help="白线饱和度上限")
    ap.add_argument("--minlen-cluster", type=float, default=60.0,
                    help="簇总长小于该值的丢掉")
    args = ap.parse_args()

    img = cv2.imread(args.img)
    if img is None:
        print("读图失败", args.img)
        return
    h, w = img.shape[:2]
    mask = white_mask(img, args.v, args.s)
    cv2.imwrite(os.path.join(OUT_DIR, "mask_" + os.path.basename(args.img)), mask)

    seg = cv2.HoughLinesP(mask, 1, np.pi / 360, args.thr,
                          minLineLength=args.minlen, maxLineGap=args.maxgap)
    if seg is None:
        print("没检出线段，调低 --thr 或 --minlen")
        return
    segs = [((int(s[0][0]), int(s[0][1])), (int(s[0][2]), int(s[0][3]))) for s in seg]

    groups = cluster_lines(segs)
    groups = [g for g in groups if g["len"] >= args.minlen_cluster]

    print(f"{args.img}  {w}x{h}")
    print(f"线段 {len(segs)} 条 -> 聚类 {len(groups)} 簇")
    print(f"{'id':>3} {'倾角°':>7} {'rho':>7} {'段数':>5} {'总长':>7} "
          f"{'x范围':>13} {'y范围':>13}")
    for i, g in enumerate(groups):
        a, b, c = g["l"]
        rho = -c
        x0, x1, y0, y1 = g["range"]
        print(f"{i:>3} {line_angle(g['l']):7.2f} {rho:7.1f} {len(g['items']):5d} "
              f"{g['len']:7.0f} {x0:6.0f}~{x1:<6.0f} {y0:6.0f}~{y1:<6.0f}")

    out = os.path.join(OUT_DIR, os.path.splitext(os.path.basename(args.img))[0] + "_lines.jpg")
    draw(img, groups, out)
    print("已存", out)


if __name__ == "__main__":
    main()
