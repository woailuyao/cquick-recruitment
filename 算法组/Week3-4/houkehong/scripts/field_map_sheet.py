# -*- coding: utf-8 -*-
"""
画一张"现场找标志用"的大尺寸场地图

目的：拿着这张图走到场地上，逐个核对每个标志在哪、长什么样、坐标多少。
比在脑子里推坐标靠谱得多。

用法：
    python scripts/field_map_sheet.py
    python scripts/field_map_sheet.py --field config/official/field.yaml \
        --geometry config/official/field_geometry_official.yaml
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from src.config import load_field

TYPE_COLOR = {
    "LCross": (0, 165, 255),      # 橙
    "TCross": (255, 170, 0),      # 蓝
    "XCross": (0, 255, 170),      # 青绿
    "PenaltyPoint": (255, 0, 255)  # 品红
}


def draw_symbol(img, cx, cy, kind, color, s=16, th=3):
    if kind == "LCross":
        cv2.line(img, (cx - s, cy + s), (cx - s, cy - s), color, th, cv2.LINE_AA)
        cv2.line(img, (cx - s, cy + s), (cx + s, cy + s), color, th, cv2.LINE_AA)
    elif kind == "TCross":
        cv2.line(img, (cx - s, cy - s), (cx + s, cy - s), color, th, cv2.LINE_AA)
        cv2.line(img, (cx, cy - s), (cx, cy + s), color, th, cv2.LINE_AA)
    elif kind == "XCross":
        cv2.line(img, (cx - s, cy - s), (cx + s, cy + s), color, th, cv2.LINE_AA)
        cv2.line(img, (cx - s, cy + s), (cx + s, cy - s), color, th, cv2.LINE_AA)
    else:
        cv2.circle(img, (cx, cy), max(6, s - 4), color, th, cv2.LINE_AA)


def put(img, text, org, scale=0.6, color=(240, 240, 240), th=1):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, th, cv2.LINE_AA)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "official", "field.yaml"))
    ap.add_argument("--geometry", default=os.path.join(
        ROOT, "config", "official", "field_geometry_official.yaml"))
    ap.add_argument("--out", default=os.path.join(ROOT, "snapshots", "field_map_sheet.png"))
    args = ap.parse_args()

    fm = load_field(args.field, args.geometry)
    minx, maxx, miny, maxy = fm.bounds()
    pad = 1.2
    minx -= pad; maxx += pad; miny -= pad; maxy += pad

    W, H = 1500, 1150
    margin = 90
    sx = (W - 2 * margin) / (maxx - minx)
    sy = (H - 2 * margin) / (maxy - miny)
    s = min(sx, sy)

    def to_px(x, y):
        px = margin + (x - minx) * s
        py = H - margin - (y - miny) * s
        return int(round(px)), int(round(py))

    img = np.full((H, W, 3), 32, np.uint8)

    # 网格（1 m）
    gx = math.ceil(minx)
    while gx <= maxx:
        p1 = to_px(gx, miny); p2 = to_px(gx, maxy)
        cv2.line(img, p1, p2, (52, 56, 62), 1)
        px, py = to_px(gx, miny)
        put(img, f"{gx:.0f}", (px - 8, py + 24), 0.45, (130, 135, 140))
        gx += 1
    gy = math.ceil(miny)
    while gy <= maxy:
        p1 = to_px(minx, gy); p2 = to_px(maxx, gy)
        cv2.line(img, p1, p2, (52, 56, 62), 1)
        px, py = to_px(minx, gy)
        put(img, f"{gy:.0f}", (px - 40, py + 6), 0.45, (130, 135, 140))
        gy += 1

    # 坐标轴
    o = to_px(0, 0)
    L = int(1.0 * s)
    cv2.arrowedLine(img, o, (o[0] + L, o[1]), (90, 160, 250), 3, tipLength=0.25)
    cv2.arrowedLine(img, o, (o[0], o[1] - L), (250, 160, 90), 3, tipLength=0.25)
    put(img, "+X", (o[0] + L + 6, o[1] + 6), 0.7, (90, 160, 250), 2)
    put(img, "+Y", (o[0] + 6, o[1] - L - 8), 0.7, (250, 160, 90), 2)
    put(img, "O(0,0)", (o[0] - 70, o[1] + 30), 0.55, (200, 200, 200))

    # 特征点
    for f in fm.features:
        px, py = to_px(f.x, f.y)
        col = TYPE_COLOR.get(f.type, (200, 200, 200))
        draw_symbol(img, px, py, f.type, col, s=18, th=3)
        label = f"{f.name or ('#' + str(f.fid))}"
        put(img, label, (px + 22, py - 14), 0.62, col, 2)
        put(img, f"({f.x:.3f}, {f.y:.3f})", (px + 22, py + 10), 0.45, (190, 190, 190))

    # 标题与图例
    put(img, "C0 FIELD MARKERS  --  官方 field.yaml 坐标（原点/轴向待现场核对）",
        (30, 42), 0.85, (255, 255, 255), 2)
    put(img, f"特征 {len(fm.features)} 个   单位 m   +X 向右  +Y 向上  yaw 从 +X 逆时针为正",
        (30, 74), 0.55, (180, 185, 190))

    # 图例
    lx = W - 330
    ly = 40
    for i, (k, c) in enumerate(TYPE_COLOR.items()):
        yy = ly + i * 34
        draw_symbol(img, lx, yy, k, c, s=10, th=2)
        cnt = len([f for f in fm.features if f.type == k])
        put(img, f"{k}  x{cnt}", (lx + 22, yy + 7), 0.58, c)

    # 类别形状说明
    put(img, "L 型 = 两条垂直线交于一点（角）", (30, H - 74), 0.56, (200, 200, 200))
    put(img, "T 型 = 横线与竖线垂直相交（边线/中线）", (30, H - 44), 0.56, (200, 200, 200))
    put(img, "X 型 = 两条线交叉（中线/中圈）", (560, H - 74), 0.56, (200, 200, 200))
    put(img, "点球点 = 一个实心圆点", (560, H - 44), 0.56, (200, 200, 200))

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    cv2.imwrite(args.out, img)
    print(f"场地图 -> {args.out}")
    print(f"{W}x{H}，范围 x[{minx:.2f},{maxx:.2f}] y[{miny:.2f},{maxy:.2f}]")

    # 控制台清单：按 y 分层，方便现场"沿一条线走"逐个找
    print("\n按场地上的位置分组（同一条线上的一般在一条边/中线）：")
    groups = {}
    for f in fm.features:
        key = round(f.y, 2)
        groups.setdefault(key, []).append(f)
    for y in sorted(groups, reverse=True):
        fs = sorted(groups[y], key=lambda t: t.x)
        print(f"\n  y = {y:+.3f}  这一行有 {len(fs)} 个:")
        for f in fs:
            print(f"    {f.name or f.fid:<14} {f.type:<14} x={f.x:+.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
