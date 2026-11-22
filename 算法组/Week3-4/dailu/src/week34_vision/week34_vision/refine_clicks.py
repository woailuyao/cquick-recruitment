# -*- coding: utf-8 -*-
"""把已保存的点击坐标再做一次"中心线几何细化"，覆盖回 json。

考核细则要求"不能直接把框中心当交点，必须几何细化"，
这个脚本就是那一步的产物：每个点都会记录细化后坐标和 how 标记。
"""
import glob
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from imgio import imread          # noqa: E402
from pick_marker import refine_intersection  # noqa: E402

CLICKS = sys.argv[1] if len(sys.argv) > 1 else r"D:\week3_4_dailu\task2_戴璐\clicks"

print(f"{'file':22s} {'type':>4s} {'原始点击':>18s} {'细化后':>18s} {'位移':>7s}  how")
total, moved = 0, 0
for f in sorted(glob.glob(os.path.join(CLICKS, "*.json"))):
    with open(f, encoding="utf-8") as fp:
        d = json.load(fp)
    img = imread(d["image"])
    if img is None:
        print(f"[ERROR] 读不到 {d['image']}")
        continue
    for p in d["points"]:
        u0, v0 = p["u"], p["v"]
        xi, yi, how = refine_intersection(img, u0, v0)
        d_ = ((xi - u0) ** 2 + (yi - v0) ** 2) ** 0.5
        total += 1
        if how == "center":
            moved += 1
        p["u_raw"], p["v_raw"] = u0, v0
        p["u"], p["v"], p["how"] = xi, yi, how
        print(f"{os.path.basename(f):22s} {p['type']:>4s} "
              f"({u0:7.1f},{v0:6.1f}) -> ({xi:7.1f},{yi:6.1f}) {d_:6.1f}px  {how}")
    with open(f, "w", encoding="utf-8") as fp:
        json.dump(d, fp, ensure_ascii=False, indent=2)

print(f"\n共 {total} 个点，其中 {moved} 个成功细化（其余保持人工点击位置）")
