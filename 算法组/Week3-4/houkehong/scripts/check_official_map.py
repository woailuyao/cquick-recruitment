# -*- coding: utf-8 -*-
"""
验证"能否正确加载 official/field.yaml 并画出场地俯视图"

官方文件是扁平键值格式：
    TCross1: [5.955, 2.930]
    PenalityPoint: [2.965, 0.885]
本脚本确认解析器能吃下这种格式，并输出一张俯视图供人工核对布局是否像足球场。
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cv2

from src import viz
from src.config import check_field, load_field

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def main():
    field_path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        ROOT, "config", "official", "field.yaml")
    geo_path = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        ROOT, "config", "official", "field_geometry_official.yaml")

    fm = load_field(field_path, geo_path)
    print("=" * 66)
    print(f"地图文件 : {fm.source_path}")
    print(f"版本     : {fm.version}")
    print(f"特征总数 : {len(fm.features)}")
    for t, lst in fm.by_type().items():
        print(f"  {t:14s} {len(lst):2d} 个  " +
              ", ".join(f"#{f.fid}({f.x:.3f},{f.y:.3f})" for f in lst))
    print(f"绘制范围 : {tuple(round(v, 3) for v in fm.bounds())}")

    warns = check_field(fm)
    if warns:
        print("\n[启动检查警告]")
        for w in warns:
            print("  -", w)
    else:
        print("\n[启动检查] 通过")

    w, h = 760, 620
    img = viz.render_map(w, h, fm, matched_fids=(), pose=(0.0, 0.0, 0.0),
                         status_text="", valid=True,
                         extra_lines=[f"{len(fm.features)} features",
                                      fm.version[:34]])
    out = os.path.join(ROOT, "snapshots", "official_field_map.png")
    os.makedirs(os.path.dirname(out), exist_ok=True)
    cv2.imwrite(out, img)
    print(f"\n俯视图 -> {out}")
    print("请人工确认：布局应该像一块足球场（两条底线、两条边线、中线、两个点球点）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
