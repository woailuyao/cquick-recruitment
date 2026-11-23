#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""参数对比实验（任务 2）—— 全部用我自己拍的实拍图。

每次只改一个主要参数，其它固定成参考配置，然后对比：
    检出了没有（漏检/误检）、中心点跑没跑偏、候选数变了没有。

3 组实验：
    A  S 下界：90（教科书值） vs 20（按实拍图标定）   -> my_01_clear
    B  V 下界：60 vs 25                              -> my_04_backlit（逆光深藏青）
    C  最小面积：800 vs 50000                        -> my_03_occluded
    D  多候选规则：largest vs closest_to_center       -> my_03_occluded

用法（在项目根目录）：
    python3 tools/sweep_params.py
输出：
    output/02_对比_A_S下界.png …… 共 3 张
    output/02_参数对比总览.png      3 组拼成一张，截图就用这张
    output/sweep_results.csv        每个变体的数值，README 直接引用
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path
from typing import Any, Dict, List, Tuple

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "cquick_vision"))

from common import ROOT, imread_bgr, make_grid, make_strip, setup_stdout  # noqa: E402
from cquick_vision.detector import BlueTargetDetector  # noqa: E402

IMG_DIR = ROOT / "data" / "images" / "photos"
OUT_DIR = ROOT / "output"

# 参考配置：没被"实验变量"改到的参数都用这一套
BASE: Dict[str, Any] = dict(
    hsv_lower=(90, 20, 25), hsv_upper=(130, 255, 255),
    min_area=800.0, morph=True, kernel=5, select="largest",
)

EXPERIMENTS: List[dict] = [
    {
        "key": "A_S下界",
        "tag": "Exp A: S lower bound",
        "image": "my_01_clear.png",
        "question": "教科书写 S>=90，可实拍物体的 S 中位数只有 31 —— 会发生什么？",
        "variants": [
            ("A: S >= 90  (textbook value)",
             dict(hsv_lower=(90, 90, 25))),
            ("B: S >= 20  (tuned on my photos)",
             dict(hsv_lower=(90, 20, 25))),
        ],
    },
    {
        "key": "B_V下界",
        "tag": "Exp B: V lower bound",
        "image": "my_04_backlit.jpg",
        "question": "逆光那张杯套的 V 中位数只有 48，V 下界设 60 会怎样？",
        "variants": [
            ("A: V >= 60", dict(hsv_lower=(90, 20, 60))),
            ("B: V >= 25  (reference)", dict(hsv_lower=(90, 20, 25))),
        ],
    },
    {
        "key": "C_最小面积",
        "tag": "Exp C: min_area",
        "image": "my_03_occluded.png",
        "question": "被白纸挡住后杯套只剩 3 万像素，min_area 设 50000 会漏检吗？",
        "variants": [
            ("A: min_area = 800", dict(min_area=800.0)),
            ("B: min_area = 50000", dict(min_area=50000.0)),
        ],
    },
]


def run_variant(bgr, overrides):
    cfg = dict(BASE)
    cfg.update(overrides)
    det = BlueTargetDetector(**cfg)
    return det, det.detect(bgr)


def main() -> int:
    setup_stdout()
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    rows: List[dict] = []
    overview: List[Tuple[str, Any]] = []

    for exp in EXPERIMENTS:
        img_path = IMG_DIR / exp["image"]
        bgr = imread_bgr(img_path)
        if bgr is None:
            print(f"[x] 读不到测试图：{img_path}")
            return 1

        print("\n" + "=" * 78)
        print(f"实验 {exp['key']}   （图：{exp['image']}）")
        print(f"问题：{exp['question']}")
        h, w = bgr.shape[:2]
        print(f"图像尺寸：{w} x {h}")
        print("-" * 78)

        panels: List[Tuple[str, Any]] = [("original", bgr)]
        rendered: List[Tuple[str, Any]] = []

        for label, overrides in exp["variants"]:
            det, res = run_variant(bgr, overrides)
            annotated = det.annotate(bgr, res)
            panels.append((label, annotated))
            rendered.append((label, annotated))
            if res.detected:
                verdict = "检测到"
                info = (f"u={res.u:7.1f}  v={res.v:7.1f}  area={res.area:8.0f}  "
                        f"候选={res.candidates}")
            else:
                verdict = "未检测到(漏检)"
                info = f"u/v/area=0  候选={res.candidates}"
            print(f"  {label:38s} {verdict:12s} {info}")
            rows.append({
                "experiment": exp["key"], "image": exp["image"], "variant": label,
                "detected": res.detected, "u": round(res.u, 2), "v": round(res.v, 2),
                "area": round(res.area, 1), "candidates": res.candidates,
                "center_method": res.center_method,
            })

        out_png = OUT_DIR / f"02_对比_{exp['key']}.png"
        make_strip(panels, out_png)
        print(f"  -> 对比图：{out_png.name}")

        # 总览：每组只放"第一个变体"和"最后一个变体"，图里只用英文，避免字体缺字
        overview.append((f"{exp['tag']}  |  {rendered[0][0]}", rendered[0][1]))
        overview.append((f"{exp['tag']}  |  {rendered[-1][0]}", rendered[-1][1]))

    make_grid(overview, OUT_DIR / "02_参数对比总览.png", cols=2, panel_w=620)

    with (OUT_DIR / "sweep_results.csv").open("w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    print("\n[ok] 3 组对比图 -> output/02_对比_*.png")
    print("[ok] 总览图     -> output/02_参数对比总览.png")
    print("[ok] 数值表     -> output/sweep_results.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
