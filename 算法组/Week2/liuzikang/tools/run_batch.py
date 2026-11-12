#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""批量跑检测（任务 1 + 任务 2 的主要证据来源）。

用法（在项目根目录）：
    python3 tools/run_batch.py                          # 跑 data/images/photos（我拍的 4 张）
    python3 tools/run_batch.py --images data/images/photos
    python3 tools/run_batch.py --hsv-lower 105 120 80 --hsv-upper 130 255 255 --min-area 500
    python3 tools/run_batch.py --select closest_to_center

每次都做四件事：
    1. 每张图检测一次，终端打印 u/v/area、候选数、用的中心点算法；
    2. 把"原图 | mask | 标注图"三合一存到 output/triplets/（方便你看 mask 对不对）；
    3. 单独存标注图到 output/annotated/，mask 存到 output/masks/；
    4. 结果写进 output/results.json 和 output/results.csv，README 的数字直接从这里抄。

如果目录里有 ground_truth.json（可选），还会算检测中心点和真值的误差（像素）。
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src" / "cquick_vision"))

from common import (ROOT, imread_bgr, imwrite_bgr, list_images, load_json,  # noqa: E402
                    mask_to_bgr, make_strip, setup_stdout)
from cquick_vision.detector import BlueTargetDetector  # noqa: E402


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="批量检测蓝色目标")
    p.add_argument("--images", default=str(ROOT / "data" / "images" / "photos"),
                   help="测试图片目录（默认我自己拍的 4 张）")
    p.add_argument("--out", default=str(ROOT / "output"), help="结果输出目录")
    p.add_argument("--hsv-lower", nargs=3, type=int, metavar=("H", "S", "V"),
                   default=[90, 20, 25], help="HSV 下界，默认 90 20 25")
    p.add_argument("--hsv-upper", nargs=3, type=int, metavar=("H", "S", "V"),
                   default=[130, 255, 255], help="HSV 上界，默认 130 255 255")
    p.add_argument("--min-area", type=float, default=800.0, help="最小轮廓面积，默认 800")
    p.add_argument("--kernel", type=int, default=5, help="形态学核大小（奇数），默认 5")
    p.add_argument("--no-morph", action="store_true", help="不做形态学处理")
    p.add_argument("--select", choices=["largest", "closest_to_center"], default="largest",
                   help="多候选时的选择规则")
    p.add_argument("--tag", default="run", help="结果文件名后缀，避免覆盖")
    p.add_argument("--brief", action="store_true",
                   help="精简输出：每张图只打一行，方便把整个运行过程截在一张图里")
    return p.parse_args()


def main() -> int:
    setup_stdout()
    args = parse_args()

    images = list_images(args.images)
    if not images:
        print(f"[x] 目录里没有图片：{args.images}")
        print("    把照片放进 data/images/photos/，或用 --images 指定别的目录")
        return 1

    det = BlueTargetDetector(
        hsv_lower=args.hsv_lower, hsv_upper=args.hsv_upper,
        min_area=args.min_area, morph=not args.no_morph,
        kernel=args.kernel, select=args.select,
    )

    out_dir = Path(args.out)
    ann_dir, mask_dir, tri_dir = out_dir / "annotated", out_dir / "masks", out_dir / "triplets"
    for d in (ann_dir, mask_dir, tri_dir):
        d.mkdir(parents=True, exist_ok=True)

    gt_all = load_json(Path(args.images) / "ground_truth.json")

    if args.brief:
        print(f"配置: HSV {det.hsv_lower}~{det.hsv_upper}  min_area={det.min_area:g}  "
              f"形态学={'开+闭 k' + str(det.kernel) if det.morph else '关'}  "
              f"规则={det.select}")
        print("-" * 78)
    else:
        print("=" * 78)
        print("配置")
        print("=" * 78)
        print(f"  图片目录 : {args.images}")
        print(f"  HSV 下界 : {det.hsv_lower}   (H 0~180, S/V 0~255)")
        print(f"  HSV 上界 : {det.hsv_upper}")
        print(f"  最小面积 : {det.min_area:g} px^2")
        print(f"  形态学   : {'开+闭, kernel' if det.morph else '关闭'} "
              f"{det.kernel if det.morph else ''}")
        print(f"  选择规则 : {det.select}")
        print("=" * 78)

    rows = []
    for i, path in enumerate(images, 1):
        bgr = imread_bgr(path)
        if not args.brief:
            print(f"\n[{i}/{len(images)}] {path.name}")
        if bgr is None:
            # "读取图片并检查是否成功"：读不到就明确报错，不能继续用空图
            print("  [x] 读取失败（文件损坏 / 不是图片 / 没有权限）")
            rows.append({"file": path.name, "detected": False, "u": 0.0, "v": 0.0,
                         "area": 0.0, "candidates": 0, "center_method": "read_error",
                         "gt_u": None, "gt_v": None, "error_px": None, "ms": 0.0})
            continue

        h, w = bgr.shape[:2]
        t0 = time.perf_counter()
        res = det.detect(bgr)
        ms = (time.perf_counter() - t0) * 1000.0

        gt = gt_all.get(path.stem)
        has_gt = bool(gt) and gt.get("gt_u") is not None
        err = None
        if has_gt and res.detected:
            err = float(np.hypot(res.u - gt["gt_u"], res.v - gt["gt_v"]))

        if args.brief:
            tag = "OK  " if res.detected else "MISS"
            extra = f"  误差 {err:6.1f} px" if err is not None else ""
            print(f"  [{tag}] {path.name:26s} u={res.u:7.1f} v={res.v:7.1f} "
                  f"area={res.area:9.0f} 候选={res.candidates}{extra}")
        else:
            print(f"  图像尺寸 : {w} x {h}")
            if res.detected:
                print(f"  结果     : 检测到目标")
                print(f"  u, v     : ({res.u:.1f}, {res.v:.1f})   # 像素坐标，原点在左上角")
                print(f"  area     : {res.area:.0f} px^2")
                print(f"  候选数   : {res.candidates}  各候选面积={[f'{a:.0f}' for a in res.all_areas]}")
                print(f"  选择     : 第 {res.chosen_index + 1} 大（规则={det.select}）")
                print(f"  中心算法 : {res.center_method}")
            else:
                print(f"  结果     : 未检测到目标（候选数={res.candidates}），u/v/area 全部置 0")
            print(f"  耗时     : {ms:.2f} ms")
            if has_gt and res.detected:
                print(f"  真值     : ({gt['gt_u']:.1f}, {gt['gt_v']:.1f})   误差 {err:.1f} px")
            elif has_gt:
                print(f"  真值     : ({gt['gt_u']:.1f}, {gt['gt_v']:.1f})   但本图未检测到 -> 漏检")
            elif gt:
                print("  真值     : 本图按设计没有目标")

        annotated = det.annotate(bgr, res)
        imwrite_bgr(ann_dir / f"{path.stem}_annotated.png", annotated)
        imwrite_bgr(mask_dir / f"{path.stem}_mask.png", res.mask if res.mask is not None else bgr)
        make_strip([("original", bgr), ("mask (HSV inRange)", mask_to_bgr(res.mask)
                                        if res.mask is not None else bgr),
                    ("annotated", annotated)],
                   tri_dir / f"{path.stem}_triplet.png")

        rows.append({
            "file": path.name, "detected": bool(res.detected),
            "u": round(float(res.u), 2), "v": round(float(res.v), 2),
            "area": round(float(res.area), 1), "candidates": int(res.candidates),
            "center_method": res.center_method,
            "gt_u": gt["gt_u"] if gt else None, "gt_v": gt["gt_v"] if gt else None,
            "error_px": round(err, 2) if err is not None else None,
            "ms": round(ms, 2),
        })

    ok = [r for r in rows if r["detected"]]
    errs = [r["error_px"] for r in rows if r["error_px"] is not None]

    print("-" * 78 if args.brief else "\n" + "=" * 78)
    if not args.brief:
        print("汇总")
        print("=" * 78)
    print(f"  共 {len(rows)} 张，检测到 {len(ok)} 张，未检测到 {len(rows) - len(ok)} 张")
    if errs:
        print(f"  中心点平均误差 {sum(errs) / len(errs):.1f} px，最大 {max(errs):.1f} px")
    if not args.brief:
        for r in rows:
            flag = "OK  " if r["detected"] else "MISS"
            extra = f"  误差 {r['error_px']:.1f} px" if r["error_px"] is not None else ""
            print(f"  [{flag}] {r['file']:26s} u={r['u']:7.1f} v={r['v']:7.1f} "
                  f"area={r['area']:9.0f} 候选={r['candidates']}{extra}")

    payload = {
        "config": {
            "hsv_lower": det.hsv_lower, "hsv_upper": det.hsv_upper,
            "min_area": det.min_area, "morph": det.morph, "kernel": det.kernel,
            "select": det.select, "images": str(args.images),
        },
        "images": rows,
        "summary": {
            "total": len(rows), "detected": len(ok),
            "mean_error_px": round(sum(errs) / len(errs), 2) if errs else None,
            "max_error_px": round(max(errs), 2) if errs else None,
        },
    }
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"results_{args.tag}.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    with (out_dir / f"results_{args.tag}.csv").open("w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        wr.writeheader()
        wr.writerows(rows)

    print(f"\n[ok] 结果已保存：{out_dir / f'results_{args.tag}.json'}")
    print(f"[ok] 标注图：{ann_dir}")
    print(f"[ok] 三合一对比图：{tri_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
