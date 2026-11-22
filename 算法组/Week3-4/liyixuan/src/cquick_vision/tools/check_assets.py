#!/usr/bin/env python3
"""体检：把手上的数据集、模型、配置文件一次盘清楚，并给出问题清单。

用法：
  python tools/check_assets.py --root D:/your_project_path
  python tools/check_assets.py --root . --marker
"""
from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from collections import Counter

IMG_EXT = (".jpg", ".jpeg", ".png", ".bmp")


def pct(vals, q):
    if not vals:
        return 0.0
    s = sorted(vals)
    return s[min(len(s) - 1, int((len(s) - 1) * q))]


def scan_images(folder: str, box_names=None):
    """统计一个文件夹里的图片 + YOLO 标签。"""
    if not os.path.isdir(folder):
        return None
    imgs = [p for p in glob.glob(os.path.join(folder, "*"))
            if p.lower().endswith(IMG_EXT)]
    out = {"folder": folder, "n_img": len(imgs), "n_lbl": 0,
           "cls": Counter(), "w": [], "h": [], "multi": 0, "bad": 0,
           "img_size": None, "unlabeled": 0}
    for p in imgs:
        lbl = os.path.splitext(p)[0] + ".txt"
        if not os.path.exists(lbl):
            out["unlabeled"] += 1
            continue
        out["n_lbl"] += 1
        try:
            with open(lbl, "r", encoding="utf-8") as f:
                lines = [ln for ln in f.read().splitlines() if ln.strip()]
        except Exception:
            out["bad"] += 1
            continue
        if len(lines) > 1:
            out["multi"] += 1
        for ln in lines:
            t = ln.split()
            if len(t) < 5:
                out["bad"] += 1
                continue
            try:
                cid = int(float(t[0]))
                w = float(t[3]) * 1280
                h = float(t[4]) * 720
            except ValueError:
                out["bad"] += 1
                continue
            out["cls"][cid] += 1
            out["w"].append(w)
            out["h"].append(h)
    if imgs:
        try:
            from PIL import Image
            with Image.open(imgs[0]) as im:
                out["img_size"] = im.size
        except Exception:
            out["img_size"] = None
    return out


def show(name: str, st, names=None):
    if st is None:
        print(f"  {name:16s} 不存在")
        return
    print(f"  {name:16s} 图片 {st['n_img']:4d}  有标签 {st['n_lbl']:4d}  "
          f"无标签 {st['unlabeled']:3d}  尺寸 {st['img_size']}")
    if st["cls"]:
        parts = []
        for cid, n in sorted(st["cls"].items()):
            nm = names.get(cid, f"id{cid}") if names else f"id{cid}"
            parts.append(f"{nm}({cid})={n}")
        print(f"  {'':16s} 类别分布: " + ", ".join(parts))
        print(f"  {'':16s} 框宽 px: 中位 {pct(st['w'],0.5):.0f}  "
              f"最小 {pct(st['w'],0.02):.0f}  最大 {pct(st['w'],0.98):.0f}")
        small = sum(1 for w in st["w"] if w < 120)
        print(f"  {'':16s} 小目标(<120px) {small} 个"
              f"{'  <- 远处小球太少，Recall 会挂' if small == 0 else ''}")
        if st["multi"] > 6:
            print(f"  {'':16s} 多框图片 {st['multi']} 张（单球场景应为 1 框）")
        if st["bad"]:
            print(f"  {'':16s} 标签格式异常 {st['bad']} 行")
        for cid in st["cls"]:
            if names and cid not in names:
                print(f"  {'':16s} [warn] 类别 id {cid} 不在 names 里，会训练出错")


def check_yaml(path: str):
    if not os.path.exists(path):
        return None
    print(f"\n--- {os.path.basename(path)} ---")
    with open(path, "r", encoding="utf-8") as f:
        print("  " + f.read().strip().replace("\n", "\n  "))
    try:
        import yaml
        d = yaml.safe_load(open(path, encoding="utf-8"))
    except Exception:
        return None
    tr, va = str(d.get("train", "")), str(d.get("val", ""))
    if tr == va:
        print("  [问题] train 和 val 指向同一个目录 -> 验证集就是训练集，"
              "mAP 会接近 1.0 但完全没有意义（评分会扣分）")
    if not d.get("test"):
        print("  [问题] 没有 test 划分 -> 无法报告独立测试集指标")
    return d


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=".")
    ap.add_argument("--ball", default="ball_images")
    ap.add_argument("--marker", default="marker_images")
    ap.add_argument("--marker-names", default="LCross,TCross,XCross,PenaltyPoint")
    ap.add_argument("--model", action="append", default=[],
                    help="要检查的权重，可重复")
    args = ap.parse_args()

    root = args.root
    print(f"=== 资产体检: {os.path.abspath(root)} ===\n")

    ball_names = {0: "ball"}
    st = scan_images(os.path.join(root, args.ball), ball_names)
    print("[足球数据]")
    show("ball_images", st, ball_names)

    mnames = {i: n.strip() for i, n in enumerate(args.marker_names.split(","))}
    st2 = scan_images(os.path.join(root, args.marker), mnames)
    print("\n[标志数据]")
    show("marker_images", st2, mnames)
    if st2 and st2["cls"]:
        got = set(st2["cls"].keys())
        miss = [mnames[i] for i in sorted(mnames) if i not in got]
        if miss:
            print(f"  [问题] 这几类一个样本都没有: {miss}")
            print("         -> 模型只能检测你标注过的类别")

    # 配置文件
    for y in ("data.yaml", "marker_data.yaml"):
        check_yaml(os.path.join(root, y))

    # 权重
    models = args.model or [p for p in
                            ("best.pt", "best_marker.pt", "yolov8n.pt")
                            if os.path.exists(os.path.join(root, p))]
    if models:
        print("\n[模型权重]")
        try:
            from ultralytics import YOLO
        except Exception as e:
            print("  (装不了 ultralytics，跳过)", e)
            YOLO = None
        if YOLO is not None:
            for m in models:
                p = m if os.path.isabs(m) else os.path.join(root, m)
                if not os.path.exists(p):
                    print(f"  {m:16s} 不存在")
                    continue
                try:
                    model = YOLO(p)
                    names = getattr(model, "names", {})
                    print(f"  {os.path.basename(p):16s} task={getattr(model,'task','?')} "
                          f"classes={names} size={os.path.getsize(p)/1e6:.1f}MB")
                except Exception as e:
                    print(f"  {os.path.basename(p):16s} 读取失败: {e}")

    # 视频
    print("\n[视频/图片素材]")
    for pat in ("*.avi", "*.mp4", "chess_images", "test_images"):
        hits = glob.glob(os.path.join(root, pat))
        for h in hits:
            if os.path.isdir(h):
                n = len([p for p in glob.glob(os.path.join(h, "*"))
                         if p.lower().endswith(IMG_EXT)])
                print(f"  {pat:16s} 目录, {n} 张图")
            else:
                print(f"  {os.path.basename(h):16s} {os.path.getsize(h)/1e6:.1f} MB")

    # 标定结果
    print("\n[标定产物]")
    found = False
    for pat in ("*.npz", "camera_calib*", "*intrinsics*.yaml"):
        for h in glob.glob(os.path.join(root, pat)):
            print(f"  {os.path.basename(h)}  {os.path.getsize(h)} bytes")
            found = True
    if not found:
        print("  没有找到任何标定结果文件（还没有成功标定过）")

    print("\n=== 体检完 ===")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
