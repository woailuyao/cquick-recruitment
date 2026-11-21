# -*- coding: utf-8 -*-
"""
check_extrinsics.py —— 用"已知长度"反查外参是否匹配当前安装
基线：中圈与中线的两个 X 交点，地图间距 = 2×0.75 = 1.50 m。
判读：实测间距 ≈ 1.50 m → 外参可用；明显偏小 → h 配小（或 pitch 配大）；
      明显偏大 → h 配大（或 pitch 配小）。

用法：
    python src/vision/check_extrinsics.py --image a.jpg [--image b.jpg ...]
    python src/vision/check_extrinsics.py --dir path/to/test_images
"""
from __future__ import annotations

import argparse
import glob
import math
import os

import cv2
import numpy as np

from ocv_field import CameraModel
from ocv_match import load_field, load_field_meta
from yolo_marker import YoloMarkerDetector

# 仓库根目录（zhengzeyang/）：默认 config 路径相对仓库根定位，不依赖运行时工作目录
ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))


def ensure_weights(path):
    """权重文件存在性检查：缺失时给出明确的获取指引后退出，不进入算法流程。"""
    if not os.path.isfile(path):
        raise SystemExit(
            "[error] 权重未就位：" + path + "\n"
            "请从 README 中的百度网盘链接（提取码 27vp）下载 best2.pt，"
            "放到对应位置，或用 --model 指定已有权重路径。")


def measure(img, cam, det, expected):
    marks, _ = det.detect(img)                   # 只要像素点，地面投影自己算
    xs = []
    for m in marks:
        if m["type"] != "X":
            continue
        a, b, ok, _ = cam.pixel_to_local_ground(m["u"], m["v"])
        if ok:
            xs.append((a, b))
    best = None
    for i in range(len(xs)):
        for j in range(i + 1, len(xs)):
            sep = math.hypot(xs[i][0] - xs[j][0], xs[i][1] - xs[j][1])
            if not (0.5 <= sep <= 6.0):
                continue
            err = abs(sep - expected)
            if best is None or err < best[0]:
                best = (err, sep)
    return (None if best is None else best[1]), len(xs)


def main():
    ap = argparse.ArgumentParser(description="用两个 X 交点的已知间距校验外参")
    ap.add_argument("--image", nargs="*", default=[], help="一张或多张图片")
    ap.add_argument("--dir", default="", help="图片目录（取前 N 张）")
    ap.add_argument("--limit", type=int, default=8)
    ap.add_argument("--config", default=os.path.join(ROOT, "calibration", "camera.yaml"))
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--model", default=r"C:\Users\wen\Desktop\best2.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    a = ap.parse_args()

    cam = CameraModel(a.config)
    meta = load_field_meta(a.field)
    r = float(meta.get("circle_r") or 0.75)
    expected = 2.0 * r
    field = load_field(a.field)
    n_x_gt = len(field.get("X", []))

    imgs = list(a.image)
    if a.dir:
        imgs += sorted(glob.glob(os.path.join(a.dir, "*.jpg")))[: a.limit]
    if not imgs:
        raise SystemExit("请给 --image 或 --dir")

    ensure_weights(a.model)
    det = YoloMarkerDetector(a.model, conf=a.conf, verbose=False)
    print("=" * 74)
    print(f"配置: 光心高度 {cam.cam_height:.3f} m   俯仰 {math.degrees(cam.pitch):.1f}°   "
          f"fx={cam.K[0,0]:.0f} fy={cam.K[1,1]:.0f}")
    print(f"地图: X 型 {n_x_gt} 个   中圈半径 {r:.3f} m   期望基线 2r = {expected:.3f} m")
    print("=" * 74)

    seps = []
    for p in imgs:
        img = cv2.imread(p)
        if img is None:
            continue
        sep, n_x = measure(img, cam, det, expected)
        name = os.path.basename(p)[:34]
        if sep is None:
            print(f"{name:<36} X 检出 {n_x} 个，未凑成基线")
            continue
        seps.append(sep)
        ratio = expected / sep if sep > 0 else float("nan")
        print(f"{name:<36} X 检出 {n_x} 个  基线 {sep:5.2f} m  "
              f"偏差 {sep - expected:+.2f} m  按比例应取 h≈{cam.cam_height * ratio:.2f} m")

    if not seps:
        raise SystemExit("\n没有可用的基线，换几张能看到中圈与中线交点的图。")
    med = float(np.median(seps))
    print("-" * 74)
    print(f"基线中位数 {med:.3f} m（期望 {expected:.3f} m，偏差 {med - expected:+.3f} m）")
    print(f"若按比例修正光心高度：{cam.cam_height:.3f} × {expected:.2f}/{med:.3f} "
          f"= {cam.cam_height * expected / med:.2f} m")
    if abs(med - expected) <= 0.10:
        print("结论：外参与当前画面基本一致（偏差 ≤10 cm），可直接用于定位。")
    else:
        print("结论：外参与当前画面不一致。请用卷尺实测光心离地高度（以及俯仰角），")
        print("      确认后再更新 calibration/camera.yaml；本脚本不改任何配置文件。")


if __name__ == "__main__":
    main()
