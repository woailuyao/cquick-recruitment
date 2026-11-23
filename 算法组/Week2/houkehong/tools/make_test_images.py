#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""备用：没有照片时生成 5 张蓝色球的合成测试图。

优先用 `tools/prepare_test_images.py` 处理你自己拍/AI 生成的真实图；这个脚本只是兜底。

    5 张图分别对应：
      blue_01  目标清晰、占画面约 7%
      blue_02  目标很小
      blue_03  大面积蓝天背景 + 小球（蓝天是蓝球检测最大的干扰源 -> 演示误检）
      blue_04  暗光，球的 V 掉到 60 以下 -> 演示漏检
      blue_05  画面里没有蓝色目标 -> 验证 detected=false

用法（在 houkehong/ 目录下）：

    python3 tools/make_test_images.py
"""

from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

W, H = 1280, 720
BALL_BGR = (200, 150, 60)        # 蓝色球（BGR），HSV 约 H=102 S=178 V=200
BALL_EDGE_BGR = (120, 85, 25)
HIGHLIGHT_BGR = (240, 235, 230)
SKY_BGR = (200, 160, 120)        # 浅蓝天空，饱和度低


def save(path: str, img: np.ndarray) -> None:
    ext = os.path.splitext(path)[1] or ".png"
    ok, buf = cv2.imencode(ext, img)
    if not ok:
        raise RuntimeError("编码失败: %s" % path)
    folder = os.path.dirname(os.path.abspath(path))
    if folder:
        os.makedirs(folder, exist_ok=True)
    buf.tofile(path)
    print("[SAVE] %s  (%dx%d)" % (path, img.shape[1], img.shape[0]))


def make_ground(rng, base=(120, 118, 112)) -> np.ndarray:
    grad = np.linspace(0.85, 1.15, H).reshape(H, 1, 1).astype(np.float32)
    img = np.ones((H, W, 3), np.float32) * np.array(base, np.float32).reshape(1, 1, 3)
    img = img * grad + rng.normal(0, 7, (H, W, 3)).astype(np.float32)
    return np.clip(img, 0, 255).astype(np.uint8)


def add_sky(img: np.ndarray, height: int) -> np.ndarray:
    """上半部分换成低饱和度的蓝天（用来验证 S 下界能不能挡住天空）。"""
    img[:height, :] = SKY_BGR
    return img


def draw_ball(img: np.ndarray, center, radius: int) -> None:
    cx, cy = center
    cv2.circle(img, (cx, cy), radius, BALL_BGR, -1, lineType=cv2.LINE_AA)
    cv2.circle(img, (cx, cy), radius, BALL_EDGE_BGR, max(2, radius // 28),
               lineType=cv2.LINE_AA)
    cv2.circle(img, (cx - int(radius * 0.34), cy - int(radius * 0.34)),
               max(3, int(radius * 0.20)), HIGHLIGHT_BGR, -1, lineType=cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description="生成 5 张蓝色球合成测试图（兜底用）")
    ap.add_argument("--out", default="src/green_detector/test_images")
    ap.add_argument("--seed", type=int, default=20261101)
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    out = args.out

    img = make_ground(rng)
    draw_ball(img, (640, 420), 95)
    save(os.path.join(out, "blue_01.png"), cv2.GaussianBlur(img, (3, 3), 0))

    img = make_ground(rng, base=(130, 128, 120))
    draw_ball(img, (1040, 520), 24)
    save(os.path.join(out, "blue_02.png"), cv2.GaussianBlur(img, (3, 3), 0))

    img = add_sky(make_ground(rng), 300)
    draw_ball(img, (300, 560), 40)
    save(os.path.join(out, "blue_03.png"), cv2.GaussianBlur(img, (3, 3), 0))

    img = make_ground(np.random.default_rng(args.seed + 1))
    draw_ball(img, (640, 420), 95)
    dark = np.clip(img.astype(np.float32) * 0.40, 0, 255).astype(np.uint8)
    save(os.path.join(out, "blue_04.png"), cv2.GaussianBlur(dark, (3, 3), 0))

    img = make_ground(rng, base=(125, 120, 118))
    cv2.circle(img, (600, 420), 80, (45, 45, 190), -1, lineType=cv2.LINE_AA)   # 红球
    cv2.rectangle(img, (950, 420), (1180, 600), (90, 160, 60), -1)             # 绿盒子
    save(os.path.join(out, "blue_05.png"), cv2.GaussianBlur(img, (3, 3), 0))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
