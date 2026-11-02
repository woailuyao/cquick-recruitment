# -*- coding: utf-8 -*-
"""生成 5 张测试图。

手上没有场地和足球，所以用脚本合成：固定随机种子，每次跑出来一样。
换成自己拍的照片时，把图片放进 data/images 覆盖同名文件即可，代码不用改。

用法: python data/make_images.py
"""

import os
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / "images"
OUT.mkdir(parents=True, exist_ok=True)

H, W = 960, 1280
SS = 4               # 画圆时超采样，边缘不会有锯齿
rng = np.random.default_rng(20261031)


def imwrite_u(path, img, quality=95):
    """cv2.imwrite 在中文路径下会失败，改用 imencode + tofile。"""
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), quality])
    if ok:
        buf.tofile(str(path))


def gradient_bg(c_tl, c_br, noise=3.0, vignette=0.12):
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    t = ((xx / W) * 0.6 + (yy / H) * 0.4)[:, :, None]
    a = np.array(c_tl, np.float32)[None, None, :]
    b = np.array(c_br, np.float32)[None, None, :]
    img = a * (1 - t) + b * t

    cx, cy = W / 2.0, H / 2.0
    r = np.sqrt(((xx - cx) / (W / 2)) ** 2 + ((yy - cy) / (H / 2)) ** 2) / 1.4142
    img *= (1.0 - vignette * np.clip(r, 0, 1))[:, :, None]
    img += rng.normal(0, noise, (H, W, 1))
    return np.clip(img, 0, 255).astype(np.uint8)


def wood_bg():
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    base = np.zeros((H, W, 3), np.float32)
    base[..., 0] = 66
    base[..., 1] = 100
    base[..., 2] = 142
    grain = 0.82 + 0.26 * (np.sin(xx / 24.0 + np.sin(yy / 150.0) * 1.4) * 0.5 + 0.5)
    base *= grain[:, :, None]
    base += rng.normal(0, 5.0, (H, W, 1))
    return np.clip(base, 0, 255).astype(np.uint8)


def grass_bg():
    """草地：绿底 + 竖直的草叶纹理 + 噪点。"""
    base = np.zeros((H, W, 3), np.float32)
    base[..., 0] = 52
    base[..., 1] = 118
    base[..., 2] = 70
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)
    blades = (np.sin(xx / 3.7 + rng.normal(0, 0.02, W)[None, :] * 8) * 0.5 + 0.5)
    base *= (0.85 + 0.3 * blades)[:, :, None]
    base += rng.normal(0, 9.0, (H, W, 1))
    return np.clip(base, 0, 255).astype(np.uint8)


def ball_mask(cx, cy, r):
    m = np.zeros((H * SS, W * SS), np.uint8)
    cv2.circle(m, (cx * SS, cy * SS), r * SS, 255, -1)
    return cv2.resize(m, (W, H), interpolation=cv2.INTER_AREA)


def put_ball(bg, cx, cy, r, color_bgr, light=(1.3, 0.55), spec=(0.35, 60),
             shadow=(-50, 35, 0.45, 28)):
    """画一个带单侧光、高光和投影的球。"""
    mask = ball_mask(cx, cy, r)
    a = (mask.astype(np.float32) / 255.0)[:, :, None]
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)

    t = np.clip((xx / W) * 0.5 + (yy / H) * 0.5, 0, 1)[:, :, None]
    col = np.array(color_bgr, np.float32)[None, None, :] * (light[0] * (1 - t) + light[1] * t)

    # 球面高光
    strike, sr = spec
    sx, sy = cx - r * 0.35, cy - r * 0.4
    g = np.exp(-(((xx - sx) ** 2 + (yy - sy) ** 2) / (2.0 * (r * 0.35) ** 2)))
    col += g[:, :, None] * strike * 255.0

    # 边缘压暗，看起来更立体
    inner = cv2.erode(mask, np.ones((15, 15), np.uint8))
    ring = cv2.subtract(mask, inner).astype(np.float32)[:, :, None] / 255.0
    col *= (1 - ring * 0.35)

    # 投影
    dx, dy, strength, blur = shadow
    sm = np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
    sm = cv2.GaussianBlur(sm, (0, 0), blur).astype(np.float32)[:, :, None] / 255.0
    bg = bg.astype(np.float32) * (1 - sm * strength)

    out = bg * (1 - a) + np.clip(col, 0, 255) * a
    return np.clip(out, 0, 255).astype(np.uint8)


def finish(img, blur=0.9):
    if blur > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    img = np.clip(img.astype(np.float32) + rng.normal(0, 2.0, img.shape), 0, 255)
    return img.astype(np.uint8)


def save(name, img):
    imwrite_u(OUT / name, img)
    print("[OK] %s  %dx%d  %d KB" % (name, img.shape[1], img.shape[0],
                                     os.path.getsize(OUT / name) // 1024))


def main():
    # 1. 目标清晰、背景干净
    save("01_clear.jpg", finish(put_ball(
        gradient_bg((205, 205, 203), (170, 170, 168)), 640, 480, 170, (40, 120, 235))))

    # 2. 目标比较小（相当于球离得远）
    save("02_small.jpg", finish(put_ball(
        gradient_bg((200, 200, 198), (165, 165, 163), noise=4.0),
        950, 600, 42, (40, 120, 235), spec=(0.25, 20), shadow=(-14, 10, 0.35, 8))))

    # 3. 背景颜色和目标接近（木桌 + 橙球）
    save("03_similar_bg.jpg", finish(put_ball(
        wood_bg(), 500, 520, 130, (38, 112, 222), light=(1.15, 0.6))))

    # 4. 暗光 + 阴影里的目标
    dark = np.clip(wood_bg().astype(np.float32) * 0.6, 0, 255).astype(np.uint8)
    img = put_ball(dark, 800, 520, 150, (30, 96, 180), light=(0.8, 0.4),
                   spec=(0.08, 40), shadow=(-60, 30, 0.5, 30))
    img = img.astype(np.float32)
    img[..., 0] += 20          # 冷光，饱和度被压下去
    save("04_shadow.jpg", finish(img.astype(np.uint8), blur=1.2))

    # 5. 绿色的球放在草地上（换颜色时用这张）
    save("05_green_ball.jpg", finish(put_ball(
        grass_bg(), 600, 500, 140, (60, 150, 58), light=(1.2, 0.6),
        spec=(0.25, 55), shadow=(-45, 30, 0.4, 25))))


if __name__ == "__main__":
    main()
