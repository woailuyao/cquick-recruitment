# -*- coding: utf-8 -*-
"""
生成练习用的 4 张测试图（固定随机种子，每次结果一样），存到 data/images。

用法：python src/make_test_images.py
"""

import os
import sys
from pathlib import Path

import cv2
import numpy as np

if hasattr(sys.stdout, "reconfigure"):          # Windows 控制台按 UTF-8 输出中文
    sys.stdout.reconfigure(encoding="utf-8")


# --------------------------------------------------------------------------- #
# Windows 下 cv2.imread / imwrite 不支持中文路径，统一用下面两个函数代替
# --------------------------------------------------------------------------- #
def imread_u(path, flags=cv2.IMREAD_COLOR):
    """读图；文件不存在/为空/损坏时返回 None（不抛异常）。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return None if data.size == 0 else cv2.imdecode(data, flags)


def imwrite_u(path, img, params=None):
    ok, buf = cv2.imencode(Path(path).suffix or ".png", img, params or [])
    if not ok:
        return False
    buf.tofile(str(path))
    return True


ROOT = Path(__file__).resolve().parents[1]
OUT_DIR = ROOT / "data" / "images"
OUT_DIR.mkdir(parents=True, exist_ok=True)

H, W = 960, 1280          # 统一分辨率 1280x960
SS = 4                    # 形状掩膜的超采样倍数，用于抗锯齿
rng = np.random.default_rng(20240501)


# --------------------------------------------------------------------------- #
# 背景
# --------------------------------------------------------------------------- #
def gradient_bg(h, w, c_tl, c_br, noise=4.0, vignette=0.22):
    """对角线性渐变 + 轻微暗角 + 高斯噪声。"""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    t = ((xx / max(w - 1, 1)) * 0.55 + (yy / max(h - 1, 1)) * 0.45)[:, :, None]
    c_tl = np.array(c_tl, np.float32)
    c_br = np.array(c_br, np.float32)
    img = c_tl[None, None, :] * (1 - t) + c_br[None, None, :] * t

    # 暗角：越靠边越暗
    cx, cy = w / 2.0, h / 2.0
    r = np.sqrt(((xx - cx) / (w / 2)) ** 2 + ((yy - cy) / (h / 2)) ** 2)
    img *= (1.0 - vignette * np.clip(r / 1.4142, 0, 1))[:, :, None]

    img += rng.normal(0, noise, (h, w, 1))
    return np.clip(img, 0, 255).astype(np.uint8)


def wood_bg(h, w):
    """木纹背景：竖向条纹 + 低频弯曲 + 纹理噪声。"""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    base = np.zeros((h, w, 3), np.float32)
    base[..., 0] = 62    # B
    base[..., 1] = 92    # G
    base[..., 2] = 128   # R
    rows = (np.sin(xx / 26.0 + np.sin(yy / 140.0) * 1.6) * 0.5 + 0.5)
    rows2 = (np.sin(xx / 7.5 + yy / 900.0) * 0.5 + 0.5)
    grain = 0.80 + 0.28 * rows + 0.10 * rows2
    base *= grain[:, :, None]
    base += rng.normal(0, 5.0, (h, w, 1))
    # 右下弱光
    light = (1.06 - 0.35 * (xx / w))[:, :, None]
    base *= light
    return np.clip(base, 0, 255).astype(np.uint8)


def cloth_bg(h, w):
    """白布背景：低频褶皱 + 织物颗粒噪声。"""
    yy, xx = np.mgrid[0:h, 0:w].astype(np.float32)
    folds = (np.sin(xx / 210.0) * 6.0 + np.sin((xx + yy) / 155.0) * 5.0)
    base = np.full((h, w, 3), 236.0, np.float32) + folds[:, :, None]
    base += rng.normal(0, 6.5, (h, w, 1))
    base -= 10.0
    return np.clip(base, 0, 255).astype(np.uint8)


# --------------------------------------------------------------------------- #
# 形状掩膜（超采样后缩小，得到抗锯齿边缘）
# --------------------------------------------------------------------------- #
def _new_mask():
    return np.zeros((H * SS, W * SS), np.uint8)


def _finish(mask_big):
    return cv2.resize(mask_big, (W, H), interpolation=cv2.INTER_AREA)


def mask_rounded_rect(cx, cy, w, h, r):
    m = _new_mask()
    x1, y1 = (cx - w // 2) * SS, (cy - h // 2) * SS
    x2, y2 = (cx + w // 2) * SS, (cy + h // 2) * SS
    r = r * SS
    cv2.rectangle(m, (x1 + r, y1), (x2 - r, y2), 255, -1)
    cv2.rectangle(m, (x1, y1 + r), (x2, y2 - r), 255, -1)
    for (px, py) in ((x1 + r, y1 + r), (x2 - r, y1 + r), (x1 + r, y2 - r), (x2 - r, y2 - r)):
        cv2.circle(m, (px, py), r, 255, -1)
    return _finish(m)


def mask_bottle(cx, cy, body_w, body_h, neck_w, neck_h):
    m = _new_mask()
    bx1, bx2 = (cx - body_w // 2) * SS, (cx + body_w // 2) * SS
    by1, by2 = (cy - body_h // 2) * SS, (cy + body_h // 2) * SS
    cv2.ellipse(m, ((bx1 + bx2) // 2, (by1 + by2) // 2),
                ((bx2 - bx1) // 2, (by2 - by1) // 2), 0, 0, 360, 255, -1)
    nx1, nx2 = (cx - neck_w // 2) * SS, (cx + neck_w // 2) * SS
    ny1, ny2 = by1 - neck_h * SS, by1 + 12 * SS
    cv2.rectangle(m, (nx1, ny1), (nx2, ny2), 255, -1)
    cv2.ellipse(m, ((nx1 + nx2) // 2, ny1), (neck_w * SS // 2, neck_w * SS // 5), 0, 0, 360, 255, -1)
    return _finish(m)


def mask_ball(cx, cy, r):
    m = _new_mask()
    cv2.circle(m, (cx * SS, cy * SS), r * SS, 255, -1)
    return _finish(m)


# --------------------------------------------------------------------------- #
# 着色：方向光 + 高光 + 边缘变暗 + 投影
# --------------------------------------------------------------------------- #
def shade_object(bg, mask, color_bgr, light=(1.25, 0.62), spec=None, edge=0.72, shadow=None):
    """
    mask  : uint8 0/255，抗锯齿
    light : (高位亮度系数, 低位亮度系数)，从左上到右下
    spec  : (x, y, r, 强度) 高光位置，配合 ball/cube 的曲面感
    shadow: (dx, dy, strength, blur) 投影偏移与强度
    """
    a = (mask.astype(np.float32) / 255.0)[:, :, None]
    yy, xx = np.mgrid[0:H, 0:W].astype(np.float32)

    # 位置决定明暗（模拟单侧光）
    t = np.clip((xx / W) * 0.6 + (yy / H) * 0.4, 0, 1)[:, :, None]
    shade = light[0] * (1 - t) + light[1] * t

    col = np.array(color_bgr, np.float32)[None, None, :] * shade

    # 高光（球形/立方体顶部反光）
    if spec is not None:
        sx, sy, sr, si = spec
        g = np.exp(-(((xx - sx) ** 2 + (yy - sy) ** 2) / (2.0 * sr ** 2)))
        col += g[:, :, None] * si * 255.0

    # 边缘变暗，增加立体感
    inner = cv2.erode(mask, np.ones((13, 13), np.uint8))
    ring = cv2.subtract(mask, inner).astype(np.float32)[:, :, None] / 255.0
    col = col * (1 - ring * (1 - edge))

    # 投影
    if shadow is not None:
        dx, dy, strength, blur = shadow
        sm = np.roll(np.roll(mask, dy, axis=0), dx, axis=1)
        sm = cv2.GaussianBlur(sm, (0, 0), blur).astype(np.float32)[:, :, None] / 255.0
        bg = bg.astype(np.float32) * (1 - sm * strength)

    col = np.clip(col, 0, 255)
    out = bg.astype(np.float32) * (1 - a) + col * a
    return np.clip(out, 0, 255).astype(np.uint8)


def finish(img, blur=0.8, jpeg_noise=2.5, path=None):
    if blur > 0:
        img = cv2.GaussianBlur(img, (0, 0), blur)
    img = np.clip(img.astype(np.float32) + rng.normal(0, jpeg_noise, img.shape), 0, 255).astype(np.uint8)
    imwrite_u(path, img, [int(cv2.IMWRITE_JPEG_QUALITY), 95])
    return img


# --------------------------------------------------------------------------- #
# 四张图
# --------------------------------------------------------------------------- #
def img1_blue_cube():
    """顺光、浅灰背景、蓝色立方体（目标大、对比强）"""
    bg = gradient_bg(H, W, (216, 214, 210), (186, 184, 181), noise=3.5, vignette=0.10)
    cx, cy, s = 430, 520, 260
    m = mask_rounded_rect(cx, cy, s, s, 18)
    img = shade_object(bg, m, (188, 116, 44),          # BGR: 蓝
                       light=(1.30, 0.72),
                       spec=(cx - 70, cy - 80, 55, 0.16),
                       edge=0.70,
                       shadow=(38, 26, 0.40, 22))
    return finish(img, blur=0.8, path=OUT_DIR / "01_blue_cube_gray.jpg")


def img2_green_bottle_wood():
    """侧光、木纹背景、绿色瓶子（目标偏小）"""
    bg = wood_bg(H, W)
    cx, cy = 905, 545
    m = mask_bottle(cx, cy, 200, 265, 58, 95)
    img = shade_object(bg, m, (58, 148, 62),           # BGR: 绿
                       light=(1.18, 0.55),
                       spec=(cx - 30, cy - 90, 34, 0.22),
                       edge=0.62,
                       shadow=(-46, 22, 0.42, 26))
    return finish(img, blur=1.0, path=OUT_DIR / "02_green_bottle_wood.jpg")


def img3_orange_ball_white():
    """强侧光、白布背景、橙色球（目标大、阴影重）"""
    bg = cloth_bg(H, W)
    cx, cy, r = 800, 500, 175
    m = mask_ball(cx, cy, r)
    img = shade_object(bg, m, (34, 116, 226),          # BGR: 橙
                       light=(1.32, 0.60),
                       spec=(cx - 60, cy - 70, 42, 0.30),
                       edge=0.68,
                       shadow=(-60, 40, 0.45, 30))
    return finish(img, blur=0.9, path=OUT_DIR / "03_orange_ball_white.jpg")


def img4_hard_case():
    """
    失败案例：橙色目标落在自身阴影里，环境光偏冷，目标饱和度下降，
    且木色背景的 R 通道也偏高 —— 直接用默认橙色阈值会漏检 / 误检。
    """
    bg = wood_bg(H, W)
    bg = np.clip(bg.astype(np.float32) * 0.72, 0, 255).astype(np.uint8)   # 整体压暗
    cx, cy, s = 950, 600, 210
    m = mask_rounded_rect(cx, cy, s, int(s * 0.78), 26)
    img = shade_object(bg, m, (30, 96, 170),           # BGR: 偏暗的橙
                       light=(0.85, 0.42),             # 目标几乎全在阴影里
                       spec=(cx - 40, cy - 50, 40, 0.06),
                       edge=0.55,
                       shadow=(-70, 34, 0.55, 34))
    # 冷色环境光：加大 B 通道，压低饱和度
    img = img.astype(np.float32)
    img[..., 0] += 26
    img[..., 1] += 8
    return finish(img, blur=1.2, path=OUT_DIR / "04_hard_orange_shadow.jpg")


def main():
    items = [
        ("01_blue_cube_gray.jpg", img1_blue_cube),
        ("02_green_bottle_wood.jpg", img2_green_bottle_wood),
        ("03_orange_ball_white.jpg", img3_orange_ball_white),
        ("04_hard_orange_shadow.jpg", img4_hard_case),
    ]
    for name, fn in items:
        img = fn()
        p = OUT_DIR / name
        print(f"[OK] {p.relative_to(ROOT)}  {img.shape[1]}x{img.shape[0]}  {os.path.getsize(p) // 1024} KB")


if __name__ == "__main__":
    main()
