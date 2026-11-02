# -*- coding: utf-8 -*-
"""
任务二：滤波、阈值、边缘、形态学对比

输出各种中间结果和一张总览图，关键指标写进 metrics.csv；
另外加噪声做了一组滤波对比。

用法：python src/task2_processing.py [--image 图片名]
"""

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")

ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task2"

DEFAULT_IMAGE = "01_blue_cube_gray.jpg"

# ---- 本脚本用到的全部参数（改这里即可做参数对比实验）----
PARAMS = {
    "mean_ksize": 5,                       # 均值滤波窗口
    "gauss_ksize_small": 5,                # 高斯滤波（小窗口）
    "gauss_ksize_large": 15,               # 高斯滤波（大窗口）
    "median_ksize": 5,                     # 中值滤波窗口
    "thresh_levels": [80, 127, 180],       # 固定阈值
    "adaptive_block": 71,                  # 自适应阈值邻域
    "adaptive_C": 5,                       # 自适应阈值偏移量
    "canny_low": (30, 90),                 # Canny 双阈值（低阈值配置，边缘更多）
    "canny_high": (120, 240),              # Canny 双阈值（高阈值配置，只留强边缘）
    "morph_ksize": 5,                      # 形态学核大小
    "sp_ratio": 0.02,                      # 噪声实验：椒盐噪声比例
    "gauss_sigma": 8.0,                    # 噪声实验：高斯噪声标准差
}

PANEL_SIZE = (400, 300)                    # 总览图每格大小 (w, h)


def imread_u(path, flags=cv2.IMREAD_COLOR):
    """读图；文件不存在/为空/损坏时返回 None（不抛异常）。"""
    try:
        data = np.fromfile(str(path), dtype=np.uint8)
    except OSError:
        return None
    return None if data.size == 0 else cv2.imdecode(data, flags)


def imwrite_u(path, img):
    ok, buf = cv2.imencode(Path(path).suffix or ".png", img)
    if ok:
        buf.tofile(str(path))
    return bool(ok)


# --------------------------------------------------------------------------- #
# 量化指标
# --------------------------------------------------------------------------- #
def noise_std(gray, roi=(20, 20, 200, 200)):
    """背景平坦区域的灰度标准差，用来衡量噪声 / 平滑程度。"""
    x, y, w, h = roi
    patch = gray[y:y + h, x:x + w]
    return float(patch.std())


def sharpness(gray):
    """拉普拉斯方差，越大越锐利（越小越模糊）。"""
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def fg_ratio(mask):
    """二值图前景像素占比。"""
    return float((mask > 0).mean())


def blob_count(mask, min_area=50):
    """连通域个数（忽略过小的噪点）。"""
    n, _, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    return int(sum(1 for i in range(1, n) if stats[i, cv2.CC_STAT_AREA] >= min_area))


def edge_count(edge):
    return int((edge > 0).sum())


# --------------------------------------------------------------------------- #
# 总览图拼接
# --------------------------------------------------------------------------- #
def to_panel(img, title):
    """把任意图转成 BGR 面板，顶部留出 26px 黑条写英文标题。"""
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    w, h = PANEL_SIZE
    panel = cv2.resize(img, (w, h), interpolation=cv2.INTER_AREA)
    strip = np.zeros((26, w, 3), np.uint8)
    cv2.putText(strip, title, (8, 19), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (255, 255, 255), 1, cv2.LINE_AA)
    return cv2.vconcat([strip, panel])


def build_grid(panels, cols=4):
    cells = [to_panel(im, t) for t, im in panels]
    while len(cells) % cols:
        cells.append(np.zeros_like(cells[0]))
    rows = [cv2.hconcat(cells[i:i + cols]) for i in range(0, len(cells), cols)]
    grid = cv2.vconcat(rows)
    cv2.rectangle(grid, (0, 0), (grid.shape[1] - 1, grid.shape[0] - 1), (90, 90, 90), 1)
    return grid


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="任务二：基础图像处理对比")
    ap.add_argument("--image", default=DEFAULT_IMAGE, help=f"图片文件名，默认 {DEFAULT_IMAGE}")
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    src = IMG_DIR / args.image
    img = imread_u(src)
    if img is None:
        print(f"[错误] 读不到图片：{src}")
        return 1

    stem = src.stem
    p = PARAMS
    print(f"[输入] {src.relative_to(ROOT)}  {img.shape[1]}x{img.shape[0]}")

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    rows = []          # 写入 CSV 的指标
    panels = []        # 总览图内容

    def save(name, image):
        imwrite_u(OUT_DIR / name, image)

    # ---------- 0. 原图与灰度图 ----------
    save(f"{stem}_00_original.png", img)
    save(f"{stem}_01_gray.png", gray)
    panels += [("00 original", img), ("01 gray", gray)]
    rows.append(dict(step="gray", param="-", noise_std=f"{noise_std(gray):.2f}",
                     sharpness=f"{sharpness(gray):.1f}", edge_pixels=edge_count(
                         cv2.Canny(gray, *p["canny_low"])),
                     fg_ratio="-", blobs="-"))

    # ---------- 1. 四种滤波 ----------
    f_mean = cv2.blur(gray, (p["mean_ksize"], p["mean_ksize"]))
    f_g5 = cv2.GaussianBlur(gray, (p["gauss_ksize_small"], p["gauss_ksize_small"]), 0)
    f_g15 = cv2.GaussianBlur(gray, (p["gauss_ksize_large"], p["gauss_ksize_large"]), 0)
    f_med = cv2.medianBlur(gray, p["median_ksize"])

    filters = [
        ("02", "mean_k5", "02 mean k=5", f_mean, "mean_ksize=5"),
        ("03", "gauss_k5", "03 gauss k=5", f_g5, "gauss_ksize=5"),
        ("04", "gauss_k15", "04 gauss k=15", f_g15, "gauss_ksize=15"),
        ("05", "median_k5", "05 median k=5", f_med, "median_ksize=5"),
    ]
    for code, tag, title, im, plabel in filters:
        save(f"{stem}_{code}_{tag}.png", im)
        panels.append((title, im))
        rows.append(dict(step=tag, param=plabel,
                         noise_std=f"{noise_std(im):.2f}",
                         sharpness=f"{sharpness(im):.1f}",
                         edge_pixels=edge_count(cv2.Canny(im, *p["canny_low"])),
                         fg_ratio="-", blobs="-"))

    # ---------- 2. 固定阈值 / Otsu ----------
    # 本图的目标（蓝色块）比浅灰背景暗，所以阈值化要用 *_INV：
    # 亮背景（≈150~180）被判为 0，暗目标（≈100~140）被判为 255。
    # 若用普通 THRESH_BINARY，得到的是"背景"而不是目标 —— 这正是极性选择的重要性。
    otsu_val, otsu_mask = cv2.threshold(gray, 0, 255,
                                        cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
    save(f"{stem}_06_otsu_inv.png", otsu_mask)
    panels.append((f"06 otsu inv (T={otsu_val:.0f})", otsu_mask))
    rows.append(dict(step="otsu_inv", param=f"threshold={otsu_val:.0f}",
                     noise_std="-", sharpness="-", edge_pixels="-",
                     fg_ratio=f"{fg_ratio(otsu_mask):.4f}",
                     blobs=blob_count(otsu_mask)))

    for lvl in p["thresh_levels"]:
        _, m = cv2.threshold(gray, lvl, 255, cv2.THRESH_BINARY_INV)
        save(f"{stem}_07_thresh_inv_{lvl}.png", m)
        panels.append((f"07 thresh inv T={lvl}", m))
        rows.append(dict(step="fixed_thresh_inv", param=f"T={lvl}",
                         noise_std="-", sharpness="-", edge_pixels="-",
                         fg_ratio=f"{fg_ratio(m):.4f}", blobs=blob_count(m)))

    # ---------- 3. 自适应阈值 ----------
    adap = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                 cv2.THRESH_BINARY_INV, p["adaptive_block"], p["adaptive_C"])
    save(f"{stem}_08_adaptive.png", adap)
    panels.append((f"08 adaptive b={p['adaptive_block']}", adap))
    rows.append(dict(step="adaptive", param=f"block={p['adaptive_block']},C={p['adaptive_C']}",
                     noise_std="-", sharpness="-", edge_pixels="-",
                     fg_ratio=f"{fg_ratio(adap):.4f}", blobs=blob_count(adap)))

    # ---------- 4. Canny 两组参数 ----------
    e1 = cv2.Canny(gray, *p["canny_low"])
    e2 = cv2.Canny(gray, *p["canny_high"])
    save(f"{stem}_09_canny_{p['canny_low'][0]}_{p['canny_low'][1]}.png", e1)
    save(f"{stem}_10_canny_{p['canny_high'][0]}_{p['canny_high'][1]}.png", e2)
    panels += [(f"09 canny {p['canny_low'][0]}/{p['canny_low'][1]}", e1),
               (f"10 canny {p['canny_high'][0]}/{p['canny_high'][1]}", e2)]
    for edge, tag, plabel in ((e1, "canny_low", f"{p['canny_low'][0]}/{p['canny_low'][1]}"),
                              (e2, "canny_high", f"{p['canny_high'][0]}/{p['canny_high'][1]}")):
        rows.append(dict(step="canny", param=tag + "=" + plabel,
                         noise_std="-", sharpness="-",
                         edge_pixels=edge_count(edge), fg_ratio="-", blobs="-"))

    # ---------- 5. 形态学：开运算去噪点，闭运算补空洞 ----------
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (p["morph_ksize"], p["morph_ksize"]))
    opened = cv2.morphologyEx(otsu_mask, cv2.MORPH_OPEN, k, iterations=1)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, k, iterations=2)
    save(f"{stem}_11_open.png", opened)
    save(f"{stem}_12_open_close.png", closed)
    panels += [("11 open 5x5", opened), ("12 open+close 5x5", closed)]

    for im, tag, plabel in ((opened, "open", "open 5x5"),
                            (closed, "open_close", "open+close 5x5")):
        rows.append(dict(step=tag, param=plabel, noise_std="-", sharpness="-",
                         edge_pixels="-", fg_ratio=f"{fg_ratio(im):.4f}",
                         blobs=blob_count(im)))

    # ---------- 7. 补充实验：噪声鲁棒性 ----------
    # 干净图上四种滤波差别很小，为了把"滤波到底解决了什么"量化出来，
    # 人为加入椒盐噪声 + 高斯噪声，再比较三种滤波的去噪效果。
    rnd = np.random.default_rng(7)
    noisy = gray.astype(np.uint8).copy()
    pepper = rnd.random(noisy.shape) < p["sp_ratio"]
    salt = rnd.random(noisy.shape) < p["sp_ratio"]
    noisy[pepper] = 0
    noisy[salt] = 255
    noisy = np.clip(noisy.astype(np.float32) + rnd.normal(0, p["gauss_sigma"], noisy.shape),
                    0, 255).astype(np.uint8)

    n_mean = cv2.blur(noisy, (p["mean_ksize"], p["mean_ksize"]))
    n_g5 = cv2.GaussianBlur(noisy, (p["gauss_ksize_small"], p["gauss_ksize_small"]), 0)
    n_med = cv2.medianBlur(noisy, p["median_ksize"])

    n_rows = []
    n_panels = []

    def add_noise_row(tag, im, plabel):
        _, m = cv2.threshold(im, 0, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        n_rows.append(dict(step=tag, param=plabel,
                           noise_std=f"{noise_std(im):.2f}",
                           edge_pixels=edge_count(cv2.Canny(im, *p["canny_low"])),
                           otsu_blobs=blob_count(m), fg_ratio=f"{fg_ratio(m):.4f}"))
        n_panels.append((tag, im))

    for tag, im, pl in (("noisy", noisy, f"sp={p['sp_ratio']}, gauss={p['gauss_sigma']}"),
                        ("noisy+mean5", n_mean, "mean k=5"),
                        ("noisy+gauss5", n_g5, "gauss k=5"),
                        ("noisy+median5", n_med, "median k=5")):
        save(f"{stem}_noise_{tag.replace('+', '_')}.png", im)
        add_noise_row(tag, im, pl)

    # 先平滑再 Canny：噪声图上差别很明显
    e_noisy = cv2.Canny(noisy, *p["canny_low"])
    e_noisy_g = cv2.Canny(n_g5, *p["canny_low"])
    save(f"{stem}_noise_canny_raw.png", e_noisy)
    save(f"{stem}_noise_canny_gauss.png", e_noisy_g)
    for tag, im in (("canny(noisy)", e_noisy), ("canny(gauss+noisy)", e_noisy_g)):
        n_rows.append(dict(step=tag, param=f"canny {p['canny_low'][0]}/{p['canny_low'][1]}",
                           noise_std="-", edge_pixels=edge_count(im),
                           otsu_blobs="-", fg_ratio="-"))
        n_panels.append((tag, im))

    noise_csv = OUT_DIR / "metrics_noise.csv"
    with open(noise_csv, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["step", "param", "noise_std", "edge_pixels",
                                          "otsu_blobs", "fg_ratio"])
        w.writeheader()
        w.writerows(n_rows)
    save(f"noise_grid.png", build_grid(n_panels, cols=3))

    print(f"\n{'噪声实验':<20}{'参数':<24}{'噪声σ':>8}{'边缘像素':>10}{'Otsu连通域':>12}")
    print("-" * 78)
    for r in n_rows:
        print(f"{r['step']:<20}{r['param']:<24}{r['noise_std']:>8}"
              f"{r['edge_pixels']:>10}{r['otsu_blobs']:>12}")

    # ---------- 8. 总览图 ----------
    grid = build_grid(panels, cols=4)
    save(f"{stem}_comparison_grid.png", grid)

    # ---------- 9. CSV 与控制台表格 ----------
    csv_path = OUT_DIR / "metrics.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["step", "param", "noise_std", "sharpness",
                                          "edge_pixels", "fg_ratio", "blobs"])
        w.writeheader()
        w.writerows(rows)

    print(f"\n{'步骤':<16}{'参数':<22}{'噪声σ':>8}{'锐度(拉氏方差)':>16}"
          f"{'边缘像素':>10}{'前景占比':>10}{'连通域':>8}")
    print("-" * 92)
    for r in rows:
        print(f"{r['step']:<16}{r['param']:<22}{r['noise_std']:>8}{r['sharpness']:>16}"
              f"{r['edge_pixels']:>10}{r['fg_ratio']:>10}{r['blobs']:>8}")

    print(f"\n[输出] {len(panels) + 1} 张主流程图片 -> {OUT_DIR.relative_to(ROOT)}")
    print(f"[输出] 指标表 -> {(OUT_DIR / 'metrics.csv').relative_to(ROOT)}、"
          f"{(OUT_DIR / 'metrics_noise.csv').relative_to(ROOT)}")
    print(f"[提示] Otsu 自动阈值 = {otsu_val:.0f}；总览图见 "
          f"{(OUT_DIR / (stem + '_comparison_grid.png')).relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except SystemExit:
        raise
    except Exception:
        # 双击运行时窗口会立刻关闭，所以把错误同时写进 error.txt 备查
        import traceback
        tb = traceback.format_exc()
        print(tb)
        try:
            (ROOT / "error.txt").write_text(tb, encoding="utf-8")
            print(f"[错误已保存] {ROOT / 'error.txt'}")
        except Exception:
            pass
        sys.exit(1)
