# -*- coding: utf-8 -*-
"""
任务三：颜色目标提取与轮廓定位

HSV 阈值 -> 开闭运算 -> 找轮廓 -> 按面积过滤 -> 外接矩形和中心点，
输出 mask_/result_ 图和 detections.csv。

用法：python src/task3_color_detect.py [--color blue|green|orange] [--sweep]
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
OUT_DIR = ROOT / "outputs" / "task3"

# --------------------------------------------------------------------------- #
# 颜色范围
#   OpenCV 的 HSV 取值：H 0~179（即 0~360° 的一半），S 0~255，V 0~255。
#   下面每个范围都是先用 cv2.cvtColor 采样目标区域的实际 H/S/V 分布（p5~p95），
#   再留出余量得到的，不是凭感觉写的：
#     blue   目标 H≈105, S≈160~205, V≈180~250；灰背景 S≈9  -> 主要靠 S 区分
#     green  目标 H≈58~62, S≈98~180, V≈50~180；木纹背景 H≈14 -> 主要靠 H 区分
#     orange 目标 H≈13,   S≈165~235, V≈190~255；白布背景 S≈0  -> 主要靠 S 区分
# --------------------------------------------------------------------------- #
COLOR_RANGES = {
    "blue": {
        "lower": (100, 70, 60),
        "upper": (130, 255, 255),
        "why": "H 100~130 覆盖蓝色相；S>=70 把灰色背景（S≈9）排除；V>=60 保留阴影里的暗像素",
    },
    "green": {
        "lower": (40, 60, 40),
        "upper": (85, 255, 255),
        "why": "H 40~85 覆盖绿/青绿；木纹背景 H≈14 被完全排除，所以 S/V 下限可以放宽",
    },
    "orange": {
        "lower": (5, 150, 120),
        "upper": (22, 255, 255),
        "why": "H 5~22 覆盖橙/橙红；S>=150、V>=120 来自顺光样本，阴影中的暗橙会被漏掉（见 README 失败案例）",
    },
}

# 每张测试图的默认目标颜色（--color 不指定时使用）
DEFAULT_COLOR = {
    "01_blue_cube_gray": "blue",
    "02_green_bottle_wood": "green",
    "03_orange_ball_white": "orange",
    "04_hard_orange_shadow": "orange",
}

MIN_AREA_RATIO = 0.002      # 面积阈值：目标至少占整幅图 0.2%（1280x960 约 2460 像素）
MIN_AREA_ABS = 800          # 面积阈值的绝对下限，避免小图上比例阈值过小


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
def build_mask(hsv, color, s_min=None, s_max=None, v_min=None):
    """按颜色范围生成并清理掩膜，返回 (mask, 说明用的实际范围)。"""
    lo = list(COLOR_RANGES[color]["lower"])
    hi = list(COLOR_RANGES[color]["upper"])
    if s_min is not None:
        lo[1] = s_min
    if s_max is not None:
        hi[1] = s_max
    if v_min is not None:
        lo[2] = v_min

    mask = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
    # 开运算：去掉零星噪点（先腐蚀后膨胀，5x5）；闭运算：连通断裂、填补小孔（9x9）
    k5 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    k9 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k5, iterations=1)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k9, iterations=2)
    return mask, (tuple(lo), tuple(hi))


def find_target(mask, min_area):
    """返回最大目标的描述字典；找不到则返回 None。"""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None
    # 先按面积阈值筛掉噪声区域，再取其中最大的一个
    valid = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not valid:
        return None
    cnt = max(valid, key=cv2.contourArea)

    area = float(cv2.contourArea(cnt))
    x, y, w, h = cv2.boundingRect(cnt)
    rect = cv2.minAreaRect(cnt)                      # ((cx,cy),(w,h),angle)
    (rcx, rcy), (rw, rh), angle = rect

    m = cv2.moments(cnt)                              # 轮廓质心
    cx = float(m["m10"] / m["m00"]) if m["m00"] else float(x + w / 2)
    cy = float(m["m01"] / m["m00"]) if m["m00"] else float(y + h / 2)

    return dict(area=area, bbox=(x, y, w, h), rect=rect,
                rect_center=(float(rcx), float(rcy)),
                rect_size=(float(rw), float(rh)), angle=float(angle),
                centroid=(cx, cy),
                n_candidates=len(valid), n_all=len(contours))


def annotate(img, info, text_lines):
    """在图上画外接矩形、最小外接矩形、中心点，并在顶部写说明文字（ASCII）。"""
    out = img.copy()
    if info is not None:
        x, y, w, h = info["bbox"]
        cv2.rectangle(out, (x, y), (x + w, y + h), (0, 255, 0), 3)

        pts = cv2.boxPoints(info["rect"]).astype(np.int32)
        cv2.drawContours(out, [pts], 0, (0, 255, 255), 2)

        cx, cy = info["centroid"]
        cv2.drawMarker(out, (int(cx), int(cy)), (0, 0, 255), cv2.MARKER_CROSS, 34, 3)
        cv2.circle(out, (int(cx), int(cy)), 16, (0, 0, 255), 2)
    # 顶部半透明黑条 + 文字
    band_h = 100
    strip = out[0:band_h, :].copy()
    cv2.rectangle(strip, (0, 0), (out.shape[1], band_h), (0, 0, 0), -1)
    out[0:band_h, :] = cv2.addWeighted(strip, 0.62, out[0:band_h, :], 0.38, 0)
    for i, line in enumerate(text_lines):
        cv2.putText(out, line, (14, 30 + i * 30), cv2.FONT_HERSHEY_SIMPLEX,
                    0.68, (60, 255, 60), 2, cv2.LINE_AA)
    return out


def sweep_thresholds(hsv, color, stem, min_area, s_vals, v_vals, selected, suffix=""):
    """
    阈值扫描：把 (S_min, V_min) 组合成网格，观察目标面积如何随阈值变化。
    这张图是"HSV 范围怎么定"的直接依据：面积从 0 突然变大的格子就是阈值的临界点；
    继续放宽则背景开始进入掩膜（面积暴涨），说明取值应该落在拐点附近。
    """
    cells = []
    for v in v_vals:
        for s in s_vals:
            m, _ = build_mask(hsv, color, s_min=s, v_min=v)
            info = find_target(m, min_area)
            area = int(info["area"]) if info else 0
            vis = cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
            if info:
                x, y, w, h = info["bbox"]
                cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)
            vis = cv2.resize(vis, (300, 225), interpolation=cv2.INTER_AREA)
            strip = np.zeros((24, 300, 3), np.uint8)
            cv2.putText(strip, f"S>={s} V>={v}  area={area}", (6, 17),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.42, (255, 255, 255), 1, cv2.LINE_AA)
            cell = cv2.vconcat([strip, vis])
            if (s, v) == selected:
                cv2.rectangle(cell, (0, 0), (cell.shape[1] - 1, cell.shape[0] - 1),
                              (0, 200, 255), 2)
            cells.append(cell)
    rows = [cv2.hconcat(cells[i:i + len(s_vals)])
            for i in range(0, len(cells), len(s_vals))]
    grid = cv2.vconcat(rows)
    cv2.putText(grid, "orange box = current setting", (6, grid.shape[0] - 8),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1, cv2.LINE_AA)
    imwrite_u(OUT_DIR / f"sweep_{stem}{suffix}.png", grid)
    return grid


# --------------------------------------------------------------------------- #
def main():
    ap = argparse.ArgumentParser(description="任务三：颜色目标提取与轮廓定位")
    ap.add_argument("--image", help="只处理指定图片文件名")
    ap.add_argument("--color", choices=list(COLOR_RANGES), help="强制指定目标颜色")
    ap.add_argument("--s-min", type=int, help="覆盖饱和度下限（做阈值实验用）")
    ap.add_argument("--s-max", type=int, help="覆盖饱和度上限")
    ap.add_argument("--v-min", type=int, help="覆盖亮度下限")
    ap.add_argument("--min-area-ratio", type=float, default=MIN_AREA_RATIO,
                    help=f"面积阈值占全图比例，默认 {MIN_AREA_RATIO}")
    ap.add_argument("--sweep", action="store_true", help="额外输出饱和度阈值扫描图")
    ap.add_argument("--tag", default="", help="给输出文件名加后缀，便于保存参数实验（如 --tag fix）")
    args = ap.parse_args()

    OUT_DIR.mkdir(parents=True, exist_ok=True)
    suffix = f"_{args.tag}" if args.tag else ""
    if args.image and not (IMG_DIR / args.image).exists():
        print(f"[错误] 图片不存在：{IMG_DIR / args.image}")
        print(f"       现有图片：{', '.join(p.name for p in sorted(IMG_DIR.glob('*')))}")
        return 2
    if args.image:
        files = [IMG_DIR / args.image]
    else:
        files = sorted(p for p in IMG_DIR.iterdir()
                       if p.suffix.lower() in {".jpg", ".jpeg", ".png", ".bmp"})
    if not files:
        print(f"[错误] {IMG_DIR} 下没有图片，请先运行 python src/make_test_images.py")
        return 1

    records = []
    print(f"{'图片':<30}{'颜色':<8}{'命中':<6}{'中心(x,y)':<18}{'宽x高':<12}"
          f"{'面积':>9}{'候选/总轮廓':>12}")
    print("-" * 100)

    for path in files:
        img = imread_u(path)
        if img is None:
            print(f"[错误] 读不到 {path}")
            continue
        stem = path.stem
        color = args.color or DEFAULT_COLOR.get(stem, "blue")
        H, W = img.shape[:2]
        min_area = max(MIN_AREA_ABS, args.min_area_ratio * H * W)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask, used = build_mask(hsv, color, args.s_min, args.s_max, args.v_min)
        info = find_target(mask, min_area)

        imwrite_u(OUT_DIR / f"mask_{stem}{suffix}.png", mask)
        if info:
            x, y, w, h = info["bbox"]
            cx, cy = info["centroid"]
            lines = [
                f"color={color}  found=True  target_area={int(info['area'])}px",
                f"center=({cx:.1f}, {cy:.1f})  bbox=({x},{y},{w},{h})",
                f"minAreaRect {info['rect_size'][0]:.1f}x{info['rect_size'][1]:.1f} "
                f"angle={info['angle']:.1f}",
            ]
            # 先把要打印的片段算成变量，避免在 f-string 里再嵌套 f-string
            # （嵌套同类引号的写法只有 Python 3.12 支持，3.10 会直接 SyntaxError）
            center_txt = "({:.1f}, {:.1f})".format(cx, cy)
            size_txt = "{}x{}".format(w, h)
            cand_txt = "{}/{}".format(info["n_candidates"], info["n_all"])
            print(f"{path.name:<30}{color:<8}{'是':<6}{center_txt:<18}"
                  f"{size_txt:<12}{int(info['area']):>9}{cand_txt:>14}")
            records.append(dict(image=path.name, color=color, found=1,
                                center_x=f"{cx:.1f}", center_y=f"{cy:.1f}",
                                bbox_x=x, bbox_y=y, bbox_w=w, bbox_h=h,
                                area_px=int(info["area"]),
                                area_ratio=f"{info['area'] / (H * W):.4f}",
                                rect_w=f"{info['rect_size'][0]:.1f}",
                                rect_h=f"{info['rect_size'][1]:.1f}",
                                angle=f"{info['angle']:.1f}",
                                candidates=f"{info['n_candidates']}/{info['n_all']}",
                                hsv_lower=str(used[0]), hsv_upper=str(used[1]),
                                min_area=int(min_area)))
        else:
            # 注意：cv2.putText 只支持 ASCII 字形，中文会变成 "???", 所以图上写英文
            lines = [f"color={color}  found=False  (no region passed area filter)",
                     f"HSV lower={used[0]} upper={used[1]}",
                     f"min_area={int(min_area)}px"]
            print(f"{path.name:<30}{color:<8}{'否':<6}{'-':<18}{'-':<12}{0:>9}{'-':>12}")
            records.append(dict(image=path.name, color=color, found=0,
                                center_x="-", center_y="-", bbox_x="-", bbox_y="-",
                                bbox_w="-", bbox_h="-", area_px=0, area_ratio="0",
                                rect_w="-", rect_h="-", angle="-", candidates="-",
                                hsv_lower=str(used[0]), hsv_upper=str(used[1]),
                                min_area=int(min_area)))

        imwrite_u(OUT_DIR / f"result_{stem}{suffix}.png", annotate(img, info, lines))

        if args.sweep:
            # 以当前取值为最严格的一角，向"更宽松"方向各取 3 档，共 3x3 组合
            s_cur, v_cur = used[0][1], used[0][2]
            s_vals = [max(0, s_cur - 90), max(0, s_cur - 45), s_cur]
            v_vals = [max(0, v_cur - 80), max(0, v_cur - 40), v_cur]
            sweep_thresholds(hsv, color, stem, min_area, s_vals, v_vals, (s_cur, v_cur), suffix)

    # ---------- CSV ----------
    cols = ["image", "color", "found", "center_x", "center_y", "bbox_x", "bbox_y",
            "bbox_w", "bbox_h", "area_px", "area_ratio", "rect_w", "rect_h",
            "angle", "candidates", "hsv_lower", "hsv_upper", "min_area"]
    csv_path = OUT_DIR / f"detections{suffix}.csv"
    with open(csv_path, "w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=cols)
        w.writeheader()
        w.writerows(records)

    n_ok = sum(r["found"] for r in records)
    print("-" * 100)
    print(f"[结果] {n_ok}/{len(records)} 张图片检测到目标")
    print(f"[输出] 掩膜/标注图 -> {OUT_DIR.relative_to(ROOT)}")
    print(f"[输出] 坐标与面积表 -> {csv_path.relative_to(ROOT)}")
    print(f"[依据] {COLOR_RANGES[args.color or 'orange']['why']}")
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

