"""任务 2：检测结果可视化 + 参数对比实验。

运行：
    cd 算法组/Week2/maohaoran
    python src/task2_visualize.py

输出到 outputs/task2/：
    vis_<图名>.png           每张图的标注结果（轮廓/外接矩形/中心点/坐标）
    02_对比_A1_S下限.png      S 下限 40 vs 80（误检案例）
    02_对比_A2_H下限.png      H 下限 100 vs 110（漏检案例）
    02_对比_B_最小面积.png    最小面积 2000 vs 5000（小目标漏检）
    02_对比_C_形态学.png      开运算 关闭 vs 11x11（噪点）
    02_对比_D_光照.png        目标清晰 vs 强高光（mask 空洞）
    compare_report.txt        每组的参数与数值结果
"""

from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from color_detector import (DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                            DEFAULT_MIN_AREA, detect)
from image_io import imread_unicode, imwrite_unicode

ROOT = Path(__file__).resolve().parent.parent
IMAGE_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task2"

FONT_PATH = r"C:\Windows\Fonts\msyh.ttc"


def draw_cn(bgr, text, xy, size=24, color=(0, 0, 255)):
    """在图上写中文（cv2.putText 不支持中文，用 PIL 画）。color 为 BGR。"""
    pil_img = Image.fromarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
    drawer = ImageDraw.Draw(pil_img)
    font = ImageFont.truetype(FONT_PATH, size)
    drawer.text(xy, text, font=font,
                fill=(color[2], color[1], color[0]),
                stroke_width=2, stroke_fill=(255, 255, 255))
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)


def annotate(bgr, result):
    """画轮廓（绿）、外接矩形（蓝）、中心点（红）和坐标文字。"""
    img = bgr.copy()

    if result.detected:
        cv2.drawContours(img, [result.contour], -1, (0, 255, 0), 2)
        x, y, w, h = result.bbox
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

        cx, cy = int(round(result.u)), int(round(result.v))
        cv2.circle(img, (cx, cy), 6, (0, 0, 255), -1)
        cv2.line(img, (cx - 18, cy), (cx + 18, cy), (0, 0, 255), 1)
        cv2.line(img, (cx, cy - 18), (cx, cy + 18), (0, 0, 255), 1)

        label = f"u={result.u:.1f}, v={result.v:.1f}, area={result.area:.0f}"
        label_y = max(4, y - 32)
        img = draw_cn(img, label, (max(4, x), label_y), 22, (0, 0, 255))
    else:
        text = "未检测到目标"
        img = draw_cn(img, text, (img.shape[1] // 2 - 110, img.shape[0] // 2 - 30),
                      42, (0, 0, 255))

    return img


def panel(img, title, width=620):
    """给图片加一条白色标题栏，并缩放固定宽度。"""
    h, w = img.shape[:2]
    new_h = int(h * width / w)
    img = cv2.resize(img, (width, new_h), interpolation=cv2.INTER_AREA)

    bar = np.full((44, width, 3), 255, dtype=np.uint8)
    bar = draw_cn(bar, title, (8, 8), 24, (0, 0, 0))
    return np.vstack([bar, img])


def side_by_side(items, path):
    """items: [(标题, 图片), ...]，纵向对齐后横向拼接后保存。"""
    panels = [panel(img, title) for title, img in items]
    height = max(p.shape[0] for p in panels)
    padded = [cv2.copyMakeBorder(p, 0, height - p.shape[0], 0, 0,
                                 cv2.BORDER_CONSTANT, value=(255, 255, 255))
              for p in panels]
    combined = np.hstack(padded)
    imwrite_unicode(path, combined)


def run_detect(bgr, lower, upper, min_area, use_morphology, open_size, close_size):
    """只返回 DetectionResult，mask 在本脚本里用不到。"""
    result, _ = detect(bgr, lower=lower, upper=upper, min_area=min_area,
                       use_morphology=use_morphology,
                       open_size=open_size, close_size=close_size)
    return result


def describe(tag, result):
    if result.detected:
        return (f"{tag}: detected=True u={result.u:.1f} v={result.v:.1f} "
                f"area={result.area:.0f} candidates={len(result.candidates)}")
    return f"{tag}: detected=False u=0.0 v=0.0 area=0 candidates=0"


def main():
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # ---------- 1. 每张图的标注结果 ----------
    images = sorted(p for p in IMAGE_DIR.iterdir()
                    if p.suffix.lower() in {".jpg", ".jpeg", ".png"})
    for path in images:
        bgr = imread_unicode(path)
        if bgr is None:
            print(f"读取失败: {path.name}")
            continue
        result, _ = detect(bgr)
        vis = annotate(bgr, result)
        imwrite_unicode(OUT_DIR / f"vis_{path.stem}.png", vis)
        print(f"已标注 {path.name}: "
              + ("检测到 (u=%.1f, v=%.1f)" % (result.u, result.v)
                 if result.detected else "未检测到目标"))

    report = []
    base = dict(lower=DEFAULT_HSV_LOWER, upper=DEFAULT_HSV_UPPER,
                min_area=DEFAULT_MIN_AREA, use_morphology=True,
                open_size=5, close_size=5)

    # ---------- 2. 对比 A1：S 下限（误检） ----------
    bgr = imread_unicode(IMAGE_DIR / "05_similar_bg.jpg")
    loose = run_detect(bgr, (100, 40, 40), (130, 255, 255), 2000, True, 5, 5)
    strict = run_detect(bgr, (100, 80, 40), (130, 255, 255), 2000, True, 5, 5)
    side_by_side([
        ("S下限=40：背景被当成目标（误检）", annotate(bgr, loose)),
        ("S下限=80：正确选中蓝色圆", annotate(bgr, strict)),
    ], OUT_DIR / "02_对比_A1_S下限.png")
    report += ["对比 A1：05_similar_bg.jpg，只改 S 下限",
               describe("  S>=40", loose), describe("  S>=80", strict),
               "  现象：S>=40 时蓝灰色背景进入 mask，面积 144145 的背景成为最大轮廓，"
               "中心点跑到 (710,300)，属于误检。", ""]

    # ---------- 3. 对比 A2：H 下限（漏检） ----------
    h100 = run_detect(bgr, (100, 80, 40), (130, 255, 255), 2000, True, 5, 5)
    h110 = run_detect(bgr, (110, 80, 40), (130, 255, 255), 2000, True, 5, 5)
    side_by_side([
        ("H下限=100：目标 H=109 被包含", annotate(bgr, h100)),
        ("H下限=110：目标 H=109 被排除（漏检）", annotate(bgr, h110)),
    ], OUT_DIR / "02_对比_A2_H下限.png")
    report += ["对比 A2：05_similar_bg.jpg，只改 H 下限",
               describe("  H>=100", h100), describe("  H>=110", h110),
               "  现象：目标蓝色像素 H=109，阈值收紧到 110 后整个目标被排除，"
               "detected=False，属于漏检。", ""]

    # ---------- 4. 对比 B：最小面积（小目标漏检） ----------
    bgr = imread_unicode(IMAGE_DIR / "04_small.jpg")
    area2000 = run_detect(bgr, DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER, 2000, True, 5, 5)
    area5000 = run_detect(bgr, DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER, 5000, True, 5, 5)
    side_by_side([
        ("最小面积=2000：小目标(3116)保留", annotate(bgr, area2000)),
        ("最小面积=5000：小目标被过滤（漏检）", annotate(bgr, area5000)),
    ], OUT_DIR / "02_对比_B_最小面积.png")
    report += ["对比 B：04_small.jpg，只改最小面积",
               describe("  min_area=2000", area2000), describe("  min_area=5000", area5000),
               "  现象：小目标面积 3116，阈值提到 5000 后没有候选轮廓，"
               "整张图报未检测到目标属于合理的拒检，但对真实存在的小目标就是漏检。", ""]

    # ---------- 5. 对比 C：形态学开运算（噪点） ----------
    bgr = imread_unicode(IMAGE_DIR / "07_noisy.jpg")
    no_open = run_detect(bgr, DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                         10, use_morphology=False, open_size=1, close_size=1)
    with_open = run_detect(bgr, DEFAULT_HSV_LOWER, DEFAULT_HSV_UPPER,
                           10, use_morphology=True, open_size=11, close_size=5)
    side_by_side([
        ("开运算关闭：100 个候选轮廓", annotate(bgr, no_open)),
        ("开运算 11x11：只剩 1 个目标", annotate(bgr, with_open)),
    ], OUT_DIR / "02_对比_C_形态学.png")
    report += ["对比 C：07_noisy.jpg，最小面积固定为 10，只改开运算",
               describe("  无形态学", no_open), describe("  开运算11x11", with_open),
               "  现象：无开运算时 mask 中保留大量蓝色噪点，候选轮廓 100 个；"
               "11x11 开运算把直径小于 11 像素的噪点全部腐蚀掉，候选降为 1 个。",
               "  注意：本轮把最小面积临时设为 10，否则面积过滤会先把噪点去掉，"
               "看不出开运算本身的作用。", ""]

    # ---------- 6. 对比 D：目标清晰 vs 强高光 ----------
    clear_img = imread_unicode(IMAGE_DIR / "sample.jpg")
    glare_img = imread_unicode(IMAGE_DIR / "sample3.jpg")
    clear_res, _ = detect(clear_img)
    glare_res, glare_mask = detect(glare_img)

    # 目标外轮廓内部、但没被阈值选中的像素比例（即"洞"占目标面积的比例）
    filled = np.zeros_like(glare_mask)
    cv2.drawContours(filled, [glare_res.contour], -1, 255, -1)
    inside_filled = np.count_nonzero(filled)
    hole_pixels = np.count_nonzero(cv2.bitwise_and(
        filled, cv2.bitwise_not(glare_mask)))
    hole_ratio = hole_pixels / inside_filled
    side_by_side([
        ("目标清晰：mask 完整", annotate(clear_img, clear_res)),
        ("强高光：mask 内部有洞，中心仍稳定", annotate(glare_img, glare_res)),
    ], OUT_DIR / "02_对比_D_光照.png")
    report += ["对比 D：sample.jpg vs sample3.jpg（不调参数，对比光照）",
               describe("  清晰图", clear_res), describe("  高光图", glare_res),
               f"  高光图轮廓内部有 {hole_ratio * 100:.1f}% 的像素没被阈值选中（高光洞）。",
               "  现象：高光处 S 只有 28（低于 S 下限 80），mask 中出现空洞；"
               "但 findContours 用 RETR_EXTERNAL 只取外轮廓，"
               "所以中心点和面积基本不受影响。若高光分裂目标或遮挡边缘，中心点才会偏移。", ""]

    report_path = OUT_DIR / "compare_report.txt"
    report_path.write_text("\n".join(report) + "\n", encoding="utf-8")
    print(f"\n对比实验记录已保存: {report_path}")
    print("对比图已保存到 outputs/task2/")


if __name__ == "__main__":
    main()
