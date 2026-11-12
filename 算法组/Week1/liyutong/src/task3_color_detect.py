# 任务三：颜色提取加轮廓定位
# 只跑一张：python src/task3_color_detect.py --image 06_blue_powerbank.jpg
import argparse
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task3"

# 颜色范围：先打印图上目标/背景的 HSV 分位数，再取中间值定的
# 红色在色环上跨 0 和 179，所以要两段
RANGES = {
    "green":      [((33, 40, 140), (55, 110, 255))],
    "orange":     [((7, 100, 60), (22, 255, 255))],
    "blue":       [((100, 65, 60), (125, 255, 200))],
    "blue_light": [((90, 35, 120), (115, 255, 255))],
    "red":        [((0, 60, 25), (8, 255, 160)), ((160, 60, 25), (179, 255, 160))],
    "orange_tight": [((6, 185, 60), (22, 255, 255))],
    "white":      [((0, 0, 150), (179, 70, 255))],
}

IMG_COLOR = {
    "01_green_notebook.jpg": "green",
    "02_orange_cream.jpg": "orange",
    "03_blue_slipper.jpg": "blue_light",
    "04_pen_small_target.jpg": "red",
    "05_orange_on_orange_trap.jpg": "orange_tight",
    "06_blue_powerbank.jpg": "blue",
    "07_white_powerbank_dark.jpg": "white",
}


def imread_cn(path):
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_cn(path, img):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", img)[1].tofile(str(path))


def make_mask(hsv, color):
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for lo, hi in RANGES[color]:
        mask = cv2.bitwise_or(mask, cv2.inRange(hsv, np.array(lo, np.uint8),
                                                np.array(hi, np.uint8)))
    return mask


def detect(img, color):
    h, w = img.shape[:2]
    # 面积下限按比例给，这批图分辨率差得多，写死数字不好用
    min_area = max(600, 0.0008 * w * h)

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = make_mask(hsv, color)

    k3 = np.ones((3, 3), np.uint8)
    k5 = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k3)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k5)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    draw = img.copy()
    found = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < min_area:
            continue
        x, y, ww, hh = cv2.boundingRect(c)
        m = cv2.moments(c)
        if m["m00"] == 0:
            continue
        cx, cy = int(m["m10"] / m["m00"]), int(m["m01"] / m["m00"])
        cv2.rectangle(draw, (x, y), (x + ww, y + hh), (0, 255, 0), 3)
        cv2.circle(draw, (cx, cy), 10, (0, 0, 255), -1)
        cv2.putText(draw, "(%d,%d)" % (cx, cy), (x, max(y - 12, 24)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
        found.append((area, cx, cy, (x, y, ww, hh)))

    found.sort(reverse=True)
    return mask, draw, found, min_area


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default=None)
    args = ap.parse_args()

    if args.image:
        images = [IMG_DIR / Path(args.image).name]
    else:
        images = sorted(p for p in IMG_DIR.iterdir()
                        if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"))

    if not images:
        print("没有图片:", IMG_DIR)
        return 1

    print("cv2", cv2.__version__, "图片数", len(images))
    rows = []
    for p in images:
        color = IMG_COLOR.get(p.name)
        if color is None:
            print("跳过", p.name, "（没配颜色范围）")
            continue

        img = imread_cn(p)
        mask, draw, found, min_area = detect(img, color)
        imwrite_cn(OUT_DIR / p.stem / "mask.png", mask)
        imwrite_cn(OUT_DIR / p.stem / "result.png", draw)

        cover = np.count_nonzero(mask) / mask.size * 100
        print()
        print("[%s] 颜色=%s  尺寸=%dx%d  面积阈值=%.0f" %
              (p.name, color, img.shape[1], img.shape[0], min_area))
        print("  掩膜白色 %.2f%%  框到 %d 个目标" % (cover, len(found)))
        for i, (area, cx, cy, box) in enumerate(found[:3]):
            print("  第%d个: 中心=(%d,%d) 面积=%.0f 外接矩形=%s" % (i + 1, cx, cy, area, box))
        if len(found) > 3:
            print("  还有 %d 个小的没算" % (len(found) - 3))

        main_one = found[0] if found else None
        rows.append((p.name, color, cover, len(found), main_one))

    # 汇总存一份，写报告时直接查
    lines = ["%-32s%-14s%-10s%-8s%-16s%s" % ("图片", "颜色", "掩膜占比", "目标数", "中心", "面积")]
    for name, color, cover, n, m in rows:
        if m:
            lines.append("%-32s%-14s%6.2f%%   %-8d%-16s%.0f" %
                         (name, color, cover, n, "(%d,%d)" % (m[1], m[2]), m[0]))
        else:
            lines.append("%-32s%-14s%6.2f%%   %-8d%-16s%s" % (name, color, cover, n, "-", "-"))
    text = "\n".join(lines)
    print()
    print(text)
    (OUT_DIR / "检测记录.txt").write_text(text, encoding="utf-8")
    print()
    print("输出在", OUT_DIR.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
