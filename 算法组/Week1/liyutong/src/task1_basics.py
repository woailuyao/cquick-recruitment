# 任务一：读图、缩放、裁剪、灰度，存图后回读验证
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task1"

# 每张图要裁的细节区域，(y1, y2, x1, x2)
ROI = {
    "01_green_notebook.jpg": (280, 720, 1100, 1660),          # 右上角标签
    "02_orange_cream.jpg": (300, 1150, 1980, 2860),           # 果蒂和果皮纹路
    "03_blue_slipper.jpg": (1400, 2250, 1000, 2700),          # 鞋面网格
    "04_pen_small_target.jpg": (1620, 2520, 1300, 1780),      # 笔身中段
    "05_orange_on_orange_trap.jpg": (650, 1650, 650, 1800),   # 果蒂特写
    "06_blue_powerbank.jpg": (150, 750, 1200, 1950),          # 二维码
    "07_white_powerbank_dark.jpg": (900, 1650, 1000, 2000),   # 印刷文字
}


def imread_cn(path, flag=cv2.IMREAD_COLOR):
    # 项目路径里有中文，cv2.imread 会返回 None，只能绕一下
    data = np.fromfile(str(path), dtype=np.uint8)
    if data.size == 0:
        print("文件不存在:", path)
        return None
    return cv2.imdecode(data, flag)


def imwrite_cn(path, img):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", img)[1].tofile(str(path))


def info(tag, img):
    h, w = img.shape[:2]
    c = img.shape[2] if img.ndim == 3 else 1
    print("  %-8s 宽=%d 高=%d 通道=%d dtype=%s" % (tag, w, h, c, img.dtype))


def resize_keep(img, target_w=640):
    # 等比缩放，直接给 (640, 480) 会拉伸变形
    h, w = img.shape[:2]
    target_h = int(round(h * target_w / w))
    return cv2.resize(img, (target_w, target_h), interpolation=cv2.INTER_AREA)


def crop_center(img):
    h, w = img.shape[:2]
    ch, cw = int(h * 0.5), int(w * 0.5)
    y1, x1 = (h - ch) // 2, (w - cw) // 2
    return img[y1:y1 + ch, x1:x1 + cw]      # 先 y 后 x


def crop_roi(img, name):
    # 指定区域：每张图手动标了目标身上的某个细节，(y1, y2, x1, x2)
    # 之前用固定比例裁右上角，结果拖鞋那张裁出来全是木纹，只能逐张标
    box = ROI.get(name)
    h, w = img.shape[:2]
    if box is None:
        box = (int(h * 0.3), int(h * 0.7), int(w * 0.3), int(w * 0.7))
    y1, y2, x1, x2 = box
    return img[y1:y2, x1:x2]


def four_panel(a, b, c, g):
    size = (320, 320)
    g3 = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)    # 灰度是 1 通道，拼之前转回三通道
    tiles = [cv2.resize(x, size) for x in (a, b, c, g3)]
    return np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:])])


def main():
    print("cv2", cv2.__version__)
    images = sorted(p for p in IMG_DIR.iterdir()
                    if p.suffix.lower() in (".jpg", ".jpeg", ".png", ".bmp"))
    if not images:
        print("没有图片:", IMG_DIR)
        return 1
    print("共 %d 张图" % len(images))

    for p in images:
        print()
        print("[%s]" % p.name)

        img = imread_cn(p)
        if img is None:
            continue
        print("  shape =", img.shape, "（高, 宽, 通道）")
        info("原图", img)

        small = resize_keep(img, 640)
        mid = crop_center(img)
        roi = crop_roi(img, p.name)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        info("缩放后", small)
        info("中心裁剪", mid)
        info("指定区域", roi)
        info("灰度图", gray)

        stem = p.stem
        files = {
            "缩放": OUT_DIR / (stem + "_resized.png"),
            "裁剪": OUT_DIR / (stem + "_cropped.png"),
            "区域": OUT_DIR / (stem + "_roi.png"),
            "灰度": OUT_DIR / (stem + "_gray.png"),
            "对比": OUT_DIR / (stem + "_compare.png"),
        }
        imwrite_cn(files["缩放"], small)
        imwrite_cn(files["裁剪"], mid)
        imwrite_cn(files["区域"], roi)
        imwrite_cn(files["灰度"], gray)
        imwrite_cn(files["对比"], four_panel(img, small, mid, gray))

        # 存完再读回来，确认不是假成功
        # 这里要用 IMREAD_UNCHANGED，否则灰度图会被读成 3 通道
        for tag, f in files.items():
            back = imread_cn(f, cv2.IMREAD_UNCHANGED)
            c = back.shape[2] if back.ndim == 3 else 1
            print("  已存%s -> %s  回读 %s 通道=%d" % (tag, f.name, back.shape, c))

    print()
    print("输出在", OUT_DIR.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
