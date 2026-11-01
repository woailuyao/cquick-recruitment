# 任务二：滤波、阈值、边缘、形态学对比
import argparse
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
IMG_DIR = ROOT / "data" / "images"
OUT_DIR = ROOT / "outputs" / "task2"


def imread_cn(path):
    # cv2.imread 读不了中文路径，先让 numpy 读成字节再解码
    data = np.fromfile(str(path), dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_cn(path, img):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", img)[1].tofile(str(path))


def white_ratio(binary):
    return np.count_nonzero(binary) / binary.size * 100


def join(items, size=(400, 400)):
    # 横向拼图，左上角标个英文名，cv2.putText 不支持中文
    out = []
    for name, img in items:
        t = cv2.resize(img, size)
        if t.ndim == 2:
            t = cv2.cvtColor(t, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(t, (0, 0), (size[0], 24), (0, 0, 0), -1)
        cv2.putText(t, name, (6, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
        out.append(t)
    return np.hstack(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", default="02_orange_cream.jpg")
    args = ap.parse_args()

    name = Path(args.image).name
    img = imread_cn(IMG_DIR / name)
    if img is None:
        print("读图失败:", IMG_DIR / name)
        return 1
    print("图片:", name, img.shape)

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    g5 = cv2.GaussianBlur(gray, (5, 5), 0)
    g15 = cv2.GaussianBlur(gray, (15, 15), 0)

    _, th127 = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
    _, th180 = cv2.threshold(gray, 180, 255, cv2.THRESH_BINARY)
    otsu_v, th_otsu = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
    th_ad = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, 11, 2)

    canny_a = cv2.Canny(gray, 50, 150)
    canny_b = cv2.Canny(gray, 30, 90)

    # 形态学做在自适应阈值的结果上，那张噪点多，开闭运算的效果才看得出来
    k = np.ones((5, 5), np.uint8)
    opened = cv2.morphologyEx(th_ad, cv2.MORPH_OPEN, k)
    closed = cv2.morphologyEx(th_ad, cv2.MORPH_CLOSE, k)

    imwrite_cn(OUT_DIR / "01_gray.png", gray)
    imwrite_cn(OUT_DIR / "02_gauss5.png", g5)
    imwrite_cn(OUT_DIR / "03_threshold_127.png", th127)
    imwrite_cn(OUT_DIR / "04_canny_50_150.png", canny_a)
    imwrite_cn(OUT_DIR / "05_morph_open.png", opened)
    imwrite_cn(OUT_DIR / "06_otsu.png", th_otsu)
    imwrite_cn(OUT_DIR / "07_adaptive.png", th_ad)
    imwrite_cn(OUT_DIR / "08_morph_close.png", closed)

    imwrite_cn(OUT_DIR / "compare_gauss.png",
               join([("gray", gray), ("gauss 5", g5), ("gauss 15", g15)]))
    imwrite_cn(OUT_DIR / "compare_threshold.png",
               join([("thresh 127", th127), ("thresh 180", th180),
                     ("otsu %.0f" % otsu_v, th_otsu), ("adaptive", th_ad)]))
    imwrite_cn(OUT_DIR / "compare_canny.png",
               join([("canny 50/150", canny_a), ("canny 30/90", canny_b)]))
    imwrite_cn(OUT_DIR / "compare_morph.png",
               join([("adaptive", th_ad), ("open 5", opened), ("close 5", closed)]))

    print("尺寸", img.shape[1], "x", img.shape[0])
    print("阈值 127 白像素", round(white_ratio(th127), 2), "%")
    print("阈值 180 白像素", round(white_ratio(th180), 2), "%")
    print("otsu 阈值", otsu_v)
    print("canny 50/150 边缘像素", np.count_nonzero(canny_a))
    print("canny 30/90 边缘像素", np.count_nonzero(canny_b))
    print("自适应阈值白像素", round(white_ratio(th_ad), 2), "%")
    print("开运算后", round(white_ratio(opened), 2), "%")
    print("闭运算后", round(white_ratio(closed), 2), "%")
    print("输出在", OUT_DIR.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
