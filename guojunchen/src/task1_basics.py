# -*- coding: utf-8 -*-
"""任务 1：图像读取 / 缩放 / 裁剪 / 灰度化（自包含，可独立运行）。

对每张输入图：读取彩色图和灰度图 → 缩放到指定边长 → 按文件名裁剪 →
打印尺寸 → 保存「缩放 / 裁剪 / 灰度」三张结果到 Outputs/task1/。

用法（在 guojunchen/ 目录下）:
    python src/task1_basics.py                    # 处理默认的 data/nailong.jpg、data/mieba.jpg
    python src/task1_basics.py data/xxx.jpg ...   # 处理指定图片
    python src/task1_basics.py --show             # 额外弹窗显示
"""
import argparse
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional, Tuple

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]  # guojunchen/
OUTPUT_DIR = ROOT / "Outputs" / "task1"

DEFAULT_IMAGES = ["data/nailong.jpg", "data/mieba.jpg"]

# 每张图的裁剪范围 (y1, y2, x1, x2)，按文件名一一对应；未列出的图默认裁左上角 size x size
CROP_BOXES: Dict[str, Tuple[int, int, int, int]] = {
    "nailong.jpg": (50, 300, 60, 500),
    "mieba.jpg": (30, 1000, 500, 1440),
}


def _imread(path: str, flags: int = cv2.IMREAD_COLOR) -> Optional[np.ndarray]:
    """读图。Windows 下 cv2.imread 对含中文的路径会失败，改用 np.fromfile + imdecode。"""
    data = np.fromfile(path, dtype=np.uint8)
    if data.size == 0:
        return None
    return cv2.imdecode(data, flags)


def _save(img: np.ndarray, path: Path) -> bool:
    """存图。中文文件名用 imencode(...).tofile() 写，避免 imwrite 静默失败。"""
    ok, buf = cv2.imencode(path.suffix or ".png", img)
    if ok:
        buf.tofile(str(path))
    return ok


def main():
    ap = argparse.ArgumentParser(description="任务1：图像读取/缩放/裁剪/灰度化")
    ap.add_argument("images", nargs="*", help="输入图片路径（缺省处理 data/ 下两张）")
    ap.add_argument("--size", type=int, default=300, help="缩放目标边长（正方形）")
    ap.add_argument("--show", action="store_true", help="额外弹窗显示结果")
    args = ap.parse_args()

    images = args.images or DEFAULT_IMAGES
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    print("姓名拼音：guojunchen")
    print("运行时间：", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("=" * 60)

    for idx, name in enumerate(images, start=1):
        color = _imread(name, cv2.IMREAD_COLOR)
        if color is None:
            print("[%s] 无法读取图像" % name)
            continue
        gray = _imread(name, cv2.IMREAD_GRAYSCALE)
        if gray is None:
            gray = cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)

        resized = cv2.resize(color, (args.size, args.size))

        # 裁剪范围按文件名查表；没查到的图默认裁左上角 size x size 区域
        y1, y2, x1, x2 = CROP_BOXES.get(Path(name).name, (0, args.size, 0, args.size))
        crop = color[y1:y2, x1:x2]

        print("[%s] 彩色 %s，灰度 %s" % (name, color.shape, gray.shape))
        stem = "result%d" % idx
        for suffix, img in (("caijian", crop), ("suofang", resized), ("huidu", gray)):
            path = OUTPUT_DIR / ("%s.%s.jpg" % (stem, suffix))
            if _save(img, path):
                print("  已保存 %s" % path)
            else:
                print("  保存失败 %s" % path)

        if args.show:
            cv2.imshow("suofang", resized)
            cv2.imshow("caijian", crop)
            cv2.imshow("huidu", gray)
            cv2.waitKey(0)
        print("-" * 60)

    if args.show:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
