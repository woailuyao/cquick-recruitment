# -*- coding: utf-8 -*-
"""任务 2：基础图像处理（自包含，可独立运行）。

对输入图依次做：灰度化 → 高斯滤波 → 二值化 → Canny 边缘检测 → 腐蚀 / 膨胀，
并把每一步的结果保存到 Outputs/task2/。

用法（在 guojunchen/ 目录下）:
    python src/task2_processing.py                 # 用 data/fennu.jpg
    python src/task2_processing.py data/xxx.jpg    # 指定图片
    python src/task2_processing.py --show          # 额外弹窗显示各步骤
"""
import argparse
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Dict, Optional

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]  # guojunchen/
OUTPUT_DIR = ROOT / "Outputs" / "task2"


@dataclass
class ProcessParams:
    """各步骤参数。blur_ksize 必须是奇数，否则高斯滤波会报错。"""
    blur_ksize: int = 101   # 高斯滤波核大小（奇数）
    thresh: int = 127       # 二值化阈值
    canny_low: int = 20     # Canny 低阈值
    canny_high: int = 100   # Canny 高阈值
    morph_ksize: int = 5    # 腐蚀 / 膨胀核大小


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


def process(bgr: np.ndarray, p: ProcessParams) -> Dict[str, np.ndarray]:
    """按顺序执行各基础操作，返回 {名字: 结果图}。"""
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    # 高斯滤波：作用在灰度图上，用于给 Canny 降噪
    blur = cv2.GaussianBlur(gray, (p.blur_ksize, p.blur_ksize), 0)

    # 二值化：> 阈值置 255，其余置 0
    _, binary = cv2.threshold(gray, p.thresh, 255, cv2.THRESH_BINARY)

    # Canny 用滤波后的图，避免把噪声误判成边缘
    canny = cv2.Canny(blur, p.canny_low, p.canny_high)

    # 腐蚀 / 膨胀：作用在同一张二值图上（腐蚀去小噪点，膨胀补空洞）
    kernel = np.ones((p.morph_ksize, p.morph_ksize), np.uint8)
    erosion = cv2.erode(binary, kernel)
    dilation = cv2.dilate(binary, kernel)

    return {
        "gray": gray,
        "blur": blur,
        "binary": binary,
        "canny": canny,
        "erosion": erosion,
        "dilation": dilation,
    }


def main():
    ap = argparse.ArgumentParser(description="任务2：基础图像处理操作演示")
    ap.add_argument("image", nargs="?", default="data/fennu.jpg", help="输入图片路径")
    ap.add_argument("--blur", type=int, default=101, help="高斯滤波核大小（奇数）")
    ap.add_argument("--thresh", type=int, default=127, help="二值化阈值")
    ap.add_argument("--canny-low", type=int, default=20, help="Canny 低阈值")
    ap.add_argument("--canny-high", type=int, default=100, help="Canny 高阈值")
    ap.add_argument("--morph", type=int, default=5, help="腐蚀/膨胀核大小")
    ap.add_argument("--show", action="store_true", help="额外弹窗显示各步骤")
    args = ap.parse_args()

    image = _imread(args.image, cv2.IMREAD_COLOR)
    if image is None:
        print("无法读取图像:", args.image)
        return

    p = ProcessParams(blur_ksize=args.blur, thresh=args.thresh,
                      canny_low=args.canny_low, canny_high=args.canny_high,
                      morph_ksize=args.morph)
    results = process(image, p)

    print("姓名拼音：guojunchen")
    print("运行时间：", datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    print("输入图像：%s（%d x %d）" % (args.image, image.shape[1], image.shape[0]))
    print("=" * 60)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    for name, img in results.items():
        path = OUTPUT_DIR / ("%s.jpg" % name)
        if _save(img, path):
            print("已保存 %-8s -> %s" % (name, path))
        else:
            print("保存失败：%s" % name)
        if args.show:
            cv2.imshow(name, img)

    if args.show:
        print("按任意键关闭所有窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
