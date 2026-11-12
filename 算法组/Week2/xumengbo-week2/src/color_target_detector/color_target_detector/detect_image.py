"""任务 1 + 任务 2：对单张图片做颜色目标检测，并在图上标注、存文件。

用法示例：
    ros2 run color_target_detector detect_image --image data/test1.png
    python3 -m color_target_detector.detect_image --image data/test1.png --min-area 2000
    ... --lower 35,80,60 --upper 85,255,255 --morph none --out out/result.png

终端会打印 u / v / area；没检测到会打印「未检测到目标」。
"""

import argparse
import os
import sys

import cv2
import numpy as np

from .detection import (DEFAULT_LOWER, DEFAULT_MIN_AREA, DEFAULT_UPPER,
                        detect, draw_result)


def imread_utf8(path):
    """读图：兼容含中文/生僻字的路径（cv2.imread 对这种路径会失败）。"""
    data = np.fromfile(path, dtype=np.uint8)
    return cv2.imdecode(data, cv2.IMREAD_COLOR)


def imwrite_utf8(path, img):
    """写图：兼容含中文/生僻字的路径。"""
    ext = os.path.splitext(path)[1]
    ok, buf = cv2.imencode(ext, img)
    if ok:
        buf.tofile(path)


def parse_triple(text: str):
    """把 "35,80,60" 解析成 (35, 80, 60)，同时检查范围是否合法。"""
    parts = [int(p.strip()) for p in text.split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError("需要 3 个用逗号分隔的整数，例如 35,80,60")
    if not all(0 <= p <= 255 for p in parts):
        raise argparse.ArgumentTypeError("HSV 三个分量都必须在 0~255 之间")
    return tuple(parts)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="HSV 颜色目标检测（单张图片）")
    p.add_argument("--image", required=True, help="输入图片路径")
    p.add_argument("--lower", type=parse_triple, default=DEFAULT_LOWER,
                   help=f"HSV 下界，默认 {','.join(map(str, DEFAULT_LOWER))}")
    p.add_argument("--upper", type=parse_triple, default=DEFAULT_UPPER,
                   help=f"HSV 上界，默认 {','.join(map(str, DEFAULT_UPPER))}")
    p.add_argument("--min-area", type=float, default=DEFAULT_MIN_AREA,
                   help=f"最小轮廓面积（像素²），默认 {DEFAULT_MIN_AREA}")
    p.add_argument("--morph", default="open",
                   choices=["none", "open", "close", "open+close"],
                   help="形态学操作，默认 open")
    p.add_argument("--ksize", type=int, default=5, help="形态学核大小，默认 5")
    p.add_argument("--out", default="", help="标注结果保存路径；不填则自动命名")
    p.add_argument("--save-mask", action="store_true",
                   help="额外保存二值掩膜，方便排查阈值问题")
    return p


def main(argv=None) -> int:
    args = build_arg_parser().parse_args(argv)

    # ---- 读图并检查是否成功（任务要求：必须做失败处理）----
    image = imread_utf8(args.image)
    if image is None:
        print(f"[ERROR] 图片读取失败: {args.image}")
        print("        检查路径是否存在、文件是否为图片、WSL 里路径是否写成了 /mnt/d/...")
        return 1

    h, w = image.shape[:2]
    print(f"[INFO] 图像   : {args.image}")
    print(f"[INFO] 尺寸   : {w} x {h}（宽 x 高，宽=W，高=H）")
    print(f"[INFO] HSV 阈值: lower={args.lower} upper={args.upper}")
    print(f"[INFO] 最小面积: {args.min_area}   形态学: {args.morph} (k={args.ksize})")

    det = detect(image, args.lower, args.upper,
                 args.min_area, args.morph, args.ksize)

    if det.detected:
        print(f"[OK]   检测到目标: u={det.u:.1f}, v={det.v:.1f}, "
              f"area={det.area:.0f}, 候选数={det.n_candidates}")
        # 判断目标落在图像的哪个区域（靠近中心就报“正中”，避免 319.9 被说成“左上”）
        half_w, half_h = w / 2.0, h / 2.0
        if abs(det.u - half_w) < w * 0.05 and abs(det.v - half_h) < h * 0.05:
            where = "正中"
        else:
            where = ("左" if det.u < half_w else "右") + ("上" if det.v < half_h else "下")
        print(f"       中心点位于图像{where}"
              f"（横向 {det.u / w * 100:.1f}% ，纵向 {det.v / h * 100:.1f}%）")
    else:
        print("[WARN] 未检测到目标：没有轮廓的面积超过 min-area")

    # ---- 标注并存文件（任务 2 要求：不能只依赖临时窗口）----
    out_path = args.out
    if not out_path:
        stem = os.path.splitext(os.path.basename(args.image))[0]
        out_path = f"out/{stem}_result.png"
    out_dir = os.path.dirname(out_path)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)

    annotated = draw_result(image, det,
                            title=f"lower={args.lower} upper={args.upper} minArea={int(args.min_area)} {args.morph}")
    imwrite_utf8(out_path, annotated)
    print(f"[INFO] 结果图已保存: {out_path}")

    if args.save_mask and det.mask is not None:
        mask_path = os.path.splitext(out_path)[0] + "_mask.png"
        imwrite_utf8(mask_path, det.mask)
        print(f"[INFO] 掩膜已保存  : {mask_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
