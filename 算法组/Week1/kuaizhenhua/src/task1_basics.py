"""任务一：读取至少两张图片，完成缩放、指定区域裁剪、灰度转换和保存验证。

示例（在项目根目录运行）：
    python src/task1_basics.py
    python src/task1_basics.py --scale 0.5 --crop 0.25 0.25 0.5 0.5

OpenCV 默认按 BGR 顺序读取彩色图；Matplotlib 显示时应先用
cv2.cvtColor(image, cv2.COLOR_BGR2RGB) 转为 RGB。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import PROJECT_ROOT, list_images, read_image, save_image, write_json, safe_output_dir


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input-dir", type=Path, default=PROJECT_ROOT / "data" / "images", help="输入图片目录，默认 data/images")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "outputs" / "task1", help="结果目录，默认 outputs/task1")
    parser.add_argument("--scale", type=float, default=0.5, help="缩放比例，默认 0.5，必须大于 0")
    parser.add_argument("--crop", type=float, nargs=4, default=(0.25, 0.25, 0.5, 0.5), metavar=("X", "Y", "W", "H"), help="裁剪区域的左上角 x/y 和宽/高，均为原图宽高的比例")
    return parser


def save_and_verify(path: Path, image: np.ndarray, source_paths: set[Path]) -> dict:
    """只生成 PNG；使用不改变通道的方式重读，检查尺寸和像素完全一致。"""
    if path.resolve() in source_paths:
        raise ValueError(f"输出路径与原图相同，已阻止覆盖：{path}")
    save_image(path, image)
    restored = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_UNCHANGED)
    if restored is None or restored.shape != image.shape or not np.array_equal(restored, image):
        raise RuntimeError(f"保存后校验失败：{path}")
    return {"file": str(path.name), "shape": list(image.shape), "read_back_ok": True, "pixels_identical": True}


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if not np.isfinite(args.scale) or args.scale <= 0:
        parser.error("--scale 必须是大于 0 的有限数字")
    x, y, width, height = args.crop
    if not all(np.isfinite(v) for v in args.crop) or x < 0 or y < 0 or width <= 0 or height <= 0 or x + width > 1 + 1e-9 or y + height > 1 + 1e-9:
        parser.error("--crop 必须满足 x/y >= 0、w/h > 0，且 x+w <= 1、y+h <= 1")
    input_dir, output_dir = args.input_dir.resolve(), args.output_dir.resolve()
    output_dir = safe_output_dir(output_dir, input_dir)
    paths = sorted(list_images(input_dir), key=lambda item: str(item).casefold())
    if len(paths) < 2:
        parser.error(f"任务一需要至少 2 张图片，当前找到 {len(paths)} 张：{input_dir}")
    sources = {path.resolve() for path in paths}
    metadata_path = output_dir / "metadata.json"
    if metadata_path.resolve() in sources:
        parser.error("metadata.json 不能覆盖源文件")
    records = []
    for index, path in enumerate(paths, start=1):
        image = read_image(path)
        original_h, original_w = image.shape[:2]
        channels = 1 if image.ndim == 2 else image.shape[2]
        print(f"[{index}/{len(paths)}] {path.name}: width={original_w}, height={original_h}, channels={channels}")

        # ndarray 的索引顺序是 [行 y, 列 x]；裁剪坐标的右边界、下边界不包含在结果中。
        left = min(original_w - 1, int(np.floor(x * original_w)))
        top = min(original_h - 1, int(np.floor(y * original_h)))
        right = min(original_w, max(left + 1, int(np.ceil((x + width) * original_w))))
        bottom = min(original_h, max(top + 1, int(np.ceil((y + height) * original_h))))
        new_size = (max(1, int(round(original_w * args.scale))), max(1, int(round(original_h * args.scale))))
        interpolation = cv2.INTER_AREA if args.scale < 1 else cv2.INTER_LINEAR
        resized = cv2.resize(image, new_size, interpolation=interpolation)
        cropped = image[top:bottom, left:right].copy()
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        folder = output_dir / f"{index:02d}_{path.stem}"
        # 文件名按处理顺序编号；JSON 中的逻辑名称保持不变，便于检索。
        results = {
            name: save_and_verify(folder / f"{step:02d}_{name}.png", result, sources)
            for step, (name, result) in enumerate(
                (("resized", resized), ("crop", cropped), ("gray", gray)), start=1
            )
        }
        records.append({"source": str(path), "source_shape": list(image.shape), "width": original_w, "height": original_h, "channels": channels, "scale_requested": args.scale, "resized_size_wh": list(new_size), "crop_fraction_xywh": list(args.crop), "crop_pixels_xyxy_exclusive": [left, top, right, bottom], "output_subfolder": folder.name, "results": results})
    write_json(metadata_path, {"task": "task1_basics", "image_count": len(records), "read_mode": "BGR", "all_saved_files_read_back_verified": True, "images": records})
    print(f"完成：处理 {len(records)} 张图片，所有 PNG 已重新读取并校验。结果：{output_dir}")


if __name__ == "__main__":
    main()
