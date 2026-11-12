"""离线检测命令行；默认路径由项目根目录的入口脚本提供。"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np

from .core import DetectorConfig, detect
from .image_io import read_image, save_image

IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff", ".webp"}


def collect_images(input_path: Path) -> list[Path]:
    """目录只读取当前层，避免混入实拍子目录或以前的实验输出。"""
    source = Path(input_path).resolve()
    if source.is_file():
        if source.suffix.lower() not in IMAGE_SUFFIXES:
            raise ValueError(f"Unsupported image extension: {source.suffix}")
        return [source]
    if not source.is_dir():
        raise FileNotFoundError(f"Input not found: {source}")
    paths = sorted((path for path in source.iterdir() if path.is_file() and path.suffix.lower() in IMAGE_SUFFIXES), key=lambda path: (path.name.casefold(), path.name))
    if not paths:
        raise ValueError(f"No supported images in: {source}")
    return paths


def check_output(input_path: Path, output: Path) -> None:
    source, output = Path(input_path).resolve(), Path(output).resolve()
    if source.is_dir() and (output == source or source in output.parents):
        raise ValueError("Output must be outside the input image folder")
    if source.is_file() and output == source:
        raise ValueError("Output cannot overwrite the input image")


def write_json(path: Path, data: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")


def image_tile(image: np.ndarray, label: str, width=320, height=240) -> np.ndarray:
    """只为对比图调整显示尺寸；单张结果仍保留输入的分辨率。"""
    bgr = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image
    canvas = np.full((height + 30, width, 3), 245, np.uint8)
    scale = min(width / bgr.shape[1], height / bgr.shape[0])
    resized = cv2.resize(bgr, (max(1, round(bgr.shape[1] * scale)), max(1, round(bgr.shape[0] * scale))), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_NEAREST)
    y, x = 30 + (height - resized.shape[0]) // 2, (width - resized.shape[1]) // 2
    canvas[y:y + resized.shape[0], x:x + resized.shape[1]] = resized
    cv2.putText(canvas, label, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (25, 25, 25), 1, cv2.LINE_AA)
    return canvas


def detection_record(path: Path, image: np.ndarray, result) -> dict:
    return {"filename": path.name, "width": int(image.shape[1]), "height": int(image.shape[0]),
            "detected": result.detected, "u": result.u, "v": result.v, "area": result.area,
            "candidate_count": len(result.candidates), "candidates": result.candidates,
            "source_sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


def build_parser(project_root: Path) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Blue-region detection: HSV, morphology, contours, largest candidate")
    parser.add_argument("--input", type=Path, default=project_root / "data" / "real_images", help="image file or folder; default: data/real_images")
    parser.add_argument("--output", type=Path, default=project_root / "outputs" / "detection", help="output folder; default: outputs/detection")
    parser.add_argument("--v-min", type=int, default=70, help="HSV value lower bound, 0..255")
    parser.add_argument("--min-area", type=float, default=500.0, help="minimum contour area in pixel squared")
    parser.add_argument("--open-kernel", type=int, default=3, help="positive odd kernel size; 1 disables opening")
    parser.add_argument("--close-kernel", type=int, default=5, help="positive odd kernel size; 1 disables closing")
    return parser


def main(argv=None, project_root=None) -> int:
    root = Path(project_root).resolve() if project_root else Path(__file__).resolve().parents[3]
    parser = build_parser(root)
    args = parser.parse_args(argv)
    config = DetectorConfig(hsv_lower=(95, 80, args.v_min), min_area=args.min_area, open_kernel=args.open_kernel, close_kernel=args.close_kernel)
    try:
        config.validate()
        paths = collect_images(args.input)
        output = args.output.resolve()
        check_output(args.input, output)
        sources = {path.resolve() for path in paths}
        records = []
        for index, path in enumerate(paths, start=1):
            image = read_image(path)
            result = detect(image, config)
            folder = output / f"{index:02d}_{path.stem}"
            images = {"01_mask_raw.png": result.raw_mask, "02_mask.png": result.mask, "03_result.png": result.annotated,
                      "04_comparison.png": np.hstack((image_tile(image, "Original"), image_tile(result.annotated, "Selected candidate")))}
            for name, rendered in images.items():
                target = folder / name
                if target.resolve() in sources:
                    raise ValueError(f"Refusing to overwrite source: {target}")
                save_image(target, rendered)
            record = detection_record(path, image, result)
            record["output_subdirectory"] = folder.name
            record["files"] = {"raw_mask": f"{folder.name}/01_mask_raw.png", "mask": f"{folder.name}/02_mask.png", "annotated": f"{folder.name}/03_result.png", "comparison": f"{folder.name}/04_comparison.png"}
            records.append(record)
            status = "已选中最大面积候选" if result.detected else "未检测到目标"
            print(f"{index:02d} {path.name}: {status} | detected={result.detected}, u={result.u:.3f}, v={result.v:.3f}, area={result.area:.1f}, candidates={len(result.candidates)}")
        write_json(output / "summary.json", {"schema_version": 1, "record_type": "automatic_run", "note": "自动运行结果，需结合图像核对；不代表本人实操已完成。", "parameters": asdict(config), "results": records})
    except (ValueError, OSError, cv2.error) as error:
        parser.exit(1, f"Error: {error}\n")
    print(f"Saved {len(records)} image records to {output / 'summary.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
