"""任务二：基础预处理和单变量参数对照，保存全部中间结果。

默认选取 data/images 按文件名排序后的第一张图片，也可指定：
    python src/task2_processing.py --image data/images/your_image.jpg

baseline 是基准；另外四组每次只改变一个参数，便于判断变化原因。
固定阈值处理比较的是亮度，不等同于任务三的颜色目标提取。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

from common import PROJECT_ROOT, list_images, read_image, save_image, write_json, safe_output_dir

# 各参数组使用一致的步骤编号，文件与结果 JSON 都引用这张表。
STAGE_FILES = {
    "gaussian": "01_gaussian.png",
    "threshold": "02_threshold.png",
    "canny": "03_canny.png",
    "morphology": "04_morphology.png",
}


def odd_kernel(text: str) -> int:
    value = int(text)
    if value < 1 or value > 99 or value % 2 == 0:
        raise argparse.ArgumentTypeError("核大小必须是 1~99 之间的奇数")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--input-dir", type=Path, default=PROJECT_ROOT / "data" / "images", help="默认图片目录")
    parser.add_argument("--image", type=Path, help="指定一张图片；优先于 --input-dir")
    parser.add_argument("--output-dir", type=Path, default=PROJECT_ROOT / "outputs" / "task2", help="结果目录")
    parser.add_argument("--gaussian-kernel", type=odd_kernel, default=5, help="高斯滤波核边长，默认 5")
    parser.add_argument("--threshold", type=int, default=127, help="固定二值阈值 0~255，默认 127")
    parser.add_argument("--canny-low", type=int, default=80, help="Canny 低阈值，默认 80")
    parser.add_argument("--canny-high", type=int, default=160, help="Canny 高阈值，默认 160")
    parser.add_argument("--morph-kernel", type=odd_kernel, default=3, help="形态学椭圆核边长，默认 3")
    parser.add_argument("--morph-operation", choices=("open", "close"), default="open", help="基准形态学操作，默认开运算 open")
    return parser


def process(gray: np.ndarray, config: dict) -> tuple[dict[str, np.ndarray], dict]:
    blurred = cv2.GaussianBlur(gray, (config["gaussian_kernel"], config["gaussian_kernel"]), 0)
    _, binary = cv2.threshold(blurred, config["threshold"], 255, cv2.THRESH_BINARY)
    edges = cv2.Canny(blurred, config["canny_low"], config["canny_high"])
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (config["morph_kernel"], config["morph_kernel"]))
    operation = cv2.MORPH_OPEN if config["morph_operation"] == "open" else cv2.MORPH_CLOSE
    morphology = cv2.morphologyEx(binary, operation, kernel)
    images = {"gaussian": blurred, "threshold": binary, "canny": edges, "morphology": morphology}
    metrics = {
        "gaussian_mean_abs_change": float(np.abs(gray.astype(np.float32) - blurred.astype(np.float32)).mean()),
        "threshold_white_fraction": float(np.count_nonzero(binary) / binary.size),
        "canny_edge_fraction": float(np.count_nonzero(edges) / edges.size),
        "canny_connected_components": int(cv2.connectedComponents(edges, connectivity=8)[0] - 1),
        "morphology_white_fraction": float(np.count_nonzero(morphology) / morphology.size),
        "morphology_changed_pixels": int(np.count_nonzero(morphology != binary)),
    }
    return images, metrics


def tile(image: np.ndarray, title: str, subtitle: str = "") -> np.ndarray:
    """保持宽高比缩放，仅在对比图中加英文标签，避免字体缺失造成乱码。"""
    canvas = np.full((244, 288, 3), 245, dtype=np.uint8)
    color = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image
    scale = min(280 / color.shape[1], 198 / color.shape[0])
    target = cv2.resize(color, (max(1, round(color.shape[1] * scale)), max(1, round(color.shape[0] * scale))), interpolation=cv2.INTER_AREA if scale < 1 else cv2.INTER_NEAREST)
    offset_x = (288 - target.shape[1]) // 2
    offset_y = 44 + (198 - target.shape[0]) // 2
    canvas[offset_y:offset_y + target.shape[0], offset_x:offset_x + target.shape[1]] = target
    cv2.putText(canvas, title, (7, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (25, 25, 25), 1, cv2.LINE_AA)
    if subtitle:
        cv2.putText(canvas, subtitle, (7, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.38, (70, 70, 70), 1, cv2.LINE_AA)
    return canvas


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()
    if not 0 <= args.threshold <= 255:
        parser.error("--threshold 必须位于 0~255")
    if not 0 <= args.canny_low < args.canny_high <= 255:
        parser.error("Canny 阈值必须满足 0 <= low < high <= 255")
    if args.image:
        source = args.image.resolve()
    else:
        paths = sorted(list_images(args.input_dir.resolve()), key=lambda item: str(item).casefold())
        if not paths:
            parser.error(f"输入目录中没有图片：{args.input_dir}")
        source = paths[0].resolve()
    image = read_image(source)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    output_dir = safe_output_dir(args.output_dir, source.parent if args.image else args.input_dir)
    # 指定目录时，任何输入图片都不能成为输出覆盖的目标。
    protected_sources = {source} if args.image else {path.resolve() for path in paths}

    def save(relative_path: str, result: np.ndarray) -> None:
        target = output_dir / relative_path
        if target.resolve() in protected_sources:
            raise ValueError(f"已阻止覆盖原图：{target}")
        save_image(target, result)

    # 改动值总是与基准不同；每次只调整一种因素。
    base = {"gaussian_kernel": args.gaussian_kernel, "threshold": args.threshold, "canny_low": args.canny_low, "canny_high": args.canny_high, "morph_kernel": args.morph_kernel, "morph_operation": args.morph_operation}
    gaussian_alt = args.gaussian_kernel + 6 if args.gaussian_kernel <= 93 else args.gaussian_kernel - 6
    threshold_alt = args.threshold + 40 if args.threshold <= 215 else args.threshold - 40
    morph_alt = args.morph_kernel + 6 if args.morph_kernel <= 93 else args.morph_kernel - 6
    low_alt, high_alt = args.canny_low // 2, max(1, args.canny_high // 2)
    if (low_alt, high_alt) == (args.canny_low, args.canny_high):
        low_alt, high_alt = 1, 2
    configurations = [
        ("baseline", "基准参数", base.copy()),
        ("gaussian_changed", "只修改高斯滤波核", {**base, "gaussian_kernel": gaussian_alt}),
        ("threshold_changed", "只修改固定二值阈值", {**base, "threshold": threshold_alt}),
        ("canny_changed", "只修改 Canny 双阈值", {**base, "canny_low": low_alt, "canny_high": high_alt}),
        ("morphology_changed", "只修改形态学核", {**base, "morph_kernel": morph_alt}),
    ]
    save("01_original.png", image)
    save("02_grayscale.png", gray)
    rows, records = [], []
    baseline_images = None
    for name, explanation, config in configurations:
        images, metrics = process(gray, config)
        if baseline_images is None:
            baseline_images = images
        for kind, result in images.items():
            save(f"{name}/{STAGE_FILES[kind]}", result)
        differences = {kind: int(np.count_nonzero(result != baseline_images[kind])) for kind, result in images.items()}
        labels = {"gaussian": f"kernel={config['gaussian_kernel']}, sigma=auto", "threshold": f"T={config['threshold']}, binary", "canny": f"low={config['canny_low']}, high={config['canny_high']}", "morphology": f"{config['morph_operation']}, kernel={config['morph_kernel']}"}
        row = [tile(gray, name, "grayscale input")]
        row.extend(tile(result, kind, labels[kind]) for kind, result in images.items())
        rows.append(np.hstack(row))
        records.append({"name": name, "change": explanation, "parameters": config, "metrics": metrics, "changed_pixels_vs_baseline": differences, "files": {kind: f"{name}/{STAGE_FILES[kind]}" for kind in images}})
        print(f"{name}: T={config['threshold']}, gaussian={config['gaussian_kernel']}, Canny=({config['canny_low']}, {config['canny_high']}), morph={config['morph_kernel']}; white={metrics['threshold_white_fraction']:.3f}, edge={metrics['canny_edge_fraction']:.3f}")
    save("03_comparison.png", np.vstack(rows))

    # 补齐学习内容中的均值/中值滤波、自适应阈值、腐蚀/膨胀/开/闭运算。
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (base["morph_kernel"], base["morph_kernel"]))
    binary = baseline_images["threshold"]
    supplements = {
        "mean_filter": cv2.blur(gray, (5, 5)),
        "median_filter": cv2.medianBlur(gray, 5),
        "adaptive_threshold": cv2.adaptiveThreshold(baseline_images["gaussian"], 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 31, 5),
        "erode": cv2.erode(binary, kernel, iterations=1),
        "dilate": cv2.dilate(binary, kernel, iterations=1),
        "opening": cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel),
        "closing": cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel),
    }
    for channel_name, channel in zip(("B", "G", "R"), cv2.split(image)):
        supplements[f"channel_{channel_name}"] = channel
    supplement_files = {
        name: f"supplement/{index:02d}_{name}.png"
        for index, name in enumerate(supplements, start=1)
    }
    for name, result in supplements.items():
        save(supplement_files[name], result)
    report_path = output_dir / "results.json"
    if report_path.resolve() == source:
        raise ValueError("results.json 不得覆盖源文件")
    write_json(report_path, {"task": "task2_processing", "source": str(source), "source_shape": list(image.shape), "comparison_strategy": "baseline + four one-factor-at-a-time changes", "configurations": records, "supplement_parameters": {"mean_kernel": 5, "median_kernel": 5, "adaptive_method": "GAUSSIAN_C", "adaptive_block_size": 31, "adaptive_C": 5, "morphology_shape": "ELLIPSE", "morphology_kernel": base["morph_kernel"], "morphology_iterations": 1}, "supplement_files": supplement_files, "interpretation_notes": ["固定阈值为 THRESH_BINARY：像素值大于 T 才变为白色；升高 T 通常减少白色区域。", "较大的高斯核会增强平滑，也可能模糊细小边缘。", "降低 Canny 阈值通常保留更多弱边缘，也可能增加噪声边缘。", "开运算可能去除小白噪声，也可能损伤细小目标；闭运算可能填孔连通，也可能合并邻近区域。", "白色占比、边缘连通分量只是观察指标，不能单独证明识别正确或边缘完整。"]})
    print(f"完成：5 组参数对照、完整中间结果和 03_comparison.png 已保存到 {output_dir}")


if __name__ == "__main__":
    main()
