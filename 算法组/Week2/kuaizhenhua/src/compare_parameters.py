"""参数 A/B 对比；同图每次只改变指定条件。统计保存在各对比组的summary.json。"""
import argparse
from dataclasses import asdict, replace
from pathlib import Path
import sys

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "color_target_vision"))
from color_target_vision.core import DetectorConfig, detect
from color_target_vision.image_io import read_image, save_image
from color_target_vision.offline import check_output, collect_images, detection_record, image_tile, write_json


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description="Compare V minimum, minimum area, and morphology on the same images")
    parser.add_argument("--input", type=Path, default=ROOT / "data" / "real_images", help="image file or folder")
    parser.add_argument("--output", type=Path, default=ROOT / "outputs" / "comparison", help="comparison output folder")
    parser.add_argument("--group", choices=["all", "01_v_min", "02_min_area", "03_morphology"], default="all", help="选择对比组；每组独立保存统计")
    args = parser.parse_args(argv)
    base = DetectorConfig()
    groups = [
        ("01_v_min", "V lower bound: 70 vs 40", base, replace(base, hsv_lower=(95, 80, 40))),
        ("02_min_area", "Minimum area: 500 vs 100", base, replace(base, min_area=100.0)),
        ("03_morphology", "Morphology: open3/close5 vs off", base, replace(base, open_kernel=1, close_kernel=1)),
    ]
    if args.group != "all":
        groups = [group for group in groups if group[0] == args.group]
    try:
        paths = collect_images(args.input)
        output = args.output.resolve()
        check_output(args.input, output)
        sources = {path.resolve() for path in paths}
        for group_name, description, config_a, config_b in groups:
            config_a.validate()
            config_b.validate()
            folder = output / group_name
            # 图内写明A/B数值，单独打开PNG时也能知道改了哪个参数。
            labels = {
                "01_v_min": ("A - Vmin=70 / original", "B - Vmin=40 / original"),
                "02_min_area": ("A - area>=500 / original", "B - area>=100 / original"),
                "03_morphology": ("A - open3+close5 / original", "B - morphology off / original"),
            }[group_name]
            rows = []
            for index, path in enumerate(paths, start=1):
                image = read_image(path)
                result_a, result_b = detect(image, config_a), detect(image, config_b)
                # 上排 A，下排 B；每排都是原图、掩膜、最终标注，不改变其他条件。
                row_a = np.hstack((image_tile(image, labels[0]), image_tile(result_a.mask, "A - mask"), image_tile(result_a.annotated, "A - result")))
                row_b = np.hstack((image_tile(image, labels[1]), image_tile(result_b.mask, "B - mask"), image_tile(result_b.annotated, "B - result")))
                filename = f"{index:02d}_{path.stem}.png"
                target = folder / filename
                if target.resolve() in sources:
                    raise ValueError(f"Refusing to overwrite source: {target}")
                save_image(target, np.vstack((row_a, row_b)))
                record_a, record_b = detection_record(path, image, result_a), detection_record(path, image, result_b)
                rows.append({"filename": path.name, "comparison_file": filename, "A": record_a, "B": record_b,
                             "mask_changed_pixels": int(np.count_nonzero(result_a.mask != result_b.mask)),
                             "detected_changed": result_a.detected != result_b.detected})
            summary = {"schema_version": 1, "record_type": "automatic_parameter_comparison", "description": description,
                       "parameters": {"A": asdict(config_a), "B": asdict(config_b)}, "results": rows}
            write_json(folder / "summary.json", summary)
            print(f"{group_name}: saved {len(rows)} comparison images")
    except (ValueError, OSError, cv2.error) as error:
        parser.exit(1, f"Error: {error}\n")
    print(f"Saved comparisons to {output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
