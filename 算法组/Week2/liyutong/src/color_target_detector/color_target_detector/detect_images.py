"""Headless batch CLI: python -m color_target_detector.detect_images INPUT ... --output DIR."""

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
from typing import Optional, Sequence

import cv2
import numpy as np

from .detector import DetectorConfig, annotate, detect


def _read_image(path: Path) -> np.ndarray:
    """Read bytes first so OpenCV never has to interpret a Unicode file path."""
    data = path.read_bytes()
    if not data:
        raise ValueError("image file is empty")
    image = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError("OpenCV could not decode image")
    return image


def _write_bytes(path: Path, data: bytes) -> None:
    with path.open("wb") as stream:
        written = stream.write(data)
        if written != len(data):
            raise OSError(f"short write: {written} of {len(data)} bytes")
        stream.flush()


def _write_png(path: Path, image: np.ndarray) -> None:
    success, encoded = cv2.imencode(".png", image)
    if not success:
        raise OSError("OpenCV PNG encoding failed")
    _write_bytes(path, encoded.tobytes())


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Detect HSV color targets without a GUI.")
    parser.add_argument("inputs", nargs="+", type=Path, help="Input image paths")
    parser.add_argument("--output", required=True, type=Path, help="Output directory")
    parser.add_argument("--hsv-lower", nargs=3, type=int, default=(5, 100, 80), metavar=("H", "S", "V"))
    parser.add_argument("--hsv-upper", nargs=3, type=int, default=(25, 255, 255), metavar=("H", "S", "V"))
    parser.add_argument("--min-area", type=float, default=100)
    parser.add_argument("--morphology", choices=("none", "open", "close", "open_close"), default="none")
    parser.add_argument("--kernel-size", type=int, default=3)
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    """Save indexed PNG annotations/masks and results.json; return 1 on any I/O error.

    Every JSON image entry contains the exact effective config and an error field
    (null on success). Invalid configuration is an argparse error (exit status 2).
    Contours/masks are image artifacts, not JSON payloads. A detection miss is a
    successful image processing result with zero centroid and area.
    """
    parser = _parser()
    args = parser.parse_args(argv)
    try:
        config = DetectorConfig(
            hsv_lower=args.hsv_lower,
            hsv_upper=args.hsv_upper,
            min_area=args.min_area,
            morphology=args.morphology,
            kernel_size=args.kernel_size,
        )
    except (TypeError, ValueError) as exc:
        parser.error(str(exc))
    try:
        args.output.mkdir(parents=True, exist_ok=True)
    except OSError as exc:
        print(f"Cannot create output directory {args.output}: {exc}", file=sys.stderr)
        return 1

    results = []
    failed = False
    for index, path in enumerate(args.inputs, start=1):
        record = {
            "input": str(path),
            "config": asdict(config),
            "detected": False,
            "u": 0.0,
            "v": 0.0,
            "area": 0.0,
            "annotated": None,
            "mask": None,
            "error": None,
        }
        # The index prevents collisions even when input stems or paths repeat.
        stem = f"{index:06d}_{path.stem}"
        try:
            image = _read_image(path)
        except (OSError, ValueError, cv2.error) as exc:
            failed = True
            record["error"] = f"load error: {exc}"
            print(f"{path}: {record['error']}", file=sys.stderr)
            results.append(record)
            continue
        try:
            result = detect(image, config)
            record.update(detected=result.detected, u=result.u, v=result.v, area=result.area,
                          width=int(image.shape[1]), height=int(image.shape[0]))
            if result.detected:
                print(f"{path}: detected=True u={result.u:.2f} v={result.v:.2f} area={result.area:.2f}")
            else:
                print(f"{path}: No target (detected=False u=0 v=0 area=0)")
            annotated_path = args.output / f"{stem}_annotated.png"
            mask_path = args.output / f"{stem}_mask.png"
            # Protect inputs even if a user explicitly chooses an existing output
            # artifact as an input and writes into that same directory.
            input_paths = {item.resolve() for item in args.inputs}
            if annotated_path.resolve() in input_paths or mask_path.resolve() in input_paths:
                raise ValueError("output filename would overwrite an input image")
            _write_png(annotated_path, annotate(image, result))
            record["annotated"] = str(annotated_path)
            _write_png(mask_path, result.mask)
            record["mask"] = str(mask_path)
        except (OSError, ValueError, cv2.error) as exc:
            failed = True
            record["error"] = f"processing/output error: {exc}"
            print(f"{path}: {record['error']}", file=sys.stderr)
        results.append(record)

    report_path = args.output / "results.json"
    try:
        if report_path.resolve() in {item.resolve() for item in args.inputs}:
            raise ValueError("results.json would overwrite an input")
        payload = json.dumps({"results": results}, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
        _write_bytes(report_path, payload.encode("utf-8"))
    except (OSError, ValueError) as exc:
        print(f"Cannot write results JSON {report_path}: {exc}", file=sys.stderr)
        return 1
    print(f"Processed {len(results)} image(s); results: {report_path}")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
