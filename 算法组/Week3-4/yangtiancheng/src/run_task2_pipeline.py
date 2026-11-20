#!/usr/bin/env python3
"""One-command field feature detection and task-2 localization."""
import argparse
import subprocess
import sys
from pathlib import Path


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--output-dir", default=str(here / "pipeline_output"))
    ap.add_argument("--intrinsics", default=str(here.parent / "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics",
                    default=str(here / "camera_extrinsics_H72p5_G140_IMG0024_provisional.yaml"))
    ap.add_argument("--field", default=str(here / "field_geometry_kid_full.yaml"))
    ap.add_argument("--threshold", type=float, default=.35)
    ap.add_argument("--prior", nargs=3, type=float, metavar=("X", "Y", "THETA_DEG"))
    ap.add_argument("--allow-provisional", action="store_true")
    args = ap.parse_args()

    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    stem = Path(args.image).stem
    features = output_dir / f"{stem}_features.json"
    preview = output_dir / f"{stem}_preview.jpg"
    result = output_dir / f"{stem}_localization.json"

    subprocess.run([
        sys.executable, str(here / "detect_field_features.py"),
        "--image", args.image, "--output", str(features), "--preview", str(preview),
    ], check=True)

    command = [
        sys.executable, str(here / "task2_localization.py"),
        "--intrinsics", args.intrinsics,
        "--extrinsics", args.extrinsics,
        "--field", args.field,
        "--features", str(features),
        "--threshold", str(args.threshold),
        "--output", str(result),
    ]
    if args.allow_provisional:
        command.append("--allow-provisional")
    if args.prior:
        command.extend(["--prior", *(str(x) for x in args.prior)])
    subprocess.run(command, check=True)
    print(f"features: {features}")
    print(f"preview: {preview}")
    print(f"localization: {result}")


if __name__ == "__main__":
    main()
