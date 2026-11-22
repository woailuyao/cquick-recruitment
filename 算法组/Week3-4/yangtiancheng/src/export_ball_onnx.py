#!/usr/bin/env python3
"""Export the existing football best.pt to ONNX without retraining."""

import argparse
import hashlib
from pathlib import Path


def sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--opset", type=int, default=12)
    parser.add_argument("--expected-sha256")
    args = parser.parse_args()

    weights = Path(args.weights)
    if not weights.is_file():
        raise SystemExit(f"weights not found: {weights}")
    actual_sha = sha256(weights)
    print(f"weights: {weights}")
    print(f"weights_sha256: {actual_sha}")
    if args.expected_sha256 and actual_sha != args.expected_sha256.upper():
        raise SystemExit(
            f"weights SHA256 mismatch: expected {args.expected_sha256.upper()}")

    from ultralytics import YOLO

    model = YOLO(str(weights))
    exported = Path(model.export(
        format="onnx",
        imgsz=args.imgsz,
        opset=args.opset,
        dynamic=False,
        simplify=False,
    ))
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    if exported.resolve() != output.resolve():
        if output.exists():
            output.unlink()
        exported.replace(output)
    print(f"onnx: {output}")
    print(f"opset: {args.opset}")
    print(f"onnx_sha256: {sha256(output)}")


if __name__ == "__main__":
    main()
