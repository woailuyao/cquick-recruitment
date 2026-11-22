#!/usr/bin/env python3
"""Evaluate a trained marker detector on the independent test split.

Reports per-class Precision / Recall / mAP50 / mAP50-95 (required by the
spec for LCross, TCross, XCross, PenaltyPoint) and optionally benchmarks
inference speed on a video or image folder.

Usage:
    python3 evaluate_markers.py --weights runs_markers/markers_v1/weights/best.pt \
        --data ./marker_dataset/data.yaml
    python3 evaluate_markers.py --weights ... --data ... --video 02_glare.avi
"""
import argparse
import json
import time
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--data", required=True)
    ap.add_argument("--split", default="test", choices=["test", "val"])
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--video", help="optional video for an FPS benchmark")
    ap.add_argument("--output", help="optional JSON report path")
    args = ap.parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("ultralytics not installed: pip install ultralytics")

    model = YOLO(args.weights)
    metrics = model.val(data=args.data, split=args.split, imgsz=args.imgsz,
                        verbose=False)

    names = metrics.names
    report = {"weights": args.weights, "split": args.split, "per_class": {}}
    box = metrics.box
    for i, cls_id in enumerate(box.ap_class_index):
        name = names[int(cls_id)]
        report["per_class"][name] = {
            "precision": round(float(box.p[i]), 4),
            "recall": round(float(box.r[i]), 4),
            "mAP50": round(float(box.ap50[i]), 4),
            "mAP50-95": round(float(box.ap[i]), 4),
        }
    report["overall"] = {
        "precision": round(float(box.mp), 4),
        "recall": round(float(box.mr), 4),
        "mAP50": round(float(box.map50), 4),
        "mAP50-95": round(float(box.map), 4),
    }

    if args.video:
        import cv2
        cap = cv2.VideoCapture(args.video)
        frames = 0
        start = time.perf_counter()
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            model.predict(frame, imgsz=args.imgsz, verbose=False)
            frames += 1
        elapsed = time.perf_counter() - start
        cap.release()
        if frames:
            report["benchmark"] = {
                "video": args.video,
                "frames": frames,
                "fps": round(frames / elapsed, 2),
                "ms_per_frame": round(1000 * elapsed / frames, 2),
            }

    print(json.dumps(report, ensure_ascii=False, indent=2))
    if args.output:
        Path(args.output).write_text(
            json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        print(f"report saved: {args.output}")


if __name__ == "__main__":
    main()
