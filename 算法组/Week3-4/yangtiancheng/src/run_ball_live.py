#!/usr/bin/env python3
"""Standalone live football detector using the existing YOLO ball model."""

import argparse
import json
import sys
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np

from run_acceptance_demo import DEFAULT_BALL_MODEL, detect_ball, load_ball_model


def parse_source(value):
    return int(value) if str(value).isdigit() else str(value)


def open_capture(source):
    if isinstance(source, int) and sys.platform.startswith("win"):
        capture = cv2.VideoCapture(source, cv2.CAP_DSHOW)
        if capture.isOpened():
            return capture
        capture.release()
    return cv2.VideoCapture(source)


def rolling_fps(timestamps, window_seconds=2.0):
    if len(timestamps) < 2:
        return 0.0
    latest = timestamps[-1]
    while len(timestamps) > 2 and latest - timestamps[0] > window_seconds:
        timestamps.popleft()
    duration = timestamps[-1] - timestamps[0]
    return (len(timestamps) - 1) / duration if duration > 1e-9 else 0.0


def annotate_ball(frame, balls, fps, processing_ms):
    camera = frame.copy()
    for ball in balls:
        x1, y1 = int(round(ball["x1"])), int(round(ball["y1"]))
        x2, y2 = int(round(ball["x2"])), int(round(ball["y2"]))
        u, v = int(round(ball["u"])), int(round(ball["v"]))
        cv2.rectangle(camera, (x1, y1), (x2, y2), (40, 235, 40), 3)
        cv2.circle(camera, (u, v), 5, (40, 235, 40), -1)
        cv2.putText(
            camera,
            f"ball {ball['confidence']:.3f} center=({u},{v})",
            (max(4, x1), max(28, y1 - 9)),
            cv2.FONT_HERSHEY_SIMPLEX, .7, (40, 235, 40), 2,
            cv2.LINE_AA,
        )
    cv2.putText(camera, "BALL / YOLO", (24, 38),
                cv2.FONT_HERSHEY_SIMPLEX, .9, (255, 255, 255), 2,
                cv2.LINE_AA)
    panel = np.full((110, camera.shape[1], 3), (22, 25, 30), np.uint8)
    lines = [
        f"display_FPS={fps:.2f}  processing={processing_ms:.1f} ms",
        f"ball_count={len(balls)}  confidence={balls[0]['confidence']:.3f}"
        if balls else "ball_count=0",
    ]
    for index, text in enumerate(lines):
        cv2.putText(panel, text, (24, 38 + index * 42),
                    cv2.FONT_HERSHEY_SIMPLEX, .72,
                    (70, 220, 90) if balls else (235, 235, 235),
                    2, cv2.LINE_AA)
    return np.vstack((camera, panel))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0")
    ap.add_argument("--ball-model", default=str(DEFAULT_BALL_MODEL))
    ap.add_argument("--ball-conf", type=float, default=.40)
    ap.add_argument("--output")
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--display-max-width", type=int, default=1400)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--duration-seconds", type=float, default=0.0)
    args = ap.parse_args()

    cap = open_capture(parse_source(args.source))
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {args.source}")
    if str(args.source).isdigit():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    model = load_ball_model(args.ball_model)
    writer = None
    timestamps = deque()
    metrics = []
    started_at = time.monotonic()
    window_name = "CQuick ball detection: YOLO"
    if not args.no_display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if frame.shape[1::-1] != (1280, 720):
            frame = cv2.resize(frame, (1280, 720))
        started = time.perf_counter()
        balls = detect_ball(model, frame, args.ball_conf)
        processing_ms = (time.perf_counter() - started) * 1000.0
        now = time.monotonic()
        timestamps.append(now)
        fps = rolling_fps(timestamps)
        combined = annotate_ball(frame, balls, fps, processing_ms)
        metrics.append({
            "frame": len(metrics),
            "ball_count": len(balls),
            "confidence": balls[0]["confidence"] if balls else None,
            "processing_ms": processing_ms,
        })
        if args.output:
            if writer is None:
                source_fps = cap.get(cv2.CAP_PROP_FPS)
                output_fps = source_fps if 0 < source_fps <= 60 else max(1.0, fps)
                writer = cv2.VideoWriter(
                    args.output, cv2.VideoWriter_fourcc(*"mp4v"),
                    max(.2, output_fps),
                    (combined.shape[1], combined.shape[0]))
            writer.write(combined)
        if not args.no_display:
            display = combined
            if args.display_max_width and combined.shape[1] > args.display_max_width:
                scale = args.display_max_width / combined.shape[1]
                display = cv2.resize(
                    combined,
                    (args.display_max_width,
                     int(round(combined.shape[0] * scale))),
                    interpolation=cv2.INTER_AREA)
            cv2.imshow(window_name, display)
            if cv2.waitKey(1) & 0xff == ord("q"):
                break
        if args.duration_seconds and now - started_at >= args.duration_seconds:
            break
        if args.max_frames and len(metrics) >= args.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
    if not args.no_display:
        cv2.destroyAllWindows()
    if not metrics:
        raise SystemExit("no frames processed")
    elapsed = max(time.monotonic() - started_at, 1e-9)
    summary = {
        "frames": len(metrics),
        "duration_seconds": elapsed,
        "display_fps": rolling_fps(timestamps),
        "mean_fps": len(metrics) / elapsed,
        "ball_frames": sum(row["ball_count"] > 0 for row in metrics),
        "mean_processing_ms": (
            sum(row["processing_ms"] for row in metrics) / len(metrics)),
        "source": str(args.source),
        "ball_model": str(args.ball_model),
        "ball_conf": args.ball_conf,
    }
    if args.output:
        Path(args.output).with_suffix(".json").write_text(
            json.dumps({"summary": summary, "frames": metrics},
                       ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
