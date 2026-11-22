#!/usr/bin/env python3
"""Generate the combined Task-2 acceptance view (camera + field map)."""
import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


COLORS = {"LCross": (80, 220, 80), "TCross": (0, 210, 255), "XCross": (70, 80, 255)}
DEFAULT_BALL_MODEL = Path(__file__).resolve().parent.parent / "ball_v3_best.pt"


def load_ball_model(path):
    """Load YOLO lazily so field-only utilities remain importable without torch."""
    from ultralytics import YOLO
    model_path = Path(path)
    if not model_path.is_file():
        raise FileNotFoundError(f"ball model not found: {model_path}")
    return YOLO(str(model_path))


def detect_ball(model, image, confidence=.40, imgsz=640):
    """Return at most one ball box in display-friendly dictionaries."""
    result = model.predict(image, imgsz=imgsz, conf=confidence, max_det=1,
                           device="cpu", verbose=False)[0]
    found = []
    for box in result.boxes:
        x1, y1, x2, y2 = (float(v) for v in box.xyxy[0].cpu().tolist())
        score = float(box.conf[0].cpu())
        found.append({"class": "ball", "confidence": score,
                      "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                      "u": (x1+x2)/2, "v": (y1+y2)/2})
    return found


def field_landmarks(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    result = []
    for item in data.get("landmarks", data.get("markers", [])):
        kind = item.get("type")
        if kind in COLORS:
            result.append((kind, float(item["x"]), float(item["y"])))
    return result


def make_field_view(result, field_path, width=720, height=720):
    canvas = np.full((height, width, 3), (38, 88, 46), np.uint8)
    scale = min((width - 90) / 9.0, (height - 90) / 6.0)
    ox, oy = width // 2, height // 2

    def p(x, y):
        return int(round(ox + x * scale)), int(round(oy - y * scale))

    white = (245, 245, 245)
    cv2.rectangle(canvas, p(-4.5, 3), p(4.5, -3), white, 3)
    cv2.line(canvas, p(0, -3), p(0, 3), white, 2)
    cv2.circle(canvas, p(0, 0), int(.75 * scale), white, 2)
    for sign in (-1, 1):
        gx, px, sx = sign * 4.5, sign * 2.5, sign * 3.5
        cv2.line(canvas, p(gx, 2.5), p(px, 2.5), white, 2)
        cv2.line(canvas, p(gx, -2.5), p(px, -2.5), white, 2)
        cv2.line(canvas, p(px, -2.5), p(px, 2.5), white, 2)
        cv2.line(canvas, p(gx, 1.5), p(sx, 1.5), white, 2)
        cv2.line(canvas, p(gx, -1.5), p(sx, -1.5), white, 2)
        cv2.line(canvas, p(sx, -1.5), p(sx, 1.5), white, 2)

    solution = result["solutions"][result.get("selected_solution", 0)]
    x, y, theta = solution["x_m"], solution["y_m"], solution["theta_rad"]
    landmarks = field_landmarks(field_path)
    c, s = math.cos(theta), math.sin(theta)
    for obs in result.get("ground_points_robot_m", []):
        mx = x + c * obs["x"] - s * obs["y"]
        my = y + s * obs["x"] + c * obs["y"]
        candidates = [q for q in landmarks if q[0] == obs["type"]]
        if candidates:
            target = min(candidates, key=lambda q: (q[1] - mx) ** 2 + (q[2] - my) ** 2)
            cv2.line(canvas, p(mx, my), p(target[1], target[2]), (0, 230, 255), 2)
            cv2.circle(canvas, p(target[1], target[2]), 7, (0, 230, 255), -1)
        cv2.circle(canvas, p(mx, my), 7, (255, 220, 30), -1)

    center = p(x, y)
    heading = p(x + .6 * math.cos(theta), y + .6 * math.sin(theta))
    cv2.circle(canvas, center, 12, (40, 40, 240), -1)
    cv2.arrowedLine(canvas, center, heading, (40, 40, 240), 5, tipLength=.3)
    cv2.putText(canvas, "FIELD MAP / MATCHES", (24, 32), cv2.FONT_HERSHEY_SIMPLEX,
                .8, white, 2, cv2.LINE_AA)
    cv2.putText(canvas, "cyan: observation  yellow: map target", (24, height - 18),
                cv2.FONT_HERSHEY_SIMPLEX, .52, (230, 230, 230), 1, cv2.LINE_AA)
    return canvas


def annotate_camera(image, features, balls=None, fps=None):
    view = image.copy()
    for ball in balls or []:
        x1, y1 = int(round(ball["x1"])), int(round(ball["y1"]))
        x2, y2 = int(round(ball["x2"])), int(round(ball["y2"]))
        u, v = int(round(ball["u"])), int(round(ball["v"]))
        color = (40, 235, 40)
        cv2.rectangle(view, (x1, y1), (x2, y2), color, 3)
        cv2.circle(view, (u, v), 5, color, -1)
        label = f"ball {ball['confidence']:.2f} center=({u},{v})"
        cv2.putText(view, label, (max(4, x1), max(24, y1-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .62, color, 2, cv2.LINE_AA)
    for item in features:
        u, v, kind = int(round(item["u"])), int(round(item["v"])), item["type"]
        color = COLORS.get(kind, (255, 255, 255))
        radius = 34
        cv2.rectangle(view, (max(0, u-radius), max(0, v-radius)),
                      (min(view.shape[1]-1, u+radius), min(view.shape[0]-1, v+radius)), color, 3)
        cv2.drawMarker(view, (u, v), color, cv2.MARKER_CROSS, 24, 3)
        cv2.putText(view, kind, (max(4, u-radius), max(25, v-radius-8)),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, color, 2, cv2.LINE_AA)
    cv2.putText(view, "CAMERA / DETECTIONS", (24, 38), cv2.FONT_HERSHEY_SIMPLEX,
                .9, (255, 255, 255), 2, cv2.LINE_AA)
    if fps is not None:
        text = f"FPS {fps:.2f}"
        size = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .82, 2)[0]
        cv2.rectangle(view, (view.shape[1]-size[0]-34, 10),
                      (view.shape[1]-10, 48), (20, 20, 20), -1)
        cv2.putText(view, text, (view.shape[1]-size[0]-22, 39),
                    cv2.FONT_HERSHEY_SIMPLEX, .82, (255, 255, 255), 2, cv2.LINE_AA)
    return view


def compose(camera, field, result, elapsed_ms, accepted, frame_index):
    h = 720
    camera = cv2.resize(camera, (1280, h))
    field = cv2.resize(field, (720, h))
    body = np.hstack([camera, field])
    panel = np.full((150, body.shape[1], 3), (22, 25, 30), np.uint8)
    solution = result["solutions"][result.get("selected_solution", 0)]
    valid = accepted and not result.get("ambiguous", False)
    status_color = (70, 220, 90) if valid else (60, 60, 240)
    line1 = (f"frame={frame_index:03d}  x_camera_map={solution['x_m']:+.3f} m  "
             f"y_camera_map={solution['y_m']:+.3f} m  yaw_camera_map={solution['theta_deg']:+.2f} deg")
    line2 = (f"marker_count={solution['inliers']}  residual={solution['rms_m']:.3f} m  "
             f"valid={str(valid).lower()}  status={'TRACKING' if valid else 'INVALID'}")
    fps = 1000.0 / max(elapsed_ms, 1e-6)
    line3 = f"processing={elapsed_ms:.1f} ms  localization_update={fps:.1f} Hz  FPS={fps:.1f}"
    for i, text in enumerate((line1, line2, line3)):
        cv2.putText(panel, text, (26, 39 + i * 42), cv2.FONT_HERSHEY_SIMPLEX,
                    .72, status_color if i == 1 else (235, 235, 235), 2, cv2.LINE_AA)
    return np.vstack([body, panel])


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--images", nargs="+", default=[str(here / "demo_images" / f"IMG_{n:04d}.jpg") for n in (25, 26, 27)])
    ap.add_argument("--output-dir", default=str(here / "acceptance_output"))
    ap.add_argument("--field", default=str(here / "field_geometry_kid_full.yaml"))
    ap.add_argument("--fps", type=float, default=15.0, help="output video playback FPS")
    ap.add_argument("--hold-seconds", type=float, default=2.0,
                    help="seconds to keep each processed image visible")
    ap.add_argument("--ball-model", default=str(DEFAULT_BALL_MODEL))
    ap.add_argument("--ball-conf", type=float, default=.40)
    args = ap.parse_args()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    prior = (-1.225, 1.233, -51.36)
    writer = None
    metrics = []
    ball_model = load_ball_model(args.ball_model)
    for index, image_name in enumerate(args.images):
        image_path = Path(image_name)
        frame_dir = output / image_path.stem
        start = time.perf_counter()
        command = [sys.executable, str(here / "run_task2_pipeline.py"), "--image", str(image_path),
                   "--output-dir", str(frame_dir), "--field", args.field, "--threshold", ".30",
                   "--prior", *(str(v) for v in prior), "--allow-provisional"]
        completed = subprocess.run(command, text=True, capture_output=True)
        elapsed_ms = (time.perf_counter() - start) * 1000.0
        result_path = frame_dir / f"{image_path.stem}_localization.json"
        valid = completed.returncode == 0 and result_path.exists()
        if not valid:
            print(completed.stdout, completed.stderr, file=sys.stderr)
            continue
        result = json.loads(result_path.read_text(encoding="utf-8"))
        features = json.loads((frame_dir / f"{image_path.stem}_features.json").read_text(encoding="utf-8"))
        solution = result["solutions"][result.get("selected_solution", 0)]
        prior = (solution["x_m"], solution["y_m"], solution["theta_deg"])
        source = cv2.imread(str(image_path))
        balls = detect_ball(ball_model, source, args.ball_conf)
        display_fps = 1000.0 / max(elapsed_ms, 1e-6)
        combined = compose(annotate_camera(source, features, balls, display_fps),
                           make_field_view(result, args.field), result, elapsed_ms, True, index)
        frame_path = output / f"acceptance_frame_{index:03d}.jpg"
        cv2.imwrite(str(frame_path), combined)
        if writer is None:
            writer = cv2.VideoWriter(str(output / "task2_acceptance_demo.mp4"),
                                     cv2.VideoWriter_fourcc(*"mp4v"), args.fps,
                                     (combined.shape[1], combined.shape[0]))
        # The prepared acceptance input contains still photographs rather than
        # a live camera stream. Repeat every processed result long enough for
        # the examiner to read the boxes, map and status fields.
        repeat = max(1, int(round(args.fps * args.hold_seconds)))
        for _ in range(repeat):
            writer.write(combined)
        metrics.append({"frame": index, "image": str(image_path), "valid": True,
                        "processing_ms": elapsed_ms, "fps": 1000.0 / elapsed_ms,
                        "x_camera_map": solution["x_m"], "y_camera_map": solution["y_m"],
                        "yaw_camera_map_deg": solution["theta_deg"],
                        "marker_count": solution["inliers"], "ball_count": len(balls),
                        "residual_m": solution["rms_m"]})
    if writer is not None:
        writer.release()
    (output / "acceptance_metrics.json").write_text(json.dumps(metrics, indent=2) + "\n", encoding="utf-8")
    if not metrics:
        raise SystemExit("no valid acceptance frames generated")
    duration = len(metrics) * args.hold_seconds
    print(f"acceptance source_frames={len(metrics)} valid={sum(row['valid'] for row in metrics)}")
    print(f"video_fps={args.fps:.1f} hold_seconds={args.hold_seconds:.1f} duration={duration:.1f}s")
    print(f"preview={output / 'acceptance_frame_000.jpg'}")
    print(f"video={output / 'task2_acceptance_demo.mp4'}")


if __name__ == "__main__":
    main()
