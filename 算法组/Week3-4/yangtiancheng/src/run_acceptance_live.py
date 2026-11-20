#!/usr/bin/env python3
"""Live/video acceptance UI: ball + field markers on RGB, localization on map."""
import argparse
import json
import subprocess
import sys
import tempfile
import threading
import time
from collections import Counter, deque
from pathlib import Path

import cv2
import numpy as np

from detect_field_features import detect as detect_markers_hsv
from run_acceptance_demo import (DEFAULT_BALL_MODEL, annotate_camera, detect_ball,
                                 load_ball_model, make_field_view)
from task2_localization import (
    load_camera as load_localization_camera,
    load_field,
    load_field_bounds,
    nearest_assignments,
    normalize_observations,
    pixel_to_ground,
)

def parse_source(value):
    return int(value) if str(value).isdigit() else str(value)


def open_capture(source):
    """Prefer DirectShow for Windows cameras; keep normal handling for videos."""
    if isinstance(source, int) and sys.platform.startswith("win"):
        capture = cv2.VideoCapture(source, cv2.CAP_DSHOW)
        if capture.isOpened():
            return capture
        capture.release()
    return cv2.VideoCapture(source)


def empty_field_view(width=720, height=720):
    canvas = np.full((height, width, 3), (38, 88, 46), np.uint8)
    scale = min((width-90)/9.0, (height-90)/6.0)
    ox, oy = width//2, height//2
    p = lambda x, y: (int(round(ox+x*scale)), int(round(oy-y*scale)))
    white = (245, 245, 245)
    cv2.rectangle(canvas, p(-4.5, 3), p(4.5, -3), white, 3)
    cv2.line(canvas, p(0, -3), p(0, 3), white, 2)
    cv2.circle(canvas, p(0, 0), int(.75*scale), white, 2)
    for sign in (-1, 1):
        gx, px, sx = sign*4.5, sign*2.5, sign*3.5
        cv2.line(canvas, p(gx, 2.5), p(px, 2.5), white, 2)
        cv2.line(canvas, p(gx, -2.5), p(px, -2.5), white, 2)
        cv2.line(canvas, p(px, -2.5), p(px, 2.5), white, 2)
        cv2.line(canvas, p(gx, 1.5), p(sx, 1.5), white, 2)
        cv2.line(canvas, p(gx, -1.5), p(sx, -1.5), white, 2)
        cv2.line(canvas, p(sx, -1.5), p(sx, 1.5), white, 2)
    cv2.putText(canvas, "FIELD MAP / LOCALIZATION INVALID", (24, 34),
                cv2.FONT_HERSHEY_SIMPLEX, .72, (50, 70, 255), 2, cv2.LINE_AA)
    return canvas


def compose(camera, field, state, display_fps, processing_fps,
            elapsed_ms, frame_index,
            marker_count=0, ball_count=0):
    camera = cv2.resize(camera, (1280, 720))
    field = cv2.resize(field, (720, 720))
    body = np.hstack((camera, field))
    panel = np.full((150, 2000, 3), (22, 25, 30), np.uint8)
    valid = state is not None
    color = (70, 220, 90) if valid else (60, 60, 240)
    if valid:
        sol = state["solutions"][state.get("selected_solution", 0)]
        line1 = (f"frame={frame_index:06d}  x_camera_map={sol['x_m']:+.3f} m  "
                 f"y_camera_map={sol['y_m']:+.3f} m  yaw_camera_map={sol['theta_deg']:+.2f} deg")
        line2 = (f"ball_count={ball_count}  marker_count={marker_count}  "
                 f"inliers={sol['inliers']}  residual={sol['rms_m']:.3f} m  "
                 "valid=true  status=TRACKING")
    else:
        line1 = f"frame={frame_index:06d}  x_camera_map=N/A  y_camera_map=N/A  yaw_camera_map=N/A"
        line2 = (f"ball_count={ball_count}  marker_count={marker_count}  "
                 "residual=N/A  valid=false  status=INVALID")
    line3 = (f"processing={elapsed_ms:.1f} ms  display_FPS={display_fps:.2f}  "
             f"processing_FPS={processing_fps:.2f}  press q to quit")
    for i, text in enumerate((line1, line2, line3)):
        cv2.putText(panel, text, (26, 39+i*42), cv2.FONT_HERSHEY_SIMPLEX, .72,
                    color if i == 1 else (235, 235, 235), 2, cv2.LINE_AA)
    return np.vstack((body, panel))


def localize(here, temp, features, prior, args):
    feature_path = temp / "features.json"
    result_path = temp / "localization.json"
    feature_path.write_text(json.dumps(features, ensure_ascii=False), encoding="utf-8")
    command = [sys.executable, str(here/"task2_localization.py"),
               "--intrinsics", args.intrinsics, "--extrinsics", args.extrinsics,
               "--field", args.field, "--features", str(feature_path),
               "--threshold", str(args.threshold), "--output", str(result_path),
               "--allow-provisional", "--prior", *(str(v) for v in prior)]
    completed = subprocess.run(command, capture_output=True, text=True)
    if completed.returncode or not result_path.exists():
        return None
    return json.loads(result_path.read_text(encoding="utf-8"))


def rolling_fps(timestamps, window_seconds=2.0):
    if len(timestamps) < 2:
        return 0.0
    latest = timestamps[-1]
    while len(timestamps) > 2 and latest - timestamps[0] > window_seconds:
        timestamps.popleft()
    duration = timestamps[-1] - timestamps[0]
    return (len(timestamps) - 1) / duration if duration > 1e-9 else 0.0


def filter_candidates_on_ground(candidates, k, dist, transform, bounds):
    kept = []
    rejected = []
    for point, kind, score in candidates:
        u, v = float(point[0]), float(point[1])
        try:
            ground = pixel_to_ground((u, v), k, dist, transform)
        except (ValueError, FloatingPointError) as exc:
            rejected.append({
                "type": kind,
                "u": round(u, 1),
                "v": round(v, 1),
                "reason": f"ground_projection_failed: {exc}",
            })
            continue
        if bounds is not None and (
                abs(float(ground[0])) > bounds[0] or
                abs(float(ground[1])) > bounds[1]):
            rejected.append({
                "type": kind,
                "u": round(u, 1),
                "v": round(v, 1),
                "ground_x": float(ground[0]),
                "ground_y": float(ground[1]),
                "reason": "outside_field_plus_1m",
            })
            continue
        kept.append((point, kind, score))
    return kept, rejected


def matched_feature_indices(features, state, k, dist, transform,
                            field, threshold):
    if state is None:
        return set()
    observations = normalize_observations(features, k, dist, transform)
    solution = state["solutions"][state.get("selected_solution", 0)]
    pose = (solution["x_m"], solution["y_m"], solution["theta_rad"])
    matches = nearest_assignments(observations, field, pose, threshold)
    return {int(match[0]["index"]) for match in matches}


def main():
    here = Path(__file__).resolve().parent
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="camera index or video path")
    ap.add_argument("--ball-model", default=str(DEFAULT_BALL_MODEL))
    ap.add_argument("--ball-conf", type=float, default=.40)
    ap.add_argument("--intrinsics", default=str(here.parent/"camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=str(here/"camera_extrinsics_H72p5_G140_IMG0024_provisional.yaml"))
    ap.add_argument("--field", default=str(here/"field_geometry_kid_full.yaml"))
    ap.add_argument("--threshold", type=float, default=.30)
    ap.add_argument("--initial-prior", nargs=3, type=float, default=(-1.225, 1.233, -51.36))
    ap.add_argument("--output", help="optional annotated MP4")
    ap.add_argument("--no-display", action="store_true")
    ap.add_argument("--display-max-width", type=int, default=1600,
                    help="resize only the preview window; processing stays 1280x720")
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--duration-seconds", type=float, default=0.0,
                    help="stop after this many seconds; 0 means no time limit")
    ap.add_argument("--frame-step", type=int, default=1,
                    help="process every Nth source frame (1 for real-time camera)")
    ap.add_argument("--start-seconds", type=float, default=0.0,
                    help="seek video input before processing")
    ap.add_argument("--crop", nargs=4, type=int, metavar=("X1", "Y1", "X2", "Y2"),
                    help="optional source-frame crop before resizing to 1280x720")
    args = ap.parse_args()

    cap = open_capture(parse_source(args.source))
    if not cap.isOpened():
        raise SystemExit(f"cannot open source: {args.source}")
    if str(args.source).isdigit():
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    elif args.start_seconds > 0:
        cap.set(cv2.CAP_PROP_POS_MSEC, args.start_seconds*1000.0)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    model = load_ball_model(args.ball_model)
    k, dist, transform, _ = load_localization_camera(
        args.intrinsics, args.extrinsics)
    field_bounds = load_field_bounds(args.field, margin=1.0)
    localization_field = load_field(args.field)
    camera_mode = str(args.source).isdigit()
    stop_event = threading.Event()
    frame_condition = threading.Condition()
    result_lock = threading.Lock()
    latest_frame = None
    latest_sequence = 0
    latest_result = {
        "sequence": 0,
        "features": [],
        "balls": [],
        "state": None,
        "marker_count": 0,
        "validated_indices": set(),
        "processing_ms": 0.0,
        "processing_fps": 0.0,
    }
    errors = []
    metrics = []
    display_total = [0]
    display_timestamps = deque()
    processing_timestamps = deque()
    window_name = "CQuick acceptance: ball + markers + map"
    if not args.no_display:
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)

    started_at = time.monotonic()
    if args.duration_seconds:
        threading.Timer(
            args.duration_seconds, stop_event.set).start()

    def display_loop():
        nonlocal latest_frame, latest_sequence
        writer = None
        display_frames = 0
        source_index = 0
        while not stop_event.is_set():
            ok, frame = cap.read()
            if not ok:
                if camera_mode:
                    errors.append(
                        f"camera read failed after display frame {display_frames}")
                stop_event.set()
                break
            if source_index % max(1, args.frame_step):
                source_index += 1
                continue
            source_index += 1
            if args.crop:
                x1, y1, x2, y2 = args.crop
                if not (0 <= x1 < x2 <= frame.shape[1] and 0 <= y1 < y2 <= frame.shape[0]):
                    raise SystemExit(f"invalid crop {args.crop} for frame {frame.shape[1]}x{frame.shape[0]}")
                frame = frame[y1:y2, x1:x2]
            if frame.shape[1::-1] != (1280, 720):
                frame = cv2.resize(frame, (1280, 720))

            with frame_condition:
                latest_frame = frame.copy()
                latest_sequence += 1
                frame_condition.notify()

            now = time.monotonic()
            display_timestamps.append(now)
            display_total[0] += 1
            display_fps = rolling_fps(display_timestamps)
            with result_lock:
                result = latest_result.copy()
            fields = result["features"]
            balls = result["balls"]
            state = result["state"]
            processing_fps = result["processing_fps"]
            processing_ms = result["processing_ms"]
            validated_indices = result["validated_indices"]
            field_view = (make_field_view(state, args.field)
                          if state is not None else empty_field_view())
            validated_features = [
                item for index, item in enumerate(fields)
                if index in validated_indices
            ]
            camera_view = annotate_camera(
                frame, validated_features, balls, display_fps)
            for index, item in enumerate(fields):
                if index not in validated_indices:
                    cv2.circle(
                        camera_view,
                        (int(round(item["u"])), int(round(item["v"]))),
                        3, (145, 145, 145), -1, cv2.LINE_AA)
            combined = compose(
                camera_view, field_view, state, display_fps,
                processing_fps, processing_ms, display_frames,
                marker_count=result["marker_count"],
                ball_count=len(balls))
            if args.output:
                if writer is None:
                    source_fps = cap.get(cv2.CAP_PROP_FPS)
                    output_fps = (source_fps/max(1, args.frame_step)
                                  if 0 < source_fps <= 60
                                  else max(1.0, display_fps))
                    output_fps = max(.2, output_fps)
                    writer = cv2.VideoWriter(args.output, cv2.VideoWriter_fourcc(*"mp4v"),
                                             output_fps, (combined.shape[1], combined.shape[0]))
                writer.write(combined)
            if not args.no_display:
                display = combined
                if args.display_max_width and combined.shape[1] > args.display_max_width:
                    scale = args.display_max_width / combined.shape[1]
                    display = cv2.resize(
                        combined,
                        (args.display_max_width,
                         int(round(combined.shape[0] * scale))),
                        interpolation=cv2.INTER_AREA,
                    )
                cv2.imshow(window_name, display)
                if cv2.waitKey(1) & 0xff == ord("q"):
                    stop_event.set()
                    break
            display_frames += 1
            if args.duration_seconds and now - started_at >= args.duration_seconds:
                stop_event.set()
                break
            if args.max_frames and display_frames >= args.max_frames:
                stop_event.set()
                break
        if writer is not None:
            writer.release()
        return display_frames

    def processing_loop():
        nonlocal latest_result
        prior = tuple(args.initial_prior)
        consumed_sequence = 0
        with tempfile.TemporaryDirectory(prefix="cquick_acceptance_") as temp_name:
            temp = Path(temp_name)
            while not stop_event.is_set():
                with frame_condition:
                    while (latest_sequence == consumed_sequence and
                           not stop_event.is_set()):
                        frame_condition.wait(.05)
                    if latest_sequence == consumed_sequence:
                        break
                    consumed_sequence = latest_sequence
                    frame = latest_frame.copy()
                started = time.perf_counter()
                balls = detect_ball(model, frame, args.ball_conf)
                marker_candidates_raw, _ = detect_markers_hsv(frame)
                marker_candidates, rejected_candidates = \
                    filter_candidates_on_ground(
                        marker_candidates_raw, k, dist, transform,
                        field_bounds)
                raw_features = [
                    {"type": kind, "u": round(float(point[0]), 1),
                     "v": round(float(point[1]), 1)}
                    for point, kind, _ in marker_candidates_raw
                ]
                features = [
                    {"type": kind, "u": round(float(point[0]), 1),
                     "v": round(float(point[1]), 1)}
                    for point, kind, _ in marker_candidates
                ]
                state = (localize(here, temp, features, prior, args)
                         if len(features) >= 2 else None)
                if state is not None:
                    solution = state["solutions"][
                        state.get("selected_solution", 0)]
                    prior = (
                        solution["x_m"], solution["y_m"],
                        solution["theta_deg"])
                validated_indices = matched_feature_indices(
                    features, state, k, dist, transform,
                    localization_field, args.threshold)
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                now = time.monotonic()
                processing_timestamps.append(now)
                processing_fps = rolling_fps(processing_timestamps)
                result = {
                    "sequence": consumed_sequence,
                    "features": features,
                    "balls": balls,
                    "state": state,
                    "marker_count": len(features),
                    "raw_marker_count": len(marker_candidates_raw),
                    "rejected_marker_count": len(rejected_candidates),
                    "validated_indices": validated_indices,
                    "processing_ms": elapsed_ms,
                    "processing_fps": processing_fps,
                }
                with result_lock:
                    latest_result = result
                solution = (state["solutions"][state.get("selected_solution", 0)]
                            if state is not None else None)
                metrics.append({
                    "processed_frame": len(metrics),
                    "source_sequence": consumed_sequence,
                    "ball_count": len(balls),
                    "marker_count": len(features),
                    "raw_marker_count": len(marker_candidates_raw),
                    "filtered_marker_count": len(features),
                    "rejected_marker_count": len(rejected_candidates),
                    "rejected_markers": rejected_candidates,
                    "raw_marker_features": raw_features,
                    "marker_classes": [item["type"] for item in features],
                    "marker_features": features,
                    "validated_indices": sorted(validated_indices),
                    "valid": state is not None,
                    "x_m": solution["x_m"] if solution else None,
                    "y_m": solution["y_m"] if solution else None,
                    "yaw_deg": solution["theta_deg"] if solution else None,
                    "residual_m": solution["rms_m"] if solution else None,
                    "processing_ms": elapsed_ms,
                    "processing_fps": processing_fps,
                })

    processing_thread = threading.Thread(
        target=processing_loop, name="latest-frame-processing", daemon=True)
    processing_thread.start()
    display_loop()
    stop_event.set()
    with frame_condition:
        frame_condition.notify_all()
    processing_thread.join(timeout=15.0)
    if processing_thread.is_alive():
        errors.append("processing thread did not stop after timeout")
    cap.release()
    if not args.no_display:
        cv2.destroyAllWindows()

    if not metrics:
        raise SystemExit("; ".join(errors) if errors else "no frames processed")
    elapsed = max(time.monotonic() - started_at, 1e-9)
    display_fps = rolling_fps(display_timestamps)
    processing_fps = (
        (len(processing_timestamps) - 1) /
        max(processing_timestamps[-1] - processing_timestamps[0], 1e-9)
        if len(processing_timestamps) > 1 else 0.0)
    summary = {
        "frames": len(metrics),
        "display_frames": display_total[0],
        "processed_frames": len(metrics),
        "duration_seconds": elapsed,
        "display_fps": display_fps,
        "display_fps_mean": display_total[0] / elapsed,
        "processing_fps": processing_fps,
        "processing_fps_mean": len(metrics) / elapsed,
        "overwritten_frames": max(
            0, display_total[0] - len(metrics)),
        "source": str(args.source),
        "ball_model": str(args.ball_model),
        "ball_conf": args.ball_conf,
        "marker_detector": "HSV detect_field_features.detect",
        "localization_threshold": args.threshold,
        "marker_class_instance_counts": dict(Counter(
            kind for row in metrics for kind in row["marker_classes"])),
        "raw_marker_candidates_total": sum(
            row["raw_marker_count"] for row in metrics),
        "filtered_marker_candidates_total": sum(
            row["filtered_marker_count"] for row in metrics),
        "rejected_marker_candidates_total": sum(
            row["rejected_marker_count"] for row in metrics),
        "same_frame_ball_and_marker": sum(
            m["ball_count"] > 0 and m["marker_count"] > 0 for m in metrics),
        "ball_frames": sum(m["ball_count"] > 0 for m in metrics),
        "marker_frames": sum(m["marker_count"] > 0 for m in metrics),
        "valid_localization_frames": sum(m["valid"] for m in metrics),
        "mean_processing_ms": (
            sum(m["processing_ms"] for m in metrics) / len(metrics)),
        "errors": errors,
    }
    if args.output:
        Path(args.output).with_suffix(".json").write_text(
            json.dumps({"summary": summary, "frames": metrics}, ensure_ascii=False, indent=2)+"\n",
            encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
