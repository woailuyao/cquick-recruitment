"""Single-frame, single-reader automatic field localization demonstration."""
import argparse
from collections import Counter, deque
import csv
from datetime import datetime
import json
from pathlib import Path
import time

import cv2
import numpy as np

from localize import CURRENT_MOUNT, HERE, ROOT, SITE, Locator, invalid
from demo import Detectors, MODEL_COLORS, WEIGHTS
from live import draw_map
from prototype import sha, utc_now
from session import Frame
from transport import Bridge

DATASET = SITE / "dataset_expansion_20261119/dataset"
VAL = ROOT / "results/site_20261119/val_targeted_20261119_160928"
WINDOW = "CQuick | YOLO automatic localization"


class Engine:
    def __init__(self, detectors, locator):
        self.detectors, self.locator = detectors, locator
        self.frames, self.valid_frames = deque(maxlen=30), deque(maxlen=30)
        self.last_session = None

    def step(self, frame, transport, read_age, read_ms, total_start, freshness=None):
        timings = dict(read_ms=read_ms, field_yolo_ms=None, ball_yolo_ms=None,
                       refinement_ms=None, projection_ms=None, matching_ms=None)
        detections = dict(ball=[], field=[])
        points, observations = [], []
        publish_age = None
        attempted = None
        if frame is None:
            self.frames.clear()
            pose = invalid("bridge_" + transport)
        else:
            if frame.session != self.last_session:
                self.frames.clear()
                self.valid_frames.clear()
            self.last_session = frame.session
            if frame.image.shape[:2] != self.locator.shape:
                pose = invalid("resolution_mismatch")
            else:
                detections, cost = self.detectors.infer(frame.image)
                timings.update(field_yolo_ms=cost["field"], ball_yolo_ms=cost["ball"])
                pose, points, observations, costs = self.locator.locate(frame.image, detections["field"])
                timings.update(costs)
            attempted = pose["status"]
            if freshness is not None:
                good, transport, publish_age = freshness(frame)
                if not good:
                    for p in points:
                        p.update(map_id=None, map_xy_m=None)
                        if p["status"] == "matched":
                            p["status"] = "stale_unpublished"
                    pose = invalid("bridge_" + transport)
            if transport == "ok":
                self.frames.append(time.perf_counter())
            else:
                self.frames.clear()
        if pose["valid"]:
            self.valid_frames.append(time.perf_counter())
        else:
            self.valid_frames.clear()

        def rate(series):
            return (len(series) - 1) / (series[-1] - series[0]) if len(series) > 1 else 0.

        candidate_count = pose["marker_count"]
        pose["marker_count"] = len(pose["matches"])
        return dict(**pose, time_utc=utc_now(), candidate_marker_count=candidate_count,
                    source_session=frame.session if frame else read_age.get("source_session"),
                    sequence=frame.number if frame else read_age.get("sequence"),
                    capture_monotonic_s=frame.capture_s if frame else None,
                    producer=frame.producer if frame else None,
                    transport_status=transport, read_age=read_age, publish_age=publish_age,
                    mount_qualification=self.locator.mount_qualification,
                    current_installation_verified=self.locator.mount_qualification == "ok",
                    unpublished_localization_status=attempted,
                    marker_observation_count=len(observations), observation_indices=observations,
                    detections=detections, intersections=points,
                    fps=rate(self.frames), pose_update_hz=rate(self.valid_frames),
                    timings_ms=dict(timings, total_to_publish_ms=(time.perf_counter() - total_start) * 1000))


def put(canvas, s, x, y, color=(233, 233, 233), scale=.53):
    cv2.putText(canvas, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def compose(frame, packet, locator, geometry_view=False):
    im = frame.image.copy() if frame is not None else np.zeros((720, 1280, 3), np.uint8)
    if frame is not None and packet["transport_status"] == "ok":
        for role, boxes in packet["detections"].items():
            color = MODEL_COLORS[role]
            for d in boxes:
                x1, y1, x2, y2 = map(round, d["bbox"])
                cv2.rectangle(im, (x1, y1), (x2, y2), color, 2)
                suffix = ""
                if role == "ball":
                    center = round(d["center_u"]), round(d["center_v"])
                    cv2.circle(im, center, 5, color, -1)
                    suffix = f" {center}"
                put(im, f'{d["class_name"]} {d["confidence"]:.2f}' + suffix,
                    x1, max(24, y1-4), color)
        for p in packet["intersections"]:
            if p["pixel_uv"] is None:
                continue
            u, v = map(round, p["pixel_uv"])
            cv2.drawMarker(im, (u, v), (70, 255, 140), cv2.MARKER_CROSS, 14, 2)
            put(im, p["map_id"] or "unmatched", min(u+6, im.shape[1]-160), max(v-7, 17),
                (70, 255, 140))
    else:
        im[:] = 0
        put(im, "No fresh RGB frame", 16, 42)
    rgb = cv2.resize(im, (960, 540), interpolation=cv2.INTER_AREA)
    panel = draw_map(locator.map, packet)
    if geometry_view:
        panel[:] = 35
        put(panel, "AUTO intersection (u,v) -> carrier XY (m) -> map XY (m)", 14, 205)
        put(panel, "Carrier: X forward, Y left. Map heading: deg CCW from +X", 14, 225)
        used = [v for v in packet["intersections"] if v["pixel_uv"] is not None]
        for i, p in enumerate(used[:18]):
            uv = ",".join(f"{a:.0f}" for a in p["pixel_uv"])
            xy = ",".join(f"{a:+.2f}" for a in p["carrier_xy_m"]) if p["carrier_xy_m"] else "--"
            mp = ",".join(f"{a:+.2f}" for a in p["map_xy_m"]) if p["map_xy_m"] else "--"
            put(panel, f"({uv}) -> ({xy}) -> ({mp})  {p['map_id'] or p['status']}", 14, 249+i*24, scale=.46)
    def number(value, precision=3):
        return "--" if value is None else f"{value:.{precision}f}"

    put(panel, "Automatic: YOLO / line intersections / ground / map", 12, 30)
    put(panel, f"valid={str(packet['valid']).lower()}  {packet['status']}", 12, 58,
        (110, 240, 140) if packet["valid"] else (65, 208, 255))
    put(panel, "Camera X/Y (m): " + number(packet["x_camera_map"]) + " / "
        + number(packet["y_camera_map"]) + "   Yaw (deg): " + number(packet["yaw_camera_map"], 1), 12, 86)
    put(panel, "Carrier X/Y (m): " + number(packet["x_carrier_map"]) + " / "
        + number(packet["y_carrier_map"]) + "   Yaw (deg): " + number(packet["yaw_carrier_map"], 1), 12, 111)
    residual = "N/A" if packet["residual"] is None else number(packet["residual"])
    put(panel, f"Matched: {packet['marker_count']}  Obs: {packet['marker_observation_count']}  RMS: {residual} m   "
        f"FPS: {packet['fps']:.1f}   Pose: {packet['pose_update_hz']:.1f} Hz", 12, 139)
    if packet["mount_qualification"] == "approximate_mount_20261116":
        put(panel, "APPROX 2026-11-16 MOUNT | CURRENT INSTALLATION UNVERIFIED",
            12, 163, (65, 208, 255), .51)
    else:
        put(panel, "G: ground details   Q/Esc: exit", 12, 163)
    put(panel, f"Sequence: {packet['sequence']}  {packet.get('input_kind', '')}", 12, 184)
    return np.hstack((rgb, cv2.resize(panel, (480, 540), interpolation=cv2.INTER_AREA)))


def fixed_images():
    manifest = DATASET / "manifest.json"
    if sha(manifest) != "c881a2aad6196179fb62b7f75b1b8ccc33e260297bf8ff4262a5aad7070e545a":
        raise ValueError("fixed validation manifest SHA256 changed")
    rows = [row for row in json.loads(manifest.read_text()) if row["split"] == "val"]
    for i, row in enumerate(rows, 1):
        path = DATASET / row["image"]
        if sha(path) != row["image_sha256"]:
            raise ValueError(f"validation image mismatch: {path}")
        image = cv2.imread(str(path))
        if image is None:
            raise ValueError(f"validation decode failed: {path}")
        yield Frame(image, "fixed_validation_images", i, 0.,
                    dict(path=str(path), source=row["source"], source_frame=row["frame"]))


def validation_video():
    path = VAL / "camera.avi"
    with (VAL / "frame_timestamps.csv").open(newline="") as f:
        observed = list(csv.DictReader(f))
    cap = cv2.VideoCapture(str(path))
    try:
        if not cap.isOpened():
            raise ValueError(f"video open failed: {path}")
        for i, row in enumerate(observed, 1):
            ok, image = cap.read()
            if not ok or int(row["frame"]) != i:
                raise ValueError(f"video or source timestamp missing: frame {i}")
            yield Frame(image, "raw_validation_video", i, float(row["elapsed_s"]),
                        dict(path=str(path), source_timestamp=row["timestamp"], timestamp_basis="elapsed_csv"))
        if cap.read()[0]:
            raise ValueError("video frames exceed source timestamp records")
    finally:
        cap.release()


def run(args, locator, detectors, source):
    out = args.out or HERE / "runs" / (("live_" if source else args.offline + "_")
                                      + detectors.metadata["field"]["sha256"][:10] + "_"
                                      + datetime.now().strftime("%Y%m%d_%H%M%S_%f"))
    out.mkdir(parents=True, exist_ok=False)
    input_kind = "windows_v2_live" if source else args.offline
    metadata = dict(**locator.provenance, detectors=detectors.metadata, input_kind=input_kind,
                    transport=source.provenance if source else None, started_utc=utc_now())
    (out / "run.json").write_text(json.dumps(metadata, indent=2, allow_nan=False) + "\n")
    print(json.dumps(metadata, indent=2, allow_nan=False), flush=True)
    print(f"Logs: {out}", flush=True)
    print("G ground details; Q/Esc close window; Ctrl-C exit headless.", flush=True)
    engine = Engine(detectors, locator)
    frames = None if source else iter(fixed_images() if args.offline == "fixed-val" else validation_video())
    counts, stage = Counter(), {name: Counter() for name in ("LCross", "TCross", "XCross")}
    durations = {key: [] for key in ("read_ms", "field_yolo_ms", "ball_yolo_ms",
                                     "refinement_ms", "projection_ms", "matching_ms", "total_to_publish_ms")}
    view = False
    saved = set()
    try:
        with (out / "frames.jsonl").open("w", buffering=1) as log:
            while True:
                base = time.perf_counter()
                if source:
                    frame, state, age = source.next_frame()
                else:
                    frame = next(frames, None)
                    if frame is None:
                        break
                    state, age = "ok", {}
                read_ms = (time.perf_counter() - base) * 1000
                packet = engine.step(frame, state, age, read_ms, base, source.fresh if source else None)
                packet["input_kind"] = input_kind
                # No rendering, evidence I/O or detector between the final freshness check and this publish.
                log.write(json.dumps(packet, allow_nan=False) + "\n")
                counts[packet["status"]] += 1
                for d, p in zip(packet["detections"]["field"], packet["intersections"]):
                    stage[d["class_name"]]["boxes"] += 1
                    stage[d["class_name"]][p["status"]] += 1
                for key, value in packet["timings_ms"].items():
                    if value is not None:
                        durations[key].append(value)
                if not args.headless or (not source and packet["status"] not in saved):
                    canvas = compose(frame, packet, locator, view)
                    if not source and packet["status"] not in saved:
                        if not cv2.imwrite(str(out / f"first_{packet['status']}.jpg"), canvas):
                            raise OSError("cannot save offline screenshot")
                        saved.add(packet["status"])
                    if not args.headless:
                        cv2.imshow(WINDOW, canvas)
                        key = cv2.waitKey(1) & 255
                        if key in (27, ord("q"), ord("Q")):
                            break
                        if key in (ord("g"), ord("G")):
                            view = not view
                        if cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1:
                            break
                if args.max_frames and sum(counts.values()) >= args.max_frames:
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if frames is not None:
            frames.close()
        if not args.headless:
            cv2.destroyAllWindows()
        summary = dict(input_kind=input_kind, mount_qualification=locator.mount_qualification,
                       status_counts=dict(counts), valid_frames=counts["ok"],
                       counts_by_class=stage, milliseconds_median={
                           k: float(np.median(v)) if v else None for k, v in durations.items()})
        (out / "summary.json").write_text(json.dumps(summary, indent=2, allow_nan=False) + "\n")
        print(json.dumps(summary, indent=2), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--check", action="store_true")
    modes.add_argument("--offline", choices=("fixed-val", "raw-val"))
    modes.add_argument("--live", help=argparse.SUPPRESS)
    mount_modes = parser.add_mutually_exclusive_group()
    mount_modes.add_argument("--mount", type=Path, default=CURRENT_MOUNT)
    mount_modes.add_argument("--approximate-mount", action="store_true",
                             help="explicit trial with the 2026-11-16 installation; unverified for current mount")
    parser.add_argument("--field-model-record", type=Path,
                        help="explicit candidate JSON containing absolute path and SHA256")
    parser.add_argument("--out", type=Path)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--max-frames", type=int, default=0)
    args = parser.parse_args()
    cv2.setNumThreads(2)
    locator = Locator(args.mount, approximate=args.approximate_mount)
    if args.field_model_record:
        record = json.loads(args.field_model_record.read_text())
        candidate = Path(record["path"])
        digest = record["sha256"]
        if not candidate.is_absolute() or not candidate.is_file() or sha(candidate) != digest:
            raise ValueError("candidate path or SHA256 mismatch")
        WEIGHTS["field"] = (candidate, digest, WEIGHTS["field"][2])
    detectors = Detectors()
    if args.check:
        print(json.dumps(dict(**locator.provenance, detectors=detectors.metadata),
                         indent=2, allow_nan=False))
        return
    detectors.warmup(locator.shape)
    source = Bridge(Path(args.live.removeprefix("bridge:"))) if args.live else None
    run(args, locator, detectors, source)


if __name__ == "__main__":
    main()
