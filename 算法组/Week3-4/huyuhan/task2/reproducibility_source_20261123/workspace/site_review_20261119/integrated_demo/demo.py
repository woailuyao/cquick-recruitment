"""Single-reader CQuick demonstration of detection and manual-init map PnP."""
import argparse
from collections import deque
from datetime import datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import torch
import yaml
from ultralytics import YOLO, __version__ as ultralytics_version

HERE = Path(__file__).resolve().parent
SITE = HERE.parent
ROOT = SITE.parent
sys.path.insert(0, str(ROOT / "task2_manual_pnp_v1"))
from display import BLUE, CAMERA_WIDTH, Display, GREEN, HEADER, INK, ORANGE, label
from evidence import AsyncEvidence
from ground_diagnostics import project_used_points
from prototype import LiveSource, run, settings, sha, utc_now

WEIGHTS = {
    "ball": (
        ROOT / "results/task1_ball_yolo/formal_v3_20261114_214759_977028/"
               "ball_yolov8s_20261115_195246_886751/weights/best.pt",
        "732ecee9f41639b6888632e1cd8417569807f570a680423b29547c9752824eae",
        {0: "ball"},
    ),
    "field": (
        SITE / "training/formal_v1/weights/best.pt",
        "2603112118bcaf24b050f3870292dd8f23ba491bd407f7eae83f516ff6c82418",
        {0: "LCross", 1: "TCross", 2: "XCross", 3: "PenaltyPoint"},
    ),
}
OPTIONS = dict(imgsz=640, conf=.25, iou=.7, device="0", verbose=False)
MODEL_COLORS = {"ball": (20, 225, 255), "field": (250, 156, 226)}


class Detectors:
    def __init__(self):
        if not torch.cuda.is_available():
            raise RuntimeError("The existing CUDA device 0 is required by this entry.")
        self.models = {}
        self.metadata = {}
        for role, (path, expected, names) in WEIGHTS.items():
            if not path.is_file() or sha(path) != expected:
                raise ValueError(f"{role}: missing or SHA256 mismatch: {path}")
            model = YOLO(str(path))
            if model.names != names:
                raise ValueError(f"{role}: unexpected classes: {model.names}")
            classes = [0] if role == "ball" else [0, 1, 2]
            self.models[role] = (model, classes)
            self.metadata[role] = dict(path=str(path), sha256=expected, names=names,
                                       classes_shown=classes, **OPTIONS)

    def warmup(self, shape):
        blank = np.zeros((*shape, 3), dtype=np.uint8)
        for _ in range(2):
            self.infer(blank)

    def infer(self, image):
        detections, durations = {}, {}
        for role, (model, classes) in self.models.items():
            start = time.perf_counter()
            boxes = model.predict(image, classes=classes, **OPTIONS)[0].boxes
            found = []
            if boxes is not None:
                for box, cls, score in zip(boxes.xyxy.cpu().tolist(), boxes.cls.cpu().tolist(),
                                           boxes.conf.cpu().tolist()):
                    cls = int(cls)
                    if cls not in classes:
                        continue
                    item = dict(bbox=box, class_id=cls, class_name=model.names[cls], confidence=score)
                    if role == "ball":
                        x1, y1, x2, y2 = box
                        item.update(center_u=(x1 + x2) / 2, center_v=(y1 + y2) / 2)
                    found.append(item)
            detections[role] = found
            durations[role] = (time.perf_counter() - start) * 1000
        return detections, durations


def residual_info(report):
    diagnostics = report["reprojection"]
    rms = diagnostics.get("rms_px")
    kind = "accepted" if report["valid"] else "rejected_candidate" if rms is not None else "unavailable"
    return rms, kind


class DemoDisplay(Display):
    def __init__(self, session, polylines, detectors):
        super().__init__(session, polylines)
        self.detectors = detectors
        self.detection_frame = None
        self.detections = {}
        self.inference_ms = {}
        self.base_scale, self.base_center = self.map_scale, self.map_center
        self.pose_times = deque(maxlen=30)
        self.last_pose_frame = None
        self.geometry_view = False
        self.extrinsics_pnp = None
        self.geometry_rows = []

    def on_key(self, key):
        if key in (ord("g"), ord("G")):
            self.geometry_view = not self.geometry_view
            return True
        return False

    def observe(self, frame):
        self.inference_ms = {}
        if frame is None:
            self.detection_frame = None
            self.detections = {}
        elif frame is not self.detection_frame:
            self.detections, self.inference_ms = self.detectors.infer(frame.image)
            self.detection_frame = frame

    def camera_image(self, frame):
        image = super().camera_image(frame)
        if frame is not self.detection_frame:
            return image
        for role, boxes in self.detections.items():
            color = MODEL_COLORS[role]
            for item in boxes:
                x1, y1, x2, y2 = item["bbox"]
                p = (round(x1 * self.sx), round(y1 * self.sy))
                q = (round(x2 * self.sx), round(y2 * self.sy))
                cv2.rectangle(image, p, q, color, 2, cv2.LINE_AA)
                text = f"{item['class_name']} {item['confidence']:.2f}"
                if role == "ball":
                    u, v = item["center_u"], item["center_v"]
                    cv2.circle(image, (round(u * self.sx), round(v * self.sy)),
                               4, color, -1, cv2.LINE_AA)
                    text += f" ({u:.0f},{v:.0f})"
                (width, _), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, .45, 1)
                x = min(max(2, p[0]), CAMERA_WIDTH - width - 6)
                y = max(15, p[1] - 4)
                cv2.rectangle(image, (x - 2, y - 13), (x + width + 3, y + 3), (22, 22, 22), -1)
                label(image, text, (x, y), color, .45)
        return image

    @property
    def pose_rate(self):
        times = self.pose_times
        return (len(times) - 1) / (times[-1] - times[0]) if len(times) > 1 else 0.

    def draw_geometry(self, canvas):
        canvas[:HEADER + self.height, self.list_left:] = 26
        label(canvas, "PnP-DERIVED ONLY", (self.list_left + 6, 21), BLUE, .45)
        label(canvas, "pixel -> local ground (m)", (self.list_left + 6, 40), INK, .40)
        if not self.geometry_rows:
            label(canvas, "No current valid pose", (self.list_left + 6, HEADER + 21), ORANGE, .43)
        for i, row in enumerate(self.geometry_rows):
            y = HEADER + 14 + i * 29
            u, v = row["pixel_uv"]
            ground = row["ground_local_pnp_xy_m"]
            text = "N/A" if ground is None else f"{ground[0]:.2f},{ground[1]:+.2f} m"
            label(canvas, f"{self.ids.index(row['map_id'])+1:02d} {row['map_id']}",
                  (self.list_left + 6, y), GREEN, .37)
            label(canvas, f"({u:.0f},{v:.0f}) -> {text}", (self.list_left + 6, y + 13), INK, .37)

    def render(self, fps, offline=False):
        s = self.session
        self.extrinsics_pnp, self.geometry_rows = project_used_points(s)
        self.map_scale, self.map_center = self.base_scale, self.base_center
        if s.pose is None:
            self.pose_times.clear()
            self.last_pose_frame = None
        else:
            if self.last_pose_frame is not s.reference:
                self.pose_times.append(time.perf_counter())
                self.last_pose_frame = s.reference
            xy = s.pose["center"][:2]
            direction = s.pose["direction"][:2]
            tip = xy + .46 * direction / np.linalg.norm(direction)
            if not all(CAMERA_WIDTH + 7 < x < self.list_left - 3 and
                       HEADER + 4 < y < HEADER + self.height - 4
                       for x, y in map(self.map_point, (xy, tip))):
                world = np.asarray([m["xyz"][:2] for m in s.markers.values()] + [tuple(xy), tuple(tip)])
                lo, hi = world.min(axis=0) - .55, world.max(axis=0) + .55
                self.map_center = (lo + hi) / 2
                self.map_scale = min(202 / (hi[0] - lo[0]), (self.height - 32) / (hi[1] - lo[1]))
        base = super().render(fps, offline)
        lower = HEADER + self.height
        canvas = cv2.copyMakeBorder(base, 0, 52, 0, 0, cv2.BORDER_CONSTANT, value=(26, 26, 26))
        canvas[:HEADER, CAMERA_WIDTH:self.list_left] = 26
        label(canvas, "field.yaml | FORMAL CENTER", (CAMERA_WIDTH + 10, 20), BLUE)
        label(canvas, "O=(0,0) | +X goal | +Y left", (CAMERA_WIDTH + 10, 40))
        origin = self.map_point((0, 0))
        for name, xy in (("+X", (.45, 0)), ("+Y", (0, .45))):
            end = self.map_point(xy)
            cv2.arrowedLine(canvas, origin, end, BLUE, 1, cv2.LINE_AA, tipLength=.24)
            label(canvas, name, (end[0] + 3, end[1] - 4), BLUE, .37)
        if self.geometry_view and s.frozen is None:
            self.draw_geometry(canvas)
        active = s.frozen or s.latest
        resolution = "1280x720" if active is None else f"{active.image.shape[1]}x{active.image.shape[0]}"
        canvas[lower:, :CAMERA_WIDTH] = 26
        label(canvas, f"{resolution} | {fps:.1f} FPS | YOLO DISPLAY ONLY - never used by PnP",
              (12, lower + 17), INK, .49)
        label(canvas, "Space: freeze   Enter: track   Backspace: undo   R: reset   G: ground view   Q/Esc: exit",
              (12, lower + 38), INK, .45)
        points_note = "Crosses: clicked L/T/X points (pending)" if s.frozen else "Crosses: PnP inliers + map IDs"
        label(canvas, f"Gold: ball / pixel center   Pink: L/T/X boxes   {points_note}",
              (12, lower + 59), INK, .45)

        if s.pose is None and s.frozen is None:
            label(canvas, "Press Space, choose map IDs, and initialize manually",
                  (12, lower + 80), ORANGE, .43)
        canvas[lower:, CAMERA_WIDTH:] = 26
        report = s.report()
        valid = report["valid"]
        x, y, yaw = (f"{report['camera_x_m']:.3f} m", f"{report['camera_y_m']:.3f} m",
                     f"{report['camera_yaw_deg']:.1f} deg") if valid else ("--", "--", "--")
        rms, kind = residual_info(report)
        residual = "N/A (px RMS)" if rms is None else f"{rms:.2f} px RMS"
        if kind == "rejected_candidate":
            residual += " rejected"
        label(canvas, "Yaw CCW from +X | local ground: forward/left",
              (CAMERA_WIDTH + 8, lower + 87), INK, .40)
        for i, text in enumerate((
            f"x_camera_map={x}   y_camera_map={y}",
            f"yaw_camera_map={yaw}   marker_count={report['marker_count']}",
            f"residual={residual}   FPS={fps:.1f}   pose updates={self.pose_rate:.1f} Hz",
            f"valid={str(valid).lower()}   status={report['status']}",
        )):
            label(canvas, text, (CAMERA_WIDTH + 8, lower + 17 + i * 20),
                  GREEN if valid else ORANGE, .45)
        return canvas

    def record(self, log, s, frame, fps, transport_status, elapsed_ms, scene):
        report = s.report()
        active = s.frozen or s.latest
        same = active is not None and active is self.detection_frame
        rms, kind = residual_info(report)
        report.update(
            time_utc=utc_now(), scene=scene,
            frame=frame.number if frame is not None else None,
            source_session=frame.session if frame is not None else None,
            producer=frame.producer if frame is not None else None,
            capture_monotonic_s=frame.capture_s if frame is not None else None,
            display_frame=active.number if active is not None else None,
            display_source_session=active.session if active is not None else None,
            display_frozen=s.frozen is not None,
            detection_frame=self.detection_frame.number if same else None,
            detection_source_session=self.detection_frame.session if same else None,
            detections=self.detections if same else {"ball": [], "field": []},
            detector_ms=self.inference_ms if frame is not None else {},
            x_camera_map=report["camera_x_m"], y_camera_map=report["camera_y_m"],
            yaw_camera_map=report["camera_yaw_deg"],
            residual=rms, residual_kind=kind,
            rejected_candidate_inlier_count=len(report["reprojection"].get("inlier_ids", []))
            if kind == "rejected_candidate" else None,
            feature_point_origin=("manual_click" if s.frozen is not None else
                                  "optical_flow_from_manually_selected_map_id" if report["valid"] else None),
            pnp_extrinsics=self.extrinsics_pnp if report["valid"] else None,
            ground_diagnostics=self.geometry_rows if report["valid"] else [],
            residual_unit="px_rms", fps=fps, pose_update_hz=self.pose_rate if report["valid"] else 0,
            processing_ms=elapsed_ms, transport_status=transport_status,
        )
        log.write(json.dumps(report, allow_nan=False) + "\n")
        log.flush()


def setup():
    session, lines, metadata = settings(SITE / "formal_map")
    if len(session.markers) != 14 or any(m["type"] == "PenaltyPoint" for m in session.markers.values()):
        raise ValueError("Expected the measured 14 L/T/X map correspondences")
    calibration = yaml.safe_load(Path(metadata["intrinsics_path"]).read_text())
    map_extent = np.asarray([point for line in lines for point in line["points"]])
    detectors = Detectors()
    metadata.update(intrinsics_calibration_views=calibration["views"],
                    intrinsics_calibration_rms_px=calibration["rms_pixels"],
                    fixed_installation_extrinsics="not loaded; current setup lacks independent verification",
                    ground_plane_map="Z=0 m, field.yaml measured landmarks on plane",
                    field_display_extent_xy_m=[map_extent.min(axis=0).tolist(), map_extent.max(axis=0).tolist()],
                    ground_projection="available after valid pose; derived from PnP, not an independent measurement",
                    ground_local_axes="origin current estimated optical-center plumb projection, +X forward, +Y left",
                    association="manual map ID then optical flow; no automated map ID search",
                    pose_ambiguity="PnP alternatives for fixed correspondences only",
                    camera_matrix=session.intr.tolist(), distortion_coefficients=session.dist.tolist(),
                    camera_reference="camera_optical_center", yaw_unit="deg_ccw_from_map_positive_x",
                    coordinate_unit="m", residual_unit="px_rms", detector_roles="visualization_only",
                    detectors=detectors.metadata, gpu=torch.cuda.get_device_name(0),
                    torch_version=torch.__version__, ultralytics_version=ultralytics_version,
                    calibration_route="planar map PnP via manual correspondences plus optical flow")
    return session, DemoDisplay(session, lines, detectors), metadata


def save_run(out, metadata):
    out.mkdir(parents=True, exist_ok=False)
    (out / "run.json").write_text(json.dumps(metadata, indent=2, ensure_ascii=False) + "\n",
                                 encoding="utf-8")
    print(f"Logs: {out}", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--check", action="store_true", help="check model/map/bridge installation paths only")
    mode.add_argument("--source", help="live input, must be bridge:/absolute/path/to/shared")
    parser.add_argument("--scene", default="preview", choices=("preview", "P1", "P2", "P3", "occlusion"))
    args = parser.parse_args()
    if args.source:
        value = args.source.removeprefix("bridge:")
        if value == args.source or not Path(value).is_absolute() or Path(value).name != "shared":
            parser.error("--source must be bridge:/absolute/path/to/shared")
        folder = Path(value).resolve()
        if not folder.is_dir():
            parser.error(f"bridge directory missing: {folder}")
    else:
        folder = Path(os.environ.get("CQUICK_BRIDGE_ROOT", "/mnt/c/Users/13991/CquickCameraBridge")) / "shared"
        folder = folder.resolve()
    transport = folder.parent / "bridge_transport.py"
    if not transport.is_file():
        parser.error(f"installed Windows v2 module missing: {transport}")
    session, display, metadata = setup()
    metadata.update(bridge_folder=str(folder), bridge_transport_path=str(transport),
                    input_kind="windows_bridge_v2" if args.source else "preflight_only")
    print("胡钰涵 huyuhan", datetime.now().astimezone().isoformat(), flush=True)
    print(json.dumps(metadata, ensure_ascii=False, indent=2), flush=True)
    if args.check:
        return
    args.offline = False
    display.detectors.warmup(session.image_shape)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out = HERE / "runs" / f"{args.scene}_{stamp}"
    save_run(out, dict(metadata, started_utc=utc_now(), scene=args.scene))
    lock_id = hashlib.sha256(os.fsencode(folder)).hexdigest()[:16]
    lockdir = ROOT / "task2_manual_pnp_v1/runs"
    lockdir.mkdir(exist_ok=True)
    source = LiveSource(folder, lockdir / f".bridge_{lock_id}.lock")
    try:
        session.evidence = AsyncEvidence(out, metadata, dict(zip(session.markers, range(1, 15))))
        try:
            with (out / "frames.jsonl").open("x", encoding="utf-8") as log:
                run(source, session, display, log, args, observe=display.observe,
                    record=display.record, on_key=display.on_key)
        finally:
            session.evidence.close()
    finally:
        source.close()


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, KeyboardInterrupt) as exc:
        sys.exit(str(exc) or "Stopped")
