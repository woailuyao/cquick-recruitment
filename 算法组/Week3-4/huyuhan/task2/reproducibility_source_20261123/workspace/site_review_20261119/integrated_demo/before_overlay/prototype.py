"""Run the prototype with the existing Windows v2 bridge or a fixed image."""
import argparse
from collections import deque
from datetime import datetime, timezone
import fcntl
import hashlib
import json
import os
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import yaml

from display import Display
from evidence import AsyncEvidence
from pose import MAX_CAPTURE_AGE_S, MAX_FRAME_GAP_S
from session import Frame, Session

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
CONFIG = ROOT / "site_20261118_curated_v1/config"
REFERENCE = ROOT / "task2_lab_night_v1_20261116/calibration/ground_20261116_152900_corrected/reference.jpg"
MODE = "manual_initialization_optical_flow_map_pnp"
WINDOW = "CQuick | Manual initialization + automatic tracking"


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def settings(config=CONFIG):
    def read(name):
        return yaml.safe_load((config / name).read_text(encoding="utf-8"))

    adapter, field, calibration = (read(name) for name in
                                   ("field_geometry.yaml", "field.yaml", "camera_intrinsics.yaml"))
    map_hash = sha(config / "field.yaml")
    axes = "x_to_goal_y_left_z_up_yaw_ccw"
    if (adapter.get("verified") is not True or adapter["field_sha256"] != map_hash
            or field["units"] != "m" or field["axes"] != axes):
        raise ValueError("field map verification, hash, or units mismatch")
    if calibration.get("verified") is not True:
        raise ValueError("camera intrinsics are not marked verified")
    intr = np.array(calibration["camera_matrix"], dtype=np.float64)
    dist = np.array(calibration["dist_coeffs"], dtype=np.float64)
    shape = (int(calibration["height"]), int(calibration["width"]))
    if (intr.shape != (3, 3) or intr[0, 0] <= 0 or intr[1, 1] <= 0
            or not np.isfinite(intr).all() or not np.isfinite(dist).all()
            or not np.allclose(intr[2], [0, 0, 1]) or dist.size not in (4, 5, 8, 12, 14)):
        raise ValueError("invalid camera matrix or distortion")
    markers = {}
    for row in field["landmarks"]:
        if row.get("verified", True) is not True:
            continue
        xyz = (float(row["x"]), float(row["y"]), 0.0)
        if row["id"] in markers or not np.isfinite(xyz).all():
            raise ValueError("duplicate map ID or non-finite coordinate")
        markers[row["id"]] = dict(xyz=xyz, type=row["type"])
    if len(markers) < 4:
        raise ValueError("fewer than four usable measured map points")
    provenance = dict(mode=MODE, axes=axes, map_path=str(config / "field.yaml"),
                      map_sha256=map_hash, map_version=adapter["version"],
                      intrinsics_path=str(config / "camera_intrinsics.yaml"),
                      intrinsics_sha256=sha(config / "camera_intrinsics.yaml"),
                      image_size_px=list(reversed(shape)), opencv_version=cv2.__version__)
    return Session(markers, intr, dist, shape), field["display_geometry"]["polylines"], provenance


def other_readers(folder):
    own = os.getpid()
    matches = []
    reader_scripts = {"live.py", "record_raw.py", "calibration.py", "ubuntu_live.py",
                      "prototype.py", "diagnostics.py", "record_detection.py", "ball_live_wsl.py"}
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit() or int(entry.name) == own:
            continue
        try:
            args = [os.fsdecode(piece) for piece in (entry / "cmdline").read_bytes().split(b"\0") if piece]
            is_reader = any(Path(arg).name in reader_scripts for arg in args)
            for arg in args:
                value = arg.split("=", 1)[-1] if arg.startswith("-") else arg
                explicit = value.startswith("bridge:")
                if explicit or (is_reader and Path(value).name == "shared"):
                    candidate = Path(value.removeprefix("bridge:"))
                    if not candidate.is_absolute():
                        candidate = (entry / "cwd").resolve() / candidate
                    if candidate.resolve() == folder:
                        matches.append(int(entry.name))
                        break
        except (OSError, RuntimeError, ValueError):
            continue
    return matches


class LiveSource:
    def __init__(self, folder, lock):
        # The lock protects instances of this prototype. Older tools are
        # checked by their bridge-source command line before each read.
        self.lock = lock.open("a")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            self.folder = folder
            self.check_conflicts()
            sys.path.insert(0, str(ROOT / "task2_lab_night_v1_20261116/src"))
            from bridge_camera import BridgeCamera
            self.camera = BridgeCamera(folder, timeout=0.22)
            self.camera.reader.max_age = MAX_CAPTURE_AGE_S
        except BaseException:
            self.lock.close()
            raise

    def check_conflicts(self):
        matches = other_readers(self.folder)
        if matches:
            raise RuntimeError(f"bridge_reader_conflict PIDs={matches}")

    def next_frame(self):
        self.check_conflicts()
        ok, image = self.camera.read()
        if not ok:
            return None, self.camera.last_status
        meta = self.camera.last_metadata
        return Frame(image, meta["session"], meta["sequence"],
                     meta["capture_monotonic_ns"] / 1e9, dict(meta)), "ok"

    def fresh(self):
        return self.camera.freshness()

    def close(self):
        self.camera.release()
        self.lock.close()


class FixedSource:
    def __init__(self):
        self.image = cv2.imread(str(REFERENCE))
        if self.image is None:
            raise ValueError(f"cannot read fixed image {REFERENCE}")
        self.number = 0

    def next_frame(self):
        time.sleep(.075)
        self.number += 1
        return Frame(self.image, "offline_fixed_image", self.number, time.perf_counter(), {}), "ok"

    def fresh(self):
        return True, "offline_fixed_image"

    def close(self):
        pass


def utc_now():
    return datetime.now(timezone.utc).isoformat()


def log_frame(log, session, frame, fps, transport_status, elapsed_ms, scene):
    log.write(json.dumps(dict(session.report(), time_utc=utc_now(), scene=scene,
                              frame=frame.number if frame is not None else None,
                              source_session=frame.session if frame is not None else None,
                              producer=frame.producer if frame is not None else None,
                              capture_monotonic_s=frame.capture_s if frame is not None else None,
                              fps=fps, transport_status=transport_status,
                              processing_ms=elapsed_ms), allow_nan=False) + "\n")
    log.flush()


def key_action(key, session):
    if key == 32:
        session.freeze()
    elif key in (ord("r"), ord("R")):
        session.clear()
    elif key in (8, 127):
        session.undo()


def window_visible():
    try:
        return cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) >= 1
    except cv2.error:
        return False


def run(source, session, display, log, args):
    timestamps = deque(maxlen=30)
    arm_initialization = False
    last_frame_time = None
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(WINDOW, display.mouse)
    cv2.imshow(WINDOW, display.render(0, args.offline))
    cv2.waitKey(1)
    try:
        while True:
            if session.evidence:
                session.evidence.check()
            frame, transport_status = source.next_frame()
            start = time.perf_counter()
            if frame is None:
                # A failed read immediately invalidates a published pose and
                # removes frozen correspondences so a gap cannot resume tracking.
                session.lost("bridge_" + transport_status, discard_frame=True)
                arm_initialization = False
                timestamps.clear()
            else:
                stalled = (last_frame_time is not None and start - last_frame_time > MAX_FRAME_GAP_S)
                if stalled and (session.pose is not None or session.frozen is not None):
                    session.lost("receive_gap", discard_frame=True)
                    arm_initialization = False
                    timestamps.clear()
                last_frame_time = start
                session.on_frame(frame)
                if arm_initialization:
                    session.initialize(frame)
                    arm_initialization = False
                fresh, reason = source.fresh()
                if not fresh:
                    transport_status = reason
                    session.lost("bridge_" + reason, discard_frame=True)
                    timestamps.clear()
                    arm_initialization = False
                else:
                    timestamps.append(start)
            fps = ((len(timestamps) - 1) / (timestamps[-1] - timestamps[0])
                   if len(timestamps) > 1 else 0)
            rendered = display.render(fps, args.offline)
            # Rendering can consume time too, so check age at publication.
            if session.pose is not None:
                fresh, reason = source.fresh()
                if not fresh:
                    transport_status = reason
                    session.lost("bridge_" + reason, discard_frame=True)
                    timestamps.clear()
                    fps = 0
                    rendered = display.render(fps, args.offline)
            log_frame(log, session, frame, fps, transport_status,
                      (time.perf_counter() - start) * 1000, args.scene)
            cv2.imshow(WINDOW, rendered)
            key = cv2.waitKey(1) & 255
            if key in (ord("q"), ord("Q"), 27) or not window_visible():
                return
            if key in (10, 13):
                arm_initialization = session.frozen is not None
            else:
                key_action(key, session)
    except RuntimeError as exc:
        session.lost(str(exc), discard_frame=True)
        log_frame(log, session, None, 0, "source_error", 0, args.scene)
        raise
    finally:
        session.lost("stopped", discard_frame=True)
        log_frame(log, session, None, 0, "stopped", 0, args.scene)
        cv2.destroyAllWindows()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument("--offline", action="store_true", help="use the existing fixed photo and no bridge")
    mode.add_argument("--check", action="store_true", help="verify source configuration without opening any capture")
    p.add_argument("--scene", choices=("preview", "P1", "P2", "P3", "occlusion"), default="preview")
    p.add_argument("--runs-root", type=Path, default=HERE / "runs")
    p.add_argument("--bridge-root", type=Path,
                   default=Path(os.environ.get("CQUICK_BRIDGE_ROOT", "/mnt/c/Users/13991/CquickCameraBridge")))
    p.add_argument("--config", type=Path, default=CONFIG)
    args = p.parse_args()
    session, lines, metadata = settings(args.config.resolve())
    folder = (args.bridge_root / "shared").resolve()
    if not args.offline:
        metadata.update(bridge_folder=str(folder), bridge_transport_path=str(folder.parent / "bridge_transport.py"))
        if not (folder.parent / "bridge_transport.py").is_file():
            p.error(f"installed Windows v2 module missing from {folder.parent}")
    print(json.dumps(metadata, indent=2), flush=True)
    if args.check:
        return
    if not args.offline and not folder.is_dir():
        p.error(f"bridge shared directory missing: {folder}. Start the Windows v2 sender first.")
    runs = args.runs_root.resolve()
    runs.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out = runs / f"{'offline' if args.offline else args.scene}_{stamp}"
    out.mkdir()
    metadata.update(started_utc=utc_now(), scene=args.scene, offline=args.offline,
                    reference_image=str(REFERENCE) if args.offline else None)
    (out / "run.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(f"Logs: {out}", flush=True)
    key = hashlib.sha256(os.fsencode(folder)).hexdigest()[:16]
    try:
        lockdir = HERE / "runs"
        lockdir.mkdir(exist_ok=True)
        source = FixedSource() if args.offline else LiveSource(folder, lockdir / f".bridge_{key}.lock")
    except BlockingIOError:
        p.exit(2, "Another manual PnP prototype already owns this bridge lock.\n")
    try:
        ui = Display(session, lines)
        session.evidence = AsyncEvidence(out, metadata, dict(zip(session.markers, range(1, len(session.markers) + 1))))
        try:
            with (out / "frames.jsonl").open("x", encoding="utf-8") as log:
                run(source, session, ui, log, args)
        finally:
            session.evidence.close()
    finally:
        source.close()


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print(str(exc) or "Stopped", file=sys.stderr)
        sys.exit(2)
