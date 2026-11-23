"""Persist diagnostic evidence separately from live pose selection."""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path

import cv2


def stamp(frame):
    return dict(source_session=frame.session, source_frame=frame.number,
                capture_monotonic_s=frame.capture_s, producer=dict(frame.producer),
                captured_unix_ns=frame.producer.get("capture_unix_ns"))


def pose_fields(pose):
    if pose is None:
        return None
    return dict(x_m=float(pose["center"][0]), y_m=float(pose["center"][1]),
                z_m=float(pose["center"][2]), yaw_deg=float(pose["yaw_deg"]),
                rms_px=float(pose["rms_px"]))


class Evidence:
    def __init__(self, out, provenance, ids, directory="initializations"):
        self.root = Path(out) / directory
        self.root.mkdir()
        self.provenance, self.ids = provenance, ids
        self.folder = None
        self.count, self.event = 0, 0
        self.image = None

    def save_image(self, file, image):
        ok, pixels = cv2.imencode(".png", image)
        if not ok:
            raise RuntimeError(f"PNG encode failed: {file}")
        with file.open("xb") as stream:
            stream.write(pixels.tobytes())
        return hashlib.sha256(pixels).hexdigest()

    def frozen(self, frame, points, status):
        self.count += 1
        self.event = 0
        self.folder = self.root / f"freeze_{self.count:04d}_{frame.number}"
        self.folder.mkdir()
        self.image = frame.image.copy()
        self.source = stamp(frame)
        self.raw_sha = self.save_image(self.folder / "raw.png", self.image)
        self.record("freeze", points, status)

    def record(self, action, points, status, *, diagnostics=None, pose=None,
               current=None, current_diagnostics=None, current_pose=None, flow=None):
        if self.folder is None:
            return
        self.event += 1
        name = f"{self.event:04d}_{action}"
        diag = diagnostics or {}
        image = self.image.copy()
        projected = diag.get("projected_pixels_by_id", {})
        px = {key: list(map(float, uv)) for key, uv in points.items()}
        for key, xy in px.items():
            pt = tuple(int(round(x)) for x in xy)
            cv2.drawMarker(image, pt, (0, 255, 255), markerSize=15, thickness=2)
            pred = projected.get(key)
            if pred is not None and all(abs(x) < 100000 for x in pred):
                repro = tuple(int(round(x)) for x in pred)
                cv2.circle(image, repro, 6, (255, 120, 20), 2)
                cv2.line(image, pt, repro, (255, 120, 20), 1)
            residual = diag.get("residual_px_by_id", {}).get(key)
            label = f"{self.ids[key]} {key}" + (f" {residual:.2f}px" if residual is not None else "")
            cv2.putText(image, label, (pt[0] + 8, max(19, pt[1] - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, .42, (0, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, f"{action}: {status}", (12, 27),
                    cv2.FONT_HERSHEY_SIMPLEX, .65, (80, 225, 255), 2, cv2.LINE_AA)
        overlay_sha = self.save_image(self.folder / (name + ".png"), image)
        packet = dict(event=action, time_utc=datetime.now(timezone.utc).isoformat(),
                      status=status, source=self.source, config=self.provenance,
                      raw_image="raw.png", raw_png_sha256=self.raw_sha,
                      overlay_image=name + ".png", overlay_png_sha256=overlay_sha,
                      clicked_pixels=px, map_numbers={key: self.ids[key] for key in px},
                      anchor_pose=pose_fields(pose), anchor_diagnostics=diag,
                      current_source=stamp(current) if current is not None else None,
                      current_pose=pose_fields(current_pose),
                      current_diagnostics=current_diagnostics, flow_by_id=flow,
                      rejection_reason=(None if action != "initialize" or status == "ok" else status),
                      note="Frozen anchor pose is diagnostic and never published as current pose.")
        tmp = self.folder / (name + ".json.partial")
        final = self.folder / (name + ".json")
        with tmp.open("x", encoding="utf-8") as stream:
            json.dump(packet, stream, indent=2, allow_nan=False)
            stream.write("\n")
        tmp.rename(final)


class AsyncEvidence:
    """FIFO writes off the capture path; failures stay visible and stop the run."""
    def __init__(self, out, provenance, ids, directory="initializations"):
        self.writer = Evidence(out, provenance, ids, directory)
        self.worker = ThreadPoolExecutor(max_workers=1)
        self.pending = deque()

    def _send(self, function, *args, **kwargs):
        self.check()
        if len(self.pending) >= 64:
            raise RuntimeError(f"evidence save failed: write backlog at {self.writer.root}")
        self.pending.append(self.worker.submit(function, *args, **kwargs))

    def frozen(self, frame, points, status):
        self._send(self.writer.frozen, frame, deepcopy(points), status)

    def record(self, event, points, status, **kwargs):
        current = kwargs.pop("current", None)
        self._send(self.writer.record, event, deepcopy(points), status,
                   current=current, **deepcopy(kwargs))

    def check(self):
        while self.pending and self.pending[0].done():
            try:
                self.pending.popleft().result()
            except Exception as exc:
                raise RuntimeError(f"evidence save failed at {self.writer.root}: {exc}") from exc

    def close(self):
        self.worker.shutdown(wait=True)
        self.check()
