"""Initialization and loss handling shared by the live and fixed-image UI."""
from dataclasses import dataclass
import cv2
import numpy as np

from pose import BORDER, MAX_FRAME_GAP_S, MIN_POINTS, inside, solve, track


@dataclass
class Frame:
    image: np.ndarray
    session: str
    number: int
    capture_s: float
    producer: dict

    def gray(self):
        return cv2.cvtColor(self.image, cv2.COLOR_BGR2GRAY)


class Session:
    def __init__(self, markers, intr, dist, image_shape):
        self.markers, self.intr, self.dist = markers, intr, dist
        self.image_shape = image_shape
        self.evidence = None
        self.latest = None
        self.clear("waiting_for_frame")

    def clear(self, status="press_space_to_initialize"):
        self.frozen = self.reference = self.reference_gray = None
        self.points, self.diagnostics, self.flow = {}, {}, {}
        self.undo_stack = []
        self.selected = None
        self.initialization = None
        self.initialization_frame = None
        self.pose = None
        self.status = status

    def lost(self, status, diagnostics=None, flow=None, discard_frame=False):
        self.clear(status)
        self.diagnostics = diagnostics or {}
        self.flow = flow or {}
        if discard_frame:
            self.latest = None

    def freeze(self):
        if self.latest is None:
            return
        self.clear("select_map_id_and_click")
        self.frozen = self.latest
        if self.evidence:
            self.evidence.frozen(self.frozen, self.points, self.status)

    def save_evidence(self, action, points=None, **kwargs):
        if self.evidence:
            self.evidence.record(action, self.points if points is None else points,
                                 self.status, **kwargs)

    def pick(self, key, pixel):
        if self.frozen is None or key not in self.markers:
            return
        if not inside(pixel, self.frozen.image.shape):
            self.status = f"click_at_least_{BORDER}px_inside_image"
            self.save_evidence("pick_rejected")
            return
        if any(other != key and np.linalg.norm(np.subtract(uv, pixel)) < 9
               for other, uv in self.points.items()):
            self.status = "pixel_too_close_to_another_point"
            self.save_evidence("pick_rejected")
            return
        self.undo_stack.append((key, self.points.get(key)))
        self.points[key] = pixel
        self.selected = None
        self.status = "select_map_id_and_click"
        self.save_evidence("pick")

    def undo(self):
        if self.frozen is None or not self.undo_stack:
            return
        key, old = self.undo_stack.pop()
        if old is None:
            self.points.pop(key)
        else:
            self.points[key] = old
        self.selected = key
        self.status = "select_map_id_and_click"
        self.save_evidence("undo")

    def initialize(self, frame):
        if self.frozen is None:
            return
        if len(self.points) < MIN_POINTS:
            self.status = "select_at_least_4_points"
            self.save_evidence("initialize", current=frame)
            return
        clicked = dict(self.points)
        if frame.session != self.frozen.session:
            self.lost("bridge_session_changed")
            self.save_evidence("initialize", clicked, current=frame)
            return
        # First check the manual correspondence on the frozen image. No
        # result from this image is published. A new frame must also pass LK.
        anchor, status, diag = solve(self.points, self.markers, self.intr, self.dist,
                                     collect_candidates=bool(self.evidence))
        if anchor is None:
            self.status, self.diagnostics = status, diag
            self.save_evidence("initialize", clicked, diagnostics=diag, current=frame)
            diag.pop("qualified_candidates", None)
            return
        gray = frame.gray()
        anchor_diag = diag
        anchor_points = {key: self.points[key] for key in diag["inlier_ids"]}
        tracked, flow = track(self.frozen.gray(), gray, anchor_points, max_motion_px=35)
        pose, status, diag = solve(tracked, self.markers, self.intr, self.dist, anchor, 0,
                                   collect_candidates=bool(self.evidence))
        if pose is None:
            self.lost("initialization_" + status, diag, flow)
            self.save_evidence("initialize", clicked, diagnostics=anchor_diag, pose=anchor,
                               current=frame, current_diagnostics=diag, flow=flow)
            diag.pop("qualified_candidates", None)
            return
        self.initialization_frame = frame.number
        self.initialization = {
            "frozen_frame": self.frozen.number,
            "frozen_capture_monotonic_s": self.frozen.capture_s,
            "clicked_pixels": {key: list(map(float, pixel)) for key, pixel in self.points.items()},
            "anchor_reprojection": anchor_diag,
        }
        self.frozen = None
        self.selected = None
        self.set_pose(frame, gray, tracked, pose, diag, flow)
        self.save_evidence("initialize", clicked, diagnostics=anchor_diag, pose=anchor,
                           current=frame, current_diagnostics=diag, current_pose=pose, flow=flow)
        # Keep candidate-only diagnostics in the evidence, not the frame stream.
        anchor_diag.pop("qualified_candidates", None)
        diag.pop("qualified_candidates", None)

    def set_pose(self, frame, gray, points, pose, diag, flow):
        self.reference, self.reference_gray = frame, gray
        self.points = {key: points[key] for key in diag["inlier_ids"]}
        self.pose, self.diagnostics, self.flow = pose, diag, flow
        self.status = "ok"

    def on_frame(self, frame):
        if frame.image.shape[:2] != self.image_shape:
            self.lost("camera_resolution_mismatch", discard_frame=True)
            return
        old = self.latest
        self.latest = frame
        if old is not None and old.session != frame.session:
            self.lost("bridge_session_changed")
            return
        if self.reference is None:
            if self.status == "waiting_for_frame":
                self.status = "press_space_to_initialize"
            return
        dt = frame.capture_s - self.reference.capture_s
        if frame.number <= self.reference.number or not 0 < dt <= MAX_FRAME_GAP_S:
            self.lost("frame_gap_or_order")
            return
        gray = frame.gray()
        tracked, flow = track(self.reference_gray, gray, self.points)
        pose, status, diag = solve(tracked, self.markers, self.intr, self.dist, self.pose, dt)
        if pose is None:
            self.lost(status, diag, flow)
        else:
            self.set_pose(frame, gray, tracked, pose, diag, flow)

    def report(self):
        pose = self.pose
        return {
            "valid": pose is not None,
            "status": self.status,
            "point_count": len(self.points),
            "marker_count": len(self.points) if pose is not None else 0,
            "points_px": {key: list(map(float, pixel)) for key, pixel in self.points.items()},
            "reprojection": self.diagnostics,
            "flow_by_id": self.flow,
            "initialization": self.initialization if self.reference is not None and self.reference.number == self.initialization_frame else None,
            "camera_center_m": pose["center"].tolist() if pose is not None else None,
            "camera_x_m": float(pose["center"][0]) if pose is not None else None,
            "camera_y_m": float(pose["center"][1]) if pose is not None else None,
            "camera_yaw_deg": pose["yaw_deg"] if pose is not None else None,
        }
