"""Image intersections, independent carrier projection, and automatic map assignment."""
import math
from pathlib import Path
import sys
import time

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
SITE = HERE.parent
ROOT = SITE.parent
FORMAL = SITE / "formal_map"
CURRENT_MOUNT = HERE / "current_installation.yaml"
LEGACY = ROOT / "task2_lab_night_v1_20261116"
LEGACY_FIT = LEGACY / "config/camera_extrinsics.yaml"
LEGACY_SHA256 = "41a131faade8a5f6d6bf59ab38629a8aeccca2b65ca05410f3a0ec537d9ba55f"
sys.path[:0] = [str(SITE / "integrated_demo"), str(ROOT / "task2_manual_pnp_v1"),
                str(LEGACY / "src")]

from geometry import invalid, match, pixel_ground, read_map, rot, transform
from prototype import settings, sha
from refine_v3 import refine_v3


def read_yaml(path):
    data = yaml.safe_load(path.read_text())
    if not isinstance(data, dict):
        raise ValueError(f"expected YAML mapping: {path}")
    return data


def validate_fit(fit, *, allow_historical=False):
    if (fit.get("schema") != "cquick_extrinsics_v1" or fit.get("verified") is not True
            or fit.get("carrier_axes") != "x_forward_y_left_z_up"
            or fit.get("camera_axes") != "x_right_y_down_z_forward"):
        raise ValueError("unverified fit or unexpected axes")
    legacy = read_yaml(LEGACY_FIT)
    R, t = transform(fit["R_carrier_camera"], fit["t_carrier_camera"])
    if (not allow_historical and np.allclose(R, legacy["R_carrier_camera"], rtol=0, atol=1e-8)
            and np.allclose(t, legacy["t_carrier_camera"], rtol=0, atol=1e-8)):
        raise ValueError("historical installation transform")
    points = fit["point_records"]
    roles = [row["role"] for row in points]
    xy = np.array([[float(row["x_forward_m"]), float(row["y_left_m"])] for row in points])
    uv = np.asarray(fit["image_pixels"], float)
    errors = np.asarray(fit["check_errors_m"], float)
    rms, height = float(fit["fit_rms_pixels"]), float(fit["measured_height_m"])
    fit_xy = xy[np.asarray(roles) == "fit"]
    check_xy = xy[np.asarray(roles) == "check"]
    if (set(roles) != {"fit", "check"} or len(fit_xy) < 6 or len(check_xy) < 2
            or uv.shape != (len(xy), 2) or not np.isfinite(xy).all()
            or not np.isfinite(uv).all() or np.any(uv < 0) or np.any(uv >= [1280, 720])
            or len({tuple(point) for point in xy}) != len(xy)
            or len({tuple(point) for point in uv}) != len(uv)
            or np.linalg.svd(fit_xy - fit_xy.mean(0), compute_uv=False)[-1] < .2
            or errors.shape != (len(check_xy),) or not np.isfinite(errors).all()
            or np.any(errors < 0) or errors.max() >= .10 or not 0 <= rms < 3
            or not math.isfinite(height) or height <= 0 or abs(t[2] - height) > .1):
        raise ValueError("installation fit/check evidence does not meet the original limits")
    return R, t


def load_mount(path, intrinsics_hash, image_size):
    path = path.resolve()
    if not path.exists():
        return None, dict(path=str(path), status="extrinsics_current_installation_missing")
    details = dict(path=str(path), sha256=sha(path))
    try:
        record = read_yaml(path)
        if (record["schema"] != "cquick_current_mount_v1"
                or record["intrinsics_sha256"] != intrinsics_hash
                or record["image_size_px"] != image_size or not str(record["installation_id"]).strip()):
            raise ValueError("installation binding, intrinsics SHA256 or resolution mismatch")
        fit_path = (path.parent / record["fit_path"]).resolve(strict=True)
        reference = (path.parent / record["reference_path"]).resolve(strict=True)
        details.update(fit_path=str(fit_path), fit_sha256=sha(fit_path),
                       reference_path=str(reference), reference_sha256=sha(reference))
        if (details["fit_sha256"] != record["fit_sha256"]
                or details["reference_sha256"] != record["reference_sha256"]):
            raise ValueError("installation evidence SHA256 mismatch")
        fit = read_yaml(fit_path)
        if Path(fit["reference_image"]).resolve() != reference:
            raise ValueError("reference does not match the calibration fit")
        if fit.get("intrinsics_sha256", intrinsics_hash) != intrinsics_hash:
            raise ValueError("fit references a different intrinsic calibration")
        R, t = validate_fit(fit)
        if not np.asarray(record["image_size_px"]).tolist() == [1280, 720]:
            raise ValueError("unexpected resolution")
        details.update(status="ok", installation_id=record["installation_id"],
                       R_carrier_camera=R.tolist(), t_carrier_camera=t.tolist(),
                       fit_rms_px=fit["fit_rms_pixels"], check_errors_m=fit["check_errors_m"])
        return (R, t), details
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        details.update(status="extrinsics_current_installation_invalid", reason=str(exc))
        return None, details


class Locator:
    def __init__(self, mount_path=CURRENT_MOUNT, *, approximate=False):
        session, polylines, provenance = settings(FORMAL)
        self.K, self.dist, self.shape = session.intr, session.dist, session.image_shape
        field = read_yaml(FORMAL / "field.yaml")
        rows = {str(row["id"]): row for row in field["landmarks"]}
        self.markers = [
            m for m in read_map(FORMAL / "field.yaml")
            if m["class_id"] in (0, 1, 2)
            and all(rows[m["id"]].get(key, True) is True
                    for key in ("verified", "visible", "detectable", "matchable"))
        ]
        boundary = {line["id"]: np.asarray(line["points"], dtype=float) for line in polylines}
        x0 = boundary["midline"][:, 0]
        x1 = boundary["endline"][:, 0]
        y0 = boundary["right_sideline"][:, 1]
        y1 = boundary["left_sideline"][:, 1]
        if (field["scope"] != "measured_positive_x_half"
                or field.get("unmeasured_half_included") is not False
                or any(v.shape != (2,) or not np.isclose(v[0], v[1]) for v in (x0, x1, y0, y1))):
            raise ValueError("field does not describe the measured rectangular half")
        self.camera_bounds = [[float(x0[0]), float(y0[0])], [float(x1[0]), float(y1[0])]]
        self.map = dict(markers=self.markers, axes=provenance["axes"],
                        display_geometry=dict(polylines=polylines, scope=field["scope"]))
        if approximate:
            fit = read_yaml(LEGACY_FIT)
            if (sha(LEGACY_FIT) != LEGACY_SHA256
                    or fit["intrinsics_sha256"] != provenance["intrinsics_sha256"]
                    or provenance["image_size_px"] != [1280, 720]):
                raise ValueError("historical fit fingerprint, intrinsics or image size mismatch")
            self.mount = validate_fit(fit, allow_historical=True)
            R, t = self.mount
            self.mount_details = dict(status="approximate_mount_20261116", path=str(LEGACY_FIT),
                                      sha256=LEGACY_SHA256, verified_for_current_installation=False,
                                      original_scope=fit["validation_scope"],
                                      fit_rms_px=fit["fit_rms_pixels"], check_errors_m=fit["check_errors_m"],
                                      R_carrier_camera=R.tolist(), t_carrier_camera=t.tolist())
        else:
            self.mount, self.mount_details = load_mount(mount_path, provenance["intrinsics_sha256"],
                                                        provenance["image_size_px"])
        self.mount_qualification = self.mount_details["status"]
        self.provenance = dict(provenance, mode="automatic_yolo_ground_map",
                               camera_matrix=self.K.tolist(), distortion_coefficients=self.dist.tolist(),
                               installation=self.mount_details,
                               fit_limits=dict(association_gate_m=.22, max_rms_m=.15,
                                               ambiguity_margin_m=.04, minimum_points=3),
                               camera_bounds_map_m=self.camera_bounds,
                               axes_carrier="x_forward_y_left_z_up",
                               ground_plane="carrier_z=0_m", yaw_unit="deg_ccw_from_map_positive_x")

    def locate(self, image, detections):
        timings = dict(refinement_ms=0., projection_ms=0., matching_ms=0.)
        if image.shape[:2] != self.shape:
            return invalid("resolution_mismatch"), [], [], timings
        points = []
        start = time.perf_counter()
        for i, det in enumerate(detections):
            cls = det["class_id"]
            uv = refine_v3(image, det["bbox"], cls) if cls in (0, 1, 2) else None
            finite = uv is not None and bool(np.isfinite(uv).all())
            points.append(dict(detection_index=i, class_id=cls,
                               pixel_uv=[float(v) for v in uv] if finite else None,
                               carrier_xy_m=None, map_xy_m=None, map_id=None,
                               status="refined" if finite else "refinement_rejected"))
        timings["refinement_ms"] = (time.perf_counter() - start) * 1000
        if self.mount is None:
            return invalid(self.mount_details["status"]), points, [], timings
        R, t = self.mount
        start = time.perf_counter()
        observations = []
        order = sorted((p for p in points if p["pixel_uv"] is not None),
                       key=lambda p: -detections[p["detection_index"]]["confidence"])
        ground = pixel_ground([p["pixel_uv"] for p in order], self.K, self.dist, R, t) if order else []
        for p, xy in zip(order, ground):
            if xy is None:
                p["status"] = "ground_ray_rejected"
                continue
            p["carrier_xy_m"] = xy.tolist()
            if any(np.linalg.norm(np.asarray(o["xy"]) - xy) < .08 for o in observations):
                p["status"] = "duplicate_ground_point"
                continue
            p["status"] = "projected_unmatched"
            observations.append(dict(xy=xy.tolist(), class_id=p["class_id"], point=p))
        timings["projection_ms"] = (time.perf_counter() - start) * 1000
        start = time.perf_counter()
        pose = match(observations, self.markers, R, t, camera_bounds=self.camera_bounds)
        timings["matching_ms"] = (time.perf_counter() - start) * 1000
        if pose["valid"]:
            rotation = rot(math.radians(pose["yaw_carrier_map"]))
            offset = np.array([pose["x_carrier_map"], pose["y_carrier_map"]])
            for observation in observations:
                p = observation["point"]
                p["map_xy_m"] = (rotation @ p["carrier_xy_m"] + offset).tolist()
            for pair in pose["matches"]:
                p = observations[pair["observation"]]["point"]
                p.update(map_id=pair["map_id"], status="matched")
        return pose, points, [o["point"]["detection_index"] for o in observations], timings

