#!/usr/bin/env python3
"""Ground projection and typed field-feature localization for task 2."""
import argparse
import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


TYPE_ALIASES = {
    "L": "LCross", "LCross": "LCross",
    "T": "TCross", "TCross": "TCross",
    "X": "XCross", "XCross": "XCross",
    "P": "PenaltyPoint", "PenaltyPoint": "PenaltyPoint",
    "PenalityPoint": "PenaltyPoint",
    "PenaltyAreaCorner": "PenaltyAreaCorner",
    "GoalAreaCorner": "GoalAreaCorner",
    "PenaltyAreaGoalLineJunction": "PenaltyAreaGoalLineJunction",
    "GoalAreaGoalLineJunction": "GoalAreaGoalLineJunction",
}


def load_yaml(path):
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def load_camera(intrinsics_path, extrinsics_path):
    intr = load_yaml(intrinsics_path)
    cm = intr["camera_matrix"]
    k = np.array([[cm["fx"], 0.0, cm["cx"]],
                  [0.0, cm["fy"], cm["cy"]],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dc = intr["distortion_coefficients"]
    dist = np.array([dc[n] for n in ("k1", "k2", "p1", "p2", "k3")],
                    dtype=np.float64)
    ext = load_yaml(extrinsics_path)
    transform = np.asarray(ext["extrin"], dtype=np.float64)
    if transform.shape != (4, 4):
        raise ValueError("extrin must be a 4x4 camera-to-robot matrix")
    return k, dist, transform, ext


def pixel_to_ground(pixel, k, dist, camera_to_robot):
    """Return robot-frame (x forward, y left) or reject a sky/parallel ray."""
    uv = np.asarray(pixel, dtype=np.float64).reshape(1, 1, 2)
    xy = cv2.undistortPoints(uv, k, dist).reshape(2)
    ray_camera = np.array([xy[0], xy[1], 1.0], dtype=np.float64)
    origin = camera_to_robot[:3, 3]
    ray_robot = camera_to_robot[:3, :3] @ ray_camera
    if ray_robot[2] >= -1e-9:
        raise ValueError(f"pixel {pixel} ray does not intersect ground in front of camera")
    scale = -origin[2] / ray_robot[2]
    if scale <= 0:
        raise ValueError(f"pixel {pixel} ground intersection is behind camera")
    p = origin + scale * ray_robot
    return p[:2]


def load_field(path):
    data = load_yaml(path)
    result = {}
    for kind, named_points in data["features"].items():
        canonical = TYPE_ALIASES.get(kind, kind)
        result[canonical] = [
            (name, np.asarray(xy, dtype=np.float64))
            for name, xy in named_points.items()
        ]
    return result


def load_field_bounds(path, margin=0.5):
    data = load_yaml(path)
    dims = data.get("dimensions") or {}
    if "length" not in dims or "width" not in dims:
        return None
    return (float(dims["length"]) / 2 + margin,
            float(dims["width"]) / 2 + margin)


def normalize_observations(raw, k, dist, transform):
    observations = []
    for i, item in enumerate(raw):
        kind = TYPE_ALIASES.get(item["type"])
        if kind is None:
            raise ValueError(f"unknown feature type at index {i}: {item['type']}")
        local = pixel_to_ground((item["u"], item["v"]), k, dist, transform)
        observations.append({"index": i, "type": kind, "pixel": [item["u"], item["v"]],
                             "local": local})
    return observations


def pose_from_two(local_a, local_b, field_a, field_b):
    dl = local_b - local_a
    df = field_b - field_a
    if np.linalg.norm(dl) < 0.08 or np.linalg.norm(df) < 0.08:
        return None
    theta = math.atan2(df[1], df[0]) - math.atan2(dl[1], dl[0])
    c, s = math.cos(theta), math.sin(theta)
    r = np.array([[c, -s], [s, c]])
    t = field_a - r @ local_a
    return t[0], t[1], math.atan2(s, c)


def transform_points(points, pose):
    x, y, theta = pose
    c, s = math.cos(theta), math.sin(theta)
    r = np.array([[c, -s], [s, c]])
    return (r @ np.asarray(points).T).T + np.array([x, y])


def nearest_assignments(observations, field, pose, threshold):
    predicted = transform_points([o["local"] for o in observations], pose)
    matches = []
    used = set()
    candidates = []
    for obs, p in zip(observations, predicted):
        for name, q in field.get(obs["type"], []):
            candidates.append((float(np.linalg.norm(p - q)), obs["index"], name, q, obs))
    for distance, obs_index, name, q, obs in sorted(candidates, key=lambda z: z[0]):
        if distance <= threshold and obs_index not in used and name not in used:
            used.add(obs_index); used.add(name)
            matches.append((obs, name, q, distance))
    return matches


def refine_pose(matches):
    local = np.asarray([m[0]["local"] for m in matches])
    target = np.asarray([m[2] for m in matches])
    a = local - local.mean(axis=0)
    b = target - target.mean(axis=0)
    u, _, vt = np.linalg.svd(a.T @ b)
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    t = target.mean(axis=0) - r @ local.mean(axis=0)
    theta = math.atan2(r[1, 0], r[0, 0])
    return float(t[0]), float(t[1]), theta


def angle_distance(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def localize(observations, field, threshold=0.35, prior=None, max_results=5):
    hypotheses = []
    for oa, ob in itertools.combinations(observations, 2):
        for _, fa in field.get(oa["type"], []):
            for _, fb in field.get(ob["type"], []):
                pose = pose_from_two(oa["local"], ob["local"], fa, fb)
                if pose is None:
                    continue
                matches = nearest_assignments(observations, field, pose, threshold)
                if len(matches) < 2:
                    continue
                pose = refine_pose(matches)
                matches = nearest_assignments(observations, field, pose, threshold)
                if len(matches) >= 2:
                    pose = refine_pose(matches)
                residuals = [m[3] for m in nearest_assignments(observations, field, pose, threshold)]
                rms = math.sqrt(sum(d * d for d in residuals) / len(residuals))
                prior_cost = 0.0
                if prior:
                    prior_cost = math.hypot(pose[0] - prior[0], pose[1] - prior[1]) + \
                                 0.5 * angle_distance(pose[2], prior[2])
                hypotheses.append((len(residuals), rms, prior_cost, pose))
    if prior is None:
        hypotheses.sort(key=lambda h: (-h[0], h[1]))
    else:
        # Symmetric field poses often have numerically indistinguishable RMS.
        # With a temporal/odometry prior, preserve the physically continuous
        # branch before considering tiny floating-point residual differences.
        hypotheses.sort(key=lambda h: (-h[0], h[2], h[1]))
    unique = []
    for h in hypotheses:
        if not any(math.hypot(h[3][0] - u[3][0], h[3][1] - u[3][1]) < 0.15 and
                   angle_distance(h[3][2], u[3][2]) < math.radians(8) for u in unique):
            unique.append(h)
        if len(unique) >= max_results:
            break
    return unique


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--intrinsics", required=True)
    parser.add_argument("--extrinsics", required=True)
    parser.add_argument("--field", required=True)
    parser.add_argument("--features", required=True,
                        help='JSON list: [{"type":"LCross","u":123,"v":456}, ...]')
    parser.add_argument("--threshold", type=float, default=0.35)
    parser.add_argument("--prior", nargs=3, type=float, metavar=("X", "Y", "THETA_DEG"))
    parser.add_argument("--allow-provisional", action="store_true",
                        help="allow development-only extrinsics that failed validation")
    parser.add_argument("--output", help="optional JSON output path")
    args = parser.parse_args()
    k, dist, transform, ext = load_camera(args.intrinsics, args.extrinsics)
    calibration_status = ext.get("calibration_status", "unknown")
    if calibration_status != "accepted" and not args.allow_provisional:
        raise SystemExit(
            f"extrinsics status is {calibration_status!r}; "
            "use --allow-provisional only for development")
    raw = json.loads(Path(args.features).read_text(encoding="utf-8"))
    observations = normalize_observations(raw, k, dist, transform)
    field = load_field(args.field)
    prior = None if args.prior is None else (args.prior[0], args.prior[1], math.radians(args.prior[2]))
    results = localize(observations, field, args.threshold, prior)
    bounds = load_field_bounds(args.field)
    if bounds is not None:
        results = [h for h in results
                   if abs(h[3][0]) <= bounds[0] and abs(h[3][1]) <= bounds[1]]
    if not results:
        raise SystemExit("localization failed: need >=2 correctly typed, geometrically distinct inliers")
    payload = {
        "calibration_status": calibration_status,
        "development_only": calibration_status != "accepted",
        "selected_solution": 0,
        "selection_basis": "prior" if prior is not None else "lowest_residual",
        "ground_points_robot_m": [
            {"index": o["index"], "type": o["type"], "x": float(o["local"][0]),
             "y": float(o["local"][1])} for o in observations
        ],
        "solutions": [
            {"x_m": h[3][0], "y_m": h[3][1], "theta_rad": h[3][2],
             "theta_deg": math.degrees(h[3][2]), "inliers": h[0], "rms_m": h[1],
             "prior_cost": h[2]} for h in results
        ],
        "ambiguous": len(results) > 1 and results[1][0] == results[0][0] and
                     abs(results[1][1] - results[0][1]) < 0.05 and
                     (prior is None or abs(results[1][2] - results[0][2]) < 0.5),
    }
    rendered = json.dumps(payload, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(rendered + "\n", encoding="utf-8")
    print(rendered)


if __name__ == "__main__":
    main()

