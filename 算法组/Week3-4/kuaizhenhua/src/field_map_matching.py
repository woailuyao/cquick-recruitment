"""Metric 2-D field matching. Inputs are calibrated ground points, not pixels.

The strict field reader accepts the simple ``Name: [x, y]`` mapping format
provided for this project. It does not silently convert or recenter coordinates.
No camera calibration or detector is implemented in this module.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from functools import lru_cache
from itertools import combinations
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np


CLASS_NAMES = ("LCross", "TCross", "XCross", "PenaltyPoint")
KEY = re.compile(r"^(LCross|TCross|XCross)([1-9]\d*)$|^(PenalityPoint|PenaltyPoint)([1-9]\d*)?$")
NUMBER = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?"
LINE = re.compile(rf"^([A-Za-z][A-Za-z0-9]*):\s*\[\s*({NUMBER})\s*,\s*({NUMBER})\s*\]\s*$")


@dataclass(frozen=True)
class Landmark:
    id: str
    class_name: str
    x: float
    y: float


def canonical_class(name: str) -> str:
    if name == "PenalityPoint":
        return "PenaltyPoint"
    if name not in CLASS_NAMES:
        raise ValueError(f"Unknown landmark class: {name!r}")
    return name


def parse_field_text(text: str) -> list[Landmark]:
    """Reject duplicate IDs, invalid coordinates and unsupported YAML syntax."""
    landmarks, ids, coordinates = [], set(), set()
    for lineno, raw in enumerate(text.lstrip("\ufeff").splitlines(), 1):
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        match = LINE.fullmatch(line)
        if not match:
            raise ValueError(f"Line {lineno}: expected Name: [finite_x, finite_y]")
        key, sx, sy = match.groups()
        kind = KEY.fullmatch(key)
        if not kind:
            raise ValueError(f"Line {lineno}: unsupported landmark ID {key!r}")
        if key in ids:
            raise ValueError(f"Line {lineno}: duplicate landmark ID {key!r}")
        class_name = canonical_class(kind.group(1) or kind.group(3))
        x, y = float(sx), float(sy)
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f"Line {lineno}: non-finite coordinate")
        if (x, y) in coordinates:
            raise ValueError(f"Line {lineno}: duplicate physical coordinate")
        ids.add(key)
        coordinates.add((x, y))
        landmarks.append(Landmark(key, class_name, x, y))
    if not landmarks:
        raise ValueError("Empty landmark map")
    return landmarks


def load_field_map(path: Path) -> tuple[list[Landmark], dict]:
    data = path.read_bytes()
    markers = parse_field_text(data.decode("utf-8-sig"))
    counts = {c: sum(p.class_name == c for p in markers) for c in CLASS_NAMES}
    return markers, {"sha256": hashlib.sha256(data).hexdigest(),
                     "marker_count": len(markers), "class_counts": counts,
                     "unit": "m", "coordinates_recentered": False,
                     "physical_field_alignment_verified_by_loader": False}


def _fit_se2(source: np.ndarray, target: np.ndarray):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source - a).T @ (target - b))
    r = vt.T @ u.T
    if np.linalg.det(r) < 0:
        vt[-1] *= -1
        r = vt.T @ u.T
    return r, b - r @ a


def _assign(points, map_xy, same_class, gate):
    """Maximize inliers, then minimize squared error; each map ID used once."""
    distances = np.linalg.norm(points[:, None, :] - map_xy[None, :, :], axis=2)
    options = [tuple(int(j) for j in np.flatnonzero(same_class[i] & (distances[i] <= gate)))
               for i in range(len(points))]

    @lru_cache(None)
    def solve(i, used):
        if i == len(points):
            return 0, 0.0, ()
        count, score, tail = solve(i + 1, used)
        best = (count, score, (-1,) + tail)
        for j in options[i]:
            if used & (1 << j):
                continue
            count, score, tail = solve(i + 1, used | (1 << j))
            proposal = (count + 1, score + float(distances[i, j] ** 2), (j,) + tail)
            if (-proposal[0], proposal[1]) < (-best[0], best[1]):
                best = proposal
        return best

    count, sse, assignment = solve(0, 0)
    return count, sse, assignment


def match_ground_points(observed, landmarks, *, min_markers=3, gate_m=0.30,
                        max_residual_m=0.15, min_baseline_m=0.10,
                        ambiguity_residual_gap_m=0.03,
                        distinct_pose_distance_m=0.25, distinct_pose_yaw_deg=10.0):
    """Return carrier->map pose; no validity claim for camera/map calibration.

    Each observed entry contains class_name, x, y in the calibrated carrier
    ground frame (metres). Exact two-point hypotheses are enumerated, gated by
    distance, assigned one-to-one and refined with SE(2). Similar-quality
    geometrically distinct candidates make the result ambiguous, not valid.
    Thresholds are configurable engineering defaults, not validated lab metrics.
    """
    failure = {"valid": False, "status": "insufficient_markers", "marker_count": 0,
               "residual": None, "residual_unit": "m", "x_carrier_map": None,
               "y_carrier_map": None, "yaw_carrier_map_deg": None, "matched_ids": []}
    thresholds = [gate_m, max_residual_m, min_baseline_m, ambiguity_residual_gap_m,
                  distinct_pose_distance_m, distinct_pose_yaw_deg]
    if min_markers < 2 or any(not math.isfinite(t) or t <= 0 for t in thresholds):
        raise ValueError("Invalid matching thresholds")
    if len(observed) < min_markers:
        return failure
    # Bound exhaustive assignment cost. A detector can NMS/select reliable,
    # spatially spread observations before calling, recording its selection.
    if len(observed) > 14 or len(landmarks) > 24:
        raise ValueError("Matcher supports at most 14 observations / 24 map landmarks")
    try:
        classes = [canonical_class(o["class_name"]) for o in observed]
        source = np.array([[float(o["x"]), float(o["y"])] for o in observed])
    except (KeyError, TypeError, ValueError) as e:
        raise ValueError("Each observation needs a known class_name and finite metric x,y") from e
    map_xy = np.array([[m.x, m.y] for m in landmarks], dtype=float)
    if not np.isfinite(source).all() or not np.isfinite(map_xy).all():
        raise ValueError("Non-finite observation or map")
    if len({m.id for m in landmarks}) != len(landmarks):
        raise ValueError("Duplicate map IDs")
    if any(np.linalg.norm(source[i] - source[j]) < 1e-6 for i,j in combinations(range(len(source)),2)):
        raise ValueError("Duplicate observed ground points")
    same = np.array([[c == m.class_name for m in landmarks] for c in classes], dtype=bool)
    candidates = []
    for i, j in combinations(range(len(source)), 2):
        local = source[j] - source[i]
        length = np.linalg.norm(local)
        if length < min_baseline_m:
            continue
        for a in np.flatnonzero(same[i]):
            for b in np.flatnonzero(same[j]):
                if a == b:
                    continue
                world = map_xy[b] - map_xy[a]
                if abs(np.linalg.norm(world) - length) > 2 * gate_m:
                    continue
                theta = math.atan2(world[1], world[0]) - math.atan2(local[1], local[0])
                r = np.array([[math.cos(theta), -math.sin(theta)],
                              [math.sin(theta), math.cos(theta)]])
                t = map_xy[a] - r @ source[i]
                for _ in range(3):
                    count, _, assignment = _assign(source @ r.T + t, map_xy, same, gate_m)
                    if count < min_markers:
                        break
                    used = [k for k,v in enumerate(assignment) if v >= 0]
                    r, t = _fit_se2(source[used], map_xy[[assignment[k] for k in used]])
                if count < min_markers:
                    continue
                count, sse, assignment = _assign(source @ r.T + t, map_xy, same, gate_m)
                if count < min_markers:
                    continue
                residual = math.sqrt(sse / count)
                yaw = math.degrees(math.atan2(r[1,0], r[0,0]))
                record = {"marker_count":count,"residual":residual,"x_carrier_map":float(t[0]),
                          "y_carrier_map":float(t[1]),"yaw_carrier_map_deg":yaw,
                          "matched_ids":[landmarks[k].id for k in assignment if k >= 0],
                          "assignments":[{"observation_index":k,"map_id":landmarks[v].id}
                                         for k,v in enumerate(assignment) if v >= 0]}
                candidates.append(record)
    if not candidates:
        return {**failure, "status":"no_consistent_match"}
    candidates.sort(key=lambda c:(-c["marker_count"],c["residual"]))
    best = candidates[0]
    if best["residual"] > max_residual_m:
        return {**failure,"status":"high_residual","marker_count":best["marker_count"],
                "residual":best["residual"],"best_candidate":best}
    for competitor in candidates[1:]:
        if competitor["marker_count"] != best["marker_count"]:
            break
        if competitor["residual"] > best["residual"] + ambiguity_residual_gap_m:
            break
        distance = math.hypot(competitor["x_carrier_map"] - best["x_carrier_map"],
                              competitor["y_carrier_map"] - best["y_carrier_map"])
        yaw_delta = abs((competitor["yaw_carrier_map_deg"] - best["yaw_carrier_map_deg"] + 180) % 360 - 180)
        if distance >= distinct_pose_distance_m or yaw_delta >= distinct_pose_yaw_deg:
            return {**failure,"status":"ambiguous","marker_count":best["marker_count"],
                    "residual":best["residual"],"best_candidate":best,"competing_candidate":competitor}
    return {**failure,**best,"valid":True,"status":"ok",
            "input_count":len(observed),"hypothesis_count":len(candidates)}


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument("--field",type=Path,required=True)
    p.add_argument("--observations",type=Path)
    p.add_argument("--output",type=Path)
    args=p.parse_args()
    markers,info=load_field_map(args.field)
    result={"map":info}
    if args.observations:
        observations=json.loads(args.observations.read_text(encoding="utf-8"))
        result["pose"]=match_ground_points(observations,markers)
    text=json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False)+"\n"
    print(text,end="")
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True)
        args.output.write_text(text,encoding="utf-8")


if __name__ == "__main__":
    main()
