"""Offline-only analysis of P1/P2/P3 bridge logs and the frozen PnP code."""
import argparse
from collections import Counter, defaultdict
from datetime import datetime, timezone
from itertools import combinations, groupby
import json
from pathlib import Path
import sys

import cv2
import numpy as np

OUT = Path(__file__).resolve().parent
APP = OUT.parent
sys.path.insert(0, str(APP))
from display import Display, HEADER
from pose import angle_delta, candidate, solve
from prototype import settings, sha

RAW_DISTANCES_CM = {"P1": (223, 85), "P2": (313, 46), "P3": (362, 71)}
IDS = ("small_front_right", "small_end_right", "large_end_right", "penalty_mark_site_20261118")
STRIPE_M = 0.05


def pose_summary(rows):
    xyz = np.asarray([row["camera_center_m"] for row in rows])
    med = np.median(xyz, axis=0)
    xy_deviation = np.linalg.norm(xyz[:, :2] - med[:2], axis=1)
    yaw = [row["camera_yaw_deg"] for row in rows]
    yaw_median = float((np.median([(a - yaw[0] + 180) % 360 - 180 for a in yaw]) + yaw[0] + 180) % 360 - 180)
    return dict(
        samples=len(rows), median_camera_xyz_m=med.tolist(), median_yaw_deg=yaw_median,
        median_rms_px=float(np.median([row["reprojection"]["rms_px"] for row in rows])),
        p95_xy_from_median_m=float(np.percentile(xy_deviation, 95)),
        max_xy_from_median_m=float(xy_deviation.max()),
        first_to_last_xy_m=float(np.linalg.norm(xyz[-1, :2] - xyz[0, :2])),
        height_min_m=float(xyz[:, 2].min()), height_max_m=float(xyz[:, 2].max()),
        p95_yaw_from_median_deg=float(np.percentile([angle_delta(a, yaw_median) for a in yaw], 95)),
        producer_time_span_s=rows[-1]["capture_monotonic_s"] - rows[0]["capture_monotonic_s"],
    )


def raw_ippe_roots(pixels, session):
    world = np.asarray([session.markers[key]["xyz"] for key in pixels])
    image = np.asarray(list(pixels.values()))
    _, rvecs, tvecs, _ = cv2.solvePnPGeneric(world, image, session.intr, session.dist,
                                            flags=cv2.SOLVEPNP_IPPE)
    result = []
    for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs), 1):
        refined = cv2.solvePnPRefineLM(world, image, session.intr, session.dist, rvec.copy(), tvec.copy())
        for label, (r, t) in (("raw", (rvec, tvec)), ("refined", refined)):
            rot = cv2.Rodrigues(r)[0]
            xyz = -rot.T @ t.reshape(3)
            depth = (world @ rot.T + t.reshape(3))[:, 2]
            uv = cv2.projectPoints(world, r, t, session.intr, session.dist)[0].reshape(-1, 2)
            errors = np.linalg.norm(uv - image, axis=1)
            eligible = candidate(world, image, session.intr, session.dist, r, t, np.arange(len(world)))
            result.append(dict(
                root=index, stage=label, camera_xyz_m=xyz.tolist(), min_depth_m=float(depth.min()),
                rms_all_px=float(np.sqrt(np.mean(errors ** 2))), max_error_px=float(errors.max()),
                passes_limits=eligible is not None,
            ))
    return result


def sensitivity(pixels, session, base):
    def check(image_points, markers):
        p, status, _ = solve(image_points, markers, session.intr, session.dist)
        if p is None:
            return dict(status=status)
        delta = p["center"] - base["center"]
        return dict(status=status, xy_change_m=float(np.linalg.norm(delta[:2])), z_change_m=float(delta[2]))

    results = {}
    for key in pixels:
        for shift in (1, 3):
            trials = [check({**pixels, key: np.add(pixels[key], dv)}, session.markers)
                      for dv in ((shift, 0), (-shift, 0), (0, shift), (0, -shift))]
            xs = [r["xy_change_m"] for r in trials if r["status"] == "ok"]
            zs = [abs(r["z_change_m"]) for r in trials if r["status"] == "ok"]
            results[f"{key}_plus_or_minus_{shift}px"] = dict(
                max_xy_change_m=max(xs, default=None), max_abs_z_change_m=max(zs, default=None),
                rejected=len(trials) - len(xs))

    display = Display(session, [])
    midpoint_offset = np.array([0.5 / display.sx - .5, 0.5 / display.sy - .5])
    results["resize_pixel_center_offset"] = check({k: np.add(v, midpoint_offset) for k, v in pixels.items()},
                                                   session.markers)
    tape = IDS[-1]
    for label, delta in (("tape_x_plus_25mm", (.025, 0, 0)), ("tape_x_minus_25mm", (-.025, 0, 0)),
                         ("tape_y_plus_25mm", (0, .025, 0)), ("tape_y_minus_25mm", (0, -.025, 0))):
        markers = dict(session.markers)
        markers[tape] = {**markers[tape], "xyz": np.add(markers[tape]["xyz"], delta)}
        results[label] = check(pixels, markers)
    return results


def read_runs(session, provenance, display):
    report = {}
    for path in sorted((APP / "runs").glob("P[123]_*")):
        log = path / "frames.jsonl"
        if not log.is_file():
            continue
        config = json.loads((path / "run.json").read_text())
        with log.open() as stream:
            rows = [json.loads(line) for line in stream if line.strip()]
        frames = [r for r in rows if r["frame"] is not None]
        valid = [r for r in frames if r["valid"]]
        segments = [list(group) for ok, group in groupby(rows, key=lambda r: r["frame"] is not None and r["valid"]) if ok]
        producers = Counter((r["producer"]["width"], r["producer"]["height"], r["producer"]["backend"],
                             r["producer"]["camera_index"]) for r in frames)
        initializations = []
        for row in valid:
            details = row.get("initialization")
            if details is None:
                continue
            pixels = details["clicked_pixels"]
            raw_display = {k: [u * display.sx, HEADER + v * display.sy] for k, (u, v) in pixels.items()}
            base, status, reprojection = solve(pixels, session.markers, session.intr, session.dist)
            assert base is not None, status
            initializations.append(dict(
                frozen_frame=details["frozen_frame"], accepted_frame=row["frame"],
                clicked_pixels=pixels, recovered_display_click_px=raw_display,
                anchor_residual_px=reprojection["residual_px_by_id"], anchor_rms_px=reprojection["rms_px"],
                inlier_ids=reprojection["inlier_ids"], camera_xyz_m=base["center"].tolist(),
                ippe=raw_ippe_roots(pixels, session),
                sensitivity=sensitivity(pixels, session, base) if set(pixels) == set(IDS) else None,
            ))
        report[path.name] = dict(
            scene=config["scene"], frame_count=len(frames), valid_count=len(valid),
            frame_valid_rate=len(valid) / len(frames) if frames else None,
            event_count=len(rows) - len(frames), status_by_frame=dict(Counter(r["status"] for r in frames)),
            run_json_sha256=sha(path / "run.json"), log_sha256=sha(log),
            config_match={key: config.get(key) == provenance[key] for key in
                          ("map_sha256", "map_version", "intrinsics_sha256", "image_size_px")},
            capture_formats=[dict(width=w, height=h, backend=b, camera=i, frames=n)
                             for (w, h, b, i), n in sorted(producers.items())],
            accepted_marker_sets=[dict(ids=list(ids), frames=n) for ids, n in Counter(
                tuple(sorted(r["points_px"])) for r in valid).items()],
            summary=pose_summary(valid) if valid else None,
            contiguous_valid_segments=[dict(first_frame=s[0]["frame"], last_frame=s[-1]["frame"],
                                             **pose_summary(s)) for s in segments],
            initializations=initializations,
        )
    return report


def references(left_y, runs):
    results = {}
    for scene, (left_cm, radius_cm) in RAW_DISTANCES_CM.items():
        valid = [r for r in runs.values() if r["scene"] == scene and r["summary"] is not None]
        estimate = valid[0]["summary"]["median_camera_xyz_m"][:2] if len(valid) == 1 else None
        radius = radius_cm / 100
        alternatives = []
        for edge, offset in (("center", 0), ("inner", STRIPE_M / 2)):
            y = left_y - offset - left_cm / 100
            x_abs = float(np.sqrt(radius ** 2 - y ** 2))
            for half, sign in (("measured_goal_half", 1), ("opposite_half", -1)):
                xy = [sign * x_abs, y]
                alternatives.append(dict(left_stripe_reference=edge, half=half, reference_xy_m=xy,
                                         comparison_to_valid_median_m=float(np.linalg.norm(
                                             np.subtract(estimate, xy))) if estimate is not None else None))
        results[scene] = dict(left_distance_cm_raw=left_cm, straight_circle_distance_cm_raw=radius_cm,
                              logged_xy_m=estimate, conditional_references=alternatives,
                              formal_reference_xy_m=None, formal_position_error_m=None)
    return results


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", type=Path, required=True)
    args = parser.parse_args()
    session, lines, provenance = settings()
    display = Display(session, lines)
    runs = read_runs(session, provenance, display)
    assert runs, "no P1/P2/P3 log directories found"
    left_y = session.markers["end_left"]["xyz"][1]
    end_x = session.markers["end_left"]["xyz"][0]
    tape_x, tape_y, _ = session.markers[IDS[-1]]["xyz"]
    report = dict(
        generated_utc=datetime.now(timezone.utc).isoformat(), configuration=provenance,
        intrinsics=session.intr.tolist(), distortion=session.dist.tolist(),
        display=dict(width=820, height=display.height, header=HEADER, scale_xy=[display.sx, display.sy]),
        tape=dict(map_xy_m=[tape_x, tape_y], distance_from_endline_center_m=end_x - tape_x,
                  distance_from_endline_inner_edge_m=end_x - STRIPE_M / 2 - tape_x,
                  distance_from_left_sideline_center_m=left_y - tape_y,
                  distance_from_left_sideline_inner_edge_m=left_y - STRIPE_M / 2 - tape_y,
                  raw_tape_measurement_sheet_filled=False),
        runs=runs, reference_diagnostics=references(left_y, runs),
    )
    with args.out.open("x", encoding="utf-8") as stream:
        json.dump(report, stream, ensure_ascii=False, indent=2, allow_nan=False)
        stream.write("\n")
    for name, run in runs.items():
        print(name, run["valid_count"], "/", run["frame_count"], run["summary"])
    print(args.out)


if __name__ == "__main__":
    main()
