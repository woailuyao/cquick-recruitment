"""Reproduce saved PnP initialization using both the frozen and current cores."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
DEMO = HERE.parent
SITE = DEMO.parent
ROOT = SITE.parent
RUN = DEMO / "runs/preview_20261119_193523_047021"
ACTIVE = "freeze_0002_21657"
SAVED = ROOT / "CQuick_20261119_light_formal_center/task2_manual_pnp_v1/pose.py"
sys.path.insert(0, str(ROOT / "task2_manual_pnp_v1"))
from display import CAMERA_WIDTH, HEADER, Display
import pose
from prototype import settings
from session import Frame


def load(path):
    return json.loads(path.read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_image(path, image):
    if not cv2.imwrite(str(path), image):
        raise OSError(path)


def inspect_candidate(world, image, intr, dist, rvec, tvec):
    rotation = cv2.Rodrigues(rvec)[0]
    center = -rotation.T @ tvec.reshape(3)
    direction = rotation.T[:, 2]
    depths = (world @ rotation.T + tvec.reshape(3))[:, 2]
    projected = cv2.projectPoints(world, rvec, tvec, intr, dist)[0].reshape(-1, 2)
    residual = np.linalg.norm(projected - image, axis=1)
    inliers = np.flatnonzero((residual <= pose.MAX_POINT_ERROR_PX) & (depths > .01))
    inlier_rms = float(np.sqrt(np.mean(residual[inliers] ** 2))) if len(inliers) else None
    if not np.isfinite(center).all() or center[2] <= 0:
        reason = "camera_center_not_above_ground"
    elif np.linalg.norm(direction[:2]) < .10:
        reason = "horizontal_optical_axis_too_short"
    elif np.any(depths <= .01):
        reason = "one_or_more_fitted_points_behind_camera"
    elif not np.isfinite(residual).all():
        reason = "non_finite_projection"
    elif len(inliers) < pose.MIN_POINTS:
        reason = "fewer_than_four_inliers_at_5_px"
    elif not pose.good_layout(world[inliers], image[inliers]):
        reason = "insufficient_inlier_spread"
    elif inlier_rms > pose.MAX_RMS_PX:
        reason = "inlier_rms_above_3_px"
    else:
        reason = "accepted"
    actual = pose.candidate(world, image, intr, dist, rvec, tvec, np.arange(len(world)))
    assert (actual is not None) == (reason == "accepted")
    return dict(camera_center_map_m=center.tolist(), depths_m=depths.tolist(),
                all_point_rms_px=float(np.sqrt(np.mean(residual ** 2))), inlier_rms_px=inlier_rms,
                inlier_indices=inliers.tolist(), residual_px=residual.tolist(),
                projection_px=projected.tolist(), reason=reason)


def inspect_points(session, clicks):
    world = np.asarray([session.markers[key]["xyz"] for key in clicks], dtype=np.float64)
    image = np.asarray(list(clicks.values()), dtype=np.float64)
    assert len(clicks) == pose.MIN_POINTS == 4
    if not pose.good_layout(world, image):
        return []
    outputs = []
    _, roots_r, roots_t, _ = cv2.solvePnPGeneric(world, image, session.intr, session.dist,
                                                flags=cv2.SOLVEPNP_IPPE)
    for idx, (rvec, tvec) in enumerate(zip(roots_r, roots_t), start=1):
        for stage in ("IPPE", "LM"):
            if stage == "LM":
                rvec, tvec = cv2.solvePnPRefineLM(world, image, session.intr, session.dist, rvec.copy(), tvec.copy())
            candidate = inspect_candidate(world, image, session.intr, session.dist, rvec, tvec)
            outputs.append(dict(root=idx, stage=stage, **candidate))
    return outputs


def annotate(raw, clicks, event, roots, out):
    image = raw.copy()
    best = min(roots, key=lambda row: row["all_point_rms_px"])
    for idx, (key, xy) in enumerate(clicks.items()):
        u, v = map(round, xy)
        cv2.drawMarker(image, (u, v), (0, 255, 255), markerSize=21, thickness=2)
        label = f"{event['map_numbers'][key]} {key} ({xy[0]:.1f},{xy[1]:.1f})"
        cv2.putText(image, label, (min(u+12, 1000), max(24, v-13)),
                    cv2.FONT_HERSHEY_SIMPLEX, .50, (0, 255, 255), 1, cv2.LINE_AA)
    write_image(out / "raw_clicks.png", image)
    for idx, (key, xy) in enumerate(clicks.items()):
        u, v = map(round, xy)
        pt = tuple(map(round, best["projection_px"][idx]))
        cv2.circle(image, pt, 6, (235, 184, 67), 2)
        cv2.line(image, (u, v), pt, (235, 184, 67), 1)
        cv2.putText(image, f"{best['residual_px'][idx]:.1f}px", (min(u+13, 1200), min(v+20, 701)),
                    cv2.FONT_HERSHEY_SIMPLEX, .47, (235, 184, 67), 1)
    write_image(out / "candidate_overlay.png", image)


def main():
    spec = importlib.util.spec_from_file_location("frozen_pose", SAVED)
    old = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(old)
    config = load(RUN / "run.json")
    camera = Path(config["intrinsics_path"])
    assert sha(camera) == config["intrinsics_sha256"]
    assert sha(Path(config["map_path"])) == config["map_sha256"]
    results = {}
    for folder in sorted((RUN / "initializations").glob("freeze_*")):
        if folder.name not in ("freeze_0001_21193", ACTIVE):
            continue
        paths = sorted(folder.glob("*_initialize.json"))
        events = [(event_path, load(event_path)) for event_path in paths]
        assert events
        ordered = {}
        for event_path, event in events:
            ordered.setdefault(json.dumps(event["clicked_pixels"], sort_keys=True), []).append(event_path.name)
        assert len(ordered) == 1
        event = events[0][1]
        clicks = event["clicked_pixels"]
        raw_path = folder / event["raw_image"]
        assert sha(raw_path) == event["raw_png_sha256"]
        raw = cv2.imread(str(raw_path))
        assert raw.shape == (720, 1280, 3)
        assert all(sha(folder/e["overlay_image"]) == e["overlay_png_sha256"] for _,e in events)
        session, lines, _ = settings(Path(config["map_path"]).parent)
        assert np.array_equal(session.intr, np.asarray(config["camera_matrix"]))
        assert np.array_equal(session.dist, np.asarray(config["distortion_coefficients"]))
        frame = Frame(raw, event["source"]["source_session"], event["source"]["source_frame"],
                      event["source"]["capture_monotonic_s"], event["source"]["producer"])
        session.on_frame(frame)
        session.freeze()
        display = Display(session, lines)
        viewport = {}
        for key, xy in clicks.items():
            idx = display.ids.index(key)
            display.mouse(cv2.EVENT_LBUTTONDOWN, display.list_left + 10,
                          HEADER + 10 + idx * display.row_height + 8)
            x, y = display.image_point(xy)
            display.mouse(cv2.EVENT_LBUTTONDOWN, x, y)
            viewport[key] = [x, y]
        assert all(np.allclose(session.points[key], xy) for key,xy in clicks.items())
        previous, previous_status, _ = old.solve(clicks, session.markers, session.intr, session.dist,
                                               collect_candidates=True)
        session.initialize(frame)
        assert session.status == previous_status == event["status"]
        assert previous is None and session.pose is None
        roots = inspect_points(session, clicks)
        target = HERE / folder.name
        target.mkdir(exist_ok=True)
        annotate(raw, clicks, event, roots, target)
        results[folder.name] = dict(
            source_raw=str(raw_path.relative_to(ROOT)), raw_sha256=event["raw_png_sha256"],
            run_event_files=[str(path.relative_to(ROOT)) for path, _ in events],
            old_core_sha256=sha(SAVED), new_core_sha256=sha(Path(pose.__file__)),
            old_status=previous_status, new_status=session.status, point_ids=list(clicks),
            point_numbers=event["map_numbers"], pixels=clicks,
            source_size_wh=list(reversed(raw.shape[:2])), canvas_size_wh=[1280,605],
            image_rect_xywh=[0, HEADER, CAMERA_WIDTH, display.height],
            src_to_display_xy_scale=[display.sx,display.sy], mouse_roundtrips=viewport,
            world_xyz_m={key:session.markers[key]["xyz"] for key in clicks}, candidates=roots)
    assert ACTIVE in results
    (HERE / "candidate_diagnostics.json").write_text(json.dumps(results,indent=2,allow_nan=False)+"\n")
    for key, row in results.items():
        print(key, row["point_ids"],row["mouse_roundtrips"])
        for case in row["candidates"]:
            print(case["root"],case["stage"],round(case["all_point_rms_px"],3),
                  [round(x,3) for x in case["residual_px"]],case["reason"])


if __name__ == "__main__":
    main()
