"""Replay saved evidence through the actual demo loop, with scripted GUI events."""
from collections import Counter
from datetime import datetime
from io import StringIO
import json
import statistics
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

from demo import CAMERA_WIDTH, GREEN, HEADER, HERE, ROOT, run, save_run, setup, sha, utc_now
from session import Frame

ANCHOR = (ROOT / "task2_manual_pnp_v1/runs/P1_20261119_144231_419596/"
          "initializations/freeze_0002_11598")
SECOND = (ROOT / "site_review_20261119/formal_dataset/images/val/validation_0424.png")
THIRD = (ROOT / "site_review_20261119/formal_dataset/images/val/"
         "frozen_old_field_1ef83347889f828e_f0000000.jpg")
LABELS = {2: "frozen_source", 3: "valid_pose", 5: "insufficient_current_rgb",
          6: "no_automatic_recovery", 9: "manual_recovery", 10: "stale_cleared", 11: "tcross_sample", 12: "xcross_sample"}


class OfflineSource:
    """Frames are known recordings except for explicitly injected loss cases."""

    def __init__(self, original, other, cross):
        self.step = 0
        self.checks = 0
        self.frames = [original, other, original, original,
                       np.full_like(original, (36, 57, 75)), original,
                       original, other, original, original, other, cross, None]

    def next_frame(self):
        index = self.step
        self.step += 1
        self.checks = 0
        image = self.frames[index]
        if image is None:
            return None, "offline_injected_input_loss"
        return Frame(image, "offline_fixture", self.step, .1 * self.step,
                     {"kind": "offline_fixture", "input_index": self.step}), "ok"

    def fresh(self):
        self.checks += 1
        if self.step == 10 and self.checks == 2:
            return False, "offline_injected_stale_after_processing"
        return True, "ok"


def checked_png(out, filename, image):
    if not cv2.imwrite(str(out / (filename + ".png")), image):
        raise OSError(f"Failed to save {filename}")


def run_fixture(out, source, session, display, clicks):
    pictures = {}
    camera_arrows = Counter()
    possible_centers = {}
    original_arrow = cv2.arrowedLine

    def arrow(image, start, end, color, *a, **kw):
        if color == GREEN:
            assert start[0] > CAMERA_WIDTH and end[0] > CAMERA_WIDTH
            possible_centers.setdefault(source.step, set()).add(start)
        return original_arrow(image, start, end, color, *a, **kw)

    def show(_window, image):
        camera_arrows[source.step] = sum(
            tuple(image[y, x]) == GREEN for x, y in possible_centers.get(source.step, ())
        )
        if source.step in LABELS:
            pictures[source.step] = image.copy()

    def key(_delay):
        step = source.step
        if step in (1, 7):
            return 32
        if step in (2, 8):
            for landmark, pixel in clicks.items():
                row = display.ids.index(landmark)
                display.mouse(cv2.EVENT_LBUTTONDOWN, display.list_left + 12,
                              HEADER + 10 + row * display.row_height + 8)
                display.mouse(cv2.EVENT_LBUTTONDOWN, *display.image_point(pixel))
            assert set(session.points) == set(clicks)
            return 13
        if step == 4:
            return ord("g")
        if step == len(source.frames):
            return ord("q")
        return -1

    with (out / "frames.jsonl").open("x", encoding="utf-8") as log:
        with patch.multiple(cv2, namedWindow=lambda *_: None, setMouseCallback=lambda *_: None,
                            imshow=show, waitKey=key, getWindowProperty=lambda *_: 1,
                            destroyAllWindows=lambda: None, arrowedLine=arrow):
            run(source, session, display, log, SimpleNamespace(offline=True, scene="offline"),
                observe=display.observe, record=display.record, on_key=display.on_key)
    for step, image in pictures.items():
        checked_png(out, LABELS[step], image)
    assert set(pictures) == set(LABELS)
    return camera_arrows


def verify_rows(out, source, arrows, session):
    rows = [json.loads(line) for line in (out / "frames.jsonl").read_text().splitlines()]
    indexed = {row["frame"]: row for row in rows if row["frame"] is not None}
    assert len(rows) == source.step + 1 == len(source.frames) + 1
    assert list(indexed) == list(range(1, len(source.frames)))
    assert all(indexed[i]["valid"] and indexed[i]["marker_count"] == 4
               and indexed[i]["residual_unit"] == "px_rms" and arrows[i] > 0 for i in (3, 4, 9))
    assert not any(arrows[i] for i in (1, 2, 5, 6, 7, 8, 10, 11, 12, 13))
    assert indexed[4]["pose_update_hz"] > 0
    stale = indexed[10]
    for row in (indexed[5], indexed[6], stale, indexed[11], indexed[12], rows[-2], rows[-1]):
        assert not row["valid"] and row["marker_count"] == 0
        assert all(row[key] is None for key in ("x_camera_map", "y_camera_map", "yaw_camera_map", "residual"))
        assert row["residual_unit"] == "px_rms" and row["residual_kind"] == "unavailable"
        assert row["pnp_extrinsics"] is None and not row["ground_diagnostics"]
    assert indexed[5]["status"] == indexed[6]["status"] == "insufficient_points"
    assert indexed[5]["display_frame"] == 5 and indexed[6]["display_frame"] == 6
    assert stale["status"] == "bridge_offline_injected_stale_after_processing"
    assert stale["display_frame"] is None and not any(stale["detections"].values())
    for i in (2, 8):
        row = indexed[i]
        assert row["display_frozen"] and row["frame"] == i
        assert row["detection_frame"] == row["display_frame"] == i - 1
        assert row["detector_ms"] == {}
    for row in indexed.values():
        assert row["detection_frame"] == row["display_frame"]
        assert row["detection_source_session"] == row["display_source_session"]
        if row["valid"]:
            assert row["frame"] == row["display_frame"]
            assert row["source_session"] == row["display_source_session"]
        assert all(box["class_id"] in (0, 1, 2) for box in row["detections"]["field"])
    visible = indexed[3]["detections"]
    assert visible["field"] and visible["ball"]
    for ball in visible["ball"]:
        x1, y1, x2, y2 = ball["bbox"]
        assert [ball["center_u"], ball["center_v"]] == [(x1 + x2) / 2, (y1 + y2) / 2]
    assert indexed[6]["detections"]["ball"] and indexed[6]["detections"]["field"]
    assert any(box["class_id"] == 1 for box in indexed[11]["detections"]["field"])
    assert any(box["class_id"] == 2 for box in indexed[12]["detections"]["field"])
    ray_errors = []
    for i in (3, 4, 9):
        row = indexed[i]
        assert row["residual_kind"] == "accepted"
        assert row["feature_point_origin"] == "optical_flow_from_manually_selected_map_id"
        ext = row["pnp_extrinsics"]
        matrix = np.asarray(ext["R_camera_from_map"])
        rvec = cv2.Rodrigues(matrix)[0]
        translation = np.asarray(ext["t_camera_from_map_m"])
        center = np.asarray(ext["optical_center_map_m"])
        assert np.allclose(center, -matrix.T @ translation)
        assert set(row["points_px"]) == {p["map_id"] for p in row["ground_diagnostics"]}
        for point in row["ground_diagnostics"]:
            assert point["status"] == "pnp_derived_ground_intersection"
            assert point["point_origin"] == row["feature_point_origin"]
            assert point["matched_map_xy_m"] == list(session.markers[point["map_id"]]["xyz"][:2])
            world = np.array([point["ground_map_xy_m"] + [0.0]])
            uv = cv2.projectPoints(world, rvec, translation, session.intr, session.dist)[0].ravel()
            ray_errors.append(float(np.linalg.norm(uv - point["pixel_uv"])))
    assert max(ray_errors) < .05, ray_errors
    timing = {role: statistics.median(row["detector_ms"][role] for row in indexed.values()
                                     if role in row["detector_ms"]) for role in ("ball", "field")}
    return dict(frames_with_input=len(indexed), valid_frames=[3, 4, 9],
                insufficient_frames=[5, 6], stale_frame=10,
                statuses=Counter(row["status"] for row in rows),
                median_detection_ms=timing, ground_roundtrip_max_px=max(ray_errors),
                median_processing_ms=statistics.median(row["processing_ms"] for row in indexed.values()),
                offline_only=True)


def main():
    session, display, metadata = setup()
    event = json.loads((ANCHOR / "0007_initialize.json").read_text())
    image_path = ANCHOR / event["raw_image"]
    assert sha(image_path) == event["raw_png_sha256"]
    original, other, cross = (cv2.imread(str(path)) for path in (image_path, SECOND, THIRD))
    assert original.shape == other.shape == cross.shape == (*session.image_shape, 3)
    clicks = {key: pixel for key, pixel in event["clicked_pixels"].items() if key in session.markers}
    assert len(clicks) == 4
    display.detectors.warmup(session.image_shape)
    starting = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    out = HERE / "runs" / f"offline_{starting}"
    save_run(out, dict(metadata, started_utc=utc_now(), input_kind="offline_saved_images_and_injected_loss",
                       fixtures=[str(image_path), str(SECOND), str(THIRD)], gui="offscreen_capture_with_scripted_events"))
    before = original.copy()
    source = OfflineSource(original, other, cross)
    arrows = run_fixture(out, source, session, display, clicks)
    assert np.array_equal(before, original)
    result = verify_rows(out, source, arrows, session)
    # Inject only the rejected status to check presentation of a computed residual.
    # The solver's automatic ID association is outside this demo.
    session.latest = Frame(original, "offline_rejection_display_fixture", 1, 0, {})
    session.lost("pnp_ambiguous", {"rms_px": 1.25, "inlier_ids": list(clicks)})
    display.observe(session.latest)
    checked_png(out, "rejected_candidate_residual", display.render(0, offline=True))
    packet = StringIO()
    # Exercise the log-only error path even if a previous rendering cached a valid projection.
    previous = next(row for row in map(json.loads, (out / "frames.jsonl").read_text().splitlines())
                    if row["valid"])
    display.extrinsics_pnp = previous["pnp_extrinsics"]
    display.geometry_rows = previous["ground_diagnostics"]
    display.record(packet, session, session.latest, 0, "offline_injected_rejection", 0, "offline")
    rejected = json.loads(packet.getvalue())
    assert not rejected["valid"] and rejected["x_camera_map"] is None
    assert rejected["residual"] == 1.25 and rejected["residual_kind"] == "rejected_candidate"
    assert rejected["marker_count"] == 0 and rejected["rejected_candidate_inlier_count"] == 4
    assert rejected["display_frame"] == rejected["detection_frame"] == 1
    assert rejected["pnp_extrinsics"] is None and rejected["ground_diagnostics"] == []
    (out / "rejected_candidate_fixture.json").write_text(json.dumps(rejected, indent=2) + "\n")
    result["rejected_candidate_status_display_checked_with_injected_diagnostics"] = True
    (out / "summary.json").write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)
    print("PASS: same frame, verified detectors, raw image unchanged, invalid pose/arrow cleared, manual recovery.")


if __name__ == "__main__":
    main()
