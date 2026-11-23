"""Exercise real fixed-image clicks and known-pose / failure cases without the bridge."""
import argparse
import json
from io import StringIO
from types import SimpleNamespace
from unittest.mock import patch
from math import cos, radians, sin
from pathlib import Path

import cv2
import numpy as np

from display import Display
from pose import angle_delta, solve
from prototype import HERE, REFERENCE, key_action, run, settings
from session import Frame

PICKS = {
    "small_end_right": (293, 411),
    "small_front_right": (185, 478),
    "large_end_right": (496, 414),
    "large_front_right": (289, 624),
    "end_right": (924, 420),
}


def frame(image, number, t, session="offline_verification"):
    return Frame(image, session, number, t, {})


def click(view, key, pixel):
    row = view.ids.index(key)
    view.mouse(cv2.EVENT_LBUTTONDOWN, view.list_left + 7, 48 + 10 + row * view.row_height + 8)
    view.mouse(cv2.EVENT_LBUTTONDOWN, *view.image_point(pixel))


def initialize(session, view, image):
    session.on_frame(frame(image, 1, 1))
    key_action(32, session)
    for key, pixel in PICKS.items():
        click(view, key, pixel)
    key_action(8, session)
    assert "end_right" not in session.points
    click(view, "end_right", PICKS["end_right"])
    assert set(session.points) == set(PICKS) and session.pose is None
    assert all(np.linalg.norm(np.subtract(session.points[k], p)) < 1.3 for k, p in PICKS.items())
    session.on_frame(frame(image, 2, 1.1))
    session.initialize(session.latest)
    assert session.pose is not None, session.report()


def project(session, world, center, yaw, pitch):
    yaw, pitch = radians(yaw), radians(pitch)
    optical = np.array([cos(yaw) * cos(pitch), sin(yaw) * cos(pitch), -sin(pitch)])
    right = np.array([sin(yaw), -cos(yaw), 0])
    rotation = np.array([right, np.cross(optical, right), optical])
    return cv2.projectPoints(world, cv2.Rodrigues(rotation)[0], -rotation @ center,
                             session.intr, session.dist)[0].reshape(-1, 2)


def mathematical_cases(session):
    ids = list(PICKS)
    world = np.float64([session.markers[k]["xyz"] for k in ids])
    center = np.array([0.16, -2.6, .68])
    yaw = 11.0
    pixels = project(session, world, center, yaw, 12)
    measured = dict(zip(ids, pixels))
    pose, status, d = solve(measured, session.markers, session.intr, session.dist)
    assert status == "ok" and len(d["inlier_ids"]) == 5, (status, d)
    location_error = float(np.linalg.norm(pose["center"] - center))
    yaw_error = angle_delta(pose["yaw_deg"], yaw)
    assert location_error < 1e-5 and yaw_error < 1e-4
    measured[ids[0]] = pixels[0] + [60, 70]
    _, status, d = solve(measured, session.markers, session.intr, session.dist)
    assert status == "ok" and ids[0] not in d["inlier_ids"]
    rotated = dict(zip(ids, project(session, world, center, yaw + 27, 12)))
    _, jump, _ = solve(rotated, session.markers, session.intr, session.dist, pose, dt=.1)
    assert jump == "pose_jump"
    line_ids = [k for k, m in session.markers.items() if m["xyz"][0] == 3.91]
    line = {key: (200 + 60 * i, 402) for i, key in enumerate(line_ids)}
    assert solve(line, session.markers, session.intr, session.dist)[1] == "poor_point_distribution"
    assert angle_delta(179, -179) == 2
    distant_center = world.mean(axis=0) + [-2.8, -.3, 20]
    ambiguous = dict(zip(ids, project(session, world, distant_center, 8, 80)))
    ambiguous_pose, ambiguity_status, amb = solve(ambiguous, session.markers, session.intr, session.dist)
    assert ambiguous_pose is None and ambiguity_status == "pnp_ambiguous", (ambiguity_status, amb)
    return dict(synthetic_center_error_m=location_error, synthetic_yaw_error_deg=yaw_error,
                collinear_rejection=True, outlier_rejection=True, jump_status=jump,
                ambiguity_status=ambiguity_status, ambiguous_second_rms_px=amb["alternative_rms_px"])


def check_loop_logging(image):
    session, lines, _ = settings()
    display = Display(session, lines)
    keys = iter([-1, 32, 13, -1, -1, -1, ord("q")])
    actual_wait = cv2.waitKey
    events = iter([(frame(image, 1, 1.0), "ok"),
                   (frame(image, 2, 1.1), "ok"),
                   (frame(image, 3, 1.2), "ok"),
                   (frame(image, 4, 1.3), "ok"),
                   (None, "duplicate_or_out_of_order"),
                   (frame(image, 7, 1.6), "ok")])
    source = SimpleNamespace(next_frame=lambda: next(events), fresh=lambda: (True, "offline"))
    log = StringIO()

    def scripted_wait(delay):
        actual_wait(delay)
        key = next(keys)
        if key == 13:
            for marker, pixel in PICKS.items():
                click(display, marker, pixel)
        return key

    with patch.object(cv2, "waitKey", side_effect=scripted_wait):
        run(source, session, display, log, SimpleNamespace(offline=True, scene="preview"))
    rows = [json.loads(line) for line in log.getvalue().splitlines()]
    assert [row["valid"] for row in rows] == [False, False, True, True, False, False, False], rows
    assert rows[2]["initialization"]["frozen_frame"] == 1
    assert rows[2]["frame"] == 3 and rows[3]["initialization"] is None
    assert rows[4]["status"] == "bridge_duplicate_or_out_of_order" and rows[4]["camera_x_m"] is None
    assert rows[-1]["status"] == "stopped" and rows[-1]["camera_yaw_deg"] is None
    return log.getvalue()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--gui", action="store_true", help="also open OpenCV HighGUI for fixed-image smoke check")
    args = p.parse_args()
    session, lines, provenance = settings()
    view = Display(session, lines)
    image = cv2.imread(str(REFERENCE))
    assert image is not None
    out = HERE / "verification"
    out.mkdir(exist_ok=True)
    initialize(session, view, image)
    still = session.report()
    snapshot = view.render(10, offline=True)
    assert cv2.imwrite(str(out / "fixed_image_split_view.png"), snapshot)
    if args.gui:
        title = "CQuick offline fixed-image UI check"
        cv2.namedWindow(title, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(title, view.mouse)
        cv2.imshow(title, snapshot)
        cv2.waitKey(300)
        cv2.destroyAllWindows()
    shift = np.float32([[1, 0, 2], [0, 1, -1]])
    shifted = cv2.warpAffine(image, shift, (image.shape[1], image.shape[0]))
    session.on_frame(frame(shifted, 3, 1.2))
    moved = session.report()
    assert moved["valid"], moved
    for key in moved["points_px"]:
        delta = np.subtract(moved["points_px"][key], still["points_px"][key])
        assert np.linalg.norm(delta - [2, -1]) < .35
    session.on_frame(frame(np.zeros_like(image), 4, 1.3))
    lost = session.report()
    assert lost["status"] == "insufficient_points" and lost["camera_center_m"] is None
    session.on_frame(frame(image, 5, 1.4))
    assert session.report()["camera_center_m"] is None
    initialize(session, view, image)
    session.on_frame(frame(image, 3, 1.8))
    gap = session.report()
    assert gap["status"] == "frame_gap_or_order" and gap["camera_yaw_deg"] is None
    initialize(session, view, image)
    session.on_frame(frame(image, 1, 2, session="simulated_new_session"))
    reconnect = session.report()
    assert reconnect["status"] == "bridge_session_changed" and reconnect["camera_x_m"] is None
    report = dict(input=str(REFERENCE), gui_window_smoke=args.gui,
                  input_method="scripted clicks through the GUI callback and scripted Space/undo handlers",
                  synthetic_pose_and_rejections=mathematical_cases(session),
                  fixed_image=still, two_pixel_translation=moved,
                  total_occlusion=lost, frame_gap=gap, session_change=reconnect, config=provenance)
    if args.gui:
        (out / "simulated_gui_frames.jsonl").write_text(check_loop_logging(image))
        report["simulated_gui_loop_logging_checked"] = True
    (out / "offline_verification.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps({"math": report["synthetic_pose_and_rejections"], "fixed_image_rms_px":
                      still["reprojection"]["rms_px"], "fixed_image_points": still["marker_count"],
                      "flow": moved["status"], "occlusion": lost["status"], "gap": gap["status"],
                      "session_change": reconnect["status"], "gui": args.gui}, indent=2))


if __name__ == "__main__":
    main()
