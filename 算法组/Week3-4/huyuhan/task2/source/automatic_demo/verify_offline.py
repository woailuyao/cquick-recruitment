"""Deterministic geometry/transport scenarios with synthetic white lines and boxes."""
import json
from pathlib import Path
import tempfile
import time

import cv2
import numpy as np

from localize import HERE, Locator, match, pixel_ground, rot
from run_auto import Engine, compose
from session import Frame
from transport import load_protocol, observed_reader

R = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
tilt = np.deg2rad(29)
R = np.array([[np.cos(tilt), 0, np.sin(tilt)], [0, 1, 0],
              [-np.sin(tilt), 0, np.cos(tilt)]]) @ R
T = np.array([.13, .08, 1.05])


def bounded_tests():
    # Two or three congruent groups separated in the map. Their classes distinguish
    # point identity within each group, leaving genuine location ambiguity.
    points = np.array([[0., 0.], [.9, .2], [.35, .75]])
    obs = [dict(xy=p.tolist(), class_id=i) for i, p in enumerate(points)]

    def markers(shifts):
        return [dict(xy=(p + shift).tolist(), class_id=i, id=f"{n}_{i}")
                for n, shift in enumerate(shifts) for i, p in enumerate(points)]

    bounds = [[0., -2.9475], [3.91, 3.0025]]
    alternatives = markers([[-1, 0], [1.1, 0.]])
    assert match(obs, alternatives, R, T)["status"] == "ambiguous"
    pose = match(obs, alternatives, R, T, camera_bounds=bounds)
    assert pose["valid"] and {p["map_id"] for p in pose["matches"]} == {"1_0", "1_1", "1_2"}
    assert np.allclose([pose["x_camera_map"], pose["y_camera_map"]], [1.23, .08])
    assert match(obs, markers([[.4, 0], [2.6, 0]]), R, T,
                 camera_bounds=bounds)["status"] == "ambiguous"
    # Carrier is within the rectangle, but the optical center goes beyond the endline.
    assert match(obs, markers([[3.83, 0]]), R, T,
                 camera_bounds=bounds)["status"] == "camera_outside_measured_half"
    assert match(obs, markers([[-1, 0]]), R, T,
                 camera_bounds=bounds)["status"] == "camera_outside_measured_half"
    # Projection at exactly the formal midline is included.
    assert match(obs, markers([[-T[0], 0]]), R, T, camera_bounds=bounds)["valid"]
    print("Bounded matching: exclusion before ranking, in-half rival, all excluded, optical offset, exact boundary")


class StubDetector:
    def __init__(self, boxes):
        self.boxes = boxes

    def infer(self, frame):
        return dict(ball=[], field=self.boxes if frame.max() else []), dict(ball=0., field=0.)


class TestClock:
    def __init__(self):
        self.age = 78.

    def age_bounds(self, meta):
        return dict(age_lower_ms=self.age-8, age_upper_ms=self.age,
                    clock_roundtrip_ms=8., calibration_age_ms=5.), "ok"


def frame_tests(output, protocol_path):
    locator = Locator()
    assert len(locator.markers) == 14 and all(m["class_id"] < 3 for m in locator.markers)
    locator.mount = R, T
    locator.mount_qualification = "synthetic_test"
    expected_uv = [(240, 408), (655, 618), (1000, 414)]
    ground = np.array(pixel_ground(expected_uv, locator.K, locator.dist, R, T))
    origin = np.array([.35, -.25])
    yaw = .1
    mapped = ground @ rot(yaw).T + origin
    locator.markers = [dict(xy=xy.tolist(), class_id=i, id=f"synthetic_{i}")
                       for i, xy in enumerate(mapped)]
    locator.map["markers"] = locator.markers
    image = np.full((*locator.shape, 3), (25, 95, 25), np.uint8)
    boxes = []
    for i, (u, v) in enumerate(expected_uv):
        arms = [(38, 0), (0, 38)]
        if i > 0:
            arms.append((-38, 0))
        if i == 2:
            arms.append((0, -38))
        for du, dv in arms:
            cv2.line(image, (u, v), (u+du, v+dv), (248, 248, 248), 5)
        boxes.append(dict(bbox=[u-35, v-35, u+35, v+35], class_id=i,
                          class_name=("LCross", "TCross", "XCross")[i], confidence=.9))

    engine = Engine(StubDetector(boxes), locator)
    clock = TestClock()
    protocol = load_protocol(protocol_path)
    reader = observed_reader(protocol, output, clock)
    statuses = []
    for sequence, name, read_age, last_age in (
            (1, "initial_pose", 78., 95.),
            (2, "expired_at_publish", 78., 511.),
            (3, "recovered_without_click", 89., 109.),
            (4, "insufficient_after_loss", 98., 112.),
            (5, "recovered_again", 78., 96.)):
        ticks = 1600000000 + sequence * 100000000
        meta = dict(session="synthetic_session", sequence=sequence, protocol_version=2,
                    capture_monotonic_ns=ticks)
        frame = Frame(image if sequence != 4 else np.zeros_like(image),
                      "synthetic_session", sequence, ticks/1e9, meta)
        clock.age = read_age
        assert reader.check_age(meta) == (True, "ok")
        age = dict(reader.checked)

        def fresh(f):
            clock.age = last_age
            passed, status = reader.check_age(f.producer)
            return passed, status, dict(reader.checked)

        packet = engine.step(frame, "ok", age, 10., time.perf_counter(), fresh)
        statuses.append(packet["status"])
        assert packet["read_age"]["age_upper_ms"] == read_age
        assert packet["publish_age"]["age_upper_ms"] == last_age
        if packet["valid"]:
            assert len({p["map_id"] for p in packet["matches"]}) == 3
            refined = np.array([p["pixel_uv"] for p in packet["intersections"]])
            assert np.max(np.linalg.norm(refined - expected_uv, axis=1)) < 4
            camera = rot(yaw) @ T[:2] + origin
            assert np.allclose([packet["x_camera_map"], packet["y_camera_map"]], camera, atol=.04)
            assert abs(packet["yaw_camera_map"] - np.rad2deg(yaw)) < 2
        else:
            assert all(packet[k] is None for k in ("x_camera_map", "y_camera_map", "yaw_camera_map"))
            assert not packet["matches"] and packet["pose_update_hz"] == 0
            assert all(p["map_id"] is None and p["map_xy_m"] is None for p in packet["intersections"])
        packet["input_kind"] = "synthetic geometry and controlled bridge clock"
        (output / f"{name}.json").write_text(json.dumps(packet, indent=2, allow_nan=False)+"\n")
        assert cv2.imwrite(str(output / f"{name}.png"), compose(frame, packet, locator))
        if sequence == 3:
            assert cv2.imwrite(str(output / "ground_transform.png"), compose(frame, packet, locator, True))
    assert statuses == ["ok", "bridge_stale_capture", "ok", "insufficient_markers", "ok"], statuses
    assert reader.max_age == .50
    print("Synthetic frame/status sequence:", statuses)


if __name__ == "__main__":
    cv2.setNumThreads(2)
    bounded_tests()
    out = Path(tempfile.mkdtemp(prefix="synthetic_", dir=HERE))
    frame_tests(out, Path("/mnt/c/Users/13991/CquickCameraBridge/bridge_transport.py"))
    print("Synthetic evidence:", out)
