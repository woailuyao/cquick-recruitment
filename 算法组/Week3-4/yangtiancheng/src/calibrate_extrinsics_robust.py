#!/usr/bin/env python3
"""Estimate camera->robot from ground chessboard images, with scale checks."""
import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml


def load_intrinsics(path):
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    c = data["camera_matrix"]
    k = np.array([[c["fx"], 0, c["cx"]], [0, c["fy"], c["cy"]], [0, 0, 1]], float)
    d = data["distortion_coefficients"]
    dist = np.array([d[x] for x in ("k1", "k2", "p1", "p2", "k3")], float)
    return data, k, dist


def symmetries(grid):
    """Corner orderings that preserve a rectangular pattern's shape."""
    yield "identity", grid
    yield "flip_rows", np.flipud(grid)
    yield "flip_cols", np.fliplr(grid)
    yield "rot180", np.flipud(np.fliplr(grid))


def estimate(image_path, k, dist, cols, rows, square, ax, ay, expected_height):
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"cannot read {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCornersSB(
        gray, (cols, rows), cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE)
    if not ok:
        ok, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if ok:
            corners = cv2.cornerSubPix(
                gray, corners, (5, 5), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 0.001))
    if not ok:
        raise ValueError(f"{cols}x{rows} corners not found: {image_path}")
    object_grid = np.zeros((rows, cols, 3), np.float64)
    for row in range(rows):
        for col in range(cols):
            object_grid[row, col] = [ax + row * square,
                                     ay + ((cols - 1) / 2 - col) * square, 0]
    object_points = object_grid.reshape(-1, 3)
    choices = []
    for name, image_grid in symmetries(corners.reshape(rows, cols, 2)):
        image_points = image_grid.reshape(-1, 2).astype(np.float64)
        ok, rv, tv = cv2.solvePnP(object_points, image_points, k, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            continue
        robot_to_camera, _ = cv2.Rodrigues(rv)
        rotation = robot_to_camera.T
        position = (-rotation @ tv).reshape(3)
        projected, _ = cv2.projectPoints(object_points, rv, tv, k, dist)
        rms = math.sqrt(float(np.mean(np.sum(
            (projected.reshape(-1, 2) - image_points) ** 2, axis=1))))
        score = rms * 0.01 + abs(position[1]) * 2 + max(0, abs(position[0]) - .4)
        if expected_height is not None:
            score += abs(position[2] - expected_height) * 10
        if position[2] <= 0:
            score += 100
        choices.append((score, name, rms, rotation, position))
    if not choices:
        raise ValueError(f"solvePnP failed: {image_path}")
    return min(choices, key=lambda x: x[0])


def write_yaml(path, result, images, square, expected_height, spread, reliable):
    _, symmetry, rms, rotation, position = result
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = position
    payload = {
        "frame_convention": "camera(x right,y down,z forward) -> robot(x forward,y left,z up)",
        "source_images": [str(p) for p in images],
        "square_size_m": square,
        "selected_grid_symmetry": symmetry,
        "reprojection_rms_px": float(rms),
        "measured_camera_height_m": expected_height,
        "pnp_camera_height_m": float(position[2]),
        "height_difference_m": None if expected_height is None else float(position[2] - expected_height),
        "repeatability_spread_m": float(spread),
        "calibration_status": "accepted" if reliable else "provisional_height_mismatch",
        "camera_position_in_robot_m": {"x": float(position[0]), "y": float(position[1]),
                                         "z": float(position[2])},
        "extrin": transform.tolist(),
    }
    Path(path).write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", required=True)
    ap.add_argument("--images", nargs="+", required=True)
    ap.add_argument("--output", required=True)
    ap.add_argument("--square", type=float, default=.027)
    ap.add_argument("--cols", type=int, default=9,
                    help="number of inner corners across the board")
    ap.add_argument("--rows", type=int, default=9,
                    help="number of inner-corner rows in the forward direction")
    ap.add_argument("--a-x", type=float, default=.750)
    ap.add_argument("--a-y", type=float, default=0.)
    ap.add_argument("--measured-height", type=float)
    ap.add_argument("--height-tolerance", type=float, default=.025)
    ap.add_argument("--accept-pnp-height", action="store_true",
                    help="write accepted status using PnP height after physical remeasurement")
    args = ap.parse_args()
    _, k, dist = load_intrinsics(args.intrinsics)
    images = [Path(x) for x in args.images]
    estimates = [estimate(p, k, dist, args.cols, args.rows, args.square, args.a_x, args.a_y,
                          args.measured_height) for p in images]
    positions = np.asarray([e[4] for e in estimates])
    median_index = int(np.argmin(np.linalg.norm(positions - np.median(positions, axis=0), axis=1)))
    chosen = estimates[median_index]
    spread = float(np.max(np.linalg.norm(positions - positions[median_index], axis=1)))
    mismatch = 0 if args.measured_height is None else abs(chosen[4][2] - args.measured_height)
    reliable = chosen[2] <= 1.5 and spread <= .02 and \
               (args.accept_pnp_height or args.measured_height is None or mismatch <= args.height_tolerance)
    write_yaml(args.output, chosen, images, args.square, args.measured_height, spread, reliable)
    print(f"PnP camera position: {chosen[4]}")
    print(f"reprojection RMS: {chosen[2]:.4f} px; repeatability spread: {spread:.4f} m")
    print(f"status: {'accepted' if reliable else 'provisional_height_mismatch'}")
    if not reliable:
        raise SystemExit("height/scale validation failed; output is provisional. "
                         "Remeasure grass-to-optical-center height and square size, or recapture intrinsics "
                         "with focus locked. Do not force only the z value.")


if __name__ == "__main__":
    main()

