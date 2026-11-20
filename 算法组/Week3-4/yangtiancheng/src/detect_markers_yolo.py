#!/usr/bin/env python3
"""YOLO field-marker detection with per-class geometric refinement.

Drop-in alternative to detect_field_features.py (the HSV detector): same
features.json output format [{"type", "u", "v"}, ...] consumed unchanged by
task2_localization.py, and the same detect(image) -> (candidates, mask)
calling convention.

The YOLO box alone is NOT the landmark. Refinement rules per class:
  - LCross / TCross / XCross: inside the (padded) box, build the white-line
    mask, skeletonize it, and take the skeleton branch point (a skeleton
    pixel with >= 3 neighbours) nearest the box centre. Fallbacks: strongest
    skeleton corner, then skeleton centroid, then box centre.
  - PenaltyPoint: the landmark is the centre of the painted dot, i.e. the
    centroid of the largest white blob inside the box. Fallback: box centre.

Usage:
    python3 detect_markers_yolo.py --image IMG_0020.jpg \
        --weights runs_markers/markers_v1/weights/best.pt
"""
import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from detect_field_features import skeletonize_opencv  # noqa: E402

CLASS_NAMES = {0: "LCross", 1: "TCross", 2: "XCross", 3: "PenaltyPoint"}
COLORS = {"LCross": (0, 255, 255), "TCross": (0, 255, 0),
          "XCross": (255, 0, 255), "PenaltyPoint": (0, 165, 255)}

_MODEL = None
_MODEL_PATH = None


def get_model(weights):
    global _MODEL, _MODEL_PATH
    if _MODEL is None or _MODEL_PATH != str(weights):
        try:
            from ultralytics import YOLO
        except ImportError:
            raise SystemExit("ultralytics not installed: pip install ultralytics")
        _MODEL = YOLO(str(weights))
        _MODEL_PATH = str(weights)
    return _MODEL


def white_mask(roi_bgr):
    hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array([0, 0, 225]), np.array([179, 60, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return mask


def branch_points(skel_bool):
    """Skeleton pixels with >= 3 skeleton neighbours (junction centres)."""
    img = skel_bool.astype(np.uint8)
    neighbours = np.zeros_like(img)
    padded = np.pad(img, 1)
    for dy in range(3):
        for dx in range(3):
            if dx == 1 and dy == 1:
                continue
            neighbours += padded[dy:dy + img.shape[0], dx:dx + img.shape[1]]
    ys, xs = np.nonzero((img == 1) & (neighbours >= 3))
    return np.column_stack((xs, ys)).astype(float)


def cluster_nearest(points, target, radius=10.0):
    """Cluster nearby points; return centroid of the cluster nearest target."""
    remaining = list(points)
    clusters = []
    while remaining:
        seed = remaining.pop(0)
        cluster = [seed]
        changed = True
        while changed:
            changed = False
            for p in list(remaining):
                if any(np.linalg.norm(p - c) <= radius for c in cluster):
                    cluster.append(p)
                    remaining.pop(next(i for i, item in enumerate(remaining)
                                       if item is p))
                    changed = True
        clusters.append(np.mean(cluster, axis=0))
    if not clusters:
        return None
    return min(clusters, key=lambda c: np.linalg.norm(c - target))


def refine_junction(image, box, kind):
    """Return (point_xy, method) for one detection box."""
    h, w = image.shape[:2]
    x1, y1, x2, y2 = box
    bw, bh = x2 - x1, y2 - y1
    pad_x, pad_y = 0.25 * bw, 0.25 * bh
    rx1, ry1 = max(0, int(x1 - pad_x)), max(0, int(y1 - pad_y))
    rx2, ry2 = min(w, int(x2 + pad_x) + 1), min(h, int(y2 + pad_y) + 1)
    roi = image[ry1:ry2, rx1:rx2]
    centre = np.array([(x1 + x2) / 2, (y1 + y2) / 2])
    if roi.size == 0:
        return centre, "box_centre_empty_roi"

    mask = white_mask(roi)
    if kind == "PenaltyPoint":
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(
            mask, 8)
        if count > 1:
            largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
            c = centroids[largest]
            return np.array([c[0] + rx1, c[1] + ry1]), "blob_centroid"
        return centre, "box_centre_no_blob"

    skel = skeletonize_opencv(mask)
    if not skel.any():
        return centre, "box_centre_no_line"
    local_centre = centre - np.array([rx1, ry1])
    branches = branch_points(skel)
    if len(branches):
        picked = cluster_nearest(branches, local_centre)
        if picked is not None:
            return picked + np.array([rx1, ry1]), "skeleton_branch"
    corners = cv2.goodFeaturesToTrack(
        (skel.astype(np.uint8)) * 255, maxCorners=8, qualityLevel=0.01,
        minDistance=5, blockSize=5, useHarrisDetector=False)
    if corners is not None:
        pts = corners.reshape(-1, 2).astype(float)
        picked = min(pts, key=lambda p: np.linalg.norm(p - local_centre))
        return picked + np.array([rx1, ry1]), "skeleton_corner"
    ys, xs = np.nonzero(skel)
    centroid = np.array([xs.mean(), ys.mean()])
    return centroid + np.array([rx1, ry1]), "skeleton_centroid"


def detect(image, weights, conf=0.35):
    """Return (candidates, None); candidate = (point_xy, kind, score)."""
    model = get_model(weights)
    results = model.predict(image, conf=conf, verbose=False)[0]
    candidates = []
    for box in results.boxes:
        cls_id = int(box.cls[0])
        kind = CLASS_NAMES.get(cls_id)
        if kind is None:
            kind = str(results.names.get(cls_id, cls_id))
        xyxy = box.xyxy[0].tolist()
        point, _method = refine_junction(image, xyxy, kind)
        candidates.append((point, kind, float(box.conf[0])))
    return candidates, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--output", default="features.json")
    ap.add_argument("--preview", default="features_preview.jpg")
    ap.add_argument("--conf", type=float, default=0.35)
    args = ap.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"cannot read image: {args.image}")
    if image.shape[1::-1] != (1280, 720):
        raise SystemExit(f"expected 1280x720, got {image.shape[1]}x{image.shape[0]}")

    model = get_model(args.weights)
    results = model.predict(image, conf=args.conf, verbose=False)[0]
    features = []
    details = []
    preview = image.copy()
    for box in results.boxes:
        cls_id = int(box.cls[0])
        kind = CLASS_NAMES.get(cls_id, str(results.names.get(cls_id, cls_id)))
        xyxy = box.xyxy[0].tolist()
        point, method = refine_junction(image, xyxy, kind)
        score = float(box.conf[0])
        features.append({"type": kind, "u": round(float(point[0]), 1),
                         "v": round(float(point[1]), 1)})
        details.append({"type": kind, "conf": round(score, 3),
                        "refine": method})
        q = tuple(np.round(point).astype(int))
        cv2.rectangle(preview, (int(xyxy[0]), int(xyxy[1])),
                      (int(xyxy[2]), int(xyxy[3])), COLORS.get(kind, (255, 255, 255)), 2)
        cv2.circle(preview, q, 10, COLORS.get(kind, (255, 255, 255)), 3)
        cv2.putText(preview, f"{kind} {score:.2f}", (q[0] + 14, q[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, .7,
                    COLORS.get(kind, (255, 255, 255)), 2, cv2.LINE_AA)
    Path(args.output).write_text(json.dumps(features, indent=2),
                                  encoding="utf-8")
    cv2.imwrite(args.preview, preview)
    print(json.dumps(features, ensure_ascii=False, indent=2))
    print("refinement:", json.dumps(details, ensure_ascii=False))
    if len(features) < 2:
        print("warning: fewer than two features; pose localization is not observable")


if __name__ == "__main__":
    main()
