#!/usr/bin/env python3
"""Detect white field-line L/T/X junctions and write features.json."""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

try:
    from numba import njit
except ImportError:
    njit = None


_NEIGHBOR_COUNT_LUT = np.zeros(256, dtype=np.uint8)
_TRANSITION_COUNT_LUT = np.zeros(256, dtype=np.uint8)
for _code in range(256):
    _bits = [(_code >> bit) & 1 for bit in range(8)]
    _NEIGHBOR_COUNT_LUT[_code] = sum(_bits)
    _TRANSITION_COUNT_LUT[_code] = sum(
        _bits[index] == 0 and _bits[(index + 1) % 8] == 1
        for index in range(8)
    )


if njit is not None:
    @njit(cache=True)
    def _skeletonize_zhang_numba(img):
        height, width = img.shape
        changed = True
        while changed:
            changed = False
            for step in (0, 1):
                remove = np.zeros_like(img)
                padded = np.zeros((height + 2, width + 2), dtype=np.uint8)
                padded[1:-1, 1:-1] = img
                step_changed = False
                for y in range(1, height + 1):
                    for x in range(1, width + 1):
                        if padded[y, x] == 0:
                            continue
                        p2 = padded[y - 1, x]
                        p3 = padded[y - 1, x + 1]
                        p4 = padded[y, x + 1]
                        p5 = padded[y + 1, x + 1]
                        p6 = padded[y + 1, x]
                        p7 = padded[y + 1, x - 1]
                        p8 = padded[y, x - 1]
                        p9 = padded[y - 1, x - 1]
                        neighbours = int(p2) + int(p3) + int(p4) + int(p5) + \
                            int(p6) + int(p7) + int(p8) + int(p9)
                        transitions = 0
                        if p2 == 0 and p3 == 1:
                            transitions += 1
                        if p3 == 0 and p4 == 1:
                            transitions += 1
                        if p4 == 0 and p5 == 1:
                            transitions += 1
                        if p5 == 0 and p6 == 1:
                            transitions += 1
                        if p6 == 0 and p7 == 1:
                            transitions += 1
                        if p7 == 0 and p8 == 1:
                            transitions += 1
                        if p8 == 0 and p9 == 1:
                            transitions += 1
                        if p9 == 0 and p2 == 1:
                            transitions += 1
                        if step == 0:
                            guard1 = p2 * p4 * p6 == 0
                            guard2 = p4 * p6 * p8 == 0
                        else:
                            guard1 = p2 * p4 * p8 == 0
                            guard2 = p2 * p6 * p8 == 0
                        if (2 <= neighbours <= 6 and transitions == 1 and
                                guard1 and guard2):
                            remove[y - 1, x - 1] = 1
                            step_changed = True
                if step_changed:
                    img -= remove
                    changed = True
        return img > 0
else:
    _skeletonize_zhang_numba = None


def skeletonize_opencv(binary):
    """Zhang-Suen thinning using only NumPy/OpenCV dependencies."""
    if _skeletonize_zhang_numba is not None:
        return _skeletonize_zhang_numba((binary > 0).astype(np.uint8))
    img = (binary > 0).astype(np.uint8)
    changed = True
    while changed:
        changed = False
        for step in (0, 1):
            p = np.pad(img, 1)
            p2, p3, p4 = p[:-2,1:-1], p[:-2,2:], p[1:-1,2:]
            p5, p6, p7 = p[2:,2:], p[2:,1:-1], p[2:,:-2]
            p8, p9 = p[1:-1,:-2], p[:-2,:-2]
            code = p2 | (p3 << 1) | (p4 << 2) | (p5 << 3) | \
                   (p6 << 4) | (p7 << 5) | (p8 << 6) | (p9 << 7)
            neighbours = _NEIGHBOR_COUNT_LUT[code]
            transitions = _TRANSITION_COUNT_LUT[code]
            if step == 0:
                guard1 = p2*p4*p6 == 0; guard2 = p4*p6*p8 == 0
            else:
                guard1 = p2*p4*p8 == 0; guard2 = p2*p6*p8 == 0
            remove = (img == 1) & (neighbours >= 2) & (neighbours <= 6) & \
                     (transitions == 1) & guard1 & guard2
            if np.any(remove):
                img[remove] = 0
                changed = True
    return img > 0


def angle_diff(a, b):
    d = abs((a - b) % math.pi)
    return min(d, math.pi - d)


def line_intersection(a, b):
    x1, y1, x2, y2 = a
    x3, y3, x4, y4 = b
    den = (x1-x2)*(y3-y4) - (y1-y2)*(x3-x4)
    if abs(den) < 1e-6:
        return None
    px = ((x1*y2-y1*x2)*(x3-x4) - (x1-x2)*(x3*y4-y3*x4)) / den
    py = ((x1*y2-y1*x2)*(y3-y4) - (y1-y2)*(x3*y4-y3*x4)) / den
    return np.array([px, py], float)


def distance_to_segment(p, line):
    a = np.array(line[:2], float); b = np.array(line[2:], float)
    ab = b-a
    t = np.clip(np.dot(p-a, ab)/(np.dot(ab, ab)+1e-9), 0, 1)
    return float(np.linalg.norm(p-(a+t*ab)))


def arm_present(mask, p, angle, sign):
    h, w = mask.shape
    hits = 0; total = 0
    direction = np.array([math.cos(angle), math.sin(angle)]) * sign
    for radius in range(28, 181, 10):
        q = p + direction * radius
        x, y = int(round(q[0])), int(round(q[1]))
        if x < 3 or y < 3 or x >= w-3 or y >= h-3:
            continue
        total += 1
        if np.mean(mask[y-3:y+4, x-3:x+4]) > 65:
            hits += 1
    return total >= 7 and hits / total >= 0.72


def merge_candidates(candidates, radius=36):
    merged = []
    for item in sorted(candidates, key=lambda z: -z[2]):
        if any(np.linalg.norm(item[0]-old[0]) < radius for old in merged):
            continue
        merged.append(item)
    return merged


def ground_support(green_mask, white_mask, p, inner=28, outer=82):
    """Measure whether a junction is embedded in green ground.

    B-Human-style field perception does not accept a white structure in
    isolation: it must be supported by the field surface.  This rejects many
    highlights, goal-frame points, walls and equipment edges.
    """
    h, w = white_mask.shape
    x, y = int(round(p[0])), int(round(p[1]))
    x0, x1 = max(0, x-outer), min(w, x+outer+1)
    y0, y1 = max(0, y-outer), min(h, y+outer+1)
    yy, xx = np.ogrid[y0:y1, x0:x1]
    ring = ((xx-x)**2 + (yy-y)**2 >= inner**2) & ((xx-x)**2 + (yy-y)**2 <= outer**2)
    non_white = ring & (white_mask[y0:y1, x0:x1] == 0)
    available = int(np.count_nonzero(non_white))
    if available < 350:
        return 0.0
    return float(np.count_nonzero((green_mask[y0:y1, x0:x1] > 0) & non_white) / available)


def local_line_width(distance, skeleton_points, p, r0=32, r1=75):
    """Estimate tape width away from the enlarged junction centre."""
    delta = skeleton_points - p
    radii = np.linalg.norm(delta, axis=1)
    samples = skeleton_points[(radii >= r0) & (radii <= r1)]
    if len(samples) < 5:
        return float("inf")
    h, w = distance.shape
    xs = np.clip(np.round(samples[:, 0]).astype(int), 0, w-1)
    ys = np.clip(np.round(samples[:, 1]).astype(int), 0, h-1)
    # distanceTransform gives half width in pixels.
    return float(2.0 * np.median(distance[ys, xs]))


def plausible_field_junction(green_mask, white_mask, distance, skeleton_points, p):
    green = ground_support(green_mask, white_mask, p)
    width = local_line_width(distance, skeleton_points, p)
    # Real tape is surrounded by turf and remains narrow along its arms.
    # Thresholds deliberately remain loose for perspective and worn tape.
    return green >= 0.24 and 2.0 <= width <= 64.0, green, width


def raw_arm_count(skeleton_points, p, r0=45, r1=65):
    """Count connected angular branches without assuming they stay straight."""
    delta = skeleton_points - p
    radii = np.linalg.norm(delta, axis=1)
    ring = delta[(radii >= r0) & (radii <= r1)]
    if len(ring) < 3:
        return 0
    angles = np.sort((np.arctan2(ring[:, 1], ring[:, 0]) + 2*np.pi) % (2*np.pi))
    groups = [[angles[0]]]
    for angle in angles[1:]:
        if angle - groups[-1][-1] < math.radians(14):
            groups[-1].append(angle)
        else:
            groups.append([angle])
    if len(groups) > 1 and groups[0][0] + 2*np.pi - groups[-1][-1] < math.radians(14):
        groups[0] = groups[-1] + [x + 2*np.pi for x in groups[0]]
        groups.pop()
    return sum(len(g) >= 8 for g in groups)


def segment_support(mask, a, b, samples=36):
    h, w = mask.shape
    hits = 0
    valid = 0
    for t in np.linspace(.06, .94, samples):
        p = a * (1-t) + b * t
        x, y = int(round(p[0])), int(round(p[1]))
        if x < 4 or y < 4 or x >= w-4 or y >= h-4:
            continue
        valid += 1
        if np.mean(mask[y-4:y+5, x-4:x+5]) > 85:
            hits += 1
    return hits / max(valid, 1)


def alternate_curved_path(mask, a, b):
    """Return true when an arc still connects both junctions after removing the chord."""
    work = mask.copy()
    cv2.line(work, tuple(np.round(a).astype(int)), tuple(np.round(b).astype(int)), 0, 55)
    _, labels = cv2.connectedComponents((work > 0).astype(np.uint8), 8)
    yy, xx = np.indices(mask.shape)
    da = np.hypot(xx-a[0], yy-a[1])
    db = np.hypot(xx-b[0], yy-b[1])
    labels_a = set(np.unique(labels[(da > 55) & (da < 110)])) - {0}
    labels_b = set(np.unique(labels[(db > 55) & (db < 110)])) - {0}
    for label in labels_a & labels_b:
        if np.count_nonzero(labels == label) >= 1200:
            return True
    return False


def detect_center_circle_crosses(mask, skel, skeleton_points, green_mask, distance):
    """Find the two intersections of a centre line with a projected circle.

    Curved circle arms fail the straight-ray test used for ordinary L/T/X
    junctions.  Here we pair high-branch candidates connected by the same
    long white centre line.
    """
    corners = cv2.goodFeaturesToTrack(skel * 255, maxCorners=45, qualityLevel=.02,
                                      minDistance=38, blockSize=11,
                                      useHarrisDetector=False)
    if corners is None:
        return []
    branch_points = []
    for p in corners.reshape(-1, 2).astype(float):
        count = raw_arm_count(skeleton_points, p)
        plausible, green, width = plausible_field_junction(
            green_mask, mask, distance, skeleton_points, p)
        if count >= 3 and plausible:
            branch_points.append((p, count, green, width))
    best_circle = None
    best_straight = None
    for i, (a, ca, ga, wa) in enumerate(branch_points):
        for b, cb, gb, wb in branch_points[i+1:]:
            distance = float(np.linalg.norm(a-b))
            if distance < 300 or distance > 1100:
                continue
            support = segment_support(mask, a, b)
            if support < .72:
                continue
            score = (support + .08 * (ca + cb) + min(distance, 800) / 4000
                     + .25 * (ga + gb) - .004 * (wa + wb))
            if alternate_curved_path(mask, a, b):
                if best_circle is None or score > best_circle[0]:
                    best_circle = (score, a, b)
            elif best_straight is None or score > best_straight[0]:
                best_straight = (score, a, b)
    if best_circle is not None:
        return [(best_circle[1], "XCross", 4.5),
                (best_circle[2], "XCross", 4.5)]
    if best_straight is not None:
        return [(best_straight[1], "TCross", 3.5),
                (best_straight[2], "TCross", 3.5)]
    return []


def detect(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # The indoor artificial turf is strongly desaturated, so fixed HSV hue
    # bounds reject valid grass. Use relative green dominance instead.
    b, g, r = cv2.split(image)
    greenish = ((g.astype(np.float32) >= .90 * r.astype(np.float32)) &
                (g.astype(np.float32) >= .90 * b.astype(np.float32)) &
                (g < 245))
    green_mask = greenish.astype(np.uint8) * 255
    # White tape is bright and weakly saturated; tolerate exposure clipping.
    mask = cv2.inRange(hsv, np.array([0, 0, 225]), np.array([179, 60, 255]))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    clean = np.zeros_like(mask)
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] >= 1800:
            clean[labels == label] = 255
    mask = clean
    distance = cv2.distanceTransform(mask, cv2.DIST_L2, 5)
    # Reduce thick tape to a one-pixel centreline, then find bends/branches.
    skel = skeletonize_opencv(mask).astype(np.uint8)
    corners = cv2.goodFeaturesToTrack(skel * 255, maxCorners=60, qualityLevel=.02,
                                      minDistance=38, blockSize=11, useHarrisDetector=False)
    if corners is None:
        return [], mask
    h, w = mask.shape
    candidates = []
    yy, xx = np.nonzero(skel)
    skeleton_points = np.column_stack((xx, yy)).astype(float)
    for raw in corners.reshape(-1, 2):
        p = raw.astype(float)
        plausible, green_score, line_width = plausible_field_junction(
            green_mask, mask, distance, skeleton_points, p)
        if not plausible:
            continue
        delta = skeleton_points - p
        radii = np.linalg.norm(delta, axis=1)
        ring = delta[(radii >= 55) & (radii <= 62)]
        if len(ring) < 3:
            continue
        angles = np.sort((np.arctan2(ring[:, 1], ring[:, 0]) + 2*np.pi) % (2*np.pi))
        # Merge ring hits into angular arms, including the wrap-around group.
        groups = [[angles[0]]]
        for angle in angles[1:]:
            if angle - groups[-1][-1] < math.radians(14):
                groups[-1].append(angle)
            else:
                groups.append([angle])
        if len(groups) > 1 and (groups[0][0] + 2*np.pi - groups[-1][-1]) < math.radians(14):
            groups[0] = groups[-1] + [x + 2*np.pi for x in groups[0]]
            groups.pop()
        arm_angles = [float(np.mean(g) % (2*np.pi)) for g in groups if len(g) >= 1]
        raw_angles = list(arm_angles)
        # Wide noisy groups can occur on grass; keep only rays supported by white tape.
        arm_angles = [a for a in arm_angles if arm_present(mask, p, a, 1)]
        count = len(arm_angles)
        if count not in (2, 3, 4):
            # Perspective and slight tape curvature can make a valid L arm fail
            # the long straight-ray test.  Two strong annular branches with a
            # clear non-straight bend are still a reliable L junction.
            if len(raw_angles) == 2:
                separation = math.degrees(abs(math.atan2(
                    math.sin(raw_angles[0]-raw_angles[1]),
                    math.cos(raw_angles[0]-raw_angles[1]))))
                if 35 <= separation <= 145:
                    score = 3.0 - abs(separation-90) / 90
                    candidates.append((p, "LCross", score + green_score - .01 * line_width))
            continue
        if count == 2:
            separation = math.degrees(abs(math.atan2(math.sin(arm_angles[0]-arm_angles[1]),
                                                     math.cos(arm_angles[0]-arm_angles[1]))))
            if separation > 145:  # straight line, not L
                continue
        kind = {2: "LCross", 3: "TCross", 4: "XCross"}[count]
        candidates.append((p, kind, float(count) + green_score - .01 * line_width))
    candidates = merge_candidates(candidates, radius=70)
    circle_crosses = detect_center_circle_crosses(
        mask, skel, skeleton_points, green_mask, distance)
    if circle_crosses:
        # A validated connected pair already supplies the two observable
        # junctions needed for localization.  Suppress lower-confidence bend
        # candidates from circle curvature, goal frames and image borders.
        candidates = list(circle_crosses)
    return merge_candidates(candidates, radius=70), mask


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image", required=True)
    ap.add_argument("--output", default="features.json")
    ap.add_argument("--preview", default="features_preview.jpg")
    args = ap.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise SystemExit(f"cannot read image: {args.image}")
    if image.shape[1::-1] != (1280, 720):
        raise SystemExit(f"expected 1280x720, got {image.shape[1]}x{image.shape[0]}")
    found, _ = detect(image)
    features = [{"type": kind, "u": round(float(p[0]), 1), "v": round(float(p[1]), 1)}
                for p, kind, _ in found]
    Path(args.output).write_text(json.dumps(features, indent=2), encoding="utf-8")
    preview = image.copy()
    colors = {"LCross": (0, 255, 255), "TCross": (0, 255, 0), "XCross": (255, 0, 255)}
    for p, kind, _ in found:
        q = tuple(np.round(p).astype(int))
        cv2.circle(preview, q, 12, colors[kind], 3)
        cv2.putText(preview, kind, (q[0]+15, q[1]-10), cv2.FONT_HERSHEY_SIMPLEX,
                    .7, colors[kind], 2, cv2.LINE_AA)
    cv2.imwrite(args.preview, preview)
    print(json.dumps(features, ensure_ascii=False, indent=2))
    if len(features) < 2:
        print("warning: fewer than two features; pose localization is not observable")


if __name__ == "__main__":
    main()
