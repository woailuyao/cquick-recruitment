"""Optical flow and ground-plane pose. Coordinates use the verified field map."""
from itertools import combinations
from math import atan2, degrees

import cv2
import numpy as np

MIN_POINTS = 4
BORDER = 11
MAX_FB_PX = 1.2
MAX_LK_ERROR = 22.0
MAX_POINT_ERROR_PX = 5.0
MAX_RMS_PX = 3.0
AMBIGUITY_MARGIN_PX = 0.8
MAX_FRAME_GAP_S = 0.55
MAX_CAPTURE_AGE_S = 0.50
LK = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_COUNT | cv2.TERM_CRITERIA_EPS, 30, 0.01))


def angle_delta(a, b):
    return abs((a - b + 180) % 360 - 180)


def inside(point, shape):
    u, v = point
    return (BORDER <= u < shape[1] - BORDER and BORDER <= v < shape[0] - BORDER)


def track(previous, current, points, max_motion_px=140):
    if not points:
        return {}, {}
    ids = list(points)
    initial = np.float32([points[key] for key in ids]).reshape(-1, 1, 2)
    forward, good, photometric = cv2.calcOpticalFlowPyrLK(previous, current, initial, None, **LK)
    if forward is None:
        return {}, {key: {"status": "lk_forward_failed"} for key in ids}
    candidates = [i for i in range(len(ids)) if good[i, 0] and inside(forward[i, 0], current.shape)]
    reverse, back_ok = {}, {}
    if candidates:
        back, ok, _ = cv2.calcOpticalFlowPyrLK(current, previous, forward[candidates], None, **LK)
        if back is not None:
            reverse = dict(zip(candidates, back[:, 0]))
            back_ok = dict(zip(candidates, ok[:, 0]))
    result, diagnostics = {}, {}
    for i, key in enumerate(ids):
        pixel = forward[i, 0]
        info = {}
        if not good[i, 0]:
            status = "lk_forward_failed"
        elif not inside(pixel, current.shape):
            status = "out_of_bounds"
        elif not back_ok.get(i):
            status = "lk_backward_failed"
        else:
            error = float(np.linalg.norm(reverse[i] - initial[i, 0]))
            motion = float(np.linalg.norm(pixel - initial[i, 0]))
            appearance = float(photometric[i, 0])
            info.update(fb_px=error, motion_px=motion, lk_error=appearance)
            if not np.isfinite([error, motion, appearance]).all() or error > MAX_FB_PX:
                status = "forward_backward_mismatch"
            elif appearance > MAX_LK_ERROR:
                status = "appearance_changed"
            elif motion > max_motion_px:
                status = "point_jump"
            else:
                status = "ok"
                result[key] = pixel.astype(float)
        diagnostics[key] = dict(info, status=status)
    return result, diagnostics


def spread(points, minimum_span, minimum_ratio, minimum_area):
    horizontal = np.ascontiguousarray(points[:, :2], dtype=np.float32)
    if len(horizontal) < MIN_POINTS:
        return False
    s = np.linalg.svd(horizontal - horizontal.mean(axis=0), compute_uv=False)
    return (s[0] >= minimum_span and s[1] >= minimum_ratio * s[0]
            and cv2.contourArea(cv2.convexHull(horizontal)) >= minimum_area)


def good_layout(world, image):
    return spread(world, 0.25, 0.06, 0.075) and spread(image, 45, 0.05, 550)


def candidate(world, image, intr, dist, rvec, tvec, fit):
    rotation = cv2.Rodrigues(rvec)[0]
    translation = tvec.reshape(3)
    center = -rotation.T @ translation
    direction = rotation.T[:, 2]
    depths = (world @ rotation.T + translation)[:, 2]
    if (not np.isfinite(center).all() or center[2] <= 0
            or np.linalg.norm(direction[:2]) < 0.10 or np.any(depths[fit] <= 0.01)):
        return None
    projected = cv2.projectPoints(world, rvec, tvec, intr, dist)[0].reshape(-1, 2)
    errors = np.linalg.norm(projected - image, axis=1)
    if not np.isfinite(errors).all():
        return None
    inlier = np.flatnonzero((errors <= MAX_POINT_ERROR_PX) & (depths > 0.01))
    if not good_layout(world[inlier], image[inlier]):
        return None
    rms = float(np.sqrt(np.mean(errors[inlier] ** 2)))
    if rms > MAX_RMS_PX:
        return None
    return dict(center=center, direction=direction, rotation=rotation,
                yaw_deg=degrees(atan2(direction[1], direction[0])),
                inlier=inlier, rms_px=rms, errors=errors, projected=projected)


def rotation_delta(a, b):
    cosine = (np.trace(a @ b.T) - 1) / 2
    return degrees(np.arccos(np.clip(cosine, -1, 1)))


def distinct(a, b):
    return (np.linalg.norm(a["center"] - b["center"]) > 0.20
            or rotation_delta(a["rotation"], b["rotation"]) > 8)


def solve(points, markers, intr, dist, previous=None, dt=None, *, collect_candidates=False):
    ids = list(points)
    if len(ids) < MIN_POINTS:
        return None, "insufficient_points", {}
    world = np.float64([markers[key]["xyz"] for key in ids])
    image = np.float64([points[key] for key in ids])
    if not good_layout(world, image):
        return None, "poor_point_distribution", {}
    candidates = []
    for count in (len(ids), len(ids) - 1):
        # Try dropping one erroneous click/track only when the full set cannot fit.
        if count < MIN_POINTS or (count < len(ids) and candidates):
            break
        for subset in combinations(range(len(ids)), count):
            fit = np.array(subset)
            obj, pix = world[fit], image[fit]
            if not good_layout(obj, pix):
                continue
            try:
                _, rotations, translations, _ = cv2.solvePnPGeneric(
                    obj, pix, intr, dist, flags=cv2.SOLVEPNP_IPPE)
                for rvec, tvec in zip(rotations, translations):
                    if not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
                        continue
                    # Keep qualified raw roots for ambiguity checks even if LM
                    # refinement from both roots converges to the same minimum.
                    roots = [(rvec, tvec)]
                    try:
                        roots.append(cv2.solvePnPRefineLM(obj, pix, intr, dist, rvec.copy(), tvec.copy()))
                    except cv2.error:
                        pass
                    for rv, tv in roots:
                        item = candidate(world, image, intr, dist, rv, tv, fit)
                        if item is not None and len(item["inlier"]) >= count:
                            candidates.append(item)
            except cv2.error:
                continue
    if not candidates:
        return None, "pnp_no_pose_within_reprojection_limits", {}
    candidates.sort(key=lambda x: (-len(x["inlier"]), x["rms_px"]))
    best = candidates[0]
    best_ids = [ids[i] for i in best["inlier"]]
    diagnostics = {
        "rms_px": best["rms_px"],
        "inlier_ids": best_ids,
        "residual_px_by_id": dict(zip(ids, best["errors"].tolist())),
        "projected_pixels_by_id": dict(zip(ids, best["projected"].tolist())),
    }
    if collect_candidates:
        diagnostics["qualified_candidates"] = [
            dict(center_m=c["center"].tolist(), yaw_deg=c["yaw_deg"], rms_px=c["rms_px"],
                 inlier_ids=[ids[i] for i in c["inlier"]])
            for c in candidates
        ]
    for other in candidates[1:]:
        if (len(other["inlier"]) == len(best_ids)
                and other["rms_px"] - best["rms_px"] <= AMBIGUITY_MARGIN_PX
                and distinct(best, other)):
            diagnostics["alternative_rms_px"] = other["rms_px"]
            return None, "pnp_ambiguous", diagnostics
    if previous is not None and dt is not None:
        # Fixed prototype guardrails, from inter-frame continuity only.
        dt = min(MAX_FRAME_GAP_S, max(0, dt))
        translation_jump = float(np.linalg.norm(best["center"] - previous["center"]))
        rotation_jump = rotation_delta(best["rotation"], previous["rotation"])
        diagnostics.update(position_jump_m=translation_jump, rotation_jump_deg=rotation_jump,
                           yaw_jump_deg=angle_delta(best["yaw_deg"], previous["yaw_deg"]))
        if translation_jump > 0.10 + 1.2 * dt or rotation_jump > 7 + 80 * dt:
            return None, "pose_jump", diagnostics
    return best, "ok", diagnostics
