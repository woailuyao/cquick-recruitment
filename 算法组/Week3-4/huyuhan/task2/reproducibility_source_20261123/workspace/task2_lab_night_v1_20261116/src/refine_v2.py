"""Isolated offline candidate refiner: expanded ROIs, shadow-tolerant masks, consensus."""
import cv2
import numpy as np


def _mask_variants(roi):
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    s, v = hsv[..., 1], hsv[..., 2]
    low_s = s <= 130
    vals = v[low_s]
    adaptive_cut = max(70, int(np.percentile(vals, 62))) if vals.size else 120
    strict = ((s <= 105) & (v >= 135)).astype(np.uint8) * 255
    shadow = (low_s & (v >= adaptive_cut)).astype(np.uint8) * 255
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    local = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                  cv2.THRESH_BINARY, 31, -3)
    local = cv2.bitwise_and(local, (s <= 145).astype(np.uint8) * 255)
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
    out = []
    for m in (strict, shadow, local):
        m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(m)
        keep = np.zeros_like(m)
        for i in range(1, n):
            if stats[i, cv2.CC_STAT_AREA] >= 8:
                keep[labels == i] = 255
        out.append(keep)
    return out


def _support(mask, point, direction):
    r = np.arange(5.0, max(11.0, min(mask.shape) * .34))
    if len(r) < 5:
        return 0.0
    p = point + r[:, None] * direction
    normal = np.array([-direction[1], direction[0]])
    scores = []
    for offset in (-2, -1, 0, 1, 2):
        q = p + offset * normal
        xx = np.rint(q[:, 0]).astype(int)
        yy = np.rint(q[:, 1]).astype(int)
        inside = (xx >= 0) & (xx < mask.shape[1]) & (yy >= 0) & (yy < mask.shape[0])
        if inside.sum() >= 4:
            scores.append(float((mask[yy[inside], xx[inside]] > 0).mean()))
    return max(scores, default=0.0)


def _fit(mask, cls):
    skeleton = np.zeros_like(mask)
    work = mask.copy()
    element = cv2.getStructuringElement(cv2.MORPH_CROSS, (3, 3))
    for _ in range(max(mask.shape)):
        eroded = cv2.erode(work, element)
        skeleton |= cv2.subtract(work, cv2.dilate(eroded, element))
        work = eroded
        if cv2.countNonZero(work) == 0:
            break
    size = min(mask.shape)
    segments = cv2.HoughLinesP(skeleton, 1, np.pi / 360,
                               threshold=max(6, size // 8),
                               minLineLength=max(8, size // 5),
                               maxLineGap=max(6, size // 12))
    if segments is None:
        return None
    seg = np.asarray(segments).reshape(-1, 4).astype(float)
    lengths = np.linalg.norm(seg[:, 2:] - seg[:, :2], axis=1)
    seg = seg[np.argsort(-lengths)[:20]]
    center = np.array([mask.shape[1] / 2, mask.shape[0] / 2], float)
    required = {0: 2, 1: 3, 2: 4}.get(cls, 4)
    candidates = []
    for ai, a in enumerate(seg):
        va = a[2:] - a[:2]
        va /= max(np.linalg.norm(va), 1e-9)
        na = np.array([-va[1], va[0]])
        ca = float(a[:2] @ na)
        for b in seg[ai + 1:]:
            vb = b[2:] - b[:2]
            vb /= max(np.linalg.norm(vb), 1e-9)
            cross = abs(float(np.linalg.det(np.column_stack([va, vb]))))
            if cross < .30:
                continue
            nb = np.array([-vb[1], vb[0]])
            try:
                point = np.linalg.solve(np.array([na, nb]), np.array([ca, float(b[:2] @ nb)]))
            except np.linalg.LinAlgError:
                continue
            if np.any(point < 2) or point[0] > mask.shape[1] - 3 or point[1] > mask.shape[0] - 3:
                continue
            if np.linalg.norm((point - center) / np.maximum(center, 1)) > 1.05:
                continue
            support = [_support(mask, point, d) for d in (va, -va, vb, -vb)]
            arms = sum(x >= .43 for x in support)
            if arms < required:
                continue
            ordered = sorted(support, reverse=True)
            score = (float(np.linalg.norm((point - center) / np.maximum(center, 1)))
                     - .12 * cross - .08 * sum(ordered[:required]))
            candidates.append((score, point))
    if not candidates:
        return None
    candidates.sort(key=lambda x: x[0])
    best_score, best = candidates[0]
    if any(score < best_score + .06 and np.linalg.norm(point - best) > max(8, size * .12)
           for score, point in candidates[1:]):
        return None
    return best


def refine_v2(image, box, cls, return_debug=False):
    h, w = image.shape[:2]
    x1, y1, x2, y2 = map(float, box)
    bw, bh = max(1.0, x2 - x1), max(1.0, y2 - y1)
    fits = []
    debug = []
    for margin in (.08, .22, .38):
        ax1 = max(0, int(np.floor(x1 - margin * bw)))
        ay1 = max(0, int(np.floor(y1 - margin * bh)))
        ax2 = min(w, int(np.ceil(x2 + margin * bw)))
        ay2 = min(h, int(np.ceil(y2 + margin * bh)))
        roi = image[ay1:ay2, ax1:ax2]
        if min(roi.shape[:2], default=0) < 12:
            continue
        for mode, mask in enumerate(_mask_variants(roi)):
            point = _fit(mask, cls)
            if point is None:
                debug.append({"margin": margin, "mode": mode, "fit": None})
                continue
            full = point + [ax1, ay1]
            fits.append((margin, mode, full))
            debug.append({"margin": margin, "mode": mode, "fit": full.tolist()})
    if len(fits) < 2:
        return (None, debug) if return_debug else None
    tolerance = max(7.0, min(bw, bh) * .14)
    clusters = []
    for item in fits:
        placed = False
        for cluster in clusters:
            center = np.median([x[2] for x in cluster], axis=0)
            if np.linalg.norm(item[2] - center) <= tolerance:
                cluster.append(item)
                placed = True
                break
        if not placed:
            clusters.append([item])
    clusters.sort(key=lambda c: (-len({(x[0], x[1]) for x in c}),
                                 np.mean([np.linalg.norm(x[2] - np.median([y[2] for y in c], axis=0)) for x in c])))
    best = clusters[0]
    if len({x[0] for x in best}) < 2:
        return (None, debug) if return_debug else None
    center = np.median([x[2] for x in best], axis=0)
    if len(clusters) > 1 and len(clusters[1]) >= len(best) and np.linalg.norm(
            np.median([x[2] for x in clusters[1]], axis=0) - center) > tolerance:
        return (None, debug) if return_debug else None
    result = center.tolist()
    return (result, debug) if return_debug else result
