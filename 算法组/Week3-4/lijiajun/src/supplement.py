import math

import cv2
import numpy as np

from p3p import solve_dual

_CLS = {"LCross": 0, "TCross": 1, "XCross": 2}


def _adaptive_field(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    H, W = img.shape[:2]
    hh, ss, vv = hsv[:, :, 0].astype(int), hsv[:, :, 1].astype(int), hsv[:, :, 2].astype(int)
    r0, r1, c0, c1 = int(0.55 * H), int(0.95 * H), int(0.15 * W), int(0.85 * W)
    sub = (slice(r0, r1), slice(c0, c1))
    sat = (ss[sub] > 25) & (vv[sub] > 45)
    if sat.sum() < 50:
        return None
    hues = hh[sub][sat]
    hmed = float(np.median(hues))
    hlo, hhi = (hmed - 28) % 180, (hmed + 28) % 180
    slo = max(25, int(np.percentile(ss[sub][sat], 15)))
    vlo = max(40, int(np.percentile(vv[sub][sat], 10)))
    hm = ((hh >= hlo) & (hh <= hhi)) if hlo <= hhi else ((hh >= hlo) | (hh <= hhi))
    field = (hm & (ss >= slo) & (vv >= vlo)).astype(np.uint8) * 255
    field = cv2.morphologyEx(field, cv2.MORPH_CLOSE, np.ones((45, 45), np.uint8))
    field = cv2.morphologyEx(field, cv2.MORPH_OPEN, np.ones((15, 15), np.uint8))
    return field


def _normal(x1, y1, x2, y2):
    th = math.atan2(y2 - y1, x2 - x1)
    if th < 0:
        th += math.pi
    nx, ny = -math.sin(th), math.cos(th)
    return th, nx, ny, x1 * nx + y1 * ny


def _merge(segs, dang=3.0, drho=14.0):
    groups = []
    for s in segs:
        th, nx, ny, rho = _normal(*s)
        for g in groups:
            dth = abs(th - g["th"])
            dth = min(dth, math.pi - dth)
            if dth < math.radians(dang) and abs(rho - g["rho"]) < drho:
                g["pts"] += [s[:2], s[2:]]
                g["th"], g["rho"] = th, rho
                break
        else:
            groups.append(dict(th=th, rho=rho, pts=[s[:2], s[2:]]))
    out = []
    for g in groups:
        P = np.array(g["pts"], np.float32)
        vx, vy, x0, y0 = cv2.fitLine(P, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
        d = np.array([vx, vy])
        t = (P - [x0, y0]) @ d
        if t.max() - t.min() < 90:
            continue
        out.append((np.array([x0, y0]) + d * t.min(), np.array([x0, y0]) + d * t.max()))
    return out


def line_intersections(img):
    H, W = img.shape[:2]
    field = _adaptive_field(img)
    if field is None:
        return []
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    top = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (19, 19)))
    roi = cv2.erode(field, np.ones((19, 19), np.uint8))
    vals = top[roi > 0]
    if vals.size < 50:
        return []
    thr = max(float(np.percentile(vals, 96)), 18)
    white = ((top > thr).astype(np.uint8) * 255) & roi
    white = cv2.morphologyEx(white, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    segs = cv2.HoughLinesP(white, 1, np.pi / 180, 35, minLineLength=45, maxLineGap=16)
    segs = [] if segs is None else [tuple(int(v) for v in np.ravel(s)[:4]) for s in segs]
    lines = _merge(segs)
    pts = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            da = lines[i][1] - lines[i][0]
            db = lines[j][1] - lines[j][0]
            a1 = np.degrees(np.arctan2(da[1], da[0])) % 180
            a2 = np.degrees(np.arctan2(db[1], db[0])) % 180
            if min(abs(a1 - a2), 180 - abs(a1 - a2)) < 20:
                continue
            d = da[0] * db[1] - da[1] * db[0]
            if abs(d) < 1e-6:
                continue
            t = ((lines[j][0][0] - lines[i][0][0]) * db[1] -
                 (lines[j][0][1] - lines[i][0][1]) * db[0]) / d
            p = lines[i][0] + t * da
            if 0 <= p[0] < W and 0 <= p[1] < H and roi[int(p[1]), int(p[0])] > 0:
                pts.append((float(p[0]), float(p[1])))
    return pts


def supplement_points(px, img, K, dist, obj, names, by_cls, gate=22.0, near=30.0):
    s = solve_dual(px, K, dist, obj, names, by_cls)
    if s is None:
        return px
    inter = line_intersections(img)
    if not inter:
        return px
    proj, _ = cv2.projectPoints(obj, s["rvec"], s["tvec"], K, dist)
    proj = proj.reshape(-1, 2)
    H, W = img.shape[:2]
    out = list(px)
    for i, nm in enumerate(names):
        u, v = proj[i]
        if not (0 <= u < W and 0 <= v < H):
            continue
        if any(math.hypot(p[1] - u, p[2] - v) < near for p in px):
            continue
        ds = [math.hypot(x - u, y - v) for x, y in inter]
        k = int(np.argmin(ds))
        if ds[k] < gate:
            x, y = inter[k]
            out.append((_CLS[nm[:6]], float(x), float(y), 0.1))
    return out