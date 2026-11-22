#!/usr/bin/env python3

import argparse
import math
import os
import re
import time

import cv2
import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CLASSES = ["LCross", "TCross", "XCross"]
ALIAS_TYPES = {"PenalityPoint": "PenaltyPoint"}

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(ROOT, "models", "marker_best.pt"))
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--camera", default=os.path.join(ROOT, "config", "camera.yaml"))
    ap.add_argument("--extrinsics", default=os.path.join(ROOT, "config", "camera_extrinsics.yaml"))
    ap.add_argument("--device", type=int, default=0)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640, help="推理分辨率，远处小标志用 960")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--source", default="camera:0",
                    help="输入源：camera:0 / image:路径 / video:路径")
    ap.add_argument("--save", default="", help="把左右分屏结果录成 mp4（可选）")
    ap.add_argument("--csv", default="", help="把每帧位姿写到 CSV（可选）")
    ap.add_argument("--dry-run", action="store_true",
                    help="不加载模型，只验证取帧、地面投影和界面")
    ap.add_argument("--no-show", action="store_true", help="不弹窗口（配合 --save 做批量测试）")
    ap.add_argument("--no-track", action="store_true", help="关闭多帧跟踪，用原始检测对比")
    return ap.parse_args()

def load_yaml(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f)

def load_field_markers(field):
    markers = field.get("markers")
    if isinstance(markers, list) and markers:
        return [{"type": ALIAS_TYPES.get(m["type"], m["type"]),
                 "x": float(m["x"]), "y": float(m["y"])} for m in markers]

    out = []
    pattern = re.compile(r"^(LCross|TCross|XCross|PenalityPoint|PenaltyPoint)\d*$")
    for key, val in field.items():
        if not isinstance(val, (list, tuple)) or len(val) != 2:
            continue
        m = pattern.match(str(key))
        if not m:
            continue
        t = m.group(1)
        out.append({"type": ALIAS_TYPES.get(t, t), "x": float(val[0]), "y": float(val[1])})
    return out

def _line_set(mask, min_len):
    edges = cv2.Canny(mask, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=18,
                            minLineLength=min_len, maxLineGap=8)
    if lines is None:
        return []
    arr = np.asarray(lines).reshape(-1, 4)
    return [tuple(int(v) for v in row) for row in arr]

def _intersect(l1, l2):
    x1, y1, x2, y2 = l1
    x3, y3, x4, y4 = l2
    d = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4)
    if abs(d) < 1e-6:
        return None
    px = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / d
    py = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / d
    return float(px), float(py)

def _angle_between(l1, l2):
    a1 = math.atan2(l1[3] - l1[1], l1[2] - l1[0])
    a2 = math.atan2(l2[3] - l2[1], l2[2] - l2[0])
    d = abs(math.degrees(a1 - a2)) % 180.0
    return min(d, 180.0 - d)

def _fit_line(segments):
    pts = []
    for x1, y1, x2, y2 in segments:
        pts.append([x1, y1])
        pts.append([x2, y2])
    pts = np.asarray(pts, dtype=np.float32)
    if len(pts) < 2:
        return None
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).reshape(-1)
    n = math.hypot(float(vx), float(vy))
    if n < 1e-6:
        return None
    return float(vx) / n, float(vy) / n, float(x0), float(y0)

def _intersect_fitted(l1, l2):
    vx1, vy1, x1, y1 = l1
    vx2, vy2, x2, y2 = l2
    det = vx1 * vy2 - vy1 * vx2
    if abs(det) < 1e-6:
        return None
    dx, dy = x2 - x1, y2 - y1
    t = (dx * vy2 - dy * vx2) / det
    return x1 + t * vx1, y1 + t * vy1

def _cluster_segments(lines, tol=12.0):
    items = []
    for ln in lines:
        a = math.degrees(math.atan2(ln[3] - ln[1], ln[2] - ln[0])) % 180.0
        length = math.hypot(ln[2] - ln[0], ln[3] - ln[1])
        items.append((a, length, ln))
    items.sort(key=lambda t: -t[1])

    clusters = []
    for a, length, ln in items:
        placed = False
        for cl in clusters:
            da = abs(a - cl["angle"]) % 180.0
            da = min(da, 180.0 - da)
            if da <= tol:
                cl["segs"].append(ln)
                s = sum(math.sin(math.radians(2 * x)) for x in cl["angles"] + [a])
                c = sum(math.cos(math.radians(2 * x)) for x in cl["angles"] + [a])
                cl["angle"] = math.degrees(math.atan2(s, c)) / 2.0 % 180.0
                cl["angles"].append(a)
                placed = True
                break
        if not placed:
            clusters.append({"angle": a, "angles": [a], "segs": [ln]})
    return clusters

def refine_landmark(frame, cls_name, box, prior=None):
    h, w = frame.shape[:2]
    x1 = max(0, min(int(round(box[0])), w - 1))
    y1 = max(0, min(int(round(box[1])), h - 1))
    x2 = max(x1 + 2, min(int(round(box[2])), w))
    y2 = max(y1 + 2, min(int(round(box[3])), h))
    cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0

    roi = frame[y1:y2, x1:x2]
    if roi.size == 0:
        return cx, cy

    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 0, 110), (180, 90, 255))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))
    if cv2.countNonZero(mask) < 10:
        return cx, cy

    if cls_name in ("PenaltyPoint", "XCross"):
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            m = cv2.moments(c)
            if m["m00"] > 0:
                return x1 + m["m10"] / m["m00"], y1 + m["m01"] / m["m00"]
        return cx, cy

    min_len = max(8, min(roi.shape[:2]) // 4)
    lines = _line_set(mask, min_len)
    cands = []
    cx_roi = roi.shape[1] / 2.0
    cy_roi = roi.shape[0] / 2.0
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            if _angle_between(lines[i], lines[j]) < 20.0:
                continue
            p = _intersect(lines[i], lines[j])
            if p is None:
                continue
            px, py = p
            if not (-0.15 * roi.shape[1] <= px <= 1.15 * roi.shape[1] and
                    -0.15 * roi.shape[0] <= py <= 1.15 * roi.shape[0]):
                continue
            dc = math.hypot(px - cx_roi, py - cy_roi)
            cands.append((dc, px, py))

    def prior_in_box(p, margin=4.0):
        return (x1 - margin <= p[0] <= x2 + margin and
                y1 - margin <= p[1] <= y2 + margin)

    if cands:
        if prior is not None:
            pu = prior[0] - x1
            pv = prior[1] - y1
            near = [c for c in cands if math.hypot(c[1] - pu, c[2] - pv) <= 15.0]
            if near:
                best = min(near, key=lambda c: math.hypot(c[1] - pu, c[2] - pv))
                return x1 + best[1], y1 + best[2]
            if prior_in_box(prior):
                return prior[0], prior[1]
            best = min(cands, key=lambda c: c[0])
            return x1 + best[1], y1 + best[2]
        best = min(cands, key=lambda c: c[0])
        return x1 + best[1], y1 + best[2]

    if prior is not None and prior_in_box(prior):
        return prior[0], prior[1]

    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if cnts:
        c = max(cnts, key=cv2.contourArea)
        m = cv2.moments(c)
        if m["m00"] > 0:
            return x1 + m["m10"] / m["m00"], y1 + m["m01"] / m["m00"]
    return cx, cy

def _box_iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    x1, y1 = max(ax1, bx1), max(ay1, by1)
    x2, y2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, x2 - x1), max(0.0, y2 - y1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0

def _box_center(b):
    return (b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0

def dedupe_detections(dets, iou_thresh=0.35):
    out = []
    for d in sorted(dets, key=lambda z: -z["conf"]):
        w = d["box"][2] - d["box"][0]
        h = d["box"][3] - d["box"][1]
        cx, cy = _box_center(d["box"])
        dup = False
        for k in out:
            if k["type"] != d["type"]:
                continue
            if _box_iou(k["box"], d["box"]) > iou_thresh:
                dup = True
                break
            kx, ky = _box_center(k["box"])
            kw = k["box"][2] - k["box"][0]
            kh = k["box"][3] - k["box"][1]
            if (math.hypot(kx - cx, ky - cy) < 0.35 * max(w, h, kw, kh) and
                    abs(math.log((w + 1) / (kw + 1))) < 0.5 and
                    abs(math.log((h + 1) / (kh + 1))) < 0.5):
                dup = True
                break
        if not dup:
            out.append(d)
    return out

class MarkerTracker:

    def __init__(self, alpha=0.45, gate=110.0, max_miss=6, min_hits=2):
        self.alpha = alpha
        self.gate = gate
        self.max_miss = max_miss
        self.min_hits = min_hits
        self.tracks = []
        self._next = 0

    def update(self, dets):
        used = set()
        for d in dets:
            bx1, by1, bx2, by2 = d["box"]
            dcx, dcy = (bx1 + bx2) / 2.0, (by1 + by2) / 2.0
            best = None
            for i, t in enumerate(self.tracks):
                if i in used or t["type"] != d["type"]:
                    continue
                tx1, ty1, tx2, ty2 = t["box"]
                tcx, tcy = (tx1 + tx2) / 2.0, (ty1 + ty2) / 2.0
                dist = math.hypot(tcx - dcx, tcy - dcy)
                if dist <= self.gate and (best is None or dist < best[0]):
                    best = (dist, i)
            if best is not None:
                t = self.tracks[best[1]]
                used.add(best[1])
                a = self.alpha
                t["u"] = a * d["u"] + (1 - a) * t["u"]
                t["v"] = a * d["v"] + (1 - a) * t["v"]
                t["conf"] = a * d["conf"] + (1 - a) * t["conf"]
                t["box"] = d["box"]
                t["hits"] += 1
                t["misses"] = 0
            else:
                self.tracks.append({"id": self._next, "type": d["type"],
                                    "u": d["u"], "v": d["v"], "conf": d["conf"],
                                    "box": d["box"], "hits": 1, "misses": 0})
                self._next += 1
        for i, t in enumerate(self.tracks):
            if i not in used:
                t["misses"] += 1
        self.tracks = [t for t in self.tracks if t["misses"] <= self.max_miss]
        confirmed = [t for t in self.tracks if t["hits"] >= self.min_hits and t["misses"] == 0]
        merged = []
        for t in sorted(confirmed, key=lambda z: (-z["hits"], -z["conf"])):
            tx, ty = _box_center(t["box"])
            if any(abs(t["box"][2] - t["box"][0]) > 0 and
                   u["type"] == t["type"] and
                   math.hypot(_box_center(u["box"])[0] - tx,
                              _box_center(u["box"])[1] - ty) < 30.0
                   for u in merged):
                continue
            merged.append(t)
        return merged

class PoseSmoother:

    def __init__(self, alpha=0.35, max_jump=1.0, max_dyaw=25.0):
        self.alpha = alpha
        self.max_jump = max_jump
        self.max_dyaw = max_dyaw
        self.prev = None

    def update(self, pose):
        if not pose.get("valid"):
            self.prev = None
            return pose
        cur = (pose["x"], pose["y"], pose["yaw_deg"])
        if self.prev is not None:
            px, py, pyaw = self.prev
            dpos = math.hypot(cur[0] - px, cur[1] - py)
            dyaw = abs((cur[2] - pyaw + 180) % 360 - 180)
            if dpos <= self.max_jump and dyaw <= self.max_dyaw:
                if dpos < 0.05 and dyaw < 2.0:
                    out = dict(pose)
                    out["x"], out["y"], out["yaw_deg"] = px, py, pyaw
                    return out
                if dpos < 0.15 and dyaw < 6.0:
                    a = 0.15
                elif dpos < 0.5 and dyaw < 15.0:
                    a = 0.35
                else:
                    a = 0.7
                x = a * cur[0] + (1 - a) * px
                y = a * cur[1] + (1 - a) * py
                diff = ((cur[2] - pyaw + 180) % 360) - 180
                yaw = (pyaw + a * diff) % 360
                pose = dict(pose)
                pose["x"], pose["y"], pose["yaw_deg"] = x, y, yaw
                cur = (x, y, yaw)
        self.prev = cur
        return pose

def pixel_to_ground(u, v, K, dist, R_base_camera, t_base_camera):
    pts = np.array([[[u, v]]], dtype=np.float32)
    und = cv2.undistortPoints(pts, K, dist).reshape(2)
    ray_c = np.array([und[0], und[1], 1.0], dtype=float)
    ray_b = R_base_camera @ ray_c
    O_b = np.asarray(t_base_camera, dtype=float)

    if abs(ray_b[2]) < 1e-6:
        return None
    lam = -O_b[2] / ray_b[2]
    if lam <= 0 or not np.isfinite(lam):
        return None

    P = O_b + lam * ray_b
    if not (-20.0 <= P[0] <= 20.0 and -20.0 <= P[1] <= 20.0):
        return None
    return float(P[0]), float(P[1])

def se2_from_pairs(p1, p2, q1, q2):
    th = math.atan2(q2[1] - q1[1], q2[0] - q1[0]) - math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    c, s = math.cos(th), math.sin(th)
    tx = q1[0] - (c * p1[0] - s * p1[1])
    ty = q1[1] - (s * p1[0] + c * p1[1])
    return th, tx, ty

def apply_se2(points, th, tx, ty):
    c, s = math.cos(th), math.sin(th)
    return [(c * p[0] - s * p[1] + tx, s * p[0] + c * p[1] + ty) for p in points]

def se2_least_squares(src, dst):
    src = np.asarray(src, dtype=float)
    dst = np.asarray(dst, dtype=float)
    cs = src.mean(axis=0)
    cd = dst.mean(axis=0)
    s = src - cs
    d = dst - cd
    num = float(np.sum(s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]))
    den = float(np.sum(s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]))
    th = math.atan2(num, den)
    c, sn = math.cos(th), math.sin(th)
    tx = cd[0] - (c * cs[0] - sn * cs[1])
    ty = cd[1] - (sn * cs[0] + c * cs[1])
    return th, tx, ty

def _assign(obs_pts, obs_cls, field_markers, th, tx, ty, thresh):
    moved = apply_se2(obs_pts, th, tx, ty)
    used, errs, pairs, mids = [], [], [], []
    for k, (mx, my) in enumerate(moved):
        cands = [(i, m) for i, m in enumerate(field_markers) if m["type"] == obs_cls[k]]
        if not cands:
            continue
        bi, best = min(cands, key=lambda im: math.hypot(mx - im[1]["x"], my - im[1]["y"]))
        d = math.hypot(mx - best["x"], my - best["y"])
        if d < thresh:
            used.append(k)
            errs.append(d)
            pairs.append((obs_pts[k], (best["x"], best["y"])))
            mids.append(bi)
    return used, errs, pairs, mids

def add_virtual_center(obs):
    xs = [o for o in obs if o["type"] == "XCross"]
    if len(xs) < 2:
        return obs
    best = None
    for i in range(len(xs)):
        for j in range(i + 1, len(xs)):
            d = math.hypot(xs[i]["x"] - xs[j]["x"], xs[i]["y"] - xs[j]["y"])
            if best is None or d < best[0]:
                best = (d, xs[i], xs[j])
    if best is None or best[0] < 1.0 or best[0] > 3.0:
        return obs
    a, b = best[1], best[2]
    obs = list(obs)
    obs.append({"type": "PenaltyPoint",
                "x": (a["x"] + b["x"]) / 2.0,
                "y": (a["y"] + b["y"]) / 2.0,
                "conf": min(a.get("conf", 1.0), b.get("conf", 1.0)) * 0.9,
                "virtual": True})
    return obs

def solve_pose(obs, field_markers, inlier_thresh=0.3, min_inliers=3, conf_thresh=0.3,
               prior=None):
    obs = [o for o in obs if o.get("conf", 1.0) >= conf_thresh]
    if len(obs) < min_inliers:
        return {"valid": False, "status": "insufficient_markers",
                "x": 0.0, "y": 0.0, "yaw_deg": 0.0,
                "marker_count": len(obs), "residual": 0.0}
    if not field_markers:
        return {"valid": False, "status": "no_map",
                "x": 0.0, "y": 0.0, "yaw_deg": 0.0,
                "marker_count": len(obs), "residual": 0.0}

    obs_pts = [(o["x"], o["y"]) for o in obs]
    obs_cls = [o["type"] for o in obs]

    candidates = []
    for i in range(len(obs)):
        for j in range(i + 1, len(obs)):
            for a in field_markers:
                if a["type"] != obs_cls[i]:
                    continue
                for b in field_markers:
                    if b["type"] != obs_cls[j] or a is b:
                        continue
                    th, tx, ty = se2_from_pairs(obs_pts[i], obs_pts[j],
                                                (a["x"], a["y"]), (b["x"], b["y"]))
                    used, errs, pairs, mids = _assign(obs_pts, obs_cls, field_markers, th, tx, ty, inlier_thresh)
                    if len(used) < min_inliers:
                        continue
                    th2, tx2, ty2 = se2_least_squares([p[0] for p in pairs], [p[1] for p in pairs])
                    used2, errs2, _, mids2 = _assign(obs_pts, obs_cls, field_markers, th2, tx2, ty2, inlier_thresh)
                    if len(used2) < min_inliers:
                        th2, tx2, ty2 = th, tx, ty
                        used2, errs2, mids2 = used, errs, mids
                    candidates.append({
                        "th": th2, "tx": tx2, "ty": ty2,
                        "n": len(used2), "resid": float(np.mean(errs2)) if errs2 else 99.0,
                        "used": used2, "mids": mids2,
                    })

    if not candidates:
        return {"valid": False, "status": "not_converged",
                "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "marker_count": 0, "residual": 0.0}

    uniq = []
    for c in sorted(candidates, key=lambda c: (c["n"], -c["resid"]), reverse=True):
        dup = False
        for u in uniq:
            if (math.hypot(c["tx"] - u["tx"], c["ty"] - u["ty"]) < 0.25 and
                    abs(math.degrees(c["th"] - u["th"])) % 360 < 8.0):
                dup = True
                break
        if not dup:
            uniq.append(c)

    best = uniq[0]
    status = "ok"
    if len(uniq) > 1:
        second = uniq[1]
        if (second["n"] >= best["n"] and
                abs(second["resid"] - best["resid"]) < 0.08 and
                (math.hypot(second["tx"] - best["tx"], second["ty"] - best["ty"]) > 0.3 or
                 abs(math.degrees(second["th"] - best["th"])) % 360 > 8.0)):
            status = "ambiguous"

    if status == "ambiguous" and prior is not None:
        px, py, pyaw = prior
        pool = [c for c in uniq
                if c["n"] >= best["n"] and abs(c["resid"] - best["resid"]) < 0.15]
        if pool:
            def pdist(c):
                d = math.hypot(c["tx"] - px, c["ty"] - py)
                da = abs(math.degrees(c["th"] - math.radians(pyaw)) % 360)
                da = min(da, 360.0 - da)
                return d + 0.02 * da
            chosen = min(pool, key=pdist)
            if chosen is not best:
                best = chosen
            status = "ok"

    xs = [m["x"] for m in field_markers]
    ys = [m["y"] for m in field_markers]
    margin = 0.6
    if not (min(xs) - margin <= best["tx"] <= max(xs) + margin and
            min(ys) - margin <= best["ty"] <= max(ys) + margin):
        status = "out_of_field"

    matches = []
    for k, mid in zip(best.get("used", []), best.get("mids", [])):
        if 0 <= mid < len(field_markers):
            matches.append({"obs_index": int(k), "map_id": int(mid),
                            "type": obs[k]["type"]})

    return {
        "valid": status == "ok",
        "status": status,
        "x": float(best["tx"]),
        "y": float(best["ty"]),
        "yaw_deg": float(math.degrees(best["th"]) % 360.0),
        "marker_count": int(best["n"]),
        "residual": float(best["resid"]),
        "matches": matches,
    }

_WINDOW_READY = set()

def show_window(name, img):
    if name not in _WINDOW_READY:
        cv2.namedWindow(name, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
        cv2.moveWindow(name, 0, 0)
        _WINDOW_READY.add(name)
    cv2.imshow(name, img)

def draw_map(size, field_markers, pose, scale, origin):
    canvas = np.full((size, size, 3), 245, np.uint8)

    def to_px(x, y):
        return int(origin[0] + x * scale), int(origin[1] - y * scale)

    p0 = to_px(0.0, -2.977)
    p1 = to_px(3.968, 2.977)
    cv2.rectangle(canvas, p0, p1, (0, 0, 0), 2)
    cv2.line(canvas, to_px(3.968, -2.977), to_px(3.968, 2.977), (0, 160, 0), 4)
    cv2.putText(canvas, "goal", (p1[0] - 40, p0[1] + 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 140, 0), 2)
    cv2.circle(canvas, to_px(0.0, 0.0), int(0.731 * scale), (0, 0, 0), 2)

    ax, ay = 40, size - 30
    cv2.arrowedLine(canvas, (ax, ay), (ax + 60, ay), (0, 0, 0), 2, tipLength=0.2)
    cv2.arrowedLine(canvas, (ax, ay), (ax, ay - 60), (0, 0, 0), 2, tipLength=0.2)
    cv2.putText(canvas, "+X", (ax + 64, ay + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.putText(canvas, "+Y", (ax - 12, ay - 64), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)

    cv2.putText(canvas, "field.yaml  C0-2026-11-21", (10, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (90, 90, 90), 1)

    for m in field_markers:
        px, py = to_px(m["x"], m["y"])
        cv2.circle(canvas, (px, py), 4, (0, 128, 255), -1)
        cv2.putText(canvas, m["type"][:2], (px + 5, py - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 128, 255), 1)

    if pose["valid"]:
        px, py = to_px(pose["x"], pose["y"])
        yaw = math.radians(pose["yaw_deg"])
        ex = int(px + 40 * math.cos(yaw))
        ey = int(py - 40 * math.sin(yaw))
        cv2.arrowedLine(canvas, (px, py), (ex, ey), (0, 0, 255), 2, tipLength=0.25)
        cv2.circle(canvas, (px, py), 6, (0, 0, 255), -1)

    return canvas

class Frames:

    def __init__(self, spec):
        kind, _, arg = spec.partition(":")
        if spec.startswith("http"):
            kind, arg = "url", spec
        self.kind = kind
        self.cap = None
        self.single = None
        self.used = False

        if kind == "image":
            img = cv2.imread(arg)
            if img is None:
                raise SystemExit(f"读不了图片：{arg}")
            self.single = img
        elif kind == "url":
            self.cap = cv2.VideoCapture(arg, cv2.CAP_FFMPEG)
            if not self.cap.isOpened():
                raise SystemExit(f"打不开网络流：{arg}")
        elif kind == "video":
            self.cap = cv2.VideoCapture(arg)
            if not self.cap.isOpened():
                raise SystemExit(f"打不开视频：{arg}")
        elif kind == "camera":
            self.cap = cv2.VideoCapture(int(arg), cv2.CAP_V4L2)
            if not self.cap.isOpened():
                raise SystemExit(f"打不开 /dev/video{arg}")
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        else:
            raise SystemExit(f"不认识的 --source：{spec}（应为 camera:/image:/video:）")

    def read(self):
        if self.kind == "image":
            if self.used:
                return False, None
            self.used = True
            return True, self.single.copy()
        return self.cap.read()

    def size(self):
        if self.kind == "image":
            h, w = self.single.shape[:2]
        else:
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        return w, h

    def release(self):
        if self.cap is not None:
            self.cap.release()

def main():
    args = parse_args()
    model = None
    if not args.dry_run:
        if not os.path.exists(args.model):
            print(f"[错误] 找不到标志模型 {args.model}，先训练或拷 best.pt 过来")
            return 1
        try:
            from ultralytics import YOLO
        except ImportError:
            print("[错误] 没装 ultralytics")
            return 1
        model = YOLO(args.model)

    cam = load_yaml(args.camera) or {}
    extr = load_yaml(args.extrinsics) or {}
    field = load_yaml(args.field) or {}

    try:
        K = np.array(cam["camera_matrix"], dtype=float)
        dist = np.array(cam["dist_coeffs"], dtype=float)
        R = np.array(extr["R_base_camera"], dtype=float)
        t = np.array(extr["t_base_camera"], dtype=float)
    except KeyError as e:
        print(f"[错误] 配置文件缺少字段 {e}，先运行 scripts/check_config.py")
        return 1
    markers = load_field_markers(field)
    if not markers:
        print("[错误] field.yaml 里没有可识别的标志点（简化格式和官方格式都没解析到）")
        return 1
    print(f"已加载 {len(markers)} 个场地标志点")

    if np.allclose(R, np.eye(3)) and np.allclose(t, 0):
        print("[警告] 外参还是占位值，地面坐标没有意义，先去量外参")

    frames = Frames(args.source)
    if args.source.startswith("camera"):
        frames.cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        frames.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 20.0, (1280, 720))
        print(f"结果将保存到 {args.save}")

    csv_f = None
    if args.csv:
        csv_f = open(args.csv, "w", encoding="utf-8")
        csv_f.write("name,x,y,yaw_deg,valid,status,marker_count,residual\n")
        print(f"位姿将写到 {args.csv}")

    xs = [m["x"] for m in markers] or [0.0]
    ys = [m["y"] for m in markers] or [0.0]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    scale = 560.0 / span
    field_w = (max(xs) - min(xs)) * scale
    origin = ((640 - field_w) / 2 - min(xs) * scale, 600 + min(ys) * scale)

    frame_id = 0
    tracker = None if args.no_track else MarkerTracker()
    smoother = PoseSmoother()
    anchor = None
    locked_pose = None
    hold_count = 0
    t_prev = time.time()
    fps = 0.0
    while True:
        ok, frame = frames.read()
        if not ok:
            break
        frame_id += 1
        now = time.time()
        fps = 1.0 / max(now - t_prev, 1e-6)
        t_prev = now

        obs = []
        det_infos = []
        if model is not None:
            results = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
            raw = []
            for box in results[0].boxes:
                name = results[0].names.get(int(box.cls[0]), "")
                if name not in CLASSES:
                    continue
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                prior = None
                if tracker is not None:
                    cx0 = (x1 + x2) / 2.0
                    cy0 = (y1 + y2) / 2.0
                    bd = None
                    for trc in tracker.tracks:
                        if trc["type"] != name:
                            continue
                        tx1, ty1, tx2, ty2 = trc["box"]
                        dd = math.hypot((tx1 + tx2) / 2.0 - cx0, (ty1 + ty2) / 2.0 - cy0)
                        if dd < 110.0 and (bd is None or dd < bd[0]):
                            bd = (dd, trc)
                    if bd is not None:
                        trc = bd[1]
                        tx1, ty1, tx2, ty2 = trc["box"]
                        tcx, tcy = (tx1 + tx2) / 2.0, (ty1 + ty2) / 2.0
                        prior = (trc["u"] + (cx0 - tcx), trc["v"] + (cy0 - tcy))
                u, v = refine_landmark(frame, name, (x1, y1, x2, y2), prior=prior)
                raw.append({"type": name, "conf": conf, "u": u, "v": v,
                            "box": (x1, y1, x2, y2), "id": -1})
                cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (110, 110, 110), 1)

            raw = dedupe_detections(raw)
            for tr in (raw if tracker is None else tracker.update(raw)):
                name = tr["type"]
                conf = tr["conf"]
                u, v = tr["u"], tr["v"]
                x1, y1, x2, y2 = [int(z) for z in tr["box"]]
                ground = pixel_to_ground(u, v, K, dist, R, t)
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.circle(frame, (int(u), int(v)), 3, (0, 0, 255), -1)
                cv2.putText(frame, f"{name} {conf:.2f} #{tr['id']}",
                            (x1, max(y1 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                info = {"name": name, "conf": conf, "u": u, "v": v,
                        "obs_index": -1}
                if ground is not None:
                    info["obs_index"] = len(obs)
                    obs.append({"type": name, "x": ground[0], "y": ground[1],
                                "conf": conf})
                det_infos.append(info)

        pix_text = ""
        for info in det_infos:
            k = info["obs_index"]
            if k < 0:
                continue
            g = obs[k]
            pix_text = "%s pix(%.0f,%.0f) -> base(%.2f,%.2f) m" % (
                info["name"], info["u"], info["v"], g["x"], g["y"])
            cv2.putText(frame, pix_text, (10, frame.shape[0] - 12),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
            break

        obs = add_virtual_center(obs)
        pose = solve_pose(obs, markers, prior=anchor)
        if anchor is None and pose.get("valid") and pose.get("marker_count", 0) < 4:
            pose = dict(pose)
            pose["valid"] = False
            pose["status"] = "ambiguous"
        pose = smoother.update(pose)
        if pose.get("valid"):
            if locked_pose is None:
                locked_pose = (pose["x"], pose["y"], pose["yaw_deg"])
            else:
                lx, ly, lyaw = locked_pose
                dpos = math.hypot(pose["x"] - lx, pose["y"] - ly)
                dyaw = abs((pose["yaw_deg"] - lyaw + 180) % 360 - 180)
                if dpos < 0.15 and dyaw < 5.0:
                    hold_count = 0
                    pose = dict(pose)
                    pose["x"], pose["y"], pose["yaw_deg"] = lx, ly, lyaw
                else:
                    hold_count += 1
                    if hold_count >= 4:
                        locked_pose = (pose["x"], pose["y"], pose["yaw_deg"])
                        hold_count = 0
                    else:
                        pose = dict(pose)
                        pose["x"], pose["y"], pose["yaw_deg"] = lx, ly, lyaw

        if pose.get("valid"):
            if anchor is None:
                if pose.get("marker_count", 0) >= 4:
                    anchor = (pose["x"], pose["y"], pose["yaw_deg"])
            else:
                ax, ay, ayaw = anchor
                dpos = math.hypot(pose["x"] - ax, pose["y"] - ay)
                dyaw = abs((pose["yaw_deg"] - ayaw + 180) % 360 - 180)
                if dpos > 0.15 or dyaw > 5.0:
                    anchor = (pose["x"], pose["y"], pose["yaw_deg"])
        for info in det_infos:
            k = info["obs_index"]
            if k < 0:
                continue
            mid = None
            for m in pose.get("matches", []):
                if m["obs_index"] == k:
                    mid = m["map_id"]
                    break
            if mid is not None:
                cv2.putText(frame, f"->#{mid}", (int(info["u"]) + 6, int(info["v"]) + 16),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        if csv_f is not None:
            tag = os.path.splitext(os.path.basename(args.source.split(":")[-1] if ":" in args.source else args.source))[0]
            csv_f.write("%s_%04d,%.4f,%.4f,%.2f,%d,%s,%d,%.4f\n" % (
                tag, frame_id, pose.get("x", 0.0), pose.get("y", 0.0),
                pose.get("yaw_deg", 0.0), 1 if pose.get("valid") else 0,
                pose.get("status", ""), pose.get("marker_count", 0),
                pose.get("residual", 0.0)))
        canvas = draw_map(640, markers, pose, scale, origin)

        left = cv2.resize(frame, (576, 324))
        panel = np.zeros((324, 576, 3), np.uint8)
        lines = [
            "frame %d   fps %.1f" % (frame_id, fps),
            "valid %s   status %s" % (pose.get("valid"), pose.get("status", "")),
            "x %.3f  y %.3f  yaw %.1f" % (pose.get("x", 0.0), pose.get("y", 0.0),
                                          pose.get("yaw_deg", 0.0)),
            "markers %d   residual %.3f" % (pose.get("marker_count", 0),
                                            pose.get("residual", 0.0)),
            pix_text,
        ]
        for i, txt in enumerate(lines):
            cv2.putText(panel, txt, (12, 48 + i * 54), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2)
        left_col = np.vstack([left, panel])
        right = np.zeros((648, 576, 3), np.uint8)
        right[36:612, :] = cv2.resize(canvas, (576, 576))
        view = np.hstack([left_col, right])

        if writer is not None:
            writer.write(view)
        if not args.no_show:
            show_window("field locate (left: rgb, right: map)", view)
            key = cv2.waitKey(0 if frames.kind == "image" else 1) & 0xFF
            if key in (ord("q"), 27):
                break
        elif frames.kind == "image":
            break

    frames.release()
    if csv_f is not None:
        csv_f.close()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
