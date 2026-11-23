import argparse
import itertools
import json
import math
import os
import time
from pathlib import Path

import cv2
import numpy as np

import ground_projection as GP


def bearings(pixels, K, dist):
    p = GP.undistort_pixels(pixels, K, dist)
    b = np.hstack([p, np.ones((len(p), 1))]) @ np.linalg.inv(K).T
    return b / np.linalg.norm(b, axis=1, keepdims=True)


def _pair(s, u, D12, D13, D23, a, b, c):
    fA = D12 ** 2 * (1 + u * u - 2 * a * u) - D13 ** 2 * (1 + s * s - 2 * b * s)
    fB = D12 ** 2 * (s * s + u * u - 2 * c * s * u) - D23 ** 2 * (1 + s * s - 2 * b * s)
    return fA, fB


def _quad_roots(A2, A1, A0):
    if abs(A2) < 1e-12:
        return [] if abs(A1) < 1e-12 else [-A0 / A1]
    disc = A1 * A1 - 4 * A2 * A0
    if disc < 0:
        return []
    r = math.sqrt(disc)
    return [(-A1 + r) / (2 * A2), (-A1 - r) / (2 * A2)]


def _branch_u(s, D12, D13, D23, a, b, c):
    A2 = D12 ** 2
    A1 = -2 * a * D12 ** 2
    A0 = D12 ** 2 - D13 ** 2 * (1 + s * s - 2 * b * s)
    return [u for u in _quad_roots(A2, A1, A0) if u > 0]


def _newton(s, u, D12, D13, D23, a, b, c, tol=1e-13, iters=80):
    h = 1e-7
    for _ in range(iters):
        fA, fB = _pair(s, u, D12, D13, D23, a, b, c)
        gA = [_pair(s * (1 + h), u, D12, D13, D23, a, b, c)[0],
              _pair(s, u * (1 + h), D12, D13, D23, a, b, c)[0]]
        gB = [_pair(s * (1 + h), u, D12, D13, D23, a, b, c)[1],
              _pair(s, u * (1 + h), D12, D13, D23, a, b, c)[1]]
        J = np.array([[(gA[0] - fA) / (s * h), (gA[1] - fA) / (u * h)],
                      [(gB[0] - fB) / (s * h), (gB[1] - fB) / (u * h)]])
        try:
            step = np.linalg.solve(J, [-fA, -fB])
        except np.linalg.LinAlgError:
            return None
        ns, nu = s + step[0], u + step[1]
        if ns <= 0 or nu <= 0 or not np.isfinite([ns, nu]).all():
            return None
        s, u = float(ns), float(nu)
        if abs(fA) + abs(fB) < tol * max(D12 ** 2, 1.0):
            break
    fA, fB = _pair(s, u, D12, D13, D23, a, b, c)
    scale = max(D12 ** 2, D13 ** 2, D23 ** 2, 1.0)
    return (s, u) if max(abs(fA), abs(fB)) < 1e-7 * scale else None


def _solve_ratios(D12, D13, D23, a, b, c, n=2000, lo=0.02, hi=300.0):
    grid = np.exp(np.linspace(math.log(lo), math.log(hi), n))
    seeds = []
    prev = []
    for s in grid:
        cur = [(u, _pair(s, u, D12, D13, D23, a, b, c)[1])
               for u in _branch_u(s, D12, D13, D23, a, b, c)]
        for i in range(min(len(cur), len(prev))):
            if cur[i][1] * prev[i][1] <= 0:
                seeds.append((s, cur[i][0]))
        prev = cur
    out = []
    for s0, u0 in seeds:
        r = _newton(s0, u0, D12, D13, D23, a, b, c)
        if r and not any(abs(r[0] - o[0]) < 1e-6 and abs(r[1] - o[1]) < 1e-6 for o in out):
            out.append(r)
    return out


def _kabsch(p_world, p_cam):
    mw, mc = p_world.mean(0), p_cam.mean(0)
    H = (p_world - mw).T @ (p_cam - mc)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, mc - R @ mw


def solve_p3p(points3d, pixel_pts, K, dist, height_prior=None):
    P = np.asarray(points3d, float).reshape(-1, 2)
    if len(P) != 3:
        return []
    P3 = np.column_stack([P[:, 0], P[:, 1], np.zeros(3)])
    f = bearings(pixel_pts, K, dist)
    D12 = float(np.linalg.norm(P3[0] - P3[1]))
    D13 = float(np.linalg.norm(P3[0] - P3[2]))
    D23 = float(np.linalg.norm(P3[1] - P3[2]))
    if min(D12, D13, D23) < 1e-6:
        return []
    cr = (P3[1, 0] - P3[0, 0]) * (P3[2, 1] - P3[0, 1]) - \
         (P3[1, 1] - P3[0, 1]) * (P3[2, 0] - P3[0, 0])
    dmax = max(D12, D13, D23)
    cond = abs(cr) / 2 / dmax ** 2
    if cond < 0.15:
        return []
    a = float(np.clip(f[0] @ f[2], -1, 1))
    b = float(np.clip(f[0] @ f[1], -1, 1))
    c = float(np.clip(f[1] @ f[2], -1, 1))
    sols = []
    for s, u in _solve_ratios(D12, D13, D23, a, b, c):
        den = 1 + s * s - 2 * b * s
        if den <= 1e-9:
            continue
        t1 = D12 / math.sqrt(den)
        p_cam = np.vstack([f[0] * t1, f[1] * (s * t1), f[2] * (u * t1)])
        R, t = _kabsch(P3, p_cam)
        C = -R.T @ t
        axis = R.T @ np.array([0.0, 0.0, 1.0])
        res = float(np.sqrt(np.mean(np.sum((p_cam - ((R @ P3.T).T + t)) ** 2, axis=1))))
        sols.append(dict(R=R, t=t, C=C, residual_m=res, cond=cond,
                         yaw=math.degrees(math.atan2(axis[1], axis[0])),
                         pitch=math.degrees(math.asin(np.clip(-axis[2], -1, 1)))))
    keep = [x for x in sols if x["C"][2] > 0.02]
    if not keep:
        return []
    if height_prior is not None:
        for x in keep:
            x["pick_score"] = abs(x["C"][2] - height_prior) / height_prior
        keep.sort(key=lambda x: x["pick_score"])
    else:
        keep.sort(key=lambda x: x["residual_m"])
    if len(keep) > 1:
        h0, h1 = keep[0]["C"][2], keep[1]["C"][2]
        keep[0]["separable"] = abs(h0 - h1) / max(h0, h1)
        keep[0]["n_solutions"] = len(keep)
    else:
        keep[0]["separable"] = 1.0
        keep[0]["n_solutions"] = 1
    return keep


_MODEL = None
_W34 = Path(os.environ.get(
    "W34_DATA", Path(__file__).resolve().parent.parent.parents[4] / "week34_data"))
_WEIGHTS = os.environ.get(
    "MARKER_WEIGHTS", str(_W34 / "runs" / "marker_v6" / "weights" / "best.pt"))
_BALL_WEIGHTS = os.environ.get(
    "BALL_WEIGHTS", str(_W34 / "runs" / "ball_v2" / "weights" / "best.pt"))
_BALL_MODEL = None


def _pick(group):
    if len(group) == 1:
        return group[0]
    cls = {g[0] for g in group}
    if 2 in cls:
        return max((g for g in group if g[0] == 2), key=lambda g: g[3])
    if 0 in cls and 1 in cls:
        return max((g for g in group if g[0] == 1), key=lambda g: g[3])
    return max(group, key=lambda g: g[3])


def _dedup(px, radius=25.0):
    items = list(px)
    used = [False] * len(items)
    order = sorted(range(len(items)), key=lambda i: -items[i][3])
    keep = []
    for oi in order:
        if used[oi]:
            continue
        used[oi] = True
        p = items[oi]
        group = [p]
        for oj in order:
            if used[oj]:
                continue
            q = items[oj]
            if (p[1] - q[1]) ** 2 + (p[2] - q[2]) ** 2 < radius * radius:
                group.append(q)
                used[oj] = True
        keep.append(_pick(group))
    return keep


def detect(img, conf=0.25, iou=0.7, imgsz=640, device="0", weights=None):
    global _MODEL
    if _MODEL is None:
        from ultralytics import YOLO
        _MODEL = YOLO(str(weights or _WEIGHTS))
    r = _MODEL.predict(img, imgsz=imgsz, conf=conf, iou=iou, device=device, verbose=False)[0]
    out = []
    for b in r.boxes:
        x1, y1, x2, y2 = b.xyxy[0].tolist()
        out.append((int(b.cls[0]), (x1 + x2) / 2.0, (y1 + y2) / 2.0, float(b.conf[0])))
    return out


def detect_points(img, **kw):
    return _dedup(detect(img, **kw))


def detect_ball(img, conf=0.25, iou=0.7, imgsz=640, device="0"):
    global _BALL_MODEL
    if _BALL_MODEL is None:
        from ultralytics import YOLO
        _BALL_MODEL = YOLO(str(_BALL_WEIGHTS))
    r = _BALL_MODEL.predict(img, imgsz=imgsz, conf=conf, iou=iou, device=device, verbose=False)[0]
    if not len(r.boxes):
        return None
    b = max(r.boxes, key=lambda b: float(b.conf[0]))
    x1, y1, x2, y2 = b.xyxy[0].tolist()
    return dict(u=round((x1 + x2) / 2.0, 1), v=round((y1 + y2) / 2.0, 1),
                conf=round(float(b.conf[0]), 3))


PX_INLIER = 40.0


def _n_assign(px, by_cls):
    g = {}
    for c, _, _, _ in px:
        g[c] = g.get(c, 0) + 1
    total = 1
    for c, k in g.items():
        m = len(by_cls[c])
        if k > m:
            return 10 ** 9
        total *= math.perm(m, k)
    return total


def solve_dual(px, K, dist, obj_all, names, by_cls, h_lo=0.4, h_hi=1.8,
               margin=1.0, max_assign=4000, prev=None, jump_m=0.35, jump_deg=15.0):
    px = _dedup(px)
    n = len(px)
    if n < 3 or _n_assign(px, by_cls) > max_assign:
        return None
    groups = {}
    for i, p in enumerate(px):
        groups.setdefault(p[0], []).append(i)
    opts = []
    for c, idxs in groups.items():
        if len(idxs) > len(by_cls[c]):
            return None
        opts.append([(idxs, pm) for pm in itertools.permutations(by_cls[c], len(idxs))])
    det = np.array([[p[1], p[2]] for p in px], np.float64)
    flag = cv2.SOLVEPNP_IPPE if n >= 4 else cv2.SOLVEPNP_SQPNP
    x_lo, x_hi = float(obj_all[:, 0].min()), float(obj_all[:, 0].max())
    y_lo, y_hi = float(obj_all[:, 1].min()), float(obj_all[:, 1].max())
    cands = []
    for combo in itertools.product(*opts):
        assign = {}
        for idxs, pm in combo:
            for i, j in zip(idxs, pm):
                assign[i] = j
        obj = obj_all[[assign[i] for i in range(n)]]
        try:
            ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, det, K, dist, flags=flag)
        except cv2.error:
            continue
        if not ok:
            continue
        for rv, tv in zip(rvecs, tvecs):
            R, _ = cv2.Rodrigues(rv)
            C = (-R.T @ tv).ravel()
            if not (h_lo <= C[2] <= h_hi):
                continue
            if C[0] > x_hi + 0.15:
                continue
            proj, _ = cv2.projectPoints(obj, rv, tv, K, dist)
            err = np.linalg.norm(proj.reshape(-1, 2) - det, axis=1)
            axis = R.T @ np.array([0, 0, 1.0])
            yaw = math.degrees(math.atan2(axis[1], axis[0]))
            inbox = (x_lo - margin <= C[0] <= x_hi + margin) and (y_lo - margin <= C[1] <= y_hi + margin)
            cands.append((inbox, float(err.mean()), C, yaw, err, assign,
                          rv.ravel(), tv.ravel()))
    if not cands:
        return None
    infield = [c for c in cands if c[0]]
    pool = infield if infield else cands
    if prev is not None:
        def _near(c):
            dxy = math.hypot(c[2][0] - prev[0], c[2][1] - prev[1])
            dyaw = abs((c[3] - prev[2] + 180.0) % 360.0 - 180.0)
            return dxy <= jump_m and dyaw <= jump_deg
        near = [c for c in pool if _near(c)]
        if not near:
            return None
        pool = near
    best = min(pool, key=lambda c: c[1])
    inbox, score, C, yaw, err, assign, rv, tv = best
    inl = int((err < PX_INLIER).sum())
    if inl < 3:
        return None
    return dict(C=C.copy(), yaw=float(yaw), err=float(score),
                err_each=[float(v) for v in err],
                inl=inl, n=n,
                names=[names[assign[i]] for i in range(n)], infield=bool(infield),
                rvec=rv, tvec=tv)


_REPO = Path(__file__).resolve().parent.parent
_CLS = {"LCross": 0, "TCross": 1, "XCross": 2}


def load_map(geometry_path=None):
    import field_geometry as FG
    frame = FG.load_frame(str(geometry_path or (_REPO / "config" / "field_geometry.yaml")))
    MAP = {k: v for k, v in frame["features"].items() if k != "PenalityPoint"}
    names = sorted(MAP)
    obj = np.array([[MAP[n][0], MAP[n][1], 0.0] for n in names], np.float64)
    cls_of = np.array([_CLS[n[:6]] for n in names])
    by_cls = {c: [i for i in range(len(names)) if cls_of[i] == c] for c in (0, 1, 2)}
    return names, obj, by_cls


def solve_image(img, K, dist, obj, names, by_cls, device="0", supplement=False,
                prev=None, jump_m=0.35, jump_deg=15.0):
    px = detect_points(img, device=device)
    n_supp = 0
    if supplement:
        from supplement import supplement_points
        px2 = supplement_points(px, img, K, dist, obj, names, by_cls)
        n_supp = len(px2) - len(px)
        px = _dedup(px2)
    use = px if _n_assign(px, by_cls) <= 4000 else sorted(px, key=lambda p: -p[3])[:4]
    s = solve_dual(use, K, dist, obj, names, by_cls,
                   prev=prev, jump_m=jump_m, jump_deg=jump_deg)
    if s is None and len(use) > 4:
        s = solve_dual(sorted(use, key=lambda p: -p[3])[:4], K, dist, obj, names, by_cls,
                       prev=prev, jump_m=jump_m, jump_deg=jump_deg)
    res = dict(marker_count=len(px), supplement_added=n_supp)
    if s is None:
        res.update(valid=False,
                   status="insufficient_markers" if len(px) < 3 else "lost")
        return res
    ok = bool(s["infield"] and s["err"] < 45)
    if ok:
        st = "ok"
    elif s["err"] >= 45:
        st = "high_residual"
    else:
        st = "ambiguous"
    res.update(x_camera_map=round(float(s["C"][0]), 3),
               y_camera_map=round(float(s["C"][1]), 3),
               yaw_camera_map=round(float(s["yaw"]), 2),
               height_m=round(float(s["C"][2]), 3),
               residual=round(float(s["err"]), 2), residual_unit="px",
               inliers=f"{s['inl']}/{s['n']}", matched=s["names"],
               point_count=int(s["n"]), infield=bool(s["infield"]),
               valid=ok, status=st)
    return res


def run(source, weights=None, conf=0.25, intrinsics=None, geometry=None,
        out=None, device="0", send=None, ball=True, ball_weights=None, supplement=False):
    global _WEIGHTS, _BALL_WEIGHTS
    if weights:
        _WEIGHTS = str(weights)
    if ball_weights:
        _BALL_WEIGHTS = str(ball_weights)

    def add_ball(row, frame):
        if not ball:
            return
        try:
            row["ball"] = detect_ball(frame, conf=conf, device=device)
        except Exception:
            row["ball"] = None
    import socket
    import field_geometry as FG
    K, dist, _ = FG.load_intrinsics(
        str(intrinsics or (_REPO / "calibration" / "camera_intrinsics.yaml")))
    names, obj, by_cls = load_map(geometry)
    sock = None
    if send:
        host, port = send.rsplit(":", 1)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.connect((host, int(port)))
    cap = cv2.VideoCapture(source)
    rows = []
    from collections import deque
    state = dict(est=None, miss=0, hist=deque(maxlen=15))

    def _advance(est, t):
        dtp = max(t - est["t"], 1e-3)
        return (est["x"] + est["vx"] * dtp, est["y"] + est["vy"] * dtp,
                (est["yaw"] + est["wy"] * dtp + 180.0) % 360.0 - 180.0)

    def _filter(row, t):
        est = state["est"]
        pred = _advance(est, t) if est is not None else None
        if row.get("valid") and row.get("point_count", 0) >= 4:
            zx, zy, zyaw = (row["x_camera_map"], row["y_camera_map"],
                            row["yaw_camera_map"])
            if est is None:
                state["est"] = dict(x=zx, y=zy, yaw=zyaw, vx=0.0, vy=0.0,
                                    wy=0.0, t=t)
            else:
                dtp = max(t - est["t"], 1e-3)
                ex, ey = zx - pred[0], zy - pred[1]
                eyaw = (zyaw - pred[2] + 180.0) % 360.0 - 180.0
                a, b = 0.6, 0.25
                state["est"] = dict(
                    x=pred[0] + a * ex, y=pred[1] + a * ey,
                    yaw=(pred[2] + a * eyaw + 180.0) % 360.0 - 180.0,
                    vx=est["vx"] + b * ex / dtp, vy=est["vy"] + b * ey / dtp,
                    wy=est["wy"] + b * eyaw / dtp, t=t)
                sp = math.hypot(state["est"]["vx"], state["est"]["vy"])
                if sp > 2.0:
                    state["est"]["vx"] *= 2.0 / sp
                    state["est"]["vy"] *= 2.0 / sp
                if abs(state["est"]["wy"]) > 90.0:
                    state["est"]["wy"] = math.copysign(90.0, state["est"]["wy"])
            state["hist"].append((t, zx, zy, zyaw))
            state["miss"] = 0
        else:
            state["miss"] += 1
            if state["miss"] >= 6:
                state["est"] = None
                state["hist"].clear()
                state["miss"] = 0
        est = state["est"]
        if est is None:
            return row
        mode = "predict+AB"
        hist = state["hist"]
        if len(hist) >= 3:
            xs = [h[1] for h in hist]
            ys = [h[2] for h in hist]
            if max(xs) - min(xs) < 0.35 and max(ys) - min(ys) < 0.35:
                a_ = np.radians([h[3] for h in hist])
                mx, my = float(np.median(xs)), float(np.median(ys))
                myaw = float(np.degrees(np.arctan2(np.sin(a_).mean(),
                                                   np.cos(a_).mean())))
                state["est"] = dict(x=mx, y=my, yaw=myaw, vx=0.0, vy=0.0,
                                    wy=0.0, t=t)
                est = state["est"]
                mode = "locked(median)"
        dtp = max(t - est["t"], 0.0)
        row["x_camera_map"] = round(est["x"] + est["vx"] * dtp, 3)
        row["y_camera_map"] = round(est["y"] + est["vy"] * dtp, 3)
        row["yaw_camera_map"] = round(
            (est["yaw"] + est["wy"] * dtp + 180.0) % 360.0 - 180.0, 2)
        row["filter"] = mode
        return row

    if not cap.isOpened():
        img = cv2.imread(str(source))
        if img is None:
            raise RuntimeError(f"打不开输入: {source}")
        rows.append(dict(frame=0, **solve_image(img, K, dist, obj, names, by_cls, device, supplement)))
        add_ball(rows[-1], img)
        if sock:
            sock.send(json.dumps(rows[-1]).encode("utf-8"))
    else:
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        is_cam = isinstance(source, int)
        t0 = time.time()
        idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            t = (time.time() - t0) if is_cam else idx / fps
            pred = _advance(state["est"], t) if state["est"] is not None else None
            row = dict(frame=idx, **solve_image(frame, K, dist, obj, names, by_cls,
                                                device, supplement, prev=pred))
            _filter(row, t)
            add_ball(row, frame)
            rows.append(row)
            if sock:
                sock.send(json.dumps(row).encode("utf-8"))
            idx += 1
        cap.release()
    stat = {}
    for r in rows:
        stat[r["status"]] = stat.get(r["status"], 0) + 1
    nvalid = sum(1 for r in rows if r["valid"])
    print(f"frames={len(rows)} valid={nvalid} status={stat}")
    for r in rows:
        if r["valid"]:
            print(f"  f{r['frame']}: x={r['x_camera_map']} y={r['y_camera_map']} "
                  f"yaw={r['yaw_camera_map']} err={r['residual']} inl={r['inliers']}")
    if out:
        Path(out).write_text(json.dumps(rows, ensure_ascii=False, indent=2), encoding="utf-8")
        print("写出", out)
    return rows


def main():
    ap = argparse.ArgumentParser(description="标记检测 + 自定位：输出 x/y/yaw/status")
    ap.add_argument("source", nargs="?", help="视频/图片路径，或相机编号(如 1)")
    ap.add_argument("--weights", help="检测权重，默认 marker_v6")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--intrinsics")
    ap.add_argument("--geometry")
    ap.add_argument("--out", help="结果 JSON 输出路径")
    ap.add_argument("--device", default="0")
    ap.add_argument("--send", help="把每帧结果 UDP 发到 host:port（VM 里的 ROS2 节点）")
    ap.add_argument("--ball-weights", help="球检测权重，默认 ball_v2")
    ap.add_argument("--no-ball", action="store_true", help="不做球检测")
    ap.add_argument("--supplement", action="store_true", help="用线交点补点（默认关，实测增益≈0）")
    ap.add_argument("--selftest", action="store_true", help="跑合成位姿自检（不需要输入）")
    a = ap.parse_args()
    if a.selftest:
        selftest(intrinsics=a.intrinsics)
        return
    if a.source is None:
        ap.error("需要 source（视频/图片/相机号），或用 --selftest")
    src = int(a.source) if a.source.isdigit() else a.source
    run(src, weights=a.weights, conf=a.conf, intrinsics=a.intrinsics,
        geometry=a.geometry, out=a.out, device=a.device, send=a.send,
        ball=not a.no_ball, ball_weights=a.ball_weights, supplement=a.supplement)


def _project(P_xy, cam, yaw, K, dist, h, pitch, roll):
    c, s = math.cos(math.radians(yaw)), math.sin(math.radians(yaw))
    R = np.array([[c, -s], [s, c]])
    loc = (np.asarray(P_xy, float) - np.asarray(cam, float)) @ R
    return GP.ground_to_pixel(loc, K, dist, h, pitch, roll)


def selftest(K=None, dist=None, trials=100, seed=0, h=0.62, pitch=7.0, roll=0.0,
             W=1280, H=720, intrinsics=None):
    import field_geometry as FG
    if K is None:
        K, dist, _ = FG.load_intrinsics(
            str(intrinsics or (_REPO / "calibration" / "camera_intrinsics.yaml")))
    names, obj, by_cls = load_map()
    P2 = obj[:, :2]
    x_lo, x_hi = float(P2[:, 0].min()), float(P2[:, 0].max())
    y_lo, y_hi = float(P2[:, 1].min()), float(P2[:, 1].max())
    cls_of = [_CLS[n[:6]] for n in names]
    rng = np.random.default_rng(seed)
    n = solved = good = rel_solved = rel_good = 0
    dpos, dyaw = [], []
    for _ in range(trials):
        cam = (float(rng.uniform(x_lo + 0.3, x_hi - 0.3)),
               float(rng.uniform(y_lo + 0.25, y_hi - 0.25)))
        tyaw = float(rng.uniform(0, 360))
        uv = _project(P2, cam, tyaw, K, dist, h, pitch, roll)
        px = [(cls_of[i], float(uv[i][0]), float(uv[i][1]), 1.0)
              for i in range(len(names)) if 0 <= uv[i][0] < W and 0 <= uv[i][1] < H]
        if len(px) < 3:
            continue
        n += 1
        s = solve_dual(px, K, dist, obj, names, by_cls)
        if s is None:
            continue
        solved += 1
        d = math.hypot(s["C"][0] - cam[0], s["C"][1] - cam[1])
        dy = abs((s["yaw"] - tyaw + 180) % 360 - 180)
        dpos.append(d)
        dyaw.append(dy)
        hit = d < 0.5 and dy < 10
        good += hit
        if s["n"] >= 4:
            rel_solved += 1
            rel_good += hit
    if solved:
        print(f"selftest: {n} 个可解场景, 解出 {solved}, 达标(<0.5m&<10°) {good}")
        print(f"  位置误差 中位 {np.median(dpos):.2f}m 最大 {max(dpos):.2f}m; "
              f"yaw 误差中位 {np.median(dyaw):.1f}°")
        print(f"  其中 ≥4点: 解出 {rel_solved}, 达标 {rel_good}"
              f"（3点解有镜像歧义，偶尔翻车属正常）")
    else:
        print(f"selftest: {n} 个场景但全部无解（异常）")
    return dict(trials=n, solved=solved, good=good,
                rel_solved=rel_solved, rel_good=rel_good, pos=dpos, yaw=dyaw)


if __name__ == "__main__":
    main()
