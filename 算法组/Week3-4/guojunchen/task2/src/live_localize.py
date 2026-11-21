"""实时定位：左侧相机画面，右侧场地俯视图（相机位姿 + 球在地面上的位置）。

输出严格按细则：x/y/yaw_camera_map（米、度）、valid、marker_count、residual、status。
当前帧解不出位姿时 status 报 insufficient_markers / ambiguous / high_residual / lost，
不会把上一帧的位姿当成这一帧的结果继续显示（只画一个灰色虚影并标 last）。
"""
import itertools
import json
import math
import re
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
TASK2 = HERE.parent
PROJ = TASK2.parent

HALF_L = 3.90
HALF_W = 3.00
CLASS_NAMES = ["L", "T", "X", "Point"]
CLASS_KEYS = {"L": 0, "T": 1, "X": 2, "Point": 3}

SOURCE = "1"
BALL_WEIGHTS = str(PROJ / "task1" / "weights" / "best.pt")
MARKER_WEIGHTS = str(TASK2 / "weights" / "marker_best.pt")
INTRINSICS = str(TASK2 / "calibration" / "intrinsics.yaml")
MAP_PATH = str(TASK2 / "config" / "field_map.png")
FIELD_YAML = str(TASK2 / "config" / "field.yaml")
R_CENTER = 0.75
CONF_BALL = 0.25
CONF_MARK = 0.20
# 场地在 180° 旋转下完全对称，单帧无法区分“站在球门这侧”还是“对面那侧”。
# -1 = 站在球门对面（视野里球门在远处，本项目实拍就是这种）
# +1 = 站在球门这侧；0 = 不指定（会持续报 ambiguous）
INIT_SIDE = -1
MAX_FRAMES = 0
SHOW = True
RECORD = ""


def white_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    v, s = hsv[..., 2], hsv[..., 1]
    m = ((v > 140) & (s < 70)).astype(np.uint8) * 255
    return cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


def segments_in(gray, roi, min_len=25.0):
    segs = []
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    found = lsd.detect(gray[roi[1]:roi[3], roi[0]:roi[2]])[0]
    if found is None:
        return segs
    for s in found.reshape(-1, 4):
        x1, y1, x2, y2 = s
        if math.hypot(x2 - x1, y2 - y1) < min_len:
            continue
        segs.append((x1 + roi[0], y1 + roi[1], x2 + roi[0], y2 + roi[1]))
    return segs


def seg_angle(seg):
    x1, y1, x2, y2 = seg
    return math.atan2(y2 - y1, x2 - x1) % math.pi


def dist_point_to_seg_line(px, py, seg):
    x1, y1, x2, y2 = seg
    dx, dy = x2 - x1, y2 - y1
    n = math.hypot(dx, dy)
    if n < 1e-6:
        return 1e9
    return abs(dy * (px - x1) - dx * (py - y1)) / n


def fit_line(segs):
    pts = []
    for x1, y1, x2, y2 in segs:
        pts.append([x1, y1])
        pts.append([x2, y2])
    pts = np.array(pts, np.float32)
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_L2, 0, 0.01, 0.01).ravel()
    return float(vx), float(vy), float(x0), float(y0)


def intersect(l1, l2):
    vx1, vy1, x1, y1 = l1
    vx2, vy2, x2, y2 = l2
    den = vx1 * vy2 - vy1 * vx2
    if abs(den) < 1e-6:
        return None
    t = ((x2 - x1) * vy2 - (y2 - y1) * vx2) / den
    return x1 + t * vx1, y1 + t * vy1


def refine(bgr, cx, cy, half):
    h, w = bgr.shape[:2]
    pad = int(half)
    roi = (max(0, int(cx) - pad), max(0, int(cy) - pad),
           min(w, int(cx) + pad), min(h, int(cy) + pad))
    if roi[2] - roi[0] < 20 or roi[3] - roi[1] < 20:
        return None
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    segs = segments_in(gray, roi, min_len=max(10, half * 0.35))
    near = [s for s in segs if dist_point_to_seg_line(cx, cy, s) <= max(10, half * 0.45)]
    if not near:
        return None
    clusters = []
    for s in near:
        a = seg_angle(s)
        for c in clusters:
            if min(abs(a - c["ang"]), math.pi - abs(a - c["ang"])) < 0.20:
                c["segs"].append(s)
                break
        else:
            clusters.append({"ang": a, "segs": [s]})
    clusters = [c for c in clusters if len(c["segs"]) >= 1][:4]
    if len(clusters) < 2:
        return None
    lines = [fit_line(c["segs"]) for c in clusters]
    best = None
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            p = intersect(lines[i], lines[j])
            if p is None:
                continue
            ang = abs(clusters[i]["ang"] - clusters[j]["ang"])
            ang = min(ang, math.pi - ang)
            if ang < 0.35:
                continue
            d = math.hypot(p[0] - cx, p[1] - cy)
            if d > half * 1.2:
                continue
            score = d - 0.5 * len(clusters[i]["segs"]) - 0.5 * len(clusters[j]["segs"])
            if best is None or score < best[0]:
                best = (score, p, len(clusters))
    if best is None:
        return None
    return best[1][0], best[1][1], best[2], len(near)


def parse_intrinsics(path):
    txt = Path(path).read_text(encoding="utf-8")
    fx = float(re.search(r"^fx:\s*([-\d.]+)", txt, re.M).group(1))
    fy = float(re.search(r"^fy:\s*([-\d.]+)", txt, re.M).group(1))
    cx = float(re.search(r"^cx:\s*([-\d.]+)", txt, re.M).group(1))
    cy = float(re.search(r"^cy:\s*([-\d.]+)", txt, re.M).group(1))
    return np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)


def parse_dist(path):
    txt = Path(path).read_text(encoding="utf-8")
    def grab(key):
        m = re.search(r"^%s:\s*([-\d.eE+]+)" % key, txt, re.M)
        return float(m.group(1)) if m else 0.0
    vals = [grab(k) for k in ("k1", "k2", "p1", "p2", "k3")]
    if not any(abs(v) > 1e-9 for v in vals):
        m = re.search(r"dist_coeffs:\s*\n?\s*-\s*\[([^\]]+)\]", txt)
        if m:
            vals = [float(t) for t in re.split(r"[,\s]+", m.group(1).strip()) if t][:5]
    return np.array(vals, np.float64)


def load_markers_yaml(path):
    """读 field.yaml（特征点坐标文件）。没有该文件或解析失败返回 None。

    只解析本仓库固定写法的 "  - {id: 12, type: LCross, x: +3.900, y: -3.000}" 行，
    不依赖 PyYAML（现场环境可能没装）。
    """
    p = Path(path)
    if not p.exists():
        return None
    type_id = {"LCross": 0, "TCross": 1, "XCross": 2, "PenaltyPoint": 3,
               "L": 0, "T": 1, "X": 2, "Point": 3}
    rows = []
    pat = re.compile(r"-\s*\{\s*id:\s*(\d+)\s*,\s*type:\s*(\w+)\s*,"
                     r"\s*x:\s*([-+0-9.]+)\s*,\s*y:\s*([-+0-9.]+)\s*\}")
    for line in p.read_text(encoding="utf-8").splitlines():
        m = pat.search(line)
        if not m:
            continue
        mid, typ, x, y = int(m.group(1)), m.group(2), float(m.group(3)), float(m.group(4))
        cls = type_id.get(typ)
        if cls is None:
            continue
        rows.append((mid, cls, x, y))
    if not rows:
        return None
    rows.sort(key=lambda r: r[0])
    return np.array([(r[1], r[2], r[3]) for r in rows], np.float64)


def field_markers(r_center):
    # 优先读 config/field.yaml（特征点坐标文件）；没有就用内置表
    loaded = load_markers_yaml(FIELD_YAML)
    if loaded is not None:
        return loaded

    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts.append((0, HALF_L * sx, HALF_W * sy))
            pts.append((0, 2.00 * sx, 2.00 * sy))
            pts.append((0, 2.96 * sx, 1.35 * sy))
            pts.append((1, HALF_L * sx, 2.00 * sy))
            pts.append((1, HALF_L * sx, 1.35 * sy))
    for sy in (-1, 1):
        pts.append((1, 0.0, HALF_W * sy))
    if r_center:
        pts.append((2, 0.0, r_center))
        pts.append((2, 0.0, -r_center))
    pts.append((3, 0.0, 0.0))
    return np.array(pts, np.float64)


def solve_all(obj_pts, img_pts, K, dist=None):
    if len(obj_pts) < 3:
        return []
    c3 = obj_pts - obj_pts.mean(axis=0)
    sv = np.linalg.svd(c3, compute_uv=False)
    if len(sv) < 2 or sv[1] < 1e-3:
        return []
    c2 = img_pts - img_pts.mean(axis=0)
    sv2 = np.linalg.svd(c2, compute_uv=False)
    if len(sv2) < 2 or sv2[1] < 1.0:
        return []
    flags = cv2.SOLVEPNP_IPPE if len(obj_pts) >= 4 else cv2.SOLVEPNP_SQPNP
    try:
        ok, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            obj_pts.reshape(-1, 1, 3), img_pts.reshape(-1, 1, 2), K, dist, flags=flags
        )
    except cv2.error:
        return []
    if not ok or rvecs is None:
        return []
    out = []
    for rvec, tvec in zip(rvecs, tvecs):
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.ravel()
        if (-R.T @ t)[2] <= 0.05:
            continue
        if any((R @ p + t)[2] <= 0 for p in obj_pts):
            continue
        proj, _ = cv2.projectPoints(obj_pts.reshape(-1, 1, 3), rvec, tvec, K, None)
        err = float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)))
        out.append((rvec, tvec, err))
    return out


def refine_all(bgr, raw):
    out = []
    for cls, cx, cy, half, conf in raw:
        rf = refine(bgr, cx, cy, half)
        if rf:
            out.append((cls, rf[0], rf[1], conf + 0.2, True))
        else:
            out.append((cls, cx, cy, conf, False))
    return out


def _correspond(dets, proj, markers, penalty=12.0):
    corr, res = [], []
    mcls = np.array([int(m[0]) for m in markers])
    for d in dets:
        dd = np.linalg.norm(proj - np.array([d[1], d[2]]), axis=1)
        dd = np.where(mcls == int(d[0]), dd, dd + penalty)
        k = int(np.argmin(dd))
        corr.append((d, k))
        res.append(float(dd[k]))
    return corr, res


def localize(dets, K, markers, prev_h=None, prev_xy=None, max_combos=1200, h_min=0.9, h_max=1.9, dist=None, prev_pose=None, init_side=0):
    info = {"n": len(dets), "residual": None, "pose": None}
    if len(dets) < 3:
        return None, "insufficient_markers", info
    dets = sorted(dets, key=lambda d: -d[3])
    kept = []
    for d in dets:
        if all(abs(d[1] - k[1]) > 25 or abs(d[2] - k[2]) > 25 for k in kept):
            kept.append(d)
    dets = kept[:8]

    cls_idx = {}
    for i, m in enumerate(markers):
        cls_idx.setdefault(int(m[0]), []).append(i)
    proj_flat = np.array([[m[1], m[2], 0.0] for m in markers], np.float64).reshape(-1, 1, 3)

    if prev_pose is not None:
        rv0 = np.array(prev_pose["rvec"], np.float64).reshape(3, 1)
        tv0 = np.array(prev_pose["tvec"], np.float64).reshape(3, 1)
        proj0, _ = cv2.projectPoints(proj_flat, rv0, tv0, K, dist)
        corr0, res0 = _correspond(dets, proj0.reshape(-1, 2), markers)
        good = [(d, i) for (d, i), r in zip(corr0, res0) if r <= 25.0]
        if len(good) >= 3:
            obj = np.array([[markers[i][1], markers[i][2], 0.0] for _, i in good], np.float64)
            imgp = np.array([[d[1], d[2]] for d, _ in good], np.float64)
            try:
                okk, rv1, tv1 = cv2.solvePnP(obj, imgp, K, dist, rv0.copy(), tv0.copy(),
                                             useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
            except cv2.error:
                okk = False
            if okk:
                R1, _ = cv2.Rodrigues(rv1)
                cam1 = (-R1.T @ tv1).ravel()
                proj1, _ = cv2.projectPoints(proj_flat, rv1, tv1, K, dist)
                corr1, res1 = _correspond(dets, proj1.reshape(-1, 2), markers)
                inl1 = [r for r in res1 if r <= 18.0]
                if h_min <= cam1[2] <= h_max and len(inl1) >= 3 and float(np.mean(inl1)) <= 12.0:
                    R1, _ = cv2.Rodrigues(rv1)
                    fwd1 = R1.T @ np.array([0.0, 0.0, 1.0])
                    return {
                        "R": R1,
                        "t": tv1.ravel(),
                        "rvec": rv1.ravel(),
                        "tvec": tv1.ravel(),
                        "cam": cam1,
                        "x": float(cam1[0]),
                        "y": float(cam1[1]),
                        "err": float(np.mean(inl1)),
                        "n": len(dets),
                        "h": float(cam1[2]),
                        "pitch": float(math.asin(max(-1.0, min(1.0, R1[2, 2])))),
                        "yaw": float(math.atan2(fwd1[1], fwd1[0])),
                        "dots": corr1,
                    }, "ok", {"n": len(dets), "residual": float(np.mean(inl1)), "pose": cam1}

    if len(dets) >= 4:
        subsets = [dets[:4]]
        if len(dets) >= 5:
            subsets += [list(c) for c in itertools.combinations(dets[:5], 4)]
        subsets.sort(key=lambda s: -len({int(d[0]) for d in s}))
    else:
        subsets = [dets[:3]]

    best = None
    second = None
    rng = np.random.default_rng(0)
    for seed in subsets:
        pools = []
        bad = False
        for d in seed:
            cand = cls_idx.get(int(d[0]))
            if not cand:
                bad = True
                break
            pools.append(cand)
        if bad:
            continue
        img = np.array([[d[1], d[2]] for d in seed], np.float64)
        total = 1
        for p in pools:
            total *= len(p)
        if total <= max_combos:
            combos = (c for c in itertools.product(*pools) if len(set(c)) == len(c))
        else:
            def _rand(pools=pools):
                for _ in range(max_combos):
                    pick = tuple(int(rng.choice(p)) for p in pools)
                    if len(set(pick)) == len(pick):
                        yield pick
            combos = _rand()
        for combo in combos:
            obj = np.array([[markers[c][1], markers[c][2], 0.0] for c in combo], np.float64)
            for rvec, tvec, _ in solve_all(obj, img, K, dist):
                R, _ = cv2.Rodrigues(rvec)
                cam = (-R.T @ tvec).ravel()
                if not (h_min <= cam[2] <= h_max):
                    continue
                if abs(cam[0]) > 6.5 or abs(cam[1]) > 5.5:
                    continue
                if prev_h is not None and len(seed) == 3:
                    if abs(cam[2] - prev_h) > max(0.25, 0.35 * prev_h):
                        continue
                proj, _ = cv2.projectPoints(proj_flat, rvec, tvec, K, dist)
                proj = proj.reshape(-1, 2)
                corr, res = _correspond(dets, proj, markers)
                if len(res) < 3:
                    continue
                inl = [r for r in res if r <= 18.0]
                if len(inl) < 3:
                    continue
                mean_in = float(np.mean(inl))
                score = (-len(inl), mean_in)
                item = (score, cam, R, tvec.ravel(), rvec, corr)
                if best is None or score < best[0]:
                    second = best
                    best = item
                elif second is None or score < second[0]:
                    second = item
        if best is not None and best[0][0] <= -4:
            break

    if best is None:
        return None, "high_residual", info

    score, cam, R, t, rvec, corr = best
    # 用全部内点做一次精修
    if len(corr) >= 4:
        obj = np.array([[markers[i][1], markers[i][2], 0.0] for _, i in corr], np.float64)
        img = np.array([[d[1], d[2]] for d, _ in corr], np.float64)
        try:
            ok, rv, tv = cv2.solvePnP(obj, img, K, dist, rvec.copy(), t.reshape(3, 1).copy(),
                                      useExtrinsicGuess=True, flags=cv2.SOLVEPNP_ITERATIVE)
        except cv2.error:
            ok = False
        if ok:
            proj, _ = cv2.projectPoints(proj_flat, rv, tv, K, None)
            corr2, res2 = _correspond(dets, proj.reshape(-1, 2), markers)
            inl2 = [r for r in res2 if r <= 18.0]
            if len(inl2) >= len([r for r in _correspond(dets, cv2.projectPoints(
                    proj_flat, rvec, t.reshape(3, 1), K, dist)[0].reshape(-1, 2), markers)[1] if r <= 18.0]):
                rvec, t = rv, tv.ravel()
                corr = corr2
                R, _ = cv2.Rodrigues(rvec)
                cam = (-R.T @ t).ravel()

    _proj, _ = cv2.projectPoints(proj_flat, rvec, t.reshape(3, 1), K, dist)
    _corr, _res = _correspond(dets, _proj.reshape(-1, 2), markers)
    _inl = [r for r in _res if r <= 18.0] or [999.0]
    info["residual"] = float(np.mean(_inl))
    if info["residual"] > 15.0 or not (h_min <= cam[2] <= h_max):
        return None, "high_residual", info

    # 场地在 180° 旋转下完全对称：显式构造镜像解，避免随机枚举把它漏掉。
    # 只有镜像解和当前解“同样好”时才需要用户给 init_side 或上一帧位姿来消歧。
    Rz = np.diag([-1.0, -1.0, 1.0])
    Rm = R @ Rz
    rvm, _ = cv2.Rodrigues(Rm)
    projm, _ = cv2.projectPoints(proj_flat, rvm, t.reshape(3, 1), K, dist)
    corrm, resm = _correspond(dets, projm.reshape(-1, 2), markers)
    inlm = [r for r in resm if r <= 18.0]
    if len(inlm) >= 3:
        mean_m = float(np.mean(inlm))
        cam_m = (-Rm.T @ t).ravel()
        gap_m = math.hypot(cam[0] - cam_m[0], cam[1] - cam_m[1])
        if len(inlm) == len(_inl) and abs(mean_m - float(np.mean(_inl))) < 1.5 and gap_m > 0.6:
            second = ((-len(inlm), mean_m), cam_m, Rm, t, rvm, corrm)
            score = (-len(_inl), float(np.mean(_inl)))

    if second is not None:
        sc2, cam2 = second[0], second[1]
        if sc2[0] == score[0] and cam2 is not None:
            gap = math.hypot(cam[0] - cam2[0], cam[1] - cam2[1])
            if abs(sc2[1] - score[1]) < 1.5 and gap > 0.6:
                if prev_xy is not None:
                    d1 = math.hypot(cam[0] - prev_xy[0], cam[1] - prev_xy[1])
                    d2 = math.hypot(cam2[0] - prev_xy[0], cam2[1] - prev_xy[1])
                    if d2 < d1:
                        cam = cam2.copy()
                        R2, _ = cv2.Rodrigues(second[4])
                        R = R2
                        t = second[3]
                elif init_side:
                    # 首帧：场地 180° 对称，用人工先验选一侧
                    if cam[0] * init_side < 0 and cam2[0] * init_side > 0:
                        cam = cam2.copy()
                        R2, _ = cv2.Rodrigues(second[4])
                        R = R2
                        t = second[3]
                else:
                    return None, "ambiguous", info

    fwd = R.T @ np.array([0.0, 0.0, 1.0])
    pose = {
        "R": R,
        "t": t,
        "rvec": np.asarray(rvec, np.float64).ravel(),
        "tvec": np.asarray(t, np.float64).ravel(),
        "x": float(cam[0]),
        "y": float(cam[1]),
        "h": float(cam[2]),
        "yaw": math.atan2(fwd[1], fwd[0]),
        "dots": corr,
        "err": info["residual"],
        "n": len(dets),
    }
    info["pose"] = pose
    return pose, "ok", info


def project_to_ground(u, v, pose, K):
    d_cam = np.linalg.inv(K) @ np.array([u, v, 1.0])
    d_world = pose["R"].T @ d_cam
    c = -pose["R"].T @ pose["t"]
    if abs(d_world[2]) < 1e-6:
        return None
    s = -c[2] / d_world[2]
    if s <= 0:
        return None
    p = c + s * d_world
    if abs(p[0]) > HALF_L + 0.5 or abs(p[1]) > HALF_W + 0.5:
        return None
    return float(p[0]), float(p[1])


class MapCanvas:
    def __init__(self, img_path, size=620):
        img = cv2.imread(str(img_path))
        if img is None:
            raise SystemExit(f"读不到场地图: {img_path}")
        self.src = img
        self.size = size
        meta_p = Path(img_path).with_suffix(".json")
        meta = json.loads(meta_p.read_text(encoding="utf-8"))
        self.scale = float(meta["px_per_m"])
        self.ox, self.oy = meta["origin_px"]

    def to_px(self, wx, wy):
        # +X 向右、+Y 向上
        return int(round(self.ox + wx * self.scale)), int(round(self.oy - wy * self.scale))

    def draw(self, pose, ball_xy, ghost=None):
        g = self.src.copy()
        if ghost is not None:
            self._camera(g, ghost, (150, 150, 150), ghost=True)
        if pose is not None:
            self._camera(g, pose, (230, 120, 20))
        if ball_xy is not None:
            bx, by = self.to_px(ball_xy[0], ball_xy[1])
            r = int(0.10 * self.scale)
            cv2.circle(g, (bx, by), int(0.18 * self.scale), (0, 200, 255), -1)
            cv2.circle(g, (bx, by), int(0.18 * self.scale), (0, 0, 0), 2)
        return cv2.resize(g, (self.size, int(self.size * g.shape[0] / g.shape[1])))

    def _camera(self, g, pose, color, ghost=False):
        """相机位置：蓝色大圆点；朝向：从圆点指向前方的小箭头。"""
        cx, cy = self.to_px(pose["x"], pose["y"])
        H, W = g.shape[:2]
        r = int(0.40 * self.scale)
        outside = not (r + 2 <= cx < W - r - 2 and r + 2 <= cy < H - r - 2)
        if outside:
            cx = min(max(cx, r + 3), W - r - 4)
            cy = min(max(cy, r + 3), H - r - 4)

        if not ghost:
            tip = self.to_px(pose["x"] + 0.9 * math.cos(pose["yaw"]),
                             pose["y"] + 0.9 * math.sin(pose["yaw"]))
            start = self.to_px(pose["x"] + 0.20 * math.cos(pose["yaw"]),
                               pose["y"] + 0.20 * math.sin(pose["yaw"]))
            tip = (min(max(tip[0], 0), W - 1), min(max(tip[1], 0), H - 1))
            cv2.arrowedLine(g, start, tip, color, max(3, r // 5), tipLength=0.30)

        cv2.circle(g, (cx, cy), r, color, -1)
        cv2.circle(g, (cx, cy), r, (0, 0, 0), 2)
        cv2.circle(g, (cx, cy), max(3, r // 3), (255, 255, 255), -1)

        if outside:
            cv2.putText(g, "cam", (cx + r + 6, cy + 8), cv2.FONT_HERSHEY_SIMPLEX,
                        max(0.6, r / 22), (0, 0, 0), 2)


def draw_panel(img, rows, x0=10, y0=None):
    fs, lh = 0.62, 30
    w = max(cv2.getTextSize(r[0] + "   " + r[1], cv2.FONT_HERSHEY_SIMPLEX, fs, 1)[0][0] for r in rows) + 24
    h = 14 + len(rows) * lh
    if y0 is None:
        y0 = img.shape[0] - h - 10
    roi = img[y0:y0 + h, x0:x0 + w]
    if roi.shape[0] == h and roi.shape[1] == w:
        img[y0:y0 + h, x0:x0 + w] = (roi * 0.18).astype(np.uint8)
    for i, (k, v) in enumerate(rows):
        y = y0 + 24 + i * lh
        cv2.putText(img, k, (x0 + 10, y), cv2.FONT_HERSHEY_SIMPLEX, fs, (150, 220, 255), 1)
        cv2.putText(img, v, (x0 + 12 + 205, y), cv2.FONT_HERSHEY_SIMPLEX, fs, (255, 255, 255), 2)


def open_source(src):
    if str(src).isdigit():
        idx = int(src)
        # 有的机器 MSMF 会返回几乎全黑的帧，所以两个后端都试，取亮的那个
        best, best_mean = None, -1.0
        for backend in (cv2.CAP_MSMF, cv2.CAP_ANY):
            cap = cv2.VideoCapture(idx, backend)
            if not cap.isOpened():
                cap.release()
                continue
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            m = -1.0
            for _ in range(40):
                ok, f = cap.read()
                if ok and f is not None:
                    m = float(f.mean())
                    if m > 20:
                        break
            if m > best_mean:
                if best is not None:
                    best.release()
                best, best_mean = cap, m
            else:
                cap.release()
            if best_mean > 20:
                break
        return best if best is not None else cv2.VideoCapture(idx, cv2.CAP_MSMF)
    return cv2.VideoCapture(str(src))


def main():
    global INIT_SIDE

    K = parse_intrinsics(INTRINSICS)
    markers = field_markers(R_CENTER)
    canvas = MapCanvas(MAP_PATH)

    from ultralytics import YOLO

    missing = [p for p in (BALL_WEIGHTS, MARKER_WEIGHTS) if not Path(p).exists()]
    if missing:
        print("找不到权重文件：")
        for p in missing:
            print("  ", p)
        print("本仓库不附带 .pt 文件。请从网盘下载后放到上面这两个路径：")
        print("  网盘: https://pan.baidu.com/s/1UhbZ_PyChPVQwycBPv7Qaw?pwd=5408  提取码 5408")
        print("  task1_best.pt        -> task1/weights/best.pt")
        print("  task2_marker_best.pt -> task2/weights/marker_best.pt")
        raise SystemExit(1)

    ball_model = YOLO(BALL_WEIGHTS)
    mark_model = YOLO(MARKER_WEIGHTS)

    cap = open_source(SOURCE)
    if not cap.isOpened():
        raise SystemExit(f"打不开视频源: {SOURCE}")

    frame = None
    for _ in range(60):
        ok, f = cap.read()
        if ok and f is not None and float(f.mean()) > 1:
            frame = f
            break
    if frame is None:
        raise SystemExit("视频源打开了但没有画面")

    writer = None
    heights = []
    last_pose = None
    last_good_t = None
    n = 0
    t0 = None
    while True:
        if frame is None:
            ok, frame = cap.read()
            if not ok:
                break
        n += 1
        if t0 is None:
            t0 = time.perf_counter()

        mres = mark_model.predict(frame, imgsz=640, conf=CONF_MARK, verbose=False)[0]
        raw = []
        for box in mres.boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            raw.append((cls, (x1 + x2) / 2, (y1 + y2) / 2, max(x2 - x1, y2 - y1) / 2, conf))
        dets = refine_all(frame, raw)
        for (cls, u, v, sc, is_ref), r in zip(dets, raw):
            x1 = int(r[1] - r[3])
            y1 = int(r[2] - r[3])
            x2 = int(r[1] + r[3])
            y2 = int(r[2] + r[3])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f"{CLASS_NAMES[cls]} {r[4]:.2f}", (x1, max(14, y1 - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
            cv2.drawMarker(frame, (int(u), int(v)), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)

        pose, status, info = localize(dets, K, markers, prev_h=(heights[-1] if heights else None),
                                      prev_xy=((last_pose["x"], last_pose["y"]) if last_pose else None),
                                      prev_pose=last_pose, init_side=INIT_SIDE)
        if pose is not None and pose.get("dots"):
            for (cls, u, v, sc, rf), mid in pose["dots"]:
                cv2.putText(frame, "#%d" % mid, (int(u) + 9, int(v) - 9),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        now = time.perf_counter()
        has_pose = pose is not None
        if has_pose:
            heights.append(pose["h"])
            last_pose = pose
            last_good_t = now
        elif last_good_t is not None and now - last_good_t > 2.5:
            last_good_t = None
            status = "lost"

        ball_xy = None
        bres = ball_model.predict(frame, imgsz=640, conf=CONF_BALL, verbose=False)[0]
        if len(bres.boxes):
            b = max(bres.boxes, key=lambda x: float(x.conf[0]))
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 165, 255), 2)
            cv2.putText(frame, f"ball {float(b.conf[0]):.2f}", (int(x1), max(14, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
            if pose is not None:
                ball_xy = project_to_ground((x1 + x2) / 2, y2, pose, K)
                if ball_xy:
                    cv2.putText(frame, f"({ball_xy[0]:+.2f},{ball_xy[1]:+.2f})m",
                                (int(x1), int(y2) + 18), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

        fps = n / max(time.perf_counter() - t0, 1e-6)
        show = pose if pose is not None else None
        map_img = canvas.draw(show, ball_xy, ghost=(last_pose if (pose is None and last_good_t is not None) else None))

        rows = [
            ("status", status),
            ("valid", "true" if pose is not None else "false"),
            ("x_camera_map", f"{pose['x']:+.3f} m" if pose else "-"),
            ("y_camera_map", f"{pose['y']:+.3f} m" if pose else "-"),
            ("yaw_camera_map", f"{math.degrees(pose['yaw']):+.1f} deg" if pose else "-"),
            ("marker_count", str(info["n"])),
            ("residual", f"{info['residual']:.2f} px" if info["residual"] is not None else "-"),
            ("fps", f"{fps:.1f}"),
        ]

        h = 560
        left = cv2.resize(frame, (int(frame.shape[1] * h / frame.shape[0]), h))
        right = cv2.resize(map_img, (int(map_img.shape[1] * h / map_img.shape[0]), h))
        draw_panel(left, rows)
        cv2.putText(left, f"side {INIT_SIDE:+d}   [f] flip   [s] save   [q] quit",
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 255), 2)
        view = np.hstack([left, right])

        if RECORD:
            if writer is None:
                writer = cv2.VideoWriter(RECORD, cv2.VideoWriter_fourcc(*"mp4v"), 20,
                                         (view.shape[1], view.shape[0]))
            writer.write(view)
        if SHOW:
            cv2.imshow("localize", view)
            k = cv2.waitKey(1) & 0xFF
            if k == ord("q"):
                break
            if k == ord("s"):
                shot = Path("shots")
                shot.mkdir(exist_ok=True)
                cv2.imwrite(str(shot / f"shot_{n:04d}.jpg"), view)
            if k == ord("f"):
                INIT_SIDE = -INIT_SIDE
                last_pose = None
                last_good_t = None
                heights.clear()
                print(f"-> 已翻转左右：INIT_SIDE={INIT_SIDE:+d}（还不对就再按一次 f）")
        frame = None
        if MAX_FRAMES and n >= MAX_FRAMES:
            break

    cap.release()
    if writer:
        writer.release()
    cv2.destroyAllWindows()
    print(f"处理 {n} 帧，{n / max(time.perf_counter() - t0, 1e-6):.1f} FPS")
    if heights:
        arr = np.array(heights)
        print(f"相机高度 中位 {np.median(arr):.2f} m")


if __name__ == "__main__":
    main()
