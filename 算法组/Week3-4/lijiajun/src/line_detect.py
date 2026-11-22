import cv2
import numpy as np


def ground_region(img, hue_lo=35, hue_hi=95, s_min=55, v_min=40, close=41, open_=15):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    Hc = hsv[:, :, 0].astype(int)
    S = hsv[:, :, 1].astype(int)
    V = hsv[:, :, 2].astype(int)
    turf = (((Hc >= hue_lo) & (Hc <= hue_hi) & (S >= s_min) & (V >= v_min)).astype(np.uint8) * 255)
    g = cv2.morphologyEx(turf, cv2.MORPH_CLOSE, np.ones((close, close), np.uint8))
    g = cv2.morphologyEx(g, cv2.MORPH_OPEN, np.ones((open_, open_), np.uint8))
    return g, turf


def tophat_mask(img, radius=13, percentile=97.5, min_abs=18):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * radius + 1, 2 * radius + 1))
    th = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, k)
    thr = max(np.percentile(th, percentile), min_abs)
    m = (th >= thr).astype(np.uint8) * 255
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m, th, float(thr)


def white_mask(img, v_min=175, s_max=55, min_row_ratio=0.0, use_ground=True):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    S = hsv[:, :, 1].astype(int)
    V = hsv[:, :, 2].astype(int)
    m = ((V >= v_min) & (S <= s_max)).astype(np.uint8) * 255
    if use_ground:
        g, _ = ground_region(img)
        m = cv2.bitwise_and(m, g)
    cut = int(img.shape[0] * min_row_ratio)
    m[:cut, :] = 0
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return m


def hough_lines(mask, rho_res=1.0, theta_res=0.5, min_votes=250):
    lines = cv2.HoughLines(mask, rho_res, np.radians(theta_res), min_votes)
    if lines is None:
        return []
    return [(float(r), float(t)) for r, t in lines.reshape(-1, 2)]


def line_support(mask, rho, theta, probe=60):
    H, W = mask.shape
    n = np.array([np.cos(theta), np.sin(theta)])
    u = np.array([-n[1], n[0]])
    base = n * rho
    ts = np.arange(-H - W, H + W, 2.0)
    pts = base[:, None] + np.outer(u, ts)
    xi = np.round(pts[0]).astype(int)
    yi = np.round(pts[1]).astype(int)
    inside = (xi >= 0) & (xi < W) & (yi >= 0) & (yi < H)
    if inside.sum() < 40:
        return 0.0, 0.0, 0, np.arange(ts.size)[inside][:0]
    idx = np.where(inside)[0]
    xs, ys = xi[idx], yi[idx]
    on = mask[ys, xs] > 0

    def run_len(x, y, sgn):
        w = 0
        for k in range(0 if sgn > 0 else 1, probe):
            px, py = int(round(x + n[0] * sgn * k)), int(round(y + n[1] * sgn * k))
            if 0 <= px < W and 0 <= py < H and mask[py, px] > 0:
                w += 1
            else:
                break
        return w

    sample = idx[::max(1, len(idx) // 12)]
    thick = [max(run_len(xi[i], yi[i], 1), run_len(xi[i], yi[i], -1)) for i in sample]
    return float(on.mean()), float(np.median(thick)), int(on.sum()), idx


def lines_from_mask(mask, rho_res=1.0, theta_res=0.5, min_votes=250,
                    min_cover=0.45, max_thick=40.0, min_run=70, ang_tol_deg=5.0,
                    rho_tol=18.0):
    H, W = mask.shape
    cands = []
    for rho, theta in hough_lines(mask, rho_res, theta_res, min_votes):
        cover, thick, run, idx = line_support(mask, rho, theta)
        if cover >= min_cover and thick <= max_thick and run >= min_run:
            n = np.array([np.cos(theta), np.sin(theta)])
            u = np.array([-n[1], n[0]])
            base = n * rho
            ts = np.arange(-H - W, H + W, 2.0)
            pts = base[:, None] + np.outer(u, ts)
            if idx.size < 2:
                continue
            p0, p1 = pts[:, idx[0]], pts[:, idx[-1]]
            cands.append(dict(rho=rho, theta=theta, ang=float(np.arctan2(u[1], u[0])),
                              cover=cover, thick=thick, run=run, p0=p0, p1=p1,
                              length=float(np.hypot(*(p1 - p0)))))
    cands.sort(key=lambda c: -c["run"])
    kept = []
    for c in cands:
        dup = False
        for k in kept:
            da = abs(np.degrees(_ang_diff(c["ang"], k["ang"])))
            da = min(da, 180.0 - da)
            if da <= ang_tol_deg and abs(c["rho"] - k["rho"]) <= rho_tol:
                dup = True
                break
        if not dup:
            kept.append(c)
    return [c for c in kept if c["length"] >= 60.0]


def _xy_of(mask):
    ys, xs = np.nonzero(mask)
    return np.column_stack([xs, ys]).astype(float)


def fit_centerline(xy, rho, theta, band=25.0, min_pts=120):
    n = np.array([np.cos(theta), np.sin(theta)])
    sel = xy[np.abs(xy @ n - rho) <= band]
    if len(sel) < min_pts:
        return None
    mean = sel.mean(axis=0)
    q = sel - mean
    w, V = np.linalg.eigh(q.T @ q / max(len(q), 1))
    u = V[:, -1]
    nrm = np.array([-u[1], u[0]])
    ts = q @ u
    return dict(ang=float(np.arctan2(u[1], u[0])), rho=float(nrm @ mean), n=nrm, u=u,
                tmin=float(ts.min()), tmax=float(ts.max()), support=len(sel),
                rms=float(np.sqrt(max(w[0], 0))))


def refine_lines(mask, lines, band=25.0, ang_tol_deg=3.0, rho_tol=45.0,
                 min_length=150.0, max_rms=12.0):
    xy = _xy_of(mask)
    fits = []
    for ln in lines:
        f = fit_centerline(xy, ln["rho"], ln["theta"], band=band)
        if f is None or f["rms"] > max_rms:
            continue
        f["cover"], f["thick"], f["rho_raw"] = ln["cover"], ln["thick"], ln["rho"]
        fits.append(f)
    fits.sort(key=lambda f: -f["support"])
    kept = []
    for f in fits:
        dup = False
        for k in kept:
            da = abs(np.degrees(_ang_diff(f["ang"], k["ang"])))
            da = min(da, 180.0 - da)
            if da <= ang_tol_deg and abs(f["rho"] - k["rho"]) <= rho_tol:
                dup = True
                break
        if not dup:
            kept.append(f)
    out = []
    for f in kept:
        base = f["n"] * f["rho"]
        p0, p1 = base + f["u"] * f["tmin"], base + f["u"] * f["tmax"]
        length = float(np.hypot(*(p1 - p0)))
        if length < min_length:
            continue
        out.append(dict(p0=p0, p1=p1, ang=f["ang"], rho=f["rho"], length=length,
                        cover=f["cover"], thick=f["thick"], support=f["support"],
                        rms=f["rms"]))
    return out


def _ang_diff(a, b):
    d = (a - b + np.pi) % (2 * np.pi) - np.pi
    return d


def detect_lines(img, method="tophat", radius=13, percentile=97.5, min_row_ratio=0.0,
                 v_min=175, s_max=55, min_votes=250,
                 min_cover=0.45, max_thick=40.0, min_run=70, band=25.0,
                 ang_tol_deg=3.0, rho_tol=45.0, min_length=150.0, max_rms=12.0):
    if method == "tophat":
        mask, _, _ = tophat_mask(img, radius=radius, percentile=percentile)
    else:
        mask = white_mask(img, v_min=v_min, s_max=s_max, min_row_ratio=min_row_ratio)
    raw = lines_from_mask(mask, min_votes=min_votes, min_cover=min_cover,
                          max_thick=max_thick, min_run=min_run,
                          ang_tol_deg=ang_tol_deg, rho_tol=rho_tol)
    return refine_lines(mask, raw, band=band, ang_tol_deg=ang_tol_deg, rho_tol=rho_tol,
                        min_length=min_length, max_rms=max_rms), mask


def draw(img, lines, path=None, scale=1):
    canvas = img.copy()
    for i, ln in enumerate(lines):
        c = (0, 0, 255) if i % 2 == 0 else (255, 128, 0)
        cv2.line(canvas, tuple(ln["p0"].astype(int)), tuple(ln["p1"].astype(int)), c, 3)
        mid = ((ln["p0"] + ln["p1"]) / 2).astype(int)
        cv2.putText(canvas, f"{i}:{np.degrees(ln['ang']):.0f}", tuple(mid),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
    return canvas
