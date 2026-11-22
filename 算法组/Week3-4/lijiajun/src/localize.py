import math

import numpy as np

import ground_projection as GP

PAR_TOL_DEG = 12.0
MIN_UNKNOWNS = 3


def ground_line(pixels, K, dist, height_m, pitch_deg, roll_deg):
    g, _, _ = GP.pixel_to_ground(np.asarray(pixels, float).reshape(-1, 2), K, dist,
                                 height_m, pitch_deg, roll_deg)
    a, b = g[0], g[1]
    v = b - a
    L = float(np.hypot(*v))
    if L < 1e-6:
        return None
    u = v / L
    n = np.array([-u[1], u[0]])
    t0, t1 = float(u @ a), float(u @ b)
    return dict(a=a, b=b, u=u, n=n, rho=float(n @ a), length=L,
                tmin=min(t0, t1), tmax=max(t0, t1))


def ground_point(pixel, K, dist, height_m, pitch_deg, roll_deg):
    g, _, _ = GP.pixel_to_ground(np.asarray([pixel], float), K, dist,
                                 height_m, pitch_deg, roll_deg)
    return g[0]


def pair_relation(l1, l2):
    c = max(-1.0, min(1.0, abs(float(l1["u"] @ l2["u"]))))
    d = math.degrees(math.acos(c))
    if d <= PAR_TOL_DEG:
        return "parallel", d
    if 90.0 - d <= PAR_TOL_DEG:
        return "cross", d
    return "other", d


def direction_rows(lines_meas, lines_map):
    rows = []
    for lm, mm in zip(lines_meas, lines_map):
        ux, uy = lm["u"]
        rows.append([ux, uy] if mm["fam"] == "H" else [-uy, ux])
    return np.array(rows, float)


def position_rows(lines_meas, lines_map, points_meas, points_map, s, c):
    M, b = [], []
    for lm, mm in zip(lines_meas, lines_map):
        nx, ny = lm["n"]
        M.append([nx * c - ny * s, nx * s + ny * c, lm["rho"]])
        b.append(mm["fixed"])
    for pm, qm in zip(points_meas, points_map):
        px, py = pm
        M.append([1.0, 0.0, c * px - s * py])
        b.append(qm[0])
        M.append([0.0, 1.0, s * px + c * py])
        b.append(qm[1])
    return np.array(M, float), np.array(b, float)


def _extent_excess(lm, mm, s, c, k, C):
    q0 = np.array([c * k * lm["a"][0] - s * k * lm["a"][1],
                   s * k * lm["a"][0] + c * k * lm["a"][1]]) + C
    q1 = np.array([c * k * lm["b"][0] - s * k * lm["b"][1],
                   s * k * lm["b"][0] + c * k * lm["b"][1]]) + C
    axis = 0 if mm["fam"] == "H" else 1
    lo, hi = min(mm["lo"], mm["hi"]), max(mm["lo"], mm["hi"])
    return max(0.0, lo - min(q0[axis], q1[axis])) + max(0.0, max(q0[axis], q1[axis]) - hi)


def _branch(lines_meas, lines_map, points_meas, points_map, s, c, free_scale, slack=0.35):
    M, b = position_rows(lines_meas, lines_map, points_meas, points_map, s, c)
    if not free_scale:
        b = b - M[:, 2]
        M = M[:, :2]
    sol, res, rank, sv = np.linalg.lstsq(M, b, rcond=None)
    Cx, Cy = float(sol[0]), float(sol[1])
    k = float(sol[2]) if free_scale else 1.0
    C = np.array([Cx, Cy])
    off = M @ sol - b
    rms = float(np.sqrt(np.mean(off ** 2))) if off.size else 0.0
    exc = sum(_extent_excess(lm, mm, s, c, k, C) for lm, mm in zip(lines_meas, lines_map))
    dof = len(b) - int(rank)
    return dict(C=(Cx, Cy), k=k, yaw=math.degrees(math.atan2(s, c)),
                residual_m=rms, extent_excess_m=float(exc), dof=dof,
                rank=int(rank), n_obs=len(b), n_unk=M.shape[1],
                per_obs=[float(v) for v in np.round(off, 4)], free_scale=free_scale)


def solve_pose(lines_meas, lines_map, points_meas=(), points_map=(), free_scale=False,
               height_m=None):
    lines_meas = list(lines_meas)
    points_meas = list(points_meas)
    n_obs = len(lines_meas) + 2 * len(points_meas)
    n_unk = 3 if free_scale else 2
    if len(lines_meas) < 1 or n_obs < n_unk + 1:
        return None, (f"underdetermined: {n_obs} 约束 vs {n_unk} 未知量，"
                      f"需要至少 {n_unk + 1} 个约束才有冗余")
    A = direction_rows(lines_meas, lines_map)
    _, sv, Vt = np.linalg.svd(A)
    if sv[-1] > 0.30 * sv[0]:
        return None, "inconsistent_families"
    sc = Vt[-1] / np.hypot(*Vt[-1])
    branches = [_branch(lines_meas, lines_map, points_meas, points_map,
                        sign * sc[0], sign * sc[1], free_scale) for sign in (1.0, -1.0)]
    if free_scale:
        branches = [b for b in branches if b["k"] > 0.05]
        if not branches:
            return None, "negative_scale: 两个分支都要求负等比，说明族配对或外参有误"
    if len(branches) < 2:
        branches = branches + [dict(branches[0], score=float("inf"),
                                    residual_m=float("inf"), extent_excess_m=0.0)]
    for br in branches:
        br["score"] = br["residual_m"] + 0.5 * br["extent_excess_m"]
    branches.sort(key=lambda x: x["score"])
    best, rival = branches[0], branches[1]
    if best["rank"] < best["n_unk"]:
        return None, (f"rank_deficient: 有效秩 {best['rank']} < 未知数 "
                      f"{best['n_unk']}，所选元素彼此相关（例如交点就在所选两条线上）")
    if best["dof"] < 1:
        return None, f"no_redundancy: {best['n_obs']} 观测只有 {best['dof']} 冗余，无法校验"
    ok_res = best["residual_m"] < 0.30 and best["extent_excess_m"] < 0.60
    best["status"] = ("ok" if ok_res and best["score"] + 0.15 < rival["score"]
                      else ("high_residual" if not ok_res else "ambiguous"))
    best["margin"] = rival["score"] - best["score"]
    best["rival_yaw"] = rival["yaw"]
    best["rival_C"] = rival["C"]
    best["height_implied_m"] = (spacing_implied_height(lines_meas, lines_map, height_m)
                                if height_m else None)
    return best, None


def spacing_implied_height(lines_meas, lines_map, height_m):
    by_fam = {}
    for lm, mm in zip(lines_meas, lines_map):
        by_fam.setdefault(mm["fam"], []).append((lm, mm))
    ratios = []
    for fam, grp in by_fam.items():
        for i in range(len(grp)):
            for j in range(i + 1, len(grp)):
                meas = abs(grp[i][0]["rho"] - grp[j][0]["rho"])
                mp = abs(grp[i][1]["fixed"] - grp[j][1]["fixed"])
                if meas > 0.05:
                    ratios.append(mp / meas)
    if not ratios:
        return None
    return height_m * float(np.median(ratios))


def to_map(pose, meas_lines, meas_points):
    s, c = math.sin(math.radians(pose["yaw"])), math.cos(math.radians(pose["yaw"]))
    C = np.array(pose["C"], float)
    k = pose["k"]
    tf = lambda p: np.array([c * k * p[0] - s * k * p[1], s * k * p[0] + c * k * p[1]]) + C
    return [(tf(lm["a"]), tf(lm["b"])) for lm in meas_lines], [tf(p) for p in meas_points]


def build_pair_table(points, lines):
    table = []
    for i in range(len(lines)):
        for j in range(i + 1, len(lines)):
            a, b = lines[i], lines[j]
            if a["fam"] == b["fam"]:
                lo = max(min(a["lo"], a["hi"]), min(b["lo"], b["hi"]))
                hi = min(max(a["lo"], a["hi"]), max(b["lo"], b["hi"]))
                if hi - lo < 0.5:
                    continue
                table.append(dict(kind="parallel", fam=a["fam"], lines=(a, b),
                                  sep=abs(a["fixed"] - b["fixed"]), overlap=(lo, hi)))
            else:
                Hl, Vl = (a, b) if a["fam"] == "H" else (b, a)
                xi, yi = Vl["fixed"], Hl["fixed"]
                if not (min(Hl["lo"], Hl["hi"]) <= xi <= max(Hl["lo"], Hl["hi"]) and
                        min(Vl["lo"], Vl["hi"]) <= yi <= max(Vl["lo"], Vl["hi"])):
                    continue
                named = [nm for nm, v in points.items()
                         if abs(v[0] - xi) < 1e-3 and abs(v[1] - yi) < 1e-3]
                table.append(dict(kind="cross", lines=(Hl, Vl),
                                  q=np.array([xi, yi]), name=named[0] if named else None))
    return table


def candidate_map_lines(l1, l2, table):
    rel, d = pair_relation(l1, l2)
    if rel == "other":
        return rel, d, []
    out = []
    for e in table:
        if e["kind"] != rel:
            continue
        out.append(e)
    return rel, d, out
