import numpy as np


def line_from_points(p, q):
    l = np.cross(np.array([p[0], p[1], 1.0]), np.array([q[0], q[1], 1.0]))
    n = np.hypot(l[0], l[1])
    return l / n if n > 0 else l


def point_rows(P, x):
    Ph = np.array([P[0], P[1], 1.0])
    r1 = np.concatenate([-Ph, np.zeros(3), x[0] * Ph])
    r2 = np.concatenate([np.zeros(3), -Ph, x[1] * Ph])
    return [r1, r2]


def _line_sample_points(a, b, n):
    ts = np.linspace(0.0, 1.0, n)
    return [np.array([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, 1.0]) for t in ts]


def line_pair_rows(map_endpoints, m):
    rows = []
    for P in _line_sample_points(map_endpoints[0], map_endpoints[1], 2):
        rows.append(np.kron(m, P))
    return rows


def build_dlt(point_pairs, line_pairs):
    rows = []
    for P, x in point_pairs:
        rows += point_rows(P, x)
    for (a, b), m in line_pairs:
        rows += line_pair_rows((a, b), m)
    return np.array(rows, dtype=float)


def dlt_rank(point_pairs, line_pairs):
    A = build_dlt(point_pairs, line_pairs)
    if A.size == 0:
        return 0, 0
    s = np.linalg.svd(A, compute_uv=False)
    rank = int((s > 1e-9 * (s[0] if s.size else 1.0)).sum())
    return A.shape[0], rank


def solve_homography(point_pairs, line_pairs):
    A = build_dlt(point_pairs, line_pairs)
    if A.shape[0] < 8:
        return None
    _, s, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    n = np.linalg.norm(H)
    return H / n if n > 0 else None


def decompose(H, K):
    A = np.linalg.inv(K) @ H
    a1, a2, a3 = A[:, 0], A[:, 1], A[:, 2]
    lam = 2.0 / (np.linalg.norm(a1) + np.linalg.norm(a2))
    out = []
    for sign in (1.0, -1.0):
        l = lam * sign
        r1, r2, t = l * a1, l * a2, l * a3
        r3 = np.cross(r1, r2)
        R = np.column_stack([r1, r2, r3])
        U, _, Vt = np.linalg.svd(R)
        R = U @ Vt
        if np.linalg.det(R) < 0:
            Vt[-1] *= -1
            R = U @ Vt
        out.append((R, t))
    return out


def euler_from_R(R_wc):
    z = R_wc.T @ np.array([0.0, 0.0, 1.0])
    x = R_wc.T @ np.array([1.0, 0.0, 0.0])
    yaw = np.degrees(np.arctan2(z[1], z[0]))
    pitch = np.degrees(np.arcsin(np.clip(-z[2], -1.0, 1.0)))
    up = np.array([0.0, 0.0, 1.0])
    x_ref = np.cross(z, up)
    n = np.linalg.norm(x_ref)
    if n < 1e-9:
        return yaw, pitch, 0.0
    x_ref = x_ref / n
    y_ref = np.cross(z, x_ref)
    roll = np.degrees(np.arctan2(float(x @ y_ref), float(x @ x_ref)))
    return yaw, pitch, roll


def pose_from_homography(H, K, ground_points, min_front_frac=0.3):
    g = np.asarray(ground_points, dtype=float).reshape(-1, 2)
    P3 = np.column_stack([g[:, 0], g[:, 1], np.zeros(len(g))])
    cands = []
    for R, t in decompose(H, K):
        C = -R.T @ t
        cam = (R @ P3.T).T + t
        yaw, pitch, roll = euler_from_R(R)
        ok = C[2] > 0.05 and (cam[:, 2] > 0.1).mean() >= min_front_frac
        cands.append(dict(R=R, t=t, C=C, yaw=yaw, pitch=pitch, roll=roll, physical=ok))
    good = [c for c in cands if c["physical"]]
    if not good:
        return None
    best = max(good, key=lambda c: c["C"][2])
    best["ambiguous"] = len(good) > 1
    return best


def project_point(H, P):
    x = H @ np.array([P[0], P[1], 1.0])
    return x[:2] / x[2]


def project_line(H, l):
    m = np.linalg.inv(H).T @ np.asarray(l, float)
    n = np.hypot(m[0], m[1])
    return m / n if n > 0 else m


def point_line_distance(p, m):
    return abs(m[0] * p[0] + m[1] * p[1] + m[2])
