"""位姿求解：观测到的场地标志（载体坐标）→ 地图位姿 (x, y, yaw)。

思路：
  1. 用「两对同类标志的间距 + 方向差」枚举候选位姿；
  2. 每个候选把全部观测点变换到地图系，按同类型最近邻统计内点与残差；
  3. 取内点最多、残差最小的候选，用 Kabsch 最小二乘精修；
  4. 内点不足或次优候选与最优同样好（对称歧义）→ 判为无效。

用法：
    python3 pose_solver.py --demo           # 用合成数据自测
    # 或在代码里调用 solve(observed, markers)

observed 格式（载体坐标系，米，x 向前 y 向左）：
    [{"type": "LCross", "x": 1.2, "y": -0.8, "conf": 0.9}, ...]
"""
import argparse
import itertools
import math

import numpy as np


def _rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _kabsch(P, Q):
    """求 R, t 使 R*P_i + t ≈ Q_i（2D 刚体）。"""
    pc = P.mean(axis=0)
    qc = Q.mean(axis=0)
    H = (P - pc).T @ (Q - qc)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    t = qc - R @ pc
    return R, t


def _apply(R, t, pts):
    return (R @ pts.T).T + t


def _candidates(obs, mp, max_dist_err=0.45):
    """枚举候选 (R, t)。obs/mp 为同结构列表。"""
    by_type_obs = {}
    for o in obs:
        by_type_obs.setdefault(o["type"], []).append(o)
    by_type_map = {}
    for m in mp:
        by_type_map.setdefault(m["type"], []).append(m)

    cands = []
    for typ, group in by_type_obs.items():
        if len(group) < 2:
            continue
        maps = by_type_map.get(typ, [])
        if len(maps) < 2:
            continue
        for oi, oj in itertools.combinations(range(len(group)), 2):
            po = np.array([group[oi]["x"], group[oi]["y"]])
            qo = np.array([group[oj]["x"], group[oj]["y"]])
            d_obs = np.linalg.norm(qo - po)
            if d_obs < 0.3:
                continue
            a_obs = math.atan2(qo[1] - po[1], qo[0] - po[0])
            for mi, mj in itertools.permutations(range(len(maps)), 2):
                pm = np.array([maps[mi]["x"], maps[mi]["y"]])
                qm = np.array([maps[mj]["x"], maps[mj]["y"]])
                d_map = np.linalg.norm(qm - pm)
                if abs(d_map - d_obs) > max_dist_err:
                    continue
                a_map = math.atan2(qm[1] - pm[1], qm[0] - pm[0])
                dyaw = a_map - a_obs
                R = _rot(dyaw)
                t = pm - R @ po
                cands.append((R, t, [type(group[oi]["type"])], d_obs))
    return cands


def _score(R, t, obs, mp, tol=0.6):
    """返回 (内点数, 残差rms, 最近匹配列表)。"""
    by_type = {}
    for m in mp:
        by_type.setdefault(m["type"], []).append(m)

    inliers = 0
    errs = []
    matches = []
    for o in obs:
        cand = by_type.get(o["type"], [])
        if not cand:
            matches.append(None)
            continue
        p = R @ np.array([o["x"], o["y"]]) + t
        best, best_d = None, 1e9
        for m in cand:
            d = math.hypot(p[0] - m["x"], p[1] - m["y"])
            if d < best_d:
                best, best_d = m, d
        if best is not None and best_d <= tol:
            inliers += 1
            errs.append(best_d)
            matches.append(best)
        else:
            matches.append(None)
    rms = float(np.sqrt(np.mean(np.square(errs)))) if errs else 9.99
    return inliers, rms, matches


def solve(observed, markers, tol=0.6, min_inliers=2,
          max_residual=0.35, ambiguous_ratio=0.8):
    """返回位姿字典。"""
    if len(observed) < 2:
        return {"valid": False, "status": "insufficient_markers",
                "marker_count": len(observed), "residual": None,
                "x": None, "y": None, "yaw": None}

    cands = _candidates(observed, markers)
    if not cands:
        return {"valid": False, "status": "not_converged",
                "marker_count": len(observed), "residual": None,
                "x": None, "y": None, "yaw": None}

    scored = []
    for R, t, _, _ in cands:
        inl, rms, matches = _score(R, t, observed, markers, tol)
        scored.append((inl, -rms, R, t, matches))
    scored.sort(key=lambda s: (s[0], s[1]), reverse=True)

    inl, neg_rms, R, t, matches = scored[0]
    if inl < min_inliers:
        return {"valid": False, "status": "insufficient_markers",
                "marker_count": inl, "residual": None,
                "x": None, "y": None, "yaw": None}

    # Kabsch 精修
    P, Q = [], []
    for o, m in zip(observed, matches):
        if m is not None:
            P.append([o["x"], o["y"]])
            Q.append([m["x"], m["y"]])
    R2, t2 = _kabsch(np.array(P), np.array(Q))
    inl2, rms2, matches2 = _score(R2, t2, observed, markers, tol)

    # 歧义检测：存在另一个位姿，内点相近但位置差异很大
    ambiguous = False
    for inl_b, neg_b, Rb, tb, _ in scored[1:6]:
        if inl_b < inl2:
            continue
        db = math.hypot(tb[0] - t2[0], tb[1] - t2[1])
        yaw_b = math.atan2(Rb[1, 0], Rb[0, 0])
        yaw_a = math.atan2(R2[1, 0], R2[0, 0])
        dyaw = abs((yaw_b - yaw_a + math.pi) % (2 * math.pi) - math.pi)
        if db > 0.6 or dyaw > math.radians(10):
            ambiguous = True
            break

    yaw = math.degrees(math.atan2(R2[1, 0], R2[0, 0]))
    res = {
        "x": float(t2[0]),
        "y": float(t2[1]),
        "yaw": float(yaw),
        "marker_count": int(inl2),
        "residual": float(rms2),
        "matches": [m["id"] if m else None for m in matches2],
        "valid": True,
        "status": "ok",
    }
    if normalized_yaw := ((yaw + 180) % 360) - 180:
        res["yaw"] = normalized_yaw
    if rms2 > max_residual:
        res["valid"] = False
        res["status"] = "high_residual"
    elif ambiguous:
        res["valid"] = False
        res["status"] = "ambiguous"
    return res


def _demo():
    markers = [
        {"id": "L1", "type": "LCross", "x": 4.02, "y": -3.0},
        {"id": "L2", "type": "LCross", "x": 4.02, "y": 3.0},
        {"id": "L3", "type": "LCross", "x": 3.09, "y": -1.495},
        {"id": "L4", "type": "LCross", "x": 3.09, "y": 1.495},
        {"id": "L5", "type": "LCross", "x": 2.09, "y": -1.985},
        {"id": "L6", "type": "LCross", "x": 2.09, "y": 1.985},
        {"id": "T5", "type": "TCross", "x": 0.0, "y": -3.0},
        {"id": "T6", "type": "TCross", "x": 0.0, "y": 3.0},
        {"id": "X1", "type": "XCross", "x": 0.0, "y": -0.755},
        {"id": "X2", "type": "XCross", "x": 0.0, "y": 0.755},
    ]
    # 真值：载体在地图 (1.2, 0.4)，yaw 30°
    true_x, true_y, true_yaw = 1.2, 0.4, math.radians(30)
    R = _rot(true_yaw)
    obs = []
    for m in markers:
        p_map = np.array([m["x"], m["y"]])
        p_body = R.T @ (p_map - np.array([true_x, true_y]))
        if p_body[0] > 0.2:  # 只保留相机前方的
            obs.append({"type": m["type"], "x": float(p_body[0]), "y": float(p_body[1])})

    print(f"合成真值: x={true_x}, y={true_y}, yaw={math.degrees(true_yaw):.1f}°")
    print(f"观测到 {len(obs)} 个标志")
    res = solve(obs[:5], markers)
    for k, v in res.items():
        print(f"  {k}: {v}")
    if res["valid"]:
        ex = res["x"] - true_x
        ey = res["y"] - true_y
        eyaw = res["yaw"] - math.degrees(true_yaw)
        print(f"\n误差: dx={ex:.3f} m, dy={ey:.3f} m, dyaw={eyaw:.2f}°")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--demo", action="store_true")
    args = ap.parse_args()
    if args.demo:
        _demo()
    else:
        print("这是一个库。用 --demo 自测，或在代码里 import solve()")


if __name__ == "__main__":
    main()
