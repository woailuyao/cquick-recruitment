"""单应初始化的严格实现（6 步）。

1. 从当前检测中选 4 个点
2. 从地图中枚举「类别相同、互不重复」的 4 个点
3. 由这 4 组对应算 图像 → 场地平面 的单应矩阵 H
4. 用 H 检查剩余检测点能否匹配到地图点
5. 用全部匹配点重新拟合 H
6. 比较不同候选解，判断是否存在对称歧义

坐标
    图像：像素 (u, v)
    场地：地图平面 (X, Y)，单位 m，Z=0
    H：把 [u, v, 1] 映射到 [X, Y, 1]

本模块只做「检测 ↔ 地图」的对应与单应，不涉及位姿分解。
"""

import itertools

import cv2
import numpy as np


def _class_of(name):
    return (name or "")[:1].upper()


def _markers_by_type(markers):
    by = {}
    for m in markers:
        t = m.get("type")
        if t == "P":
            continue
        by.setdefault(t, []).append(m)
    return by


def _quad_assignments(det_quad, by_type):
    """第 2 步：给定 4 个检测，枚举同类且互不重复的地图四点。"""
    groups = {}
    for idx, d in enumerate(det_quad):
        cls = _class_of(d.get("class_name"))
        if cls not in by_type:
            return []
        groups.setdefault(cls, []).append(idx)

    keys = list(groups)
    cand_lists = [by_type[k] for k in keys]
    ranges = [itertools.permutations(range(len(c)), len(groups[k]))
              for k, c in zip(keys, cand_lists)]

    out = []
    for assign in itertools.product(*ranges):
        used, pairs, ok = set(), [], True
        for k, idxs in zip(keys, assign):
            cand = by_type[k]
            for di, mi in zip(groups[k], idxs):
                if (k, mi) in used:
                    ok = False
                    break
                used.add((k, mi))
                pairs.append((det_quad[di], cand[mi]))
            if not ok:
                break
        if ok:
            out.append(pairs)
    return out


def _h_from_pairs(pairs):
    """第 3 步：4 组对应 → 图像到场地的单应（精确 4 点解）。"""
    src = np.array([list(d["pixel"]) for d, _ in pairs], np.float32)
    dst = np.array([[m["x"], m["y"]] for _, m in pairs], np.float32)
    return cv2.getPerspectiveTransform(src, dst)


def _apply_h(H, pixel):
    p = np.array([pixel[0], pixel[1], 1.0], float)
    q = H @ p
    if abs(q[2]) < 1e-12:
        return None
    return q[0] / q[2], q[1] / q[2]


def _match_rest(H, dets, used_det, by_type, used_marker, tol_m):
    """第 4 步：用 H 把剩余检测投到场地平面，看能否匹配到同类地图点。"""
    matches = []
    for i, d in enumerate(dets):
        if i in used_det:
            continue
        cls = _class_of(d.get("class_name"))
        if cls not in by_type:
            continue
        xy = _apply_h(H, d["pixel"])
        if xy is None or not np.all(np.isfinite(xy)):
            continue
        best, best_d = None, tol_m
        for mi, m in enumerate(by_type[cls]):
            if (cls, mi) in used_marker:
                continue
            dist = float(np.hypot(m["x"] - xy[0], m["y"] - xy[1]))
            if dist < best_d:
                best, best_d = (cls, mi), dist
        if best is not None:
            matches.append((i, best, by_type[cls][best[1]], best_d))
    return matches


def _refit(H, pairs):
    """第 5 步：用全部匹配点重新拟合 H（≥4 组时用最小二乘）。"""
    if len(pairs) < 4:
        return H, None, None
    src = np.array([list(d["pixel"]) for d, _ in pairs], np.float32)
    dst = np.array([[m["x"], m["y"]] for _, m in pairs], np.float32)
    H2, _ = cv2.findHomography(src, dst, 0)
    if H2 is None or not np.all(np.isfinite(H2)):
        return H, None, None
    proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H2).reshape(-1, 2)
    err = np.linalg.norm(proj - dst, axis=1)
    return H2, float(np.median(err)), float(np.sqrt(np.mean(err ** 2)))


def init_by_homography(dets, markers, tol_m=0.35, max_quad_subsets=40,
                       max_assignments=4000, ambiguity_ratio=0.25,
                       min_points=4, height=None, K=None, dist=None,
                       field=None, height_tol=0.40):
    """执行 6 步，返回最优对应、H 以及歧义判定。

    dets   : [{"class_name": "Lcross", "pixel": (u,v), ...}]
    markers: field_c0.yaml 的 markers 列表（含 type / x / y / id）
    """
    if len(dets) < min_points:
        return {"status": "INSUFFICIENT",
                "reason": "至少需要 %d 个检测才能算单应" % min_points}

    by_type = _markers_by_type(markers)

    # ---- 第 1 步：枚举「选 4 个检测」 ----
    quads = list(itertools.combinations(range(len(dets)), 4))
    if len(quads) > max_quad_subsets:
        quads = quads[:max_quad_subsets]

    solutions = []
    tried = 0
    for quad_idx in quads:
        det_quad = [dets[i] for i in quad_idx]

        # ---- 第 2 步：同类、互不重复的地图四点 ----
        for pairs in _quad_assignments(det_quad, by_type):
            tried += 1
            if tried > max_assignments:
                break

            # ---- 第 3 步：4 点单应 ----
            H0 = _h_from_pairs(pairs)
            if H0 is None or not np.all(np.isfinite(H0)):
                continue

            used_det = set(quad_idx)
            # 已被这 4 组占用的地图点（按类别 + 下标标识）
            used_marker = set()
            for d, m in pairs:
                cls = _class_of(d.get("class_name"))
                for mi, cand in enumerate(by_type[cls]):
                    if cand is m:
                        used_marker.add((cls, mi))
                        break

            # ---- 第 4 步：剩余点验证 ----
            extra = _match_rest(H0, dets, used_det, by_type, used_marker, tol_m)

            # ---- 第 5 步：全部匹配点重拟合 ----
            all_pairs = list(pairs) + [(dets[i], m) for i, _, m, _ in extra]
            H1, med, rms = _refit(H0, all_pairs)
            if med is None:
                continue

            sol = {
                "H": H1,
                "pairs": all_pairs,
                "inliers": len(all_pairs),
                "matched_extra": len(extra),
                "residual_median": med,
                "residual_rms": rms,
                "quad": quad_idx,
                "matched_ids": [(dets[i], by_type[c][mi]["id"] if "id" in by_type[c][mi]
                                 else "%s%d" % (c, mi))
                                for i, (c, mi), _, _ in extra],
            }
            # 已知高度时：先用 H 分解出位姿，高度不符的直接淘汰
            if height is not None and K is not None and dist is not None:
                pose = pose_from_homography(H1, K, dist, all_pairs, field=field,
                                            height=height, height_tol=height_tol)
                if pose is None:
                    continue
                sol["pose"] = pose
            solutions.append(sol)
        if tried > max_assignments:
            break

    if not solutions:
        return {"status": "NO_SOLUTION", "reason": "没有可用候选", "tried": tried}

    # ---- 第 6 步：比较候选，判断对称歧义 ----
    solutions.sort(key=lambda s: (-s["inliers"], s["residual_median"]))
    best = solutions[0]
    ambiguous = False
    second = None
    for s in solutions[1:]:
        if s["inliers"] != best["inliers"]:
            continue
        rel = abs(s["residual_median"] - best["residual_median"]) / max(
            best["residual_median"], 1e-6)
        if rel < ambiguity_ratio:
            # 残差接近且匹配结果不同 → 对称歧义
            same = {id(p[1]) for p in s["pairs"]} == {id(p[1]) for p in best["pairs"]}
            if not same:
                ambiguous = True
                second = s
                break

    return {
        "status": "AMBIGUOUS" if ambiguous else "VALID",
        "best": best,
        "second": second,
        "candidate_count": len(solutions),
        "tried_assignments": tried,
    }


# ------------------------------------------------------------------ 第 7 步：H → 位姿
def _yaw_pitch_roll(R_camera_world):
    """R（camera→world）→ (yaw, pitch, roll) 度。z 轴为光轴。"""
    R = np.asarray(R_camera_world, float)
    z = R[:, 2]
    yaw = float(np.degrees(np.arctan2(z[1], z[0])))
    pitch = float(np.degrees(np.arcsin(max(-1.0, min(1.0, z[2])))))
    x0 = np.cross(z, [0.0, 0.0, 1.0])
    n = np.linalg.norm(x0)
    if n < 1e-9:
        return yaw, pitch, 0.0
    x0 = x0 / n
    xa = R[:, 0]
    roll = float(np.degrees(np.arctan2(float(xa @ np.cross(z, x0)),
                                       float(xa @ x0))))
    return yaw, pitch, roll


def _project(obj_xyz, R_cw, C, K, dist):
    rvec, _ = cv2.Rodrigues(R_cw.T)
    uv, _ = cv2.projectPoints(np.asarray(obj_xyz, float).reshape(-1, 3),
                              rvec, -R_cw.T @ np.asarray(C, float), K, dist)
    return uv.reshape(-1, 2)


def decompose_homography(H, K):
    """H ~ K [r1 r2 t] → 两个候选（镜像解相反）。返回 [(R_cw, C), ...]。"""
    A = np.linalg.inv(np.asarray(K, float)) @ np.asarray(H, float)
    n1, n2 = np.linalg.norm(A[:, 0]), np.linalg.norm(A[:, 1])
    if n1 < 1e-12 or n2 < 1e-12:
        return []
    lam = 2.0 / (n1 + n2)
    r1, r2, tt = A[:, 0] * lam, A[:, 1] * lam, A[:, 2] * lam
    out = []
    for sgn in (1.0, -1.0):
        M = np.column_stack([sgn * r1, sgn * r2, np.cross(sgn * r1, sgn * r2)])
        U, _, Vt = np.linalg.svd(M)
        Rct = U @ np.diag([1.0, 1.0, float(np.sign(np.linalg.det(U @ Vt)))]) @ Vt
        t = sgn * tt
        R_cw = Rct.T
        C = -R_cw @ t
        out.append((R_cw, C))
    return out


def pose_from_homography(H, K, dist, pairs, field=None,
                         height=None, height_tol=0.40,
                         max_roll=35.0, pitch_range=(-40.0, 5.0),
                         field_margin=0.5, verbose=False):
    """H → 位姿：分解、镜像筛选、物理校验、按重投影残差选优。

    pairs : [(det, marker)]，用于算重投影残差
    height: 已知相机高度（m）；给了就按 height ± height_tol 过滤
    返回 {"C","R_cw","R_wc","yaw","pitch","roll","rms"} 或 None
    """
    if H is None or not np.all(np.isfinite(H)):
        return None
    K = np.asarray(K, float)
    obj = np.array([[m["x"], m["y"], 0.0] for _, m in pairs], float)
    img = np.array([list(d["pixel"]) for d, _ in pairs], float)

    L = field["length"] / 2.0 + field_margin if field else 1e9
    W = field["width"] / 2.0 + field_margin if field else 1e9

    cands = []
    for i, (R_cw, C) in enumerate(decompose_homography(H, K)):
        why = []
        if C[2] <= 0:
            why.append("相机在地面下")
        if height is not None and abs(C[2] - height) > height_tol:
            why.append("高度 %.2f 偏离 %.2f" % (C[2], height))
        if abs(C[0]) > L or abs(C[1]) > W:
            why.append("位置出场")
        yaw, pitch, roll = _yaw_pitch_roll(R_cw)
        if abs(roll) > max_roll:
            why.append("roll %.0f° 过大" % roll)
        if not (pitch_range[0] <= pitch <= pitch_range[1]):
            why.append("pitch %.0f° 不合" % pitch)

        uv = _project(obj, R_cw, C, K, dist)
        rms = float(np.sqrt(np.mean(np.sum((uv - img) ** 2, axis=1))))
        if verbose:
            print("   [H分解%d] C=(%+6.2f,%+6.2f,%+6.2f) yaw %+7.1f pitch %+6.1f roll %+6.1f"
                  "  重投影 %7.2f px  %s"
                  % (i, C[0], C[1], C[2], yaw, pitch, roll, rms,
                     "通过" if not why else "否决: " + "; ".join(why)))
        if why:
            continue
        cands.append({"C": C, "R_cw": R_cw, "R_wc": R_cw.T,
                      "yaw": yaw, "pitch": pitch, "roll": roll, "rms": rms})

    if not cands:
        return None
    cands.sort(key=lambda s: s["rms"])
    return cands[0]
