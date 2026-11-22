#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
pose_solver.py —— 用观测到的场地标志反解相机(机器人)在地图里的位姿

为什么不能"按类别最近邻硬配"：
    地图里有 6 个 TCross、5 个 LCross，它们**同类之间完全无法区分**。
    如果你把检测到的第一个 TCross 配给地图里最近的 TCross，一旦错一位，
    整个位姿就会平移一米甚至镜像翻转，而且残差看起来还挺小——这是最坑的错法。

正确做法（也是考核文档要求的）：
    1) 枚举"两两配对"的假设：任取两个观测 + 任取两个同类的地图标志，
       由这两对点直接解出一个 SE(2) 位姿（旋转 + 平移，共 3 个自由度）；
    2) 用这个假设去给**所有**观测做一致性投票（最近点 + 距离阈值内的算 inlier）；
    3) 按 inlier 数量选最优，再用全部 inlier 做最小二乘精修；
    4) 检查"是否存在第二个同样好、但位姿差很远的假设"——有就是 ambiguous，
       并如实报出来，而不是硬给一个数。

这就是 RANSAC 在 SE(2) 上的具体形态。
"""

import itertools
import math

import numpy as np


class Observation(object):
    """一个检测到的场地标志（已经过几何细化 + 地面投影）。"""

    __slots__ = ("cls_id", "cls_name", "p_body", "pixel", "confidence", "depth")

    def __init__(self, cls_id, cls_name, p_body, pixel=None, confidence=1.0, depth=None):
        self.cls_id = int(cls_id)
        self.cls_name = cls_name
        self.p_body = np.asarray(p_body, dtype=float).reshape(2)
        self.pixel = None if pixel is None else np.asarray(pixel, dtype=float).reshape(2)
        self.confidence = float(confidence)
        self.depth = depth

    def __repr__(self):
        return "Obs(%s @ %.2f,%.2f conf=%.2f)" % (
            self.cls_name, self.p_body[0], self.p_body[1], self.confidence)


# ---------------------------------------------------------------------------
def _se2_from_pair(p1, p2, m1, m2):
    """由两对点求 SE(2)：使 R(psi) p + t 尽量把 p→m。"""
    d_p = np.asarray(p2) - np.asarray(p1)
    d_m = np.asarray(m2) - np.asarray(m1)
    if np.linalg.norm(d_p) < 1e-6 or np.linalg.norm(d_m) < 1e-6:
        return None
    psi = math.atan2(d_m[1], d_m[0]) - math.atan2(d_p[1], d_p[0])
    psi = math.atan2(math.sin(psi), math.cos(psi))
    c, s = math.cos(psi), math.sin(psi)
    R = np.array([[c, -s], [s, c]])
    t = np.asarray(m1) - R @ np.asarray(p1)
    return psi, R, t


def _fit_se2(P, M):
    """最小二乘 SE(2)（Kabsch 2D），P/M 为 N×2 对应点。"""
    P = np.asarray(P, dtype=float)
    M = np.asarray(M, dtype=float)
    pc, mc = P.mean(axis=0), M.mean(axis=0)
    H = (P - pc).T @ (M - mc)
    U, _S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:                 # 反射修正
        Vt[-1, :] *= -1.0
        R = Vt.T @ U.T
    t = mc - R @ pc
    psi = math.atan2(R[1, 0], R[0, 0])
    return psi, R, t


def _wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def _score_hypothesis(obs_pts, obs_cls, map_pts, map_cls, psi, R, t, tol,
                      l_as_t=False):
    """返回 (inliers 索引列表, 对应地图索引, 残差列表)。

    l_as_t: 允许"被检成 LCross 的观测"匹配 TCross 地图点。
            本场地 TCross 与 LCross 在 40 px 标注框里几乎无法区分（模型对 T 的召回极低），
            但两者几何上都只是"两条白线的交点"，只是臂数不同；这条兜底能让
            TCross 地图点不至于因为分类问题全部失效。默认关闭，由上层显式打开。
    """
    P = obs_pts @ R.T + t                     # 把观测投到地图
    d = np.linalg.norm(P[:, None, :] - map_pts[None, :, :], axis=2)
    same = (obs_cls[:, None] == map_cls[None, :])
    if l_as_t:
        same = same | ((obs_cls[:, None] == 0) & (map_cls[None, :] == 1))
    d = np.where(same, d, 1e9)
    nn = np.argmin(d, axis=1)
    nd = d[np.arange(len(P)), nn]
    inl = np.where(nd < tol)[0]
    return inl, nn, nd


def solve(obs_list, field_map, tol=0.35, ambiguity_penalty=0.6,
          prior=None, min_pair_sep=0.25, max_hypotheses=4000,
          bounds=None, bounds_margin=0.10, l_as_t=False,
          min_inliers=3, prior_gate_sigma=3.0):
    """
    主入口。

    obs_list : [Observation]，p_body 是"载体系下"的地面坐标（x 前 / y 左，米）
    field_map: FieldMap
    tol      : inlier 距离阈值(米)。场地标志位置误差 + 检测误差，0.3~0.5 合适
    prior    : 可选 (x, y, yaw_deg, sig_xy, sig_yaw)，给上一帧位姿做跟踪，可大幅降低歧义
    min_inliers      : 判定 valid 所需的最少内点数（细则要求 valid 仅当 ok 且内点 ≥3）
    prior_gate_sigma : 给了 prior 时，解与先验的偏差超过该倍数 σ 就判为不可信
                       （防止"只有 2 个观测时，一个错误配对把航向拧到 158°"还被当成有效解）
    bounds   : 可选 (x0, y0, x1, y1)，相机必须落在这个矩形内（默认 None 不约束）
               用途：只有 2 个标志时，2 点约束欠定，会出现"相机站在场外"的假解；
               把相机限制在场内能直接剔掉这类解。margin 向外放 bounds_margin 米。
    返回 dict：
        valid, x, y, yaw_deg, residual, marker_count, status, inliers, second
    """
    out = {"valid": False, "x": 0.0, "y": 0.0, "yaw_deg": 0.0, "residual": float("nan"),
           "marker_count": len(obs_list), "status": "lost", "inliers": [],
           "second": None, "n_hypotheses": 0}

    obs_list = [o for o in obs_list if o is not None]
    n = len(obs_list)
    if n == 0:
        out["status"] = "lost"
        return out
    if n < 2:
        out["status"] = "insufficient_markers"
        return out

    obs_pts = np.array([o.p_body for o in obs_list], dtype=float)
    obs_cls = np.array([o.cls_id for o in obs_list], dtype=int)
    map_pts = field_map.points
    map_cls = field_map.class_ids

    # 候选：(观测 i, 地图 a) 组合，按类别
    obs_by_cls = {}
    map_by_cls = {}
    for i, c in enumerate(obs_cls):
        obs_by_cls.setdefault(c, []).append(i)
    for j, c in enumerate(map_cls):
        map_by_cls.setdefault(c, []).append(j)

    hypotheses = []
    for i, k in itertools.combinations(range(n), 2):
        if np.linalg.norm(obs_pts[i] - obs_pts[k]) < min_pair_sep:
            continue
        for a in map_by_cls.get(obs_cls[i], []):
            for b in map_by_cls.get(obs_cls[k], []):
                if a == b:
                    continue
                r = _se2_from_pair(obs_pts[i], obs_pts[k], map_pts[a], map_pts[b])
                if r is None:
                    continue
                psi, R, t = r
                if bounds is not None:
                    bx0, by0, bx1, by1 = bounds
                    if not (bx0 - bounds_margin <= t[0] <= bx1 + bounds_margin and
                            by0 - bounds_margin <= t[1] <= by1 + bounds_margin):
                        continue
                if prior is not None:
                    px, py, pyaw, sxy, syaw = prior
                    if np.hypot(*(np.array([px, py]) - t)) > 2.0 * max(sxy, 0.2):
                        continue
                    if abs(_wrap_deg(math.degrees(psi) - pyaw)) > 2.0 * max(syaw, 5.0):
                        continue
                hypotheses.append((psi, R, t))
                if len(hypotheses) >= max_hypotheses:
                    break
            if len(hypotheses) >= max_hypotheses:
                break
        if len(hypotheses) >= max_hypotheses:
            break

    if not hypotheses:
        out["status"] = "insufficient_markers"
        return out
    out["n_hypotheses"] = len(hypotheses)

    # 逐个假设打分
    scored = []
    for psi, R, t in hypotheses:
        inl, nn, nd = _score_hypothesis(obs_pts, obs_cls, map_pts, map_cls, psi, R, t, tol, l_as_t)
        if len(inl) < 2:
            continue
        scored.append((len(inl), -float(np.median(nd[inl])), inl, nn, nd, psi, R, t))
    if not scored:
        out["status"] = "high_residual"
        return out

    scored.sort(key=lambda x: (x[0], x[1]), reverse=True)
    best = scored[0]
    n_inl, _neg_med, inl, nn, nd, psi, R, t = best

    # 用全部 inlier 精修
    P = obs_pts[inl]
    M = map_pts[nn[inl]]
    psi, R, t = _fit_se2(P, M)
    # 精修后可能丢掉部分 inlier，再算一次
    inl2, nn2, nd2 = _score_hypothesis(obs_pts, obs_cls, map_pts, map_cls, psi, R, t, tol, l_as_t)
    if len(inl2) >= 2:
        inl, nn, nd = inl2, nn2, nd2
        P = obs_pts[inl]
        M = map_pts[nn[inl]]
        psi, R, t = _fit_se2(P, M)
        inl, nn, nd = _score_hypothesis(obs_pts, obs_cls, map_pts, map_cls, psi, R, t, tol, l_as_t)

    x, y = float(t[0]), float(t[1])
    yaw_deg = math.degrees(psi)
    residual = float(np.sqrt(np.mean(nd[inl] ** 2))) if len(inl) else float("nan")

    out.update({
        "x": x, "y": y, "yaw_deg": _wrap_deg(yaw_deg),
        "residual": residual, "marker_count": len(inl),
        "inliers": [(int(i), int(nn[i]), float(nd[i])) for i in inl],
    })

    # ---- 硬门 1：内点数不足就不许输出位姿 ----
    # 细则要求"valid 仅当 status == ok 且内点 ≥ 3"。2 个观测时位姿欠定：
    # 任意一组同类配对都能把残差压到很小（实测出现过航向偏 158°、residual 仍只有 0.13 m）。
    if len(inl) < min_inliers:
        out["status"] = "insufficient_markers"
        out["valid"] = False
        out["marker_count"] = len(inl)
        return out

    # ---- 硬门 2：解必须与先验自洽（防止错误配对把航向锁到反方向）----
    if prior is not None:
        px, py, pyaw, sxy, syaw = prior
        d_prior = float(np.hypot(px - x, py - y))
        yaw_prior = abs(_wrap_deg(yaw_deg - pyaw))
        if d_prior > prior_gate_sigma * max(sxy, 0.2) or \
           yaw_prior > prior_gate_sigma * max(syaw, 5.0):
            out["status"] = "high_residual"
            out["valid"] = False
            out["reason"] = "prior_gate"
            out["prior_dev_m"] = d_prior
            out["prior_dev_deg"] = yaw_prior
            return out

    # ---- 歧义检查 ----
    # 找"和最优解一样能解释同样多观测，但位姿明显不同"的另一个假设。
    # 关键：**必须连残差都差不多**才算真歧义。若次优解的中位残差明显更大，
    # 说明它只是"碰巧也能配 2 个点"，不该让正确答案变成 ambiguous。
    best_med = -scored[0][1]
    second = None
    second_cand = None
    for cand in scored[1:]:
        c_ninl = cand[0]
        c_psi = cand[5]
        c_t = cand[7]
        c_med = -cand[1]
        if c_ninl < max(2, n_inl - 0):
            continue
        if c_med > best_med + 0.02:      # 拟合明显更差 -> 不是等价解
            continue
        dpos = float(np.hypot(c_t[0] - x, c_t[1] - y))
        dyaw = abs(_wrap_deg(math.degrees(c_psi) - yaw_deg))
        if dpos > ambiguity_penalty or dyaw > 10.0:
            second = {"x": float(c_t[0]), "y": float(c_t[1]),
                      "yaw_deg": _wrap_deg(math.degrees(c_psi)),
                      "inliers": c_ninl, "dpos": dpos, "dyaw": dyaw,
                      "med_residual": float(c_med)}
            second_cand = cand
            break

    # ---- 用先验消歧 ----
    # 本场地关于中线左右镜像对称，单帧观测必然存在两个等价解。
    # 若给了先验（上一帧位姿 / 启动粗位姿），且"次优解"明显更贴近先验，
    # 就采用次优解，并标记 resolved_by_prior（而不是一律判 ambiguous）。
    if second is not None and prior is not None:
        px, py, pyaw, sxy, syaw = prior
        d_best = float(np.hypot(px - x, py - y))
        d_sec = float(np.hypot(px - second["x"], py - second["y"]))
        yaw_best = abs(_wrap_deg(yaw_deg - pyaw))
        yaw_sec = abs(_wrap_deg(second["yaw_deg"] - pyaw))
        better_second = (d_sec + 0.30 < d_best) or (yaw_sec + 10.0 < yaw_best)
        # 反过来：最优解离先验很近、次优解远在先验门外 -> 直接采用最优解，
        # 不再把这一帧判成 ambiguous（镜像孪生解被先验排除掉了）
        gate_xy = 2.0 * max(sxy, 0.2)
        gate_yaw = 2.0 * max(syaw, 5.0)
        if (not better_second) and d_best <= gate_xy and yaw_best <= gate_yaw:
            out.update({
                "x": x, "y": y, "yaw_deg": _wrap_deg(yaw_deg),
                "residual": residual, "marker_count": len(inl),
                "inliers": [(int(i), int(nn[i]), float(nd[i])) for i in inl],
                "second": second, "resolved_by_prior": True,
            })
            if residual > max(0.5 * tol, 0.12):
                out["status"] = "high_residual"
                out["valid"] = False
                return out
            out["status"] = "ok"
            out["valid"] = True
            return out
        if better_second:
            c_ninl, c_negmed, c_inl, c_nn, c_nd, c_psi, c_R, c_t = second_cand
            x, y = float(c_t[0]), float(c_t[1])
            yaw_deg = math.degrees(c_psi)
            inl, nn, nd = c_inl, c_nn, c_nd
            residual = float(np.sqrt(np.mean(nd[inl] ** 2))) if len(inl) else float("nan")
            out.update({
                "x": x, "y": y, "yaw_deg": _wrap_deg(yaw_deg),
                "residual": residual, "marker_count": len(inl),
                "inliers": [(int(i), int(nn[i]), float(nd[i])) for i in inl],
                "second": second, "resolved_by_prior": True,
            })
            if residual > max(0.5 * tol, 0.12):
                out["status"] = "high_residual"
                out["valid"] = False
                return out
            out["status"] = "ok"
            out["valid"] = True
            return out

    # 残差门限：残差超过 0.5*tol 基本说明配错了（或地图/标定有问题）
    if residual > max(0.5 * tol, 0.12):
        out["status"] = "high_residual"
        out["valid"] = False
        out["second"] = second
        return out
    if second is not None:
        out["status"] = "ambiguous"
        out["valid"] = False
        out["second"] = second
        return out
    if len(inl) < 2:
        out["status"] = "insufficient_markers"
        out["valid"] = False
        return out

    out["status"] = "ok"
    out["valid"] = True
    return out


# ---------------------------------------------------------------------------
# 自测：造一组已知位姿的观测，看能否解回来（不依赖任何数据/模型）
# ---------------------------------------------------------------------------
def _selftest():
    try:
        from .field_map import FieldMap
    except ImportError:                      # 直接跑单文件时
        import os
        import sys
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from field_map import FieldMap

    markers = [
        ("TCross1", "TCross", 1, 5.955, 2.930),
        ("TCross2", "TCross", 1, 4.945, 2.930),
        ("TCross3", "TCross", 1, 1.000, 2.930),
        ("LCross1", "LCross", 0, 7.465, 2.930),
        ("LCross5", "LCross", 0, 5.955, 0.000),
        ("XCross1", "XCross", 2, 4.505, -4.010),
        ("PenalityPoint", "PenaltyPoint", 3, 2.965, 0.885),
    ]
    fm = FieldMap(markers)

    rng = np.random.default_rng(1)
    ok_cnt = 0
    for trial in range(200):
        gt = np.array([rng.uniform(0, 6), rng.uniform(-2, 2), rng.uniform(-180, 180)])
        c, s = math.cos(math.radians(gt[2])), math.sin(math.radians(gt[2]))
        R = np.array([[c, -s], [s, c]])
        # 只取视野前方 8 m 内的标志
        obs = []
        for name, cls_name, cid, mx, my in fm.markers:
            p_b = R.T @ (np.array([mx, my]) - gt[:2])
            if 0.3 < p_b[0] < 8.0 and abs(p_b[1]) < 4.0:
                obs.append(Observation(cid, cls_name, p_b + rng.normal(0, 0.02, 2)))
        if len(obs) < 3:
            continue
        res = solve(obs, fm, tol=0.35)
        if not res["valid"]:
            continue
        # 位姿是"等效"的：允许 yaw 差 360°
        dyaw = abs(_wrap_deg(res["yaw_deg"] - gt[2]))
        if abs(res["x"] - gt[0]) < 0.15 and abs(res["y"] - gt[1]) < 0.15 and dyaw < 2.0:
            ok_cnt += 1
        else:
            print("  [失败] gt=%s 解=%s" % (np.round(gt, 2),
                                          (round(res["x"], 2), round(res["y"], 2),
                                           round(res["yaw_deg"], 1))))
    print("自测：随机 %d 个位姿场景中成功 %d 个" % (200, ok_cnt))


if __name__ == "__main__":
    _selftest()
