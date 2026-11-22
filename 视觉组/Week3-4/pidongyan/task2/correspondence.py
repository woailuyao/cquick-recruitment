"""对应关系解算：检测类别只有"类"没有"编号"（L_cross 有 8 个候选点），
需要在不预先知道相机位姿的前提下联合解出「谁是谁 + 相机在哪」。

策略（窄视场实测教训：单帧往往只有 2~5 个标志点可见）：
  1. 假设生成：枚举「4 个检测点的子集 × 其候选点分配」，用 P3P 解
     （OpenCV P3P 需要 4 点：3 点生假设、第 4 点消歧；Generic 接口
      可能返回多个解，全收）。候选乘积小的子集优先（X 类只有 2 个
     候选、点球点唯一），预算内几乎必然覆盖真值假设。
  2. 假设评估：在候选位姿下把**所有**检测点做一次整体重投影，每个检测取
     误差最小候选，按（内点数, 平均误差）打分。
  3. 精化：软分配 -> 内点 PnP -> 再软分配，迭代 3 轮。
  4. 先验消歧：多假设并立时，用测量得到的相机高度/俯仰角先验
     （--h-prior / --pitch-prior）给假设加权/剔除。
"""
import itertools

import cv2
import numpy as np

from .geometry import pose_from_Rt


def _hypothesis_solutions(names_xy, uvs, K, dist):
    """4 点 P3P，返回 [(R, C)]（0~4 个解）。"""
    obj = np.array([[*xy, 0.0] for _, xy in names_xy],
                   dtype=np.float64).reshape(-1, 1, 3)
    img = np.asarray(uvs, dtype=np.float64).reshape(-1, 1, 2)
    try:
        res = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_P3P)
    except cv2.error:
        return []
    if len(res) < 3 or not res[0]:
        return []
    rvecs, tvecs = res[1], res[2]
    sols = []
    for rvec, tvec in zip(rvecs, tvecs):
        R, _ = cv2.Rodrigues(rvec)
        C, _, _ = pose_from_Rt(R, tvec)
        sols.append((R, C))
    return sols


def solve_pose_pnp(names_xy, uvs, K, dist, prefer_h=None):
    """已知对应关系的平面 PnP（≥4 点）。返回 (ok, R, C, per_point_err, rms)。

    prefer_h:
      None  -> 原有单解 SOLVEPNP_IPPE 行为。**RANSAC 内部精化走这条，零改动**，
               避免影响 selftest 与已验证的检测链路。
      数值（米，现场卷尺实测的相机光心高度）-> 用 solvePnPGeneric 取 IPPE 的
               全部解，按「h>0 优先 / rms 最小 / 最接近 prefer_h」选物理可置信解。
               平面 PnP 天然存在地下镜像分支（h<0、pitch<0，位姿看起来像另一个
               机位），clicks 模式用它消歧；2026-11-20 pose_B 实测
               无先验 RANSAC 会给出 h=-0.737 的镜像解。
    """
    obj = np.array([[*xy, 0.0] for _, xy in names_xy],
                   dtype=np.float64).reshape(-1, 1, 3)
    img = np.asarray(uvs, dtype=np.float64).reshape(-1, 1, 2)

    if prefer_h is None:
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            return False, None, None, None, None
        candidates = [(rvec, tvec)]
    else:
        res = cv2.solvePnPGeneric(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if len(res) < 3 or not res[0]:
            return False, None, None, None, None
        candidates = list(zip(res[1], res[2]))

    best = None                      # (score, R, C, err, rms)
    for rvec, tvec in candidates:
        R, _ = cv2.Rodrigues(rvec)
        C, _, _ = pose_from_Rt(R, tvec)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
        rms = float(np.sqrt((err ** 2).mean()))
        if prefer_h is not None and C[2] <= 0:
            score = (-1e9 - rms, 0.0)          # 地下镜像解降权，但全地下时仍可返回
        elif prefer_h is not None:
            score = (-rms, -abs(C[2] - prefer_h))
        else:
            score = (-rms, 0.0)
        if best is None or score > best[0]:
            best = (score, R, C, err, rms)
    if best is None:
        return False, None, None, None, None
    _, R, C, err, rms = best
    return True, R, C, err, rms


class _Scorer:
    """在候选位姿下把所有检测一次性重投影并做软分配。"""

    def __init__(self, dets, candidates, K, dist):
        self.uvs = np.array([uv for _, uv in dets], dtype=np.float64)
        self.K, self.dist = K, dist
        self.segments = []                      # 每个检测的候选数
        self.names_flat = []                    # 所有候选点名（顺序展开）
        obj = []
        for cands in candidates:
            self.segments.append(len(cands))
            for n, xy in cands:
                self.names_flat.append(n)
                obj.append([xy[0], xy[1], 0.0])
        self.obj = np.array(obj, dtype=np.float64).reshape(-1, 1, 3)
        self.offsets = np.cumsum([0] + self.segments)
        self.tol = 1e-6

    def eval(self, R, C):
        """返回 (n_inlier, mean_err, errs[k], names[k])。"""
        rvec, _ = cv2.Rodrigues(R)
        tvec = -R @ np.asarray(C).reshape(3)
        proj, _ = cv2.projectPoints(self.obj, rvec, tvec, self.K, self.dist)
        proj = proj.reshape(-1, 2)
        errs, names = [], []
        for i, uv in enumerate(self.uvs):
            lo, hi = self.offsets[i], self.offsets[i + 1]
            e = np.linalg.norm(proj[lo:hi] - uv, axis=1)
            j = int(np.argmin(e))
            errs.append(float(e[j]))
            names.append(self.names_flat[lo + j])
        errs = np.array(errs)
        return errs, names


def ransac_pose_from_detections(dets, field_by_class, K, dist,
                                px_thresh=10.0, max_combos=20000,
                                prior=None, seed=0):
    """从「类别 + 像素」检测解相机位姿。

    dets: [(cls_name, (u, v)), ...]      ≥4 个
    field_by_class: {cls: [(点名, xy), ...]}
    prior: dict(h=光心高度先验, h_sigma=容差, pitch_deg=俯仰先验, pitch_sigma)
            —— 多假设并立时靠它消歧；None 表示不用先验。
    返回 None 或 dict(R_CF, C_F, yaw_deg, pitch_deg, names, errs,
                      inlier, n_inlier, rms)。
    """
    dets = [(c, uv) for c, uv in dets if c in field_by_class]
    if len(dets) < 4:
        return None
    candidates = [field_by_class[c] for c, _ in dets]
    if any(len(c) == 0 for c in candidates):
        return None
    k = len(dets)
    counts = [len(c) for c in candidates]

    scorer = _Scorer(dets, candidates, K, dist)

    # ---- 先验评分函数 ----
    def prior_penalty(R, C):
        if not prior:
            return 0.0
        pen = 0.0
        if prior.get("h") is not None:
            pen += abs(C[2] - prior["h"]) / max(prior.get("h_sigma", 0.1), 1e-6)
        if prior.get("pitch_deg") is not None:
            fwd = R[2, :]
            pitch = np.degrees(np.arcsin(np.clip(-fwd[2], -1, 1)))
            pen += (abs(pitch - prior["pitch_deg"])
                    / max(prior.get("pitch_sigma", 8.0), 1e-6))
        return pen

    # ---- 假设生成：4 点子集按候选乘积升序枚举 ----
    quads = []
    for quad in itertools.combinations(range(k), 4):
        prod = int(np.prod([counts[i] for i in quad]))
        quads.append((prod, quad))
    quads.sort()

    best = None          # (score, R, C)
    n_solves = 0
    budget_left = max_combos
    uvs = [uv for _, uv in dets]
    for prod, quad in quads:
        if prod > 1024 or budget_left <= 0:
            continue
        cand_idx = [range(counts[i]) for i in quad]
        for assign in itertools.product(*cand_idx):
            budget_left -= 1
            if budget_left < 0:
                break
            names_xy = [candidates[i][a] for i, a in zip(quad, assign)]
            uv4 = [uvs[i] for i in quad]
            for R, C in _hypothesis_solutions(names_xy, uv4, K, dist):
                n_solves += 1
                pen = prior_penalty(R, C)
                if prior and pen > 3.0:       # 与先验差 3σ 以上，直接弃
                    continue
                errs, _ = scorer.eval(R, C)
                n_in = int((errs < px_thresh).sum())
                score = (n_in - 0.001 * pen, -float(errs.mean()))
                if best is None or score > best[0]:
                    best = (score, R, C)
        if best is not None and best[0][0] >= k and n_solves > 30:
            break                       # 已有全内点解，够用
    if n_solves == 0 or best is None:
        return None
    if best[0][0] < min(4, k):
        return None

    # ---- 精化：软分配 -> 内点 PnP（≥4 点时）-> 迭代 ----
    _, R, C = best
    name_xy = {n: xy for cands in candidates for n, xy in cands}
    for _ in range(3):
        errs, names = scorer.eval(R, C)
        mask = errs < px_thresh
        if mask.sum() < 3:
            break
        names_xy = [(names[i], name_xy[names[i]]) for i in np.where(mask)[0]]
        uv_in = [uvs[i] for i in np.where(mask)[0]]
        if mask.sum() >= 4:
            ok, R2, C2, _, _ = solve_pose_pnp(names_xy, uv_in, K, dist)
        else:
            sols = _hypothesis_solutions(names_xy, uv_in, K, dist)
            if sols:
                R2, C2 = sols[0]
                ok = True
            else:
                ok = False
        if not ok:
            break
        R, C = R2, C2

    errs, names = scorer.eval(R, C)
    mask = errs < px_thresh
    if mask.sum() < 3:
        return None

    C_f, yaw, pitch = pose_from_Rt(R, (-R @ np.asarray(C).reshape(3)).reshape(3, 1))
    return {
        "R_CF": R, "C_F": np.asarray(C).reshape(3),
        "yaw_deg": yaw, "pitch_deg": pitch,
        "names": names, "errs": errs, "inlier": mask,
        "n_inlier": int(mask.sum()),
        "rms": float(np.sqrt((errs[mask] ** 2).mean())),
    }
