"""观测标志 <-> 地图标志 的匹配与二维位姿求解（SE(2)）。

思路（不做"按类别找最近点"）：
  1. 取观测中的任意两点 (i, j)（基线足够长）；
  2. 在地图里找**同类型**的点对 (a, b)，要求距离一致 |d_map - d_obs| < pair_len_tol；
  3. 每个可行组合直接解一个刚体变换（2 点定 SE(2)：平移 + 旋转）；
  4. 用该候选位姿把所有观测投到地图，逐点找最近同类型地图点，
     距离 < inlier_thresh 记内点，按置信度/距离加权打分；
  5. 取内点最多、分数最高的候选，用内点做 Gauss-Newton 精修；
  6. 检查歧义（第二个不同位姿的内点/分数接近 -> ambiguous）；
  7. 残差过大 -> high_residual；内点不足 -> insufficient_markers。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .field_map import FieldMap  # noqa: F401  (类型提示用)


# ----------------------------------------------------------------------
@dataclass
class Observation:
    """一个标志在载体系下的地面观测点。"""
    type: str
    x: float
    y: float
    conf: float = 1.0
    range_m: float = 0.0
    pixel: tuple = (0.0, 0.0)

    def xy(self):
        return np.array([self.x, self.y], dtype=np.float64)


@dataclass
class MatchResult:
    valid: bool = False
    status: str = "not_run"
    x: float = 0.0
    y: float = 0.0
    yaw_deg: float = 0.0
    marker_count: int = 0
    residual: float = 0.0
    inliers: list = field(default_factory=list)      # [{obs_index, map_id, dist}]
    n_obs: int = 0
    n_candidates: int = 0
    second_score_ratio: float = 0.0
    second_gap_m: float = 0.0

    def as_dict(self):
        return {
            "valid": self.valid, "status": self.status,
            "x_carrier_map": round(self.x, 3),
            "y_carrier_map": round(self.y, 3),
            "yaw_carrier_map": round(self.yaw_deg, 2),
            "marker_count": self.marker_count,
            "residual": round(self.residual, 3),
        }


def _rot(th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s], [s, c]], dtype=np.float64)


def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


# ----------------------------------------------------------------------
class Locator:
    def __init__(self, field_map, pair_len_tol=0.35, min_baseline=0.40,
                 inlier_thresh=0.45, max_rms=0.35, min_inliers=3,
                 allow_two_markers=True, ambiguity_score_ratio=0.85,
                 ambiguity_pos_m=0.6, ambiguity_yaw_deg=8.0,
                 prior_gate_m=1.0, prior_gate_yaw_deg=35.0, bounds=None):
        self.map = field_map
        self.pair_len_tol = float(pair_len_tol)
        self.min_baseline = float(min_baseline)
        self.inlier_thresh = float(inlier_thresh)
        self.max_rms = float(max_rms)
        self.min_inliers = int(min_inliers)
        self.allow_two_markers = bool(allow_two_markers)
        self.ambiguity_score_ratio = float(ambiguity_score_ratio)
        self.ambiguity_pos_m = float(ambiguity_pos_m)
        self.ambiguity_yaw = math.radians(float(ambiguity_yaw_deg))
        # 上一帧位姿先验：用于消解对称场地的 180 度歧义
        self.prior_gate_m = float(prior_gate_m)
        self.prior_gate_yaw = math.radians(float(prior_gate_yaw_deg))
        # (xmin, ymin, xmax, ymax)：解算出的位姿超出此范围就判无效，
        # 防止"地图只测了半场、却在没测的半场里给出一个自信的错误位姿"
        self.bounds = tuple(bounds) if bounds else None

    # ------------------------------------------------------------------
    def _score(self, obs, th, t):
        R = _rot(th)
        inliers, score = [], 0.0
        for idx, o in enumerate(obs):
            p = R @ o.xy() + t
            best_m, best_d = None, 1e9
            for m in self.map.by_type(o.type):
                d = math.hypot(p[0] - m.x, p[1] - m.y)
                if d < best_d:
                    best_d, best_m = d, m
            if best_m is not None and best_d <= self.inlier_thresh:
                w = o.conf / (1.0 + o.range_m / 5.0)
                score += w * (1.0 - 0.5 * best_d / self.inlier_thresh)
                inliers.append({"obs_index": idx, "map_id": best_m.id,
                                "dist": float(best_d)})
        return score, inliers

    # ------------------------------------------------------------------
    def _candidates(self, obs):
        cands = []
        n = len(obs)
        for i in range(n):
            for j in range(i + 1, n):
                pi, pj = obs[i].xy(), obs[j].xy()
                d_obs = float(np.linalg.norm(pj - pi))
                if d_obs < self.min_baseline:
                    continue
                ang_obs = math.atan2(pj[1] - pi[1], pj[0] - pi[0])
                for a in self.map.by_type(obs[i].type):
                    for b in self.map.by_type(obs[j].type):
                        if a.id == b.id:
                            continue
                        pa = np.array([a.x, a.y])
                        pb = np.array([b.x, b.y])
                        d_map = float(np.linalg.norm(pb - pa))
                        if abs(d_map - d_obs) > self.pair_len_tol:
                            continue
                        ang_map = math.atan2(pb[1] - pa[1], pb[0] - pa[0])
                        th = _wrap(ang_map - ang_obs)
                        t = pa - _rot(th) @ pi
                        # 第二点校验（同一刚体变换下应重合）
                        res = float(np.linalg.norm(_rot(th) @ pj + t - pb))
                        if res > self.pair_len_tol:
                            continue
                        cands.append((th, t, (i, j), (a.id, b.id), res))
        return cands

    # ------------------------------------------------------------------
    @staticmethod
    def _dedupe(cands, pos_step=0.4, yaw_step_deg=5.0):
        best = {}
        yaw_step = math.radians(yaw_step_deg)
        for c in cands:
            th, t = c[0], c[1]
            key = (round(t[0] / pos_step), round(t[1] / pos_step),
                   round(_wrap(th) / yaw_step))
            if key not in best:
                best[key] = c
        return list(best.values())

    # ------------------------------------------------------------------
    def _refine(self, obs, inliers, th, t, iters=6):
        pts = [o.xy() for o in obs]
        ms = [np.array([self.map.get(inf["map_id"]).x,
                        self.map.get(inf["map_id"]).y]) for inf in inliers]
        idx = [inf["obs_index"] for inf in inliers]
        w0 = [obs[k].conf / (1.0 + obs[k].range_m / 5.0) for k in idx]
        for _ in range(iters):
            c, s = math.cos(th), math.sin(th)
            R = np.array([[c, -s], [s, c]])
            Rp = np.array([[-s, -c], [c, -s]])          # dR/dth
            H = np.zeros((3, 3))
            g = np.zeros(3)
            for k, (pi, mi, wi) in enumerate(zip(idx, ms, w0)):
                p = pts[pi]
                r = R @ p + t - mi
                J = np.zeros((2, 3))
                J[:, 0] = [1.0, 0.0]
                J[:, 1] = [0.0, 1.0]
                J[:, 2] = Rp @ p
                nr = float(np.linalg.norm(r))
                w = wi if nr < 0.15 else wi * 0.15 / max(nr, 1e-6)
                H += w * (J.T @ J)
                g += w * (J.T @ r)
            try:
                d = np.linalg.solve(H + 1e-6 * np.eye(3), -g)
            except np.linalg.LinAlgError:
                break
            t = t + d[:2]
            th = _wrap(th + d[2])
            if np.linalg.norm(d) < 1e-5:
                break
        return th, t

    # ------------------------------------------------------------------
    def locate(self, observations, prior_pose=None):
        """observations: list[Observation]；prior_pose=(x, y, yaw_deg) 可选（仅用于记录）。"""
        res = MatchResult(n_obs=len(observations))
        obs = self._nms(observations)
        if len(obs) < 2:
            res.status = "insufficient_markers"
            res.marker_count = len(obs)
            return res

        cands = self._dedupe(self._candidates(obs))
        res.n_candidates = len(cands)
        if not cands:
            res.status = "no_match"
            return res

        scored = []
        for c in cands:
            th, t = c[0], c[1]
            score, inliers = self._score(obs, th, t)
            if len(inliers) >= 2:
                scored.append({"th": th, "t": t, "score": score,
                               "inliers": inliers})
        if not scored:
            res.status = "no_match"
            return res

        scored.sort(key=lambda s: (len(s["inliers"]), s["score"]), reverse=True)

        # ---- 有先验时：在内点并列的候选里挑离先验最近的那个 ----
        best = scored[0]
        used_prior = False
        prior_xy = prior_yaw = None
        if prior_pose is not None:
            px, py, pyaw = prior_pose
            prior_xy = np.array([px, py])
            prior_yaw = math.radians(pyaw)
            top_n = len(scored[0]["inliers"])
            near = []
            for s in scored:
                if len(s["inliers"]) < top_n:
                    continue
                dpos = float(np.linalg.norm(s["t"] - prior_xy))
                dyaw = abs(math.degrees(_wrap(s["th"] - prior_yaw)))
                if dpos <= self.prior_gate_m and dyaw <= math.degrees(self.prior_gate_yaw):
                    near.append((dpos, s))
            if near:
                near.sort(key=lambda kv: kv[0])
                best = near[0][1]
                used_prior = True
        for s in scored:
            s.setdefault("score_adj", s["score"])

        # ---- 歧义：存在另一个"不同位姿"的解，内点不更少且分数接近 ----
        worst_ratio = 0.0
        gap = 1e9
        for other in scored:
            if other is best:
                continue
            dpos = float(np.linalg.norm(best["t"] - other["t"]))
            dyaw = abs(math.degrees(_wrap(best["th"] - other["th"])))
            if not (dpos > self.ambiguity_pos_m or dyaw > math.degrees(self.ambiguity_yaw)):
                continue
            if len(other["inliers"]) < len(best["inliers"]):
                continue
            if used_prior:
                # 先验已经排除掉的解，不算歧义；
                # 只有当两个解都落在先验门限内，才承认真的分不开
                dpos_p = float(np.linalg.norm(other["t"] - prior_xy))
                dyaw_p = abs(math.degrees(_wrap(other["th"] - prior_yaw)))
                if dpos_p > self.prior_gate_m or dyaw_p > math.degrees(self.prior_gate_yaw):
                    continue
            ratio = other.get("score_adj", other["score"]) / max(best.get("score_adj", best["score"]), 1e-9)
            worst_ratio = max(worst_ratio, ratio)
            gap = min(gap, dpos)
        res.second_score_ratio = worst_ratio
        res.second_gap_m = 0.0 if gap > 1e8 else gap

        th, t = self._refine(obs, best["inliers"], best["th"], best["t"])
        score, inliers = self._score(obs, th, t)
        if len(inliers) < 2:                       # 精修后崩了，退回原解
            th, t = best["th"], best["t"]
            score, inliers = best["score"], best["inliers"]

        rms = math.sqrt(sum(i["dist"] ** 2 for i in inliers) / max(1, len(inliers)))

        res.x = float(t[0])
        res.y = float(t[1])
        res.yaw_deg = float(math.degrees(_wrap(th)))
        res.marker_count = len(inliers)
        res.residual = float(rms)
        res.inliers = inliers

        if self.bounds is not None:
            xmin, ymin, xmax, ymax = self.bounds
            if not (xmin <= res.x <= xmax and ymin <= res.y <= ymax):
                res.valid = False
                res.status = "outside_map"
                return res

        if worst_ratio >= self.ambiguity_score_ratio and res.marker_count >= 2:
            res.valid = False
            res.status = "ambiguous"
        elif res.marker_count < 2:
            res.valid = False
            res.status = "insufficient_markers"
        elif res.marker_count < self.min_inliers:
            if self.allow_two_markers and worst_ratio < self.ambiguity_score_ratio:
                res.valid = True
                res.status = "ok_low_markers"
            else:
                res.valid = False
                res.status = "insufficient_markers"
        elif rms > self.max_rms:
            res.valid = False
            res.status = "high_residual"
        else:
            res.valid = True
            res.status = "ok_prior" if used_prior else "ok"
        return res

    # ------------------------------------------------------------------
    @staticmethod
    def _nms(obs, dist_m=0.30):
        """同一位置重复检出的同类标志只留置信度最高的一个。"""
        out = []
        for o in sorted(obs, key=lambda o: -o.conf):
            if any(o.type == p.type and math.hypot(o.x - p.x, o.y - p.y) < dist_m
                   for p in out):
                continue
            out.append(o)
        return out