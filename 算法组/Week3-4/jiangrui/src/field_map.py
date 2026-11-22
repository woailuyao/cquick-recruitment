# -*- coding: utf-8 -*-
"""field.yaml 加载 + 观测特征与地图特征的 SE(2) 位姿匹配。

流程:
    1. 观测点(载体坐标)与地图点按“类别”分组；
    2. 枚举两两观测 -> 两两同类地图点的对应关系，直接解出候选 (x, y, yaw)；
    3. 用全部观测对每个候选打分（内点数、残差），取最优；
    4. 对最优候选做最小二乘精化，并重新一一对应；
    5. 若存在另一个同样好、但位姿差很多的候选，判为 ambiguous。

这样处理了“同类标志多、场地对称”导致的身份歧义，而不是把每个框硬匹配到
最近的同类点。
"""

import math

import numpy as np

import minyaml

TYPE_ALIASES = {
    "t": "T", "tcross": "T", "t_cross": "T",
    "l": "L", "lcross": "L", "l_cross": "L",
    "x": "X", "xcross": "X", "x_cross": "X",
    "p": "P", "penalitypoint": "P", "penaltypoint": "P", "point": "P",
}


def normalize_type(name):
    if name is None:
        return None
    key = str(name).strip().lower().replace(" ", "").replace("-", "")
    return TYPE_ALIASES.get(key)


def _type_from_key(key):
    k = str(key).strip().lower()
    if k.startswith("tcross"):
        return "T"
    if k.startswith("lcross"):
        return "L"
    if k.startswith("xcross"):
        return "X"
    if k.startswith("penalitypoint") or k.startswith("penaltypoint"):
        return "P"
    return None


class FieldMap:
    def __init__(self, path):
        raw = minyaml.load(path) or {}
        self.path = path
        self.markers = []
        for key, value in raw.items():
            mtype = _type_from_key(key)
            if mtype is None:
                continue
            if not isinstance(value, (list, tuple)) or len(value) < 2:
                continue
            self.markers.append({
                "id": str(key),
                "type": mtype,
                "x": float(value[0]),
                "y": float(value[1]),
            })
        if not self.markers:
            raise SystemExit(f"field.yaml 里没有解析到标志点: {path}")

    def bounds(self, expand=1.5):
        xs = [m["x"] for m in self.markers]
        ys = [m["y"] for m in self.markers]
        return (min(xs) - expand, max(xs) + expand,
                min(ys) - expand, max(ys) + expand)


def _fit_rigid(points, targets):
    """2D 刚体最小二乘：target ≈ R(theta) * point + t。"""
    p = np.asarray(points, dtype=float)
    m = np.asarray(targets, dtype=float)
    pc = p.mean(axis=0)
    mc = m.mean(axis=0)
    h = (p - pc).T @ (m - mc)
    theta = math.atan2(h[0, 1] - h[1, 0], h[0, 0] + h[1, 1])
    c, s = math.cos(theta), math.sin(theta)
    rot = np.array([[c, -s], [s, c]], dtype=float)
    t = mc - rot @ pc
    return theta, t


def _wrap_deg(angle):
    return (angle + 180.0) % 360.0 - 180.0


class PoseSolver:
    def __init__(self, field, inlier_thresh=0.35, max_residual=0.30,
                 min_inliers=3, pair_tol=0.60,
                 ambiguity_m=0.40, ambiguity_deg=12.0):
        self.field = field
        self.inlier_thresh = float(inlier_thresh)
        self.max_residual = float(max_residual)
        self.min_inliers = int(min_inliers)
        self.pair_tol = float(pair_tol)
        self.ambiguity_m = float(ambiguity_m)
        self.ambiguity_deg = float(ambiguity_deg)

        self.types = sorted({m["type"] for m in field.markers})
        self.group_index = {}
        self.group_xy = {}
        for t in self.types:
            idx = [i for i, m in enumerate(field.markers) if m["type"] == t]
            self.group_index[t] = idx
            self.group_xy[t] = np.array(
                [[field.markers[i]["x"], field.markers[i]["y"]] for i in idx],
                dtype=float)

    # ------------------------------------------------------------------
    def solve(self, obs_types, obs_xy):
        obs_types = list(obs_types)
        xy_all = np.asarray(obs_xy, dtype=float).reshape(-1, 2)
        n_all = len(obs_types)
        result = {
            "valid": False, "status": "insufficient_markers",
            "x": float("nan"), "y": float("nan"), "yaw": float("nan"),
            "residual": float("nan"), "marker_count": 0,
            "matches": {}, "inliers": np.zeros(n_all, dtype=bool),
            "reason": "观测点不足",
        }

        # 只保留地图里存在的类别
        keep = [i for i, t in enumerate(obs_types)
                if t in self.group_xy and np.isfinite(xy_all[i]).all()]
        if len(keep) < 2:
            return result

        # 注意：这里的 xy 是载体坐标系（原点为相机光心地面投影），
        # 不能拿地图边界去筛；只丢掉明显异常的距离。
        keep = [i for i in keep if np.hypot(xy_all[i, 0], xy_all[i, 1]) < 30.0]
        if len(keep) < 2:
            result["status"] = "not_converged"
            result["reason"] = "地面投影点距离异常"
            return result

        types = [obs_types[i] for i in keep]
        xy = xy_all[keep]
        n = len(types)

        hyp = self._build_hypotheses(types, xy)
        if len(hyp) == 0:
            result["status"] = "not_converged"
            result["reason"] = "没有几何一致的候选位姿"
            return result

        H = np.asarray(hyp, dtype=float)
        theta, tx, ty = H[:, 0], H[:, 1], H[:, 2]
        c, s = np.cos(theta), np.sin(theta)
        X = c[:, None] * xy[None, :, 0] - s[:, None] * xy[None, :, 1] + tx[:, None]
        Y = s[:, None] * xy[None, :, 0] + c[:, None] * xy[None, :, 1] + ty[:, None]

        nearest_d = np.full((len(H), n), np.inf)
        nearest_gid = np.full((len(H), n), -1, dtype=int)
        for t in set(types):
            cols = [k for k, tt in enumerate(types) if tt == t]
            M = self.group_xy[t]
            d = np.hypot(X[:, cols, None] - M[None, None, :, 0],
                         Y[:, cols, None] - M[None, None, :, 1])
            nearest_d[:, cols] = d.min(axis=2)
            nearest_gid[:, cols] = np.array(self.group_index[t])[d.argmin(axis=2)]

        inl = nearest_d < self.inlier_thresh
        n_unique = np.zeros(len(H), dtype=int)
        for h in range(len(H)):
            n_unique[h] = len(set(nearest_gid[h][inl[h]].tolist()))
        mean_d = np.where(inl, nearest_d, 0.0).sum(axis=1) / np.maximum(inl.sum(axis=1), 1)
        mean_d = np.where(inl.sum(axis=1) > 0, mean_d, 1e9)

        order = np.lexsort((mean_d, -n_unique))
        best_h = int(order[0])
        if n_unique[best_h] < self.min_inliers:
            result["status"] = "insufficient_markers"
            result["reason"] = f"仅 {int(n_unique[best_h])} 个有效匹配"
            return result

        # 两轮：一一对应 -> 最小二乘拟合
        pose = (float(theta[best_h]), float(tx[best_h]), float(ty[best_h]))
        pairs = []
        for _ in range(2):
            pairs = self._assign(types, xy, pose)
            if len(pairs) < 2:
                break
            P = xy[[p[0] for p in pairs]]
            M = np.array([[self.field.markers[p[1]]["x"],
                           self.field.markers[p[1]]["y"]] for p in pairs])
            th_fit, t_fit = _fit_rigid(P, M)
            pose = (th_fit, t_fit[0], t_fit[1])

        if len(pairs) < self.min_inliers:
            result["status"] = "insufficient_markers"
            result["reason"] = f"精化后仅 {len(pairs)} 个有效匹配"
            return result

        th, ttx, tty = pose
        P = xy[[p[0] for p in pairs]]
        M = np.array([[self.field.markers[p[1]]["x"],
                       self.field.markers[p[1]]["y"]] for p in pairs])
        resid = np.linalg.norm(
            P @ np.array([[math.cos(th), math.sin(th)],
                          [-math.sin(th), math.cos(th)]]) + np.array([ttx, tty]) - M,
            axis=1)
        rmse = float(np.sqrt(np.mean(resid ** 2)))

        inlier_keep = np.zeros(n, dtype=bool)
        inlier_keep[[p[0] for p in pairs]] = True
        full_inliers = np.zeros(n_all, dtype=bool)
        full_inliers[keep] = inlier_keep

        yaw_deg = _wrap_deg(math.degrees(th))
        result.update({
            "x": float(ttx), "y": float(tty), "yaw": float(yaw_deg),
            "residual": rmse, "marker_count": len(pairs),
            "inliers": full_inliers,
        })
        for oi, gi, d in pairs:
            src = keep[oi]
            result["matches"][int(src)] = {
                "id": self.field.markers[gi]["id"],
                "type": self.field.markers[gi]["type"],
                "dist": float(d),
            }

        # 解出的位姿在地图坐标系里，这里才应该用场地边界检查
        bx0, bx1, by0, by1 = self.field.bounds(expand=2.0)
        if not (bx0 <= ttx <= bx1 and by0 <= tty <= by1):
            result["status"] = "not_converged"
            result["reason"] = "解出的位置在场地范围外"
            return result
        if rmse > self.max_residual:
            result["status"] = "high_residual"
            result["reason"] = f"匹配残差 {rmse:.2f} m 超过阈值"
            return result

        # 歧义检查：另一个同样优的候选，位姿却明显不同
        for k in range(min(len(order), 600)):
            h = int(order[k])
            if n_unique[h] < len(pairs) or mean_d[h] > rmse + 0.06:
                continue
            dxy = math.hypot(tx[h] - ttx, ty[h] - tty)
            dyaw = abs(_wrap_deg(math.degrees(theta[h]) - yaw_deg))
            if dxy > self.ambiguity_m or dyaw > self.ambiguity_deg:
                result["status"] = "ambiguous"
                result["reason"] = "存在同样优的对称候选位姿"
                result["ambiguous_with"] = {
                    "x": float(tx[h]), "y": float(ty[h]),
                    "yaw": float(_wrap_deg(math.degrees(theta[h]))),
                }
                return result

        result["valid"] = True
        result["status"] = "ok"
        result["reason"] = ""
        return result

    # ------------------------------------------------------------------
    def _build_hypotheses(self, types, xy):
        hyps = []
        n = len(xy)
        for i in range(n):
            for j in range(i + 1, n):
                dvec = xy[j] - xy[i]
                d_obs = float(np.linalg.norm(dvec))
                if d_obs < 0.15:
                    continue
                obs_ang = math.atan2(dvec[1], dvec[0])
                ma_list = self.group_xy[types[i]]
                mb_list = self.group_xy[types[j]]
                for a in range(len(ma_list)):
                    for b in range(len(mb_list)):
                        if types[i] == types[j] and a == b:
                            continue
                        dmap = mb_list[b] - ma_list[a]
                        d_map = float(np.linalg.norm(dmap))
                        if d_map < 1e-6 or abs(d_obs - d_map) > self.pair_tol:
                            continue
                        theta = math.atan2(dmap[1], dmap[0]) - obs_ang
                        c, s = math.cos(theta), math.sin(theta)
                        px, py = ma_list[a]
                        t = np.array([px, py]) - np.array(
                            [[c, -s], [s, c]]) @ xy[i]
                        hyps.append((theta, t[0], t[1]))
        if not hyps:
            return []
        arr = np.asarray(hyps, dtype=float)
        _, uniq = np.unique(np.round(arr, 3), axis=0, return_index=True)
        return arr[np.sort(uniq)]

    def _assign(self, types, xy, pose):
        """按距离做一一对应，返回 [(观测索引, 地图点全局索引, 距离)]。"""
        th, tx, ty = pose
        c, s = math.cos(th), math.sin(th)
        pred = np.column_stack([c * xy[:, 0] - s * xy[:, 1] + tx,
                                s * xy[:, 0] + c * xy[:, 1] + ty])
        cand = []
        for oi, t in enumerate(types):
            for gi in self.group_index[t]:
                gm = self.field.markers[gi]
                d = math.hypot(pred[oi, 0] - gm["x"], pred[oi, 1] - gm["y"])
                if d < self.inlier_thresh:
                    cand.append((d, oi, gi))
        used_obs, used_map, pairs = set(), set(), []
        for d, oi, gi in sorted(cand):
            if oi in used_obs or gi in used_map:
                continue
            used_obs.add(oi)
            used_map.add(gi)
            pairs.append((oi, gi, d))
        pairs.sort()
        return pairs
