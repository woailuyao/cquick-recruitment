"""任务二核心：把「看到的标志」和「地图上的标志」匹配，解出相机在地图里的位姿。

坐标约定（和 config/field_geometry.yaml 一致）：
    地图坐标系 M：原点 = 中圈圆心，+X 指向球门，+Y 朝左，单位米
    载体坐标系 B：原点 = 相机光心在地面的投影点，+X = 相机正前方，+Y = 左侧
    待求位姿：(x, y, yaw) 满足  P_map = R(yaw) @ P_carrier + [x, y]
    yaw 从地图 +X 轴逆时针为正，单位度。

为什么不能「按类别找最近点」：
    场地里有 6 个 T、5 个 L，左右还基本对称。随便配一对就可能"看着也挺顺"，
    但位姿完全不同（镜像解）。所以这里做的是：
      1) 用两对点生成候选位姿（每对都试一遍）
      2) 每个候选位姿算一个"代价"：对得上的点按距离计分，对不上的点按门限罚分
      3) 取代价最小的，再用最小二乘迭代精细化（门限逐步收紧）
      4) 检查"第二名"是不是几乎一样好 —— 是的话判为歧义，valid=False
"""

import math

import numpy as np

try:
    from .yaml_mini import load_yaml
except ImportError:  # pragma: no cover
    from yaml_mini import load_yaml

NAME_TO_TYPE = {"L": "L", "T": "T", "X": "X", "P": "P", "C": "C"}


def type_of(name):
    """把 yaml 里的键名（TCross3 / PenalityPoint / CenterSpot ...）归一成类别。"""
    if name.startswith("TCross"):
        return "T"
    if name.startswith("LCross"):
        return "L"
    if name.startswith("XCross"):
        return "X"
    if name in ("PenalityPoint", "PenaltyPoint"):
        return "P"
    if name in ("CenterSpot", "CenterPoint"):
        return "C"
    if name in NAME_TO_TYPE:
        return NAME_TO_TYPE[name]
    return "?"


class Feature:
    __slots__ = ("name", "type", "x", "y")

    def __init__(self, name, type_, x, y):
        self.name = name
        self.type = type_
        self.x = float(x)
        self.y = float(y)

    def __repr__(self):
        return "%s(%s, %.2f, %.2f)" % (self.name, self.type, self.x, self.y)


class FieldMap:
    def __init__(self, features):
        self.features = list(features)
        self.by_type = {}
        for f in self.features:
            self.by_type.setdefault(f.type, []).append(f)

    @classmethod
    def load(cls, path):
        data = load_yaml(path)
        feats = []
        for k, v in data.items():
            if not isinstance(v, (list, tuple)) or len(v) != 2:
                continue
            t = type_of(k)
            if t != "?":
                feats.append(Feature(k, t, v[0], v[1]))
        return cls(feats)

    def of(self, type_):
        return self.by_type.get(type_, [])

    def __len__(self):
        return len(self.features)


def _rot(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]])


def _align(obs_i, obs_j, map_a, map_b):
    """用两对点求刚体变换：obs_i → map_a，obs_j → map_b。"""
    v_obs = np.asarray(obs_j, float) - np.asarray(obs_i, float)
    v_map = np.asarray(map_b, float) - np.asarray(map_a, float)
    if np.linalg.norm(v_obs) < 1e-6:
        return None
    theta = math.atan2(v_map[1], v_map[0]) - math.atan2(v_obs[1], v_obs[0])
    theta = (theta + math.pi) % (2 * math.pi) - math.pi
    R = _rot(theta)
    t = np.asarray(map_a, float) - R @ np.asarray(obs_i, float)
    return theta, t


def _refine(pairs_obs, pairs_map):
    """最小二乘（SE2）：给若干对已配好的点，求最优 (θ, t)。"""
    A = np.asarray(pairs_obs, float)
    B = np.asarray(pairs_map, float)
    ca, cb = A.mean(0), B.mean(0)
    A0, B0 = A - ca, B - cb
    num = float(np.sum(A0[:, 0] * B0[:, 1] - A0[:, 1] * B0[:, 0]))
    den = float(np.sum(A0[:, 0] * B0[:, 0] + A0[:, 1] * B0[:, 1]))
    if abs(num) < 1e-12 and abs(den) < 1e-12:
        return None
    theta = math.atan2(num, den)
    R = _rot(theta)
    return theta, cb - R @ ca


def _assign(obs, fmap, theta, t, gating):
    """把每个观测配到最近的同类地图点上（超出 gating 算未匹配）。"""
    R = _rot(theta)
    inl = []
    for k, o in enumerate(obs):
        p = R @ np.array([o["x"], o["y"]], float) + t
        best_d, best_f = 1e9, None
        for f in fmap.of(o["type"]):
            d = math.hypot(p[0] - f.x, p[1] - f.y)
            if d < best_d:
                best_d, best_f = d, f
        if best_f is not None and best_d <= gating:
            inl.append((k, best_f, best_d))
    return inl


def _cost(obs, fmap, theta, t, gating):
    """连续代价：匹配上的点按距离计，未匹配的点按门限计（离群点的代价被截断）。"""
    inl = _assign(obs, fmap, theta, t, gating)
    matched = {k for k, _, _ in inl}
    cost = sum(d for _, _, d in inl)
    cost += gating * (len(obs) - len(matched))
    return cost, inl


def match(obs, fmap, gating=0.8, min_inliers=2, refine_iters=3, refine_ratio=0.6,
          bounds=None, bounds_margin=0.5,
          ambiguity_cost_ratio=1.5, ambiguity_pos=0.5, ambiguity_yaw=10.0,
          max_resid=0.35, max_hypotheses=6000):
    """观测 → 地图位姿。详见文件头注释。"""
    out = {
        "valid": False,
        "status": "insufficient_markers",
        "x": None,
        "y": None,
        "yaw_deg": None,
        "marker_count": 0,
        "residual": None,
        "inliers": [],
        "candidates": 0,
        "cost": None,
        "second_cost": None,
    }
    if len(obs) < min_inliers:
        return out

    hyps, seen = [], set()
    n = len(obs)
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = obs[i], obs[j]
            for a in fmap.of(oi["type"]):
                for b in fmap.of(oj["type"]):
                    if a is b:
                        continue
                    if len(hyps) >= max_hypotheses:
                        break
                    res = _align((oi["x"], oi["y"]), (oj["x"], oj["y"]), (a.x, a.y), (b.x, b.y))
                    if res is None:
                        continue
                    theta, t = res
                    key = (round(theta, 2), round(t[0], 1), round(t[1], 1))
                    if key in seen:
                        continue
                    seen.add(key)
                    hyps.append((theta, t))
    out["candidates"] = len(hyps)
    if not hyps:
        return out

    scored = []
    for theta, t in hyps:
        if bounds is not None and not _inside(t, bounds, bounds_margin):
            continue
        cost, inl = _cost(obs, fmap, theta, t, gating)
        if len(inl) < min_inliers:
            continue
        scored.append({"theta": theta, "t": t, "cost": cost})
    if not scored:
        return out

    scored.sort(key=lambda r: r["cost"])

    def polish(h):
        theta, t = h["theta"], h["t"]
        for it in range(refine_iters):
            g = gating * (refine_ratio ** it)
            inl = _assign(obs, fmap, theta, t, g)
            if len(inl) < 2:
                break
            refined = _refine([(obs[k]["x"], obs[k]["y"]) for k, _, _ in inl],
                              [(f.x, f.y) for _, f, _ in inl])
            if refined is None:
                break
            theta, t = refined
        cost, inl = _cost(obs, fmap, theta, t, gating)
        return theta, t, cost, inl

    polished = []
    for h in scored[:6]:
        theta, t, cost, inl = polish(h)
        polished.append({"theta": theta, "t": t, "cost": cost, "inl": inl})
    polished.sort(key=lambda r: r["cost"])

    best = polished[0]
    resid = float(np.mean([d for _, _, d in best["inl"]])) if best["inl"] else 9.9

    second_cost = None
    ambiguous = False
    for r in polished[1:]:
        dpos = math.hypot(r["t"][0] - best["t"][0], r["t"][1] - best["t"][1])
        dyaw = abs(math.degrees(_angle_diff(r["theta"] - best["theta"])))
        if dpos <= ambiguity_pos and dyaw <= ambiguity_yaw:
            continue
        second_cost = r["cost"]
        if r["cost"] <= best["cost"] * ambiguity_cost_ratio:
            ambiguous = True
        break

    out.update({
        "x": float(best["t"][0]),
        "y": float(best["t"][1]),
        "yaw_deg": float(math.degrees(_angle_diff(best["theta"]))),
        "marker_count": len(best["inl"]),
        "residual": resid,
        "inliers": [(obs[k]["type"], f.name, d) for k, f, d in best["inl"]],
        "cost": float(best["cost"]),
        "second_cost": float(second_cost) if second_cost is not None else None,
    })

    if ambiguous:
        out["valid"] = False
        out["status"] = "ambiguous"
        return out
    if len(best["inl"]) < 3:
        out["valid"] = False
        out["status"] = "insufficient_markers"
        return out
    if resid > max_resid:
        out["valid"] = False
        out["status"] = "high_residual"
        return out

    out["valid"] = True
    out["status"] = "ok"
    return out


def _inside(t, bounds, margin):
    """位姿必须落在场地范围内（半场之外不可能，直接否掉；能干掉一多半镜像解）。"""
    x_min, x_max, y_min, y_max = bounds
    return (x_min - margin) <= t[0] <= (x_max + margin) and (y_min - margin) <= t[1] <= (y_max + margin)


def bounds_from_geometry(geom, margin_ratio=0.0):
    """从 field_geometry.yaml 里取场地范围。"""
    depth = float(geom["field"]["depth_m"])
    width = float(geom["field"]["width_m"])
    return (0.0, depth, -width / 2.0, width / 2.0)


def _angle_diff(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def visible_observations(fmap, x, y, yaw_deg, max_range=7.0, half_fov_deg=45.0):
    """给定相机在地图里的真实位姿，算出"理论上能看到哪些标志"（仿真/自测用）。"""
    yaw = math.radians(yaw_deg)
    R_inv = _rot(-yaw)
    obs = []
    for f in fmap.features:
        v = np.array([f.x - x, f.y - y])
        if np.linalg.norm(v) > max_range:
            continue
        p = R_inv @ v
        ang = math.degrees(math.atan2(p[1], p[0]))
        if abs(ang) > half_fov_deg:
            continue
        obs.append({"type": f.type, "x": float(p[0]), "y": float(p[1]), "name": f.name})
    return obs
