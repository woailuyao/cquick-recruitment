# -*- coding: utf-8 -*-
"""
ocv_match.py —— field.yaml 解析 + 局部系→场地系刚体匹配 + 位姿求解
核心思想：假设"观测 i ↔ 地图 A、观测 j ↔ 地图 B"即可解出唯一 SE(2) 位姿，
再用该位姿检验其余观测，能同时对上足够多标志的假设才可信。
"""
import math
import re
import numpy as np
import yaml

# 键名前缀 -> 内部类别。注意官方把罚球点拼成 PenalityPoint（少一个 n），
# 这里同时兼容 CenterPoint / PenaltyPoint / Spot，任意写法都能解析。
# field.yaml 里除标志点之外还可以有场地尺寸，这些键不参与匹配
FIELD_META_KEYS = {"CenterCircleRadius", "CircleRadius", "Comment"}

TYPE_ALIAS = {"LCross": "L", "TCross": "T", "XCross": "X",
              "PenalityPoint": "P", "PenaltyPoint": "P",
              "CenterPoint": "P", "Spot": "P"}

DEFAULT_MATCH = dict(
    inlier_thr=0.30,       # 内点判定阈值 (m)
    min_inliers=3,         # 至少匹配上几个标志才算定位有效
    max_residual=0.25,     # 内点残差 RMS 上限 (m)
    amb_pose_m=0.60,       # 两个假设相距小于此值视为同一个
    amb_yaw_deg=20.0,      # 朝向差小于此角度视为同一个
    margin=2.0,            # 相机允许离场地边界的余量 (m)
    prefer_yaw=None,       # 给定偏好朝向(度)时，平局按朝向最接近者选解
)


def load_field(path):
    """解析 field.yaml -> {'L': [(x,y,'LCross1'),...], 'T': [...], 'X': [...], 'P': [...]}"""
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    field = {"L": [], "T": [], "X": [], "P": []}
    for key, val in raw.items():
        if str(key) in FIELD_META_KEYS:
            continue
        m = re.match(r"^([A-Za-z]+)(\d*)$", str(key))
        base = m.group(1) if m else str(key)
        t = TYPE_ALIAS.get(base)
        if t is None:
            print(f"[warn] field.yaml 未知类型 {key}，跳过")
            continue
        try:
            x, y = float(val[0]), float(val[1])
        except Exception:
            print(f"[warn] {key} 坐标格式不对: {val}")
            continue
        field[t].append((x, y, str(key)))
    return field


def load_field_meta(path, default_r=0.0):
    """读场地尺寸（中圈半径）。中圈半径已知时，圆心反算的条件数会好很多。"""
    with open(path, "r", encoding="utf-8") as f:
        raw = yaml.safe_load(f) or {}
    for k in ("CenterCircleRadius", "CircleRadius"):
        if k in raw:
            try:
                return {"circle_r": float(raw[k])}
            except Exception:
                pass
    return {"circle_r": float(default_r)}


def field_bounds(field, margin=1.0):
    """从标志点集推出场地范围（组里没给尺寸时的推荐做法）"""
    pts = [(m[0], m[1]) for lst in field.values() for m in lst]
    if not pts:
        return (-margin, margin, -margin, margin)
    xs = [p[0] for p in pts]
    ys = [p[1] for p in pts]
    return (min(xs) - margin, max(xs) + margin, min(ys) - margin, max(ys) + margin)


def rigid_fit_2d(src, dst):
    """最小二乘刚体变换(只旋转+平移)。返回 (R, t, theta_rad)。
    src: 相机局部系 (a,b)  N×2     dst: 场地系 (x,y)  N×2"""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    cs = src.mean(axis=0)
    cd = dst.mean(axis=0)
    s = src - cs
    d = dst - cd
    num = float((s[:, 0] * d[:, 1] - s[:, 1] * d[:, 0]).sum())
    den = float((s[:, 0] * d[:, 0] + s[:, 1] * d[:, 1]).sum())
    th = math.atan2(num, den)
    c, sn = math.cos(th), math.sin(th)
    R = np.array([[c, -sn], [sn, c]])
    return R, cd - R @ cs, th


def _match_obs(R, t, obs, field, thr):
    """用候选位姿把观测投到地图上，找同类地图点，返回内点"""
    inl = []
    for o in obs:
        cands = field.get(o["type"], [])
        if not cands:
            continue
        p = R @ o["xy"] + t
        best = min(cands, key=lambda m: (p[0] - m[0]) ** 2 + (p[1] - m[1]) ** 2)
        d = math.hypot(p[0] - best[0], p[1] - best[1])
        if d <= thr:
            inl.append((o, best, d))
    return inl


def _dedup_alts(top, amb_m, amb_yaw, k=4):
    """候选解去重（同一位姿可能来自不同的观测对组合），供界面/日志展示。"""
    out = []
    for c in top:
        if all(math.hypot(c["x"] - a["x"], c["y"] - a["y"]) >= amb_m
               or abs((c["yaw"] - a["yaw"] + 180.0) % 360.0 - 180.0) >= amb_yaw for a in out):
            out.append({"x": c["x"], "y": c["y"], "yaw": c["yaw"], "n": c["n"], "res": c["res"]})
        if len(out) >= k:
            break
    return out


def solve_pose(obs, field, cfg=None):
    """obs: [{'type':'T','xy':np.array([a,b]),'conf':0.9}, ...]
    返回 x_camera_map / y_camera_map / yaw_camera_map / marker_count / residual / valid / status"""
    cfg = dict(DEFAULT_MATCH, **(cfg or {}))

    def fail(status):
        # 失败路径键集合必须与成功路径完全一致（调用方会直接访问 symmetry / alts 等字段）
        return {"valid": False, "status": status, "x": 0.0, "y": 0.0, "yaw": 0.0,
                "marker_count": 0, "residual": float("nan"), "inliers": [],
                "symmetry": False, "resolved_by_continuity": False,
                "resolved_by_preference": False, "alts": []}

    if len(obs) < 2:
        return fail("insufficient_markers")

    xmin, xmax, ymin, ymax = field_bounds(field, cfg["margin"])

    # ---- 枚举假设：观测对 x 地图对
    hyps = []
    for i in range(len(obs)):
        for j in range(i + 1, len(obs)):
            oi, oj = obs[i], obs[j]
            ci, cj = field.get(oi["type"], []), field.get(oj["type"], [])
            if not ci or not cj:
                continue
            for m1 in ci:
                for m2 in cj:
                    if oi["type"] == oj["type"] and m1[2] == m2[2]:
                        continue
                    R, t, th = rigid_fit_2d(np.array([oi["xy"], oj["xy"]]),
                                            np.array([m1[:2], m2[:2]]))
                    if not (xmin <= t[0] <= xmax and ymin <= t[1] <= ymax):
                        continue                  # 剪枝：相机不可能在场地外
                    inl = _match_obs(R, t, obs, field, cfg["inlier_thr"])
                    if len(inl) >= 2:
                        hyps.append((R, t, th, inl))
    if not hyps:
        return fail("no_match")

    def res_of(inl):
        return math.sqrt(sum(d * d for _, _, d in inl) / max(len(inl), 1))

    hyps.sort(key=lambda h: (-len(h[3]), res_of(h[3])))

    # ---- 逐个精修候选假设（SE(2) 最小二乘）；全部精修，交给下面的消歧逻辑挑
    # （场地点位中心对称，只精修第一名会在真值解与镜像解之间随机跳变）
    cands = []
    for h in hyps[:8]:
        R0, t0, th0, inl0 = h
        src = np.array([o["xy"] for o, _, _ in inl0])
        dst = np.array([[m[0], m[1]] for _, m, _ in inl0])
        R, t, th = rigid_fit_2d(src, dst)
        inl = _match_obs(R, t, obs, field, cfg["inlier_thr"])
        cands.append({"R": R, "t": t, "th": th, "inl": inl,
                      "n": len(inl), "res": res_of(inl),
                      "x": float(t[0]), "y": float(t[1]),
                      "yaw": float(math.degrees(th) % 360.0)})
    if not cands:
        return fail("no_match")

    best_n = max(c["n"] for c in cands)
    top = [c for c in cands if c["n"] == best_n]

    # 中心对称场地的"绕原点转 180°"孪生解：显式评估并加入候选，如实反映到 ambiguous
    def _R(th):
        c_, s_ = math.cos(th), math.sin(th)
        return np.array([[c_, -s_], [s_, c_]])
    for base in list(top[:2]):
        th_t = base["th"] + math.pi
        t_t = -base["t"]
        inl_t = _match_obs(_R(th_t), t_t, obs, field, cfg["inlier_thr"])
        if len(inl_t) >= best_n - 1 and len(inl_t) >= 2:
            yaw_t = float(math.degrees(th_t) % 360.0)
            if all(math.hypot(t_t[0] - c["x"], t_t[1] - c["y"]) >= cfg["amb_pose_m"]
                   or abs((yaw_t - c["yaw"] + 180.0) % 360.0 - 180.0) >= cfg["amb_yaw_deg"]
                   for c in top):
                top.append({"R": _R(th_t), "t": t_t, "th": th_t, "inl": inl_t,
                            "n": len(inl_t), "res": res_of(inl_t),
                            "x": float(t_t[0]), "y": float(t_t[1]), "yaw": yaw_t})

    def distinct(c, ref):
        dp = math.hypot(c["x"] - ref["x"], c["y"] - ref["y"])
        dy = abs((c["yaw"] - ref["yaw"] + 180.0) % 360.0 - 180.0)
        return dp >= cfg["amb_pose_m"] or dy >= cfg["amb_yaw_deg"]

    # ---- 消歧：优先用上一帧位姿（时序连续性），没有上一帧就按残差选
    prev = cfg.get("prev_pose")
    resolved_by_continuity = False
    if prev is not None and len(top) > 1:
        px, py, pyaw = float(prev[0]), float(prev[1]), float(prev[2])
        def cost(c):
            dp = math.hypot(c["x"] - px, c["y"] - py)
            dy = abs((c["yaw"] - pyaw + 180.0) % 360.0 - 180.0)
            return dp + math.radians(dy) * 0.6      # 1 rad 的朝向差折算成 0.6 m
        chosen = min(top, key=cost)
        resolved_by_continuity = any(distinct(c, chosen) for c in top)
    else:
        pref = cfg.get("prefer_yaw")
        if pref is None:
            chosen = min(top, key=lambda c: (c["res"], c["n"]))
        else:
            chosen = min(top, key=lambda c: (abs((c["yaw"] - float(pref) + 180.0) % 360.0 - 180.0),
                                             c["res"]))

    R, t, th, inl = chosen["R"], chosen["t"], chosen["th"], chosen["inl"]
    res = chosen["res"]
    n = len(inl)

    # ---- 歧义检测：还有没有"位置/朝向明显不同、却同样解释得了观测"的竞争者
    rivals = [c for c in top if distinct(c, chosen)]
    resolved_by_pref = bool(rivals) and cfg.get("prefer_yaw") is not None
    ambiguous = len(rivals) > 0 and not (resolved_by_continuity or resolved_by_pref)

    if n < cfg["min_inliers"]:
        status = "insufficient_markers"
    elif ambiguous:
        status = "ambiguous"
    elif res > cfg["max_residual"]:
        status = "high_residual"
    else:
        status = "ok"
    return {"valid": status == "ok", "status": status,
            "x": float(t[0]), "y": float(t[1]),
            "yaw": float(math.degrees(th) % 360.0),
            "marker_count": n, "residual": float(res), "inliers": inl,
            # 对称歧义信息：界面用来提示"这是靠上一帧消的歧义"
            "symmetry": bool(rivals),
            "resolved_by_continuity": bool(resolved_by_continuity),
            "resolved_by_preference": bool(resolved_by_pref),
            "alts": _dedup_alts(top, cfg["amb_pose_m"], cfg["amb_yaw_deg"])}
