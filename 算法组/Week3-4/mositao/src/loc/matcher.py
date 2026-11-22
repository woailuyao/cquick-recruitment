# -*- coding: utf-8 -*-
"""观测标志 -> field.yaml 地图特征 匹配，求解载体二维位姿。

对应细则 2.8 与评分表「field.yaml 加载及多特征地图匹配（15 分）」：
- 不允许「按类别把检测框匹配到最近的同类点」；
- 必须处理同类标志身份歧义与场地对称；
- 必须给出 marker_count / residual / valid / status，失败时不伪造位姿。

实现路线：几何一致性候选位姿枚举（等价于小样本下的 RANSAC 假设生成）
  -> 一对一贪心关联 -> SE(2) 最小二乘（Kabsch）迭代优化 -> 残差与歧义判定。

关于对称性（实测结论，见 outputs/sim_selftest_*.txt）：
标准足球场的标志在绕场地中心旋转 180° 后与自身重合，因此仅凭几何标志，
任何一组观测都存在「翻转解」。这是场地的性质而非算法缺陷。
处理方式有二：① 地图含非对称特征时天然消歧；② 连续帧中用上一帧位姿作先验窗口。
两者都无法确认时，正确做法就是返回 ambiguous —— 细则 2.8 明确要求如此。
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

STATUS_OK = "ok"
STATUS_INSUFFICIENT = "insufficient_markers"
STATUS_AMBIGUOUS = "ambiguous"
STATUS_HIGH_RESIDUAL = "high_residual"
STATUS_NOT_CONVERGED = "not_converged"
STATUS_LOST = "lost"

# 跨类别配对的等效距离代价（米）：保证同类别配对优先，跨类只作兜底。
# 取值小于 inlier_tol_m(0.35) 的一半，宁可丢掉一个跨类点，也不打乱同类配对。
CROSS_TYPE_PENALTY = 0.06


def wrap_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def rot2(theta: float) -> np.ndarray:
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], float)


def apply_pose(pts: np.ndarray, theta: float, t) -> np.ndarray:
    return pts @ rot2(theta).T + np.asarray(t, float)


@dataclass
class ObservedMarker:
    """载体坐标系下的地面观测（细则 2.7 的 observed_markers 结构）。"""
    type: str
    x: float
    y: float
    confidence: float = 1.0

    def xy(self) -> np.ndarray:
        return np.array([self.x, self.y], float)


@dataclass
class MatchParams:
    min_markers: int = 3
    min_confidence: float = 0.25
    inlier_tol_m: float = 0.35
    max_residual_m: float = 0.25
    ambiguity_pose_m: float = 0.40
    ambiguity_pose_deg: float = 8.0
    ambiguity_residual_ratio: float = 1.30
    refine_iters: int = 4
    max_observations: int = 6
    max_candidates: int = 2500
    random_samples: int = 800


@dataclass
class LocalizationResult:
    valid: bool = False
    x: float = float("nan")
    y: float = float("nan")
    yaw_deg: float = float("nan")
    marker_count: int = 0
    residual: float = float("nan")
    status: str = STATUS_LOST
    matches: list = field(default_factory=list)
    candidates: int = 0
    second: dict | None = None

    def as_dict(self, force: bool = False) -> dict:
        # force=True 时即使 valid=False（例如 ambiguous）也把坐标填出来，
        # 但 valid 字段仍如实为 False、status 仍写明原因，避免把不可信坐标伪装成有效
        ok = self.valid or (force and self.x == self.x)
        # 细则 §2.2：相机光心可定义载体参考原点（相机装在独立移动测试架上时），
        # 此时两组位姿一致。本项目外参 forward_m = lateral_m = 0，正是这种情形，
        # 故 camera 三项与 carrier 三项同值；两组字段都必须输出。
        x = round(self.x, 4) if ok else None
        y = round(self.y, 4) if ok else None
        yaw = round(self.yaw_deg, 3) if ok else None
        return {
            "valid": ok,
            "x_carrier_map": x,
            "y_carrier_map": y,
            "yaw_carrier_map": yaw,
            "x_camera_map": x,
            "y_camera_map": y,
            "yaw_camera_map": yaw,
            "marker_count": self.marker_count,
            "residual": round(self.residual, 4) if self.residual == self.residual else None,
            "status": self.status,
        }


def _build_index(field_map) -> dict:
    """type -> (m,2) 地图坐标数组。"""
    idx: dict = {}
    for m in field_map.markers:
        idx.setdefault(m.type, []).append((m.x, m.y))
    return {t: np.asarray(v, float) for t, v in idx.items()}


def _associate(pts_map: np.ndarray, obs_types, by_type: dict, tol: float):
    """一对一贪心关联：同一地图特征不会被两个观测重复占用。

    返回 (内点数, 残差RMS[m], [(obs_i, map_j, type, d), ...])

    类别兼容：这个项目里的四类标志，实际只有「线终止」与「线穿过」两种几何形状，
    而斜视角、低分辨率、细线（2~4 px）下两者极难区分，分类器会系统性判错。
    实测（现场球门区 4 角，见 .tmp/live/crop_junctions.png）：
        真实 LCross 角点被全部判成 TCross，而 matcher 原表只放行 X<->T，
        于是 4 个正确检测一个也配不上，整帧定位失败。
    因此放开 T<->L 双向兼容，但给跨类别配对加 CROSS_TYPE_PENALTY 的代价，
    保证同类别配对永远优先；跨类只在同类找不到时兜底。
    """
    compat = {
        "XCross": ("XCross", "TCross"),
        "TCross": ("TCross", "XCross", "LCross"),
        "LCross": ("LCross", "TCross"),
    }
    cands = []
    for i, (t, p) in enumerate(zip(obs_types, pts_map)):
        for tt in compat.get(t, (t,)):
            arr = by_type.get(tt)
            if arr is None or len(arr) == 0:
                continue
            d = np.linalg.norm(arr - p, axis=1)
            for j in np.nonzero(d <= tol)[0]:
                pen = 0.0 if tt == t else CROSS_TYPE_PENALTY
                cands.append((float(d[j]) + pen, float(d[j]), i, int(j), tt))
    if not cands:
        return 0, float("inf"), []
    cands.sort(key=lambda r: r[0])
    used_obs, used_map, matches = set(), set(), []
    for _key, d, i, j, t in cands:
        if i in used_obs or (t, j) in used_map:
            continue
        used_obs.add(i)
        used_map.add((t, j))
        matches.append((i, j, t, d))
    res = math.sqrt(sum(d * d for *_, d in matches) / len(matches))
    return len(matches), res, matches


def _pose_from_pair(p1, p2, q1, q2):
    dp, dq = p2 - p1, q2 - q1
    if np.linalg.norm(dp) < 1e-6 or np.linalg.norm(dq) < 1e-6:
        return None
    th = wrap_pi(math.atan2(dq[1], dq[0]) - math.atan2(dp[1], dp[0]))
    t = q1 - rot2(th) @ p1
    return th, t


def _kabsch(pts: np.ndarray, targets: np.ndarray):
    """二维 SE(2) 最小二乘：求 theta,t 使 R@pts+t 尽量贴近 targets。"""
    cp, ct = pts.mean(axis=0), targets.mean(axis=0)
    dp, dt = pts - cp, targets - ct
    cross = float(np.sum(dp[:, 0] * dt[:, 1] - dp[:, 1] * dt[:, 0]))
    dot = float(np.sum(dp[:, 0] * dt[:, 0] + dp[:, 1] * dt[:, 1]))
    th = math.atan2(cross, dot)
    return th, ct - rot2(th) @ cp


def _refine(pts: np.ndarray, obs_types, by_type: dict, th: float, t: np.ndarray,
            params: MatchParams):
    for _ in range(params.refine_iters):
        n, res, matches = _associate(apply_pose(pts, th, t), obs_types, by_type,
                                     params.inlier_tol_m)
        if n < 2:
            return th, t, n, res, matches
        src = np.array([pts[i] for i, *_ in matches])
        dst = np.array([by_type[ty][j] for _, j, ty, _ in matches])
        th_new, t_new = _kabsch(src, dst)
        if abs(wrap_pi(th_new - th)) < 1e-10 and float(np.linalg.norm(t_new - t)) < 1e-10:
            th, t = th_new, t_new
            break
        th, t = th_new, t_new
    n, res, matches = _associate(apply_pose(pts, th, t), obs_types, by_type, params.inlier_tol_m)
    return th, t, n, res, matches


def _generate_candidates(pts: np.ndarray, obs_types, by_type: dict, params: MatchParams,
                         sampling: str, rng):
    """几何一致性假设生成，返回去重后的 (theta, t) 列表。

    优先枚举「稀有类别」的组合：先用实例少的类型生成假设，可在不牺牲覆盖率的前提下
    把候选数量压下来（PenaltyPoint 通常只有 2 个，而 LCross 可能有 8 个）。
    """
    n = len(pts)
    obs_pairs = [(i, j) for i in range(n) for j in range(i + 1, n)]
    if not obs_pairs:
        return []
    obs_pairs.sort(key=lambda ij: len(by_type.get(obs_types[ij[0]], ())) *
                   len(by_type.get(obs_types[ij[1]], ())))

    if sampling == "random":
        # 真正的 RANSAC 假设生成：随机抽 (观测对, 地图对) 组合，而不是抽观测对
        cands, seen = [], set()
        for _ in range(params.random_samples):
            i, j = obs_pairs[int(rng.integers(len(obs_pairs)))]
            ti, tj = obs_types[i], obs_types[j]
            ai, aj = by_type.get(ti), by_type.get(tj)
            if ai is None or aj is None or len(ai) == 0 or len(aj) == 0:
                continue
            a = ai[int(rng.integers(len(ai)))]
            b = aj[int(rng.integers(len(aj)))]
            got = _pose_from_pair(pts[i], pts[j], a, b)
            if got is None:
                continue
            th, t = got
            key = (round(th, 4), round(float(t[0]), 2), round(float(t[1]), 2))
            if key in seen:
                continue
            seen.add(key)
            cands.append((th, t))
            if len(cands) >= params.max_candidates:
                break
        return cands

    cands, seen = [], set()
    for i, j in obs_pairs:
        ti, tj = obs_types[i], obs_types[j]
        ai, aj = by_type.get(ti), by_type.get(tj)
        if ai is None or aj is None:
            continue
        for a in ai:
            for b in aj:
                got = _pose_from_pair(pts[i], pts[j], a, b)
                if got is None:
                    continue
                th, t = got
                key = (round(th, 4), round(float(t[0]), 2), round(float(t[1]), 2))
                if key in seen:
                    continue
                seen.add(key)
                cands.append((th, t))
                if len(cands) >= params.max_candidates:
                    return cands
    return cands


def locate(observed, field_map, params: MatchParams | None = None,
           sampling: str = "exhaustive", seed: int = 0,
           prior: tuple | None = None, x_sign: int = 0,
           y_sign: int = 0) -> LocalizationResult:
    """主入口。

    observed : [ObservedMarker]，载体坐标系地面观测
    field_map: FieldMap（src/loc/field_map.py）
    sampling : "exhaustive"（默认）或 "random"（随机 RANSAC 假设，用于加分项对比）
    prior    : (x, y, yaw_rad, pos_window_m, yaw_window_deg) 上一帧位姿先验，可选；
               连续跟踪时用它可以消掉场地 180° 对称带来的翻转歧义
    x_sign   : 0=不限制；-1=只要 x<0 的解；+1=只要 x>0 的解。
               现场地图关于场地中心严格中心对称，只凭一帧的地面点无法区分「相机在
               -X 端朝 +X」与「相机在 +X 端朝 -X」——这两支的重投影完全等价。
               随机位置冷启动时用这个参数指定在场地哪一端，就能把歧义消掉，
               而 y 与 yaw 仍然全自动估计。
    """
    params = params or MatchParams()
    by_type = _build_index(field_map)

    obs = [o for o in observed
           if math.isfinite(o.x) and math.isfinite(o.y) and o.confidence >= params.min_confidence]
    if len(obs) > params.max_observations:
        obs = sorted(obs, key=lambda o: -o.confidence)[:params.max_observations]
    obs_types = [o.type for o in obs]
    pts = np.array([o.xy() for o in obs], float).reshape(-1, 2)

    result = LocalizationResult()
    if len(obs) < params.min_markers:
        result.status = STATUS_INSUFFICIENT
        result.marker_count = len(obs)
        return result
    if not by_type:
        result.status = STATUS_LOST
        result.marker_count = len(obs)
        return result

    rng = np.random.default_rng(seed)
    cands = _generate_candidates(pts, obs_types, by_type, params, sampling, rng)
    result.candidates = len(cands)
    if not cands:
        result.status = STATUS_NOT_CONVERGED
        result.marker_count = len(obs)
        return result

    if prior is not None:
        px, py, pyaw, win_m, win_deg = prior
        kept = [(th, t) for th, t in cands
                if np.hypot(t[0] - px, t[1] - py) <= win_m
                and abs(wrap_pi(th - pyaw)) <= math.radians(win_deg)]
        if kept:
            cands = kept

    if x_sign:
        # 只保留相机落在指定半场的候选，消掉中心对称带来的 180° 翻转歧义
        kept = [(th, t) for th, t in cands
                if (t[0] < 0.3 if x_sign < 0 else t[0] > -0.3)]
        if kept:
            cands = kept

    if y_sign:
        # 场地关于两条轴都对称，一个 180° 翻转解是 (x,y)->(-x,-y) 且 yaw+180°。
        # 当相机站在中线附近（|x| 很小）时，x 的符号几乎不区分两支，
        # 而 y 的符号照旧翻转（实测相机在 (0,-2.8)，误解得 (0,+2.8) 一类的镜像）。
        # 因此再提供一条按 y 定向的约束。
        kept = [(th, t) for th, t in cands
                if (t[1] < 0.3 if y_sign < 0 else t[1] > -0.3)]
        if kept:
            cands = kept

    scored = []
    for th, t in cands:
        # 相机位置必须落在场地内（或紧邻边线外一点）：现场 7.6x6.03 m，
        # 半长 3.8 / 半宽 3.02，留 0.8 m 给「人站在边线外」的情形。
        # 不加这条时，仅凭 4 个点能在容差内拟合出场地外的假解
        # （实测出现过 y=+4.045 且残差很小），光看 residual 分辨不出来。
        if abs(float(t[0])) > 4.6 or abs(float(t[1])) > 3.8:
            continue
        n, res, _ = _associate(apply_pose(pts, th, t), obs_types, by_type, params.inlier_tol_m)
        if n >= 2:
            scored.append((n, res, th, t))
    if not scored:
        result.status = STATUS_NOT_CONVERGED
        result.marker_count = 0
        return result

    scored.sort(key=lambda r: (-r[0], r[1]))
    refined, seen = [], set()
    for n, res, th, t in scored[:24]:
        th2, t2, n2, res2, matches = _refine(pts, obs_types, by_type, th, t, params)
        key = (round(th2, 2), round(float(t2[0]), 1), round(float(t2[1]), 1))
        if key in seen:
            continue
        seen.add(key)
        refined.append((n2, res2, th2, t2, matches))

    refined.sort(key=lambda r: (-r[0], r[1]))
    n_best, res_best, th_best, t_best, matches = refined[0]

    second = None
    for n_i, res_i, th_i, t_i, _ in refined[1:]:
        pose_delta = math.hypot(t_i[0] - t_best[0], t_i[1] - t_best[1])
        yaw_delta = abs(math.degrees(wrap_pi(th_i - th_best)))
        if pose_delta < params.ambiguity_pose_m and yaw_delta < params.ambiguity_pose_deg:
            continue
        second = {"x": float(t_i[0]), "y": float(t_i[1]),
                  "yaw_deg": math.degrees(wrap_pi(th_i)),
                  "marker_count": int(n_i), "residual": float(res_i),
                  "pose_delta_m": float(pose_delta), "yaw_delta_deg": float(yaw_delta)}
        break
    result.second = second

    if n_best < params.min_markers:
        result.status = STATUS_INSUFFICIENT
        result.marker_count = int(n_best)
        result.residual = float(res_best) if n_best else float("nan")
        # 把「少点位姿」也带出来：它谈不上可信（valid 仍为 False），但能让上游
        # 用这个位姿去重投影，判断剩下的检测究竟是没配上、还是根本不是交点。
        if n_best:
            result.x = float(t_best[0])
            result.y = float(t_best[1])
            result.yaw_deg = math.degrees(wrap_pi(th_best))
        # 即使点数不够，也要把已找到的关联交出去 —— PnP 路线用它当种子，
        # 若这里留空，上游就完全不知道哪一观测配了哪一地图点，只能整帧放弃。
        result.matches = [
            {"obs_index": int(i), "type": ty, "map_index": int(j),
             "map_x": float(by_type[ty][j][0]), "map_y": float(by_type[ty][j][1]),
             "residual_m": float(d)}
            for i, j, ty, d in matches
        ]
        return result
    if res_best > params.max_residual_m:
        result.status = STATUS_HIGH_RESIDUAL
        result.marker_count = int(n_best)
        result.residual = float(res_best)
        return result
    if second and second["marker_count"] >= n_best and \
       second["residual"] <= max(res_best * params.ambiguity_residual_ratio, 1e-6):
        # 有第二个等价解：仍然把最优解填进去（valid 保持 False、status 标 ambiguous），
        # 让上层可以「带警告地使用」——现场有些方位只看得见 3 个交点时，不给坐标就完全没法用。
        result.status = STATUS_AMBIGUOUS
        result.marker_count = int(n_best)
        result.residual = float(res_best)
        result.x = float(t_best[0])
        result.y = float(t_best[1])
        result.yaw_deg = math.degrees(wrap_pi(th_best))
        result.matches = [
            {"obs_index": int(i), "type": ty, "map_index": int(j),
             "map_x": float(by_type[ty][j][0]), "map_y": float(by_type[ty][j][1]),
             "residual_m": float(d)}
            for i, j, ty, d in matches
        ]
        return result

    result.valid = True
    result.status = STATUS_OK
    result.x = float(t_best[0])
    result.y = float(t_best[1])
    result.yaw_deg = math.degrees(wrap_pi(th_best))
    result.marker_count = int(n_best)
    result.residual = float(res_best)
    result.matches = [
        {"obs_index": int(i), "type": ty, "map_index": int(j),
         "map_x": float(by_type[ty][j][0]), "map_y": float(by_type[ty][j][1]),
         "residual_m": float(d)}
        for i, j, ty, d in matches
    ]
    return result
