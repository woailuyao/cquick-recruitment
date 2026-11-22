# -*- coding: utf-8 -*-
"""路线 B：用 4~N 个「已知地图点 <-> 像素」直接解相机位姿（PnP），不依赖外参。

为什么需要它（实测，见 .tmp/pnp_vs_matcher.py，field_v1.mp4 全片）：

| 路线 | 需要外参 | 同帧解出相机高度 | 与另一路线水平差 |
| --- | --- | --- | --- |
| A 反投影 + SE(2) 匹配 | 是（h=1.08 m 写死） | 沿用 1.08 | 中位 0.37 m，最大 9.8 m |
| B PnP（本模块） | **否** | 1.19~1.32 m（自洽约 1.22） | — |

外参的高度是卷尺量的 1.08 m，而 PnP 独立解出约 1.22 m：外参偏矮约 0.14 m，
反投影出的地面点整体内缩，SE(2) 拟合的平移随之偏移。PnP 不吃外参，故不受此误差影响，
并且顺带解出真实高度与俯角，可用于反哺外参。

「谁对应谁」仍需要初始关联；本模块在其上做 EM 迭代：
    关联 -> PnP -> 用 PnP 位姿重投影重新关联 -> 再 PnP -> … -> 内点 / RMS 判定
这样即使初始关联被外参误差带偏一点点，也能收敛回几何自洽的解。
内点是在**像素域**判定的（重投影误差），比地面域更严格、也更直观。

另加几何合理性门限（高度、场地范围）：路线 A 曾在 field_v1 第 75 帧给出 x=-8.67 m
却仍报 status=ok，这类解在本模块会被直接判为 not_converged，不可能伪装成有效。
"""

from __future__ import annotations

import itertools
import math
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera_model import CameraModel
from .matcher import (
    STATUS_HIGH_RESIDUAL, STATUS_INSUFFICIENT, STATUS_LOST, STATUS_NOT_CONVERGED,
    STATUS_OK,
)

# 可见交点几乎落在一条直线上：共面位姿此时在几何上不可定解。
# 与「点太少」分开报告，因为它不是算法问题，而是要调机位（转/退相机）。
STATUS_DEGENERATE = "degenerate_geometry"

# 与 matcher 保持一致的类别兼容：低分辨率/斜视角下「线穿过」与「线终止」极难分辨，
# 实测现场球门区 4 角全被判成 T（其中两个实为 L），故放开 T<->L 双向兼容。
COMPAT = {
    "XCross": ("XCross", "TCross"),
    "TCross": ("TCross", "XCross", "LCross"),
    "LCross": ("LCross", "TCross"),
}


@dataclass
class PnpParams:
    min_markers: int = 4            # 4 点即可定解（共面 IPPE）
    reproj_tol_px: float = 12.0     # 内点的重投影门限（像素）
    max_residual_m: float = 0.30    # 等效地面残差上限
    min_height_m: float = 0.30      # 相机高度合理区间
    max_height_m: float = 3.00
    x_limit_m: float = 4.5          # 相机水平位置合理范围：场地半长 3.8 / 半宽 3.02，
    y_limit_m: float = 3.6          # 再留 0.6 m 给人站在边线外的余量。
                                    # 早先设 6.5/5.0，放过了解在 y=+4.06（场地外）的错解。
    max_iter: int = 5
    max_combos: int = 240           # 4 点子集上限；超过则改为随机采样，控制单帧耗时


@dataclass
class PnpResult:
    valid: bool = False
    x: float = float("nan")
    y: float = float("nan")
    yaw_deg: float = float("nan")
    height_m: float = float("nan")
    pitch_deg: float = float("nan")
    roll_deg: float = float("nan")
    rms_px: float = float("nan")
    residual_m: float = float("nan")
    marker_count: int = 0
    n_observed: int = 0
    status: str = STATUS_LOST
    matches: list = field(default_factory=list)   # [{obs_index, type, map_x, map_y, residual_px}]
    iterations: int = 0
    rvec: np.ndarray | None = None
    tvec: np.ndarray | None = None

    def as_dict(self, force: bool = False) -> dict:
        ok = self.valid or (force and self.x == self.x)
        x = round(self.x, 4) if ok else None
        y = round(self.y, 4) if ok else None
        yaw = round(self.yaw_deg, 3) if ok else None
        # 本项目载体参考原点 = 相机光心正下方地面点，两组位姿同值（细则 §2.2）
        return {
            "valid": ok,
            "x_carrier_map": x,
            "y_carrier_map": y,
            "yaw_carrier_map": yaw,
            "x_camera_map": x,
            "y_camera_map": y,
            "yaw_camera_map": yaw,
            "marker_count": self.marker_count,
            "residual": round(self.residual_m, 4) if self.residual_m == self.residual_m else None,
            "status": self.status,
            "height_m": round(self.height_m, 3) if ok else None,
            "pitch_deg": round(self.pitch_deg, 2) if ok else None,
            "rms_px": round(self.rms_px, 2) if self.rms_px == self.rms_px else None,
        }


# ---------------------------------------------------------------------------
# 基础几何
# ---------------------------------------------------------------------------
def _map_index(field_map) -> dict:
    idx: dict = {}
    for m in field_map.markers:
        idx.setdefault(m.type, []).append((m.x, m.y))
    return {t: np.asarray(v, float).reshape(-1, 2) for t, v in idx.items()}


def _cand_types(t: str):
    return COMPAT.get(t, (t,))


def _reproj_rms(obj3, img2, cam: CameraModel, rvec, tvec) -> np.ndarray:
    proj, _ = cv2.projectPoints(obj3, rvec, tvec, cam.K, cam.dist.reshape(-1, 1))
    return np.linalg.norm(proj.reshape(-1, 2) - np.asarray(img2, float), axis=1)


def _decompose(rvec, tvec):
    """p_cam = R p_map + t  ->  相机光心在地图中的位置、光轴 yaw/pitch、roll。"""
    R, _ = cv2.Rodrigues(np.asarray(rvec, float).reshape(3, 1))
    cam_pos = -R.T @ np.asarray(tvec, float).reshape(3)
    M = R.T                      # 相机各轴在地图中的方向
    axis = M[:, 2]               # 光轴
    pitch = math.degrees(math.asin(float(np.clip(-axis[2], -1.0, 1.0))))
    yaw = math.degrees(math.atan2(float(axis[1]), float(axis[0])))
    # 零滚转时的相机 +Y 方向（与 camera_model._rot 约定一致）
    psi, p = math.radians(yaw), math.radians(pitch)
    ideal_y = np.array([-math.sin(p) * math.cos(psi),
                        -math.sin(p) * math.sin(psi),
                        -math.cos(p)])
    actual_y = M[:, 1]
    cross = np.cross(ideal_y, actual_y)
    roll = math.degrees(math.atan2(float(np.dot(cross, axis)), float(np.dot(ideal_y, actual_y))))
    return cam_pos, float(yaw), float(pitch), float(roll), R


def _planarity_ratio(pts_xy: np.ndarray) -> float:
    """点集的「扁平度」：第二主成分 / 第一主成分。

    接近 0 表示这些点几乎排在一条直线上。判据用地图坐标（米），
    与机位无关——所以它反映的是「现场能看见的标志本身就共线」，而不是数值误差。
    """
    if pts_xy.shape[0] < 3:
        return 0.0
    c = pts_xy - pts_xy.mean(axis=0)
    w = np.linalg.svd(c, compute_uv=False)
    return float(w[1] / max(float(w[0]), 1e-9))


def _geom_ok(cam_pos, params: PnpParams) -> bool:
    h = float(cam_pos[2])
    if not (params.min_height_m <= h <= params.max_height_m):
        return False
    return abs(float(cam_pos[0])) <= params.x_limit_m and abs(float(cam_pos[1])) <= params.y_limit_m


def _solve_quad(obj4, img4, cam: CameraModel, params: PnpParams):
    """4 点最小解集。

    求解器选择（实测，现场帧）：
        场地几何有一个陷阱——球门线 x=3.80 上并排挤着 6 个标志（2 个 L 场地角 +
        4 个 T）。只要相机朝着球门线看，关联出来的 4 点里就常有 3 点共线。
        IPPE 要求任意三点不共线，遇到这种配置直接返回 0 个解，整帧定位失败，
        而现场看到的只是「认不出来」。同一组点换 SQPNP 可以解出
        h=1.106 m、pitch=19.4°、rms=2.63 px —— 与标定的 1.08 m / 19.75° 吻合。
        因此 SQPNP 优先，IPPE 作为非退化情形的备选。
    """
    out = []
    for flag in (cv2.SOLVEPNP_SQPNP, cv2.SOLVEPNP_IPPE):
        try:
            ret, rvecs, tvecs = cv2.solvePnPGeneric(
                obj4, img4, cam.K, cam.dist.reshape(-1, 1), flags=flag)[:3]
        except (cv2.error, ValueError, AttributeError):
            continue
        if not ret or len(rvecs) == 0:
            continue
        for rv, tv in zip(rvecs, tvecs):
            rv = np.asarray(rv, float).reshape(3, 1)
            tv = np.asarray(tv, float).reshape(3, 1)
            # 解析解未考虑畸变，必须跑一次 LM 精化，否则会给出假残差
            try:
                rv, tv = cv2.solvePnPRefineLM(obj4, img4, cam.K,
                                              cam.dist.reshape(-1, 1), rv, tv)
            except cv2.error:
                pass
            cam_pos, yaw, pitch, roll, _ = _decompose(rv, tv)
            if not _geom_ok(cam_pos, params):
                continue
            out.append((rv, tv, cam_pos, yaw, pitch, roll))
        if out:          # SQPNP 成功就不必再试 IPPE
            break
    return out


def _solve(obj3, img2, cam: CameraModel, params: PnpParams, rng=None):
    """以「4 点」为最小解集求最一致位姿（穷举，超限则随机采样）。

    为什么必须是 4 点子集而不是一次性最小二乘（实测，field_v1 第 90 帧）：
        5 个关联里混进 1 个错配（TCross 观测配错地图点），一次性求解被它带偏，
        解出 x=-2.965 y=+0.013 h=1.319 yaw=-0.56，5 个点的重投影误差变成
        [28.3, 3.6, 7.6, 35.7, 3.0] px —— 每个都超标，于是被判成「标志不足」。
        而只用 4 点、把这一个错配剔掉，解是 x=-2.816 y=-0.001 h=1.226 yaw=-0.95，
        四个正确点的重投影误差全部 <1.2 px。
    共面 PnP 的最小配置正是 4 点，因此逐个子集求解再按「内点数、内点 RMS」评分，
    就能把错配挤出去；这也是本文件把 4 点定位作为主路线的原因。
    """
    n = int(obj3.shape[0])
    if n < 4:
        return None
    combos = list(itertools.combinations(range(n), 4))
    if len(combos) > params.max_combos:
        rng = rng if rng is not None else np.random.default_rng(0)
        picked = set()
        for _ in range(params.max_combos):
            picked.add(tuple(sorted(rng.choice(n, 4, replace=False).tolist())))
        combos = sorted(picked)

    best = None
    for cb in combos:
        idx = list(cb)
        for rv, tv, cam_pos, yaw, pitch, roll in _solve_quad(obj3[idx], img2[idx], cam, params):
            d = _reproj_rms(obj3, img2, cam, rv, tv)
            inl = d <= params.reproj_tol_px
            k = int(inl.sum())
            rms = float(np.sqrt(np.mean(d[inl] ** 2))) if k else float("inf")
            if best is None or (k, -rms) > (best["k"], -best["rms"]):
                best = {"rvec": rv, "tvec": tv, "cam_pos": cam_pos, "yaw": yaw,
                        "pitch": pitch, "roll": roll, "k": k, "rms": rms, "inl": inl}
    return best


def _associate_by_reprojection(uv, obs_types, by_type, rvec, tvec, cam: CameraModel,
                               params: PnpParams):
    """用 PnP 位姿把所有地图点投回像素，再与观测做一对一贪心关联。

    这一步是 EM 的 E 步：它能把初始种子漏掉、或类别判错的点重新找回来，
    因为它只依赖当前位姿的几何预测，不依赖外参反投影。
    """
    cands = []
    proj_cache = {}
    for i, (u, v) in enumerate(uv):
        for t in _cand_types(obs_types[i]):
            arr = by_type.get(t)
            if arr is None or len(arr) == 0:
                continue
            if t not in proj_cache:
                obj3 = np.hstack([arr, np.zeros((len(arr), 1))]).astype(np.float64)
                p2, _ = cv2.projectPoints(obj3, rvec, tvec, cam.K, cam.dist.reshape(-1, 1))
                proj_cache[t] = p2.reshape(-1, 2)
            p2 = proj_cache[t]
            dd = np.linalg.norm(p2 - np.array([u, v], float), axis=1)
            for j in np.nonzero(dd <= params.reproj_tol_px)[0]:
                cands.append((float(dd[j]), i, t, int(j)))
    if not cands:
        return []
    cands.sort(key=lambda r: r[0])
    used_obs, used_map, pairs = set(), set(), []
    for d, i, t, j in cands:
        if i in used_obs or (t, j) in used_map:
            continue
        used_obs.add(i)
        used_map.add((t, j))
        pairs.append((i, t, j, d))
    return pairs


def _arrays_from_pairs(pairs, by_type, uv):
    obj, img, keep = [], [], []
    for i, t, j, _d in pairs:
        obj.append((float(by_type[t][j][0]), float(by_type[t][j][1]), 0.0))
        img.append(uv[i])
        keep.append((i, t, j))
    return (np.asarray(obj, float).reshape(-1, 3),
            np.asarray(img, float).reshape(-1, 2), keep)


def _pairs_from_seed(seed_matches, by_type, uv, obs_types):
    """把 matcher 的关联（带地图坐标）转成 (obs_idx, type, map_idx)。"""
    pairs, used_map = [], set()
    for sm in (seed_matches or []):
        try:
            oi = int(sm["obs_index"])
            mty = str(sm["type"])
            mx, my = float(sm["map_x"]), float(sm["map_y"])
        except (KeyError, TypeError, ValueError):
            continue
        if not (0 <= oi < len(uv)):
            continue
        arr = by_type.get(mty)
        if arr is None or len(arr) == 0:
            continue
        j = int(np.argmin(np.linalg.norm(arr - np.array([mx, my], float), axis=1)))
        if (mty, j) in used_map:
            continue
        used_map.add((mty, j))
        pairs.append((oi, mty, j, 0.0))
    return pairs


def locate_pnp(obs_uv, obs_types, field_map, cam: CameraModel,
               seed_matches=None, params: PnpParams | None = None,
               x_sign: int = 0, y_sign: int = 0) -> PnpResult:
    """主入口。

    obs_uv     : (N,2) 检测到的像素坐标（不必先反投影，PnP 直接吃像素）
    obs_types  : list[str] 每个观测的类别
    seed_matches : matcher.localocate 的 result.matches（可选）；给定时作为初始关联，
                   obs_index 必须是 obs_uv 的下标
    x_sign     : -1 / +1 指定相机在场地哪一端（消 180° 对称歧义）；0 不限制
    """
    params = params or PnpParams()
    uv = np.asarray(obs_uv, float).reshape(-1, 2)
    res = PnpResult(status=STATUS_LOST, n_observed=int(len(uv)))
    if len(uv) < params.min_markers:
        res.status = STATUS_INSUFFICIENT
        return res

    by_type = _map_index(field_map)
    if not by_type:
        return res

    pairs = _pairs_from_seed(seed_matches, by_type, uv, obs_types)
    rng = np.random.default_rng(0)

    best = None
    for it in range(params.max_iter):
        if len(pairs) < params.min_markers:
            break
        obj3, img2, keep = _arrays_from_pairs(pairs, by_type, uv)
        sol = _solve(obj3, img2, cam, params, rng=rng)
        if sol is None:
            break
        if x_sign or y_sign:
            cx = float(sol["cam_pos"][0])
            cy = float(sol["cam_pos"][1])
            if x_sign and ((x_sign < 0 and cx > 0.3) or (x_sign > 0 and cx < -0.3)):
                break
            if y_sign and ((y_sign < 0 and cy > 0.3) or (y_sign > 0 and cy < -0.3)):
                break
        best = sol
        best["keep"] = keep
        res.iterations = it + 1

        new_pairs = _associate_by_reprojection(uv, obs_types, by_type,
                                               sol["rvec"], sol["tvec"], cam, params)
        key_old = {(i, t, j) for i, t, j, _ in pairs}
        key_new = {(i, t, j) for i, t, j, _ in new_pairs}
        if key_new == key_old:
            break
        if len(new_pairs) < params.min_markers:
            break
        pairs = new_pairs

    if best is None:
        res.marker_count = len(pairs)
        if res.status == STATUS_INSUFFICIENT or len(pairs) < params.min_markers:
            res.status = STATUS_INSUFFICIENT
            return res
        # 区分「点太少」与「点几乎共线」：后者转一下相机就能解决，
        # 前者要退后让视野含更多标志——两者的操作完全不同，不能混成一个 not_converged。
        if len(pairs) >= params.min_markers:
            arr = np.array([[by_type[t][j][0], by_type[t][j][1]]
                            for _i, t, j, _d in pairs], float)
            if _planarity_ratio(arr) < 0.18:
                res.status = STATUS_DEGENERATE
                return res
        res.status = STATUS_NOT_CONVERGED
        return res

    obj3, img2, keep = _arrays_from_pairs(
        [(i, t, j, 0.0) for i, t, j in best["keep"]], by_type, uv)
    dpx = _reproj_rms(obj3, img2, cam, best["rvec"], best["tvec"])
    inl = dpx <= params.reproj_tol_px
    if int(inl.sum()) >= params.min_markers and int(inl.sum()) < len(dpx):
        # 用内点再精解一次
        try:
            rv, tv = cv2.solvePnPRefineLM(obj3[inl], img2[inl], cam.K,
                                          cam.dist.reshape(-1, 1),
                                          best["rvec"], best["tvec"])
            d2 = _reproj_rms(obj3[inl], img2[inl], cam, rv, tv)
            if float(np.sqrt(np.mean(d2 * d2))) <= best["rms"]:
                best["rvec"], best["tvec"] = rv, tv
                cam_pos, yaw, pitch, roll, _ = _decompose(rv, tv)
                best.update({"cam_pos": cam_pos, "yaw": yaw, "pitch": pitch,
                             "roll": roll, "rms": float(np.sqrt(np.mean(d2 * d2)))})
        except cv2.error:
            pass

    inl_pairs = [(i, t, j) for (i, t, j), ok_i in zip(best["keep"], inl) if ok_i]
    res.marker_count = len(inl_pairs)
    if res.marker_count < params.min_markers:
        res.status = STATUS_INSUFFICIENT
        return res

    cam_pos = best["cam_pos"]
    res.x, res.y = float(cam_pos[0]), float(cam_pos[1])
    res.height_m = float(cam_pos[2])
    res.yaw_deg = best["yaw"]
    res.pitch_deg = best["pitch"]
    res.roll_deg = best["roll"]
    res.rvec, res.tvec = best["rvec"], best["tvec"]

    # 像素残差 -> 等效地面残差：一阶换算 depth/f（depth 取该点到相机的水平距离）
    f = float(cam.K[0, 0])
    dpx_in = _reproj_rms(obj3, img2, cam, best["rvec"], best["tvec"])
    m_pairs = np.array([[by_type[t][j][0], by_type[t][j][1]] for i, t, j in inl_pairs], float)
    depth = np.linalg.norm(m_pairs - np.array([res.x, res.y], float), axis=1)
    depth = np.clip(depth, 0.5, None)
    dm = dpx_in[dpx_in <= params.reproj_tol_px] * depth / f
    res.residual_m = float(np.sqrt(np.mean(dm * dm))) if dm.size else float("inf")
    res.rms_px = float(np.sqrt(np.mean(dpx_in[dpx_in <= params.reproj_tol_px] ** 2))) \
        if dpx_in.size else float("inf")

    res.matches = [
        {"obs_index": int(i), "type": t,
         "map_x": float(by_type[t][j][0]), "map_y": float(by_type[t][j][1]),
         "residual_px": float(d)}
        for (i, t, j), d in zip(inl_pairs,
                                dpx_in[dpx_in <= params.reproj_tol_px])
    ]
    # obs 类型与地图类型可能不同（X 观测配 T 地图点），界面据此显示箭头
    for m in res.matches:
        m["obs_type"] = str(obs_types[int(m["obs_index"])])

    if res.residual_m > params.max_residual_m:
        res.status = STATUS_HIGH_RESIDUAL
        return res

    res.valid = True
    res.status = STATUS_OK
    return res

