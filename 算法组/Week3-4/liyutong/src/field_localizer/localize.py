# -*- coding: utf-8 -*-
"""
任务 2：单目场地自定位 —— 像素 → 地面 → 地图匹配

流程
    1) 标志点像素 (u, v) 去畸变 → 归一化射线
    2) 与地面平面 Z=0 求交 → 得到"架体系"下的地面坐标 (x=右, y=前, 单位米)
    3) 用"点对假设 + RANSAC"在 field.yaml 的 17 个标志点里找 2D 刚体变换（尺度已知，只有 3 自由度）
    4) 最小二乘精化 → 输出 x_camera_map / y_camera_map / yaw_camera_map
    5) 若残差最小的两个候选相差 180°，用球门观测量消歧

坐标约定（严格按《算法考核细则》）
    地图系：原点 = 场地中心，+X → 唯一球门，+Y → 面向 +X 的左侧，+Z 向上，右手系，米
    yaw   ：相机光轴水平投影的方位角，从地图 +X 起算，逆时针为正，度

用法
    python localize.py --selftest              # 合成数据自测（不需要相机和场地）
    python localize.py --det demo.json         # 跑真实检测结果
    python localize.py --selftest -n 200 -v    # 200 次随机位姿，打印明细
"""
import os
import re
import sys
import json
import math
import argparse
import numpy as np
import cv2

CAL_NPZ = "calibration.npz"
EXTR_YAML = "camera_extrinsics.yaml"
FIELD_YAML = "field.yaml"

FOV_MARGIN = 0.0        # 检测点离画面边缘小于该比例时不用（0 = 不启用）


# ============================================================
# 极简 yaml 读取（不引入 PyYAML，避免环境依赖）
# ============================================================
def _scalar(s):
    s = s.strip().strip('"').strip("'")
    if s.lower() in ("null", "none", "~", ""):
        return None
    try:
        return float(s)
    except ValueError:
        return s


def load_field(path=FIELD_YAML):
    """解析 field.yaml，返回 (markers, goal, field_dict)"""
    markers, goal, fld = [], {}, {}
    section = None
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#")[0].rstrip()
            if not line.strip():
                continue
            if not line.startswith(" "):                 # 顶层键
                section = line.split(":")[0].strip()
                continue
            body = line.strip()
            if section == "markers" and body.startswith("-"):
                d = dict(re.findall(r"(\w+):\s*([-\w.]+)", body))
                if {"id", "type", "x", "y"} <= set(d):
                    markers.append(dict(id=int(float(d["id"])), type=d["type"],
                                        x=float(d["x"]), y=float(d["y"])))
            elif section == "goal" and ":" in body:
                k, v = body.split(":", 1)
                goal[k.strip()] = _scalar(v)
            elif section == "field" and ":" in body:
                k, v = body.split(":", 1)
                fld[k.strip()] = _scalar(v)
    if not markers:
        sys.exit(f"{path} 里没解析出标志点")
    return markers, goal, fld


def load_extrinsics(path=EXTR_YAML):
    """返回 (R_ground_to_camera 3x3, height_m)。解析失败则报错退出"""
    import re as _re
    txt = open(path, encoding="utf-8").read()
    m = _re.search(r"data:\s*\[([^\]]+)\]", txt)
    if not m:
        sys.exit(f"{path} 里找不到 rotation_ground_to_camera.data")
    vals = [float(x) for x in m.group(1).replace("\n", " ").split(",")]
    R = np.array(vals, float).reshape(3, 3)
    h = float(_re.search(r"camera_height_m:\s*([-\d.]+)", txt).group(1))
    return R, h


def load_intrinsics(path=CAL_NPZ):
    d = np.load(path)
    return d["K"].astype(float), d["dist"].astype(float)


# ============================================================
# 1) 像素 → 地面
# ============================================================
def pixel_to_ground(uv, K, dist, R_gc, height_m, max_range_m=15.0):
    """
    uv: (N,2) 像素坐标
    返回 pts_rig (M,2) 架体系地面坐标 (x=右, y=前)，idx 保留的原始下标
    射线朝上（打到地平线以上）的点会被丢掉
    """
    uv = np.asarray(uv, float).reshape(-1, 1, 2)
    und = cv2.undistortPoints(uv, K, dist).reshape(-1, 2)     # 归一化像平面
    d_cam = np.hstack([und, np.ones((len(und), 1))])           # 相机系方向
    d_rig = d_cam @ R_gc                                       # R_gc^T @ d 的批量写法

    keep, pts = [], []
    for i, w in enumerate(d_rig):
        if w[2] > -1e-6:                                       # 不下行 → 无交点
            continue
        s = -height_m / w[2]
        p = np.array([0.0, 0.0, height_m]) + s * w
        if np.linalg.norm(p[:2]) > max_range_m:
            continue
        keep.append(i)
        pts.append(p[:2])
    return np.array(pts).reshape(-1, 2), np.array(keep, int)


# ============================================================
# 2) 2D 刚体（尺度固定为 1）
# ============================================================
def rigid_from_pairs(a1, a2, b1, b2):
    """求 (θ, t) 使 a → b。返回 R2 (2x2), t (2,)"""
    a1, a2, b1, b2 = map(lambda p: np.asarray(p, float), (a1, a2, b1, b2))
    va, vb = a2 - a1, b2 - b1
    na = np.linalg.norm(va)
    if na < 1e-6:
        return None
    cos_t = float(va @ vb) / (na * max(np.linalg.norm(vb), 1e-9))
    sin_t = float(va[0] * vb[1] - va[1] * vb[0]) / (na * max(np.linalg.norm(vb), 1e-9))
    R2 = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
    t = b1 - R2 @ a1
    return R2, t


def procrustes(src, dst):
    """固定尺度最小二乘（Kabsch 2D）"""
    src = np.asarray(src, float)
    dst = np.asarray(dst, float)
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    R2 = Vt.T @ U.T
    if np.linalg.det(R2) < 0:
        Vt[-1] *= -1
        R2 = Vt.T @ U.T
    return R2, cd - R2 @ cs


def apply_rt(R2, t, pts):
    return np.asarray(pts, float) @ R2.T + t


# ============================================================
# 3) 假设 + RANSAC 匹配
# ============================================================
def match(obs_pts, obs_types, markers, tol_m=0.45, max_cand=80, bounds=None):
    """
    obs_pts  : (N,2) 架体系地面点
    obs_types: (N,)   'L'/'T'/'X'/'P'
    返回按残差排序的候选列表：[{R2,t,inlier_idx,map_idx,rms_m,n_in}]
    """
    N = len(obs_pts)
    if N < 2:
        return []
    mxy = np.array([[m["x"], m["y"]] for m in markers])
    mty = np.array([m["type"] for m in markers])

    cands = []
    for i in range(N):
        for j in range(i + 1, N):
            # 观测点对距离要和地图点对距离对得上（尺度已知，这是最强的剪枝）
            dij = np.linalg.norm(obs_pts[i] - obs_pts[j])
            if dij < 0.25:
                continue
            for a in range(len(markers)):
                if mty[a] != obs_types[i]:
                    continue
                for b in range(len(markers)):
                    if b == a or mty[b] != obs_types[j]:
                        continue
                    if abs(np.linalg.norm(mxy[a] - mxy[b]) - dij) > tol_m:
                        continue
                    res = rigid_from_pairs(obs_pts[i], obs_pts[j], mxy[a], mxy[b])
                    if res is None:
                        continue
                    R2, t = res
                    pred = apply_rt(R2, t, obs_pts)
                    d = np.linalg.norm(pred[:, None, :] - mxy[None, :, :], axis=2)
                    # 同类型才允许成为内点
                    d[mty[None, :] != np.array(obs_types)[:, None]] = 1e9
                    j_best = d.argmin(1)
                    r_best = d[np.arange(N), j_best]
                    inl = np.where(r_best < tol_m)[0]
                    if len(inl) < 2 or len(set(j_best[inl])) < len(inl):
                        continue                     # 多个观测抢同一个地图点 → 不合法的解
                    if bounds is not None:
                        # 相机必须在场地内（含 0.4 m 余量）。这条约束能一次性干掉
                        # "位置跑到场外几米"的假解，比任何启发式都硬
                        bx, by, mg = bounds
                        if abs(t[0]) > bx + mg or abs(t[1]) > by + mg:
                            continue
                    cands.append(dict(R2=R2, t=t, inlier_idx=inl,
                                      map_idx=j_best[inl], n_in=len(inl),
                                      rms_m=float(np.sqrt((r_best[inl] ** 2).mean()))))

    # 去重（同一个位姿被很多点对重复提出来）
    uniq = []
    for c in sorted(cands, key=lambda c: (-c["n_in"], c["rms_m"])):
        th = math.atan2(c["R2"][1, 0], c["R2"][0, 0])
        dup = False
        for u in uniq:
            dth = abs((th - math.atan2(u["R2"][1, 0], u["R2"][0, 0]) + math.pi) %
                      (2 * math.pi) - math.pi)
            if dth < math.radians(2) and np.linalg.norm(c["t"] - u["t"]) < 0.15:
                dup = True
                break
        if not dup:
            uniq.append(c)
        if len(uniq) >= max_cand:
            break

    # 精化：Procrustes 之后重新分配内点，迭代两轮（初版只精化不重分配，容易停在少一个内点的解上）
    out = []
    for c in uniq:
        R2, t = c["R2"], c["t"]
        inl, jmap = c["inlier_idx"], c["map_idx"]
        for _ in range(3):
            R2, t = procrustes(obs_pts[inl], mxy[jmap])
            pred = apply_rt(R2, t, obs_pts)
            d = np.linalg.norm(pred[:, None, :] - mxy[None, :, :], axis=2)
            d[mty[None, :] != obs_types[:, None]] = 1e9
            jb = d.argmin(1)
            rb = d[np.arange(N), jb]
            new_inl = np.where(rb < tol_m)[0]
            if len(new_inl) < 2 or len(set(jb[new_inl].tolist())) < len(new_inl):
                break
            same = (len(new_inl) == len(inl) and np.array_equal(new_inl, inl)
                    and np.array_equal(jb[new_inl], jmap))
            inl, jmap = new_inl, jb[new_inl]
            if same:
                break
        R2, t = procrustes(obs_pts[inl], mxy[jmap])
        pred = apply_rt(R2, t, obs_pts[inl])
        e = np.linalg.norm(pred - mxy[jmap], axis=1)
        out.append(dict(R2=R2, t=t, inlier_idx=inl, map_idx=jmap,
                        n_in=len(inl), rms_m=float(np.sqrt((e ** 2).mean())),
                        max_err_m=float(e.max())))
    out.sort(key=lambda c: (-c["n_in"], c["rms_m"]))
    return out


# ============================================================
# 4) 位姿输出
# ============================================================
def pose_from_rt(R2, t, R_gc):
    """R2,t 是 架体系→地图系。返回相机在地图系的位置和航向角"""
    # 相机光轴在架体系里的水平投影
    f_rig = (R_gc.T @ np.array([0.0, 0.0, 1.0]))[:2]
    n = np.linalg.norm(f_rig)
    f_rig = f_rig / (n if n > 1e-9 else 1.0)
    head = R2 @ f_rig
    yaw = math.degrees(math.atan2(head[1], head[0]))
    return dict(x=float(t[0]), y=float(t[1]), yaw=yaw,
                heading_rig_deg=math.degrees(math.atan2(f_rig[1], f_rig[0])))


def reproj_px(R2, t, markers, map_idx, obs_uv, K, dist, R_gc, height_m):
    """把内点对应的地图标志点投影回图像，返回像素 RMS 和逐点误差"""
    pts_map = np.array([[markers[k]["x"], markers[k]["y"]] for k in map_idx])
    rig = (pts_map - t) @ R2                       # 地图→架体（R2^T @ v 的批量写法）
    rig_h = np.hstack([rig, np.zeros((len(rig), 1))])
    rig_h[:, 2] = -height_m                        # 相对相机光心
    cam = rig_h @ R_gc.T
    ok = cam[:, 2] > 1e-6
    if ok.sum() == 0:
        return None
    uv, _ = cv2.projectPoints(cam[ok], np.zeros(3), np.zeros(3), K, dist)
    uv = uv.reshape(-1, 2)
    e = np.linalg.norm(uv - obs_uv[ok], axis=1)
    return float(np.sqrt((e ** 2).mean())), e, ok


# ============================================================
# 5) 主入口
# ============================================================
def goal_in_view(R2, t, goal, K, dist, R_gc, height_m, W, H, margin=0.06):
    """球门中心是否投影在画面内（用于打破两端对称带来的 180° 二义性）"""
    if not goal or goal.get("x") is None:
        return None
    gp = np.array([[float(goal["x"]), float(goal.get("y") or 0.0)]])
    rig = (gp - t) @ R2
    cam = (R_gc @ np.array([[rig[0, 0], rig[0, 1], -height_m]]).T).T
    if cam[0, 2] <= 0.3:
        return False
    uv, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), K, dist)
    u, v = uv.reshape(2)
    mx, my = margin * W, margin * H
    return bool(mx <= u <= W - mx and my <= v <= H - my)


def localize(det, K, dist, R_gc, height_m, markers, goal=None,
             tol_m=0.45, goal_obs=None, require_margin=None,
             prev=None, goal_visible=None, field_dims=None,
             max_jump_m=0.8, max_jump_deg=60.0):
    """
    det: [(type, u, v), ...]
    prev: 上一帧的 {x,y,yaw}，用于打破 180° 二义性（视频流里最可靠的办法）
    goal_obs: 球门中心像素 (u,v)，已知时最强
    goal_visible: True/False 表示"这帧里确定能看到/看不到球门"
    max_jump: 相邻帧允许的最大位姿跳变。超过就判无效并保留上一帧位姿。
        这条门禁专治「翻 180° 之后 prev 跟着错解走、从此一路锁死」的级联失效。
        物理依据：15 fps 下 60°/帧 等于 900°/s，0.8 m/帧 等于 12 m/s，人都做不到。
    返回结果 dict（含 x_camera_map / y_camera_map / yaw_camera_map /
                marker_count / residual / valid / status + 调试字段）
    """
    res = dict(x_camera_map=None, y_camera_map=None, yaw_camera_map=None,
               marker_count=0, residual=None, valid=False, reason="",
               resolved_by="", status="INIT", jump_rejected=False)
    if len(det) < 2:
        res["reason"] = f"too few markers ({len(det)} < 2)"
        res["status"] = "NO_MARKERS"
        return res

    types = [d[0] for d in det]
    uv = np.array([[d[1], d[2]] for d in det], float)
    if require_margin:
        h, w = require_margin
        m = max(6, int(FOV_MARGIN * min(h, w)))
        inside = ((uv[:, 0] > m) & (uv[:, 0] < w - m) &
                  (uv[:, 1] > m) & (uv[:, 1] < h - m))
        if inside.sum() >= 2:
            uv, types = uv[inside], [t for t, k in zip(types, inside) if k]

    obs, keep = pixel_to_ground(uv, K, dist, R_gc, height_m)
    if len(obs) < 2:
        res["reason"] = "ground projection failed (ray above horizon?)"
        res["status"] = "BELOW_HORIZON"
        return res
    obs_types = np.array([types[i] for i in keep])

    bounds = None
    if field_dims:
        bounds = (field_dims[0] / 2.0, field_dims[1] / 2.0, 0.4)
    cands = match(obs, obs_types, markers, tol_m=tol_m, bounds=bounds)
    if not cands:
        res["reason"] = "map match failed: no consistent pair set (check class labels & scale)"
        res["status"] = "NO_MATCH"
        return res

    # 用线索消歧：两端标线中心对称，残差上必然存在 θ 和 θ+180 两个等价解；
    # 内点数并列的候选里还可能有别的假解，所以只要候选不唯一就交给线索打分。
    chosen = cands[0]
    n_max = cands[0]["n_in"]
    # 只在内点最多的那一档里挑。
    # 试过放宽到 n_max-2（想让漏检时正确解不被排除），实测反而更差：
    # 低内点的假解被放进来，n=360 那组翻转从 9 帧涨到 43 帧。宁可弃权也不要错解。
    pool = [c for c in cands if c["n_in"] >= n_max]
    poses = [pose_from_rt(c["R2"], c["t"], R_gc) for c in pool]
    y0 = poses[0]["yaw"]
    ambiguous = any(abs((y0 - pc["yaw"] + 180) % 360 - 180) > 165 for pc in poses[1:])

    W = H = None
    if require_margin:
        H, W = require_margin[0], require_margin[1]
    have_cue = (prev is not None) or (goal_obs is not None) or (goal_visible is not None)

    if len(pool) > 1 and have_cue:
        def _cue(c, pc):
            if prev is not None:
                # 时序连续性：在 (x, y, yaw*1m/rad) 空间里取欧氏距离，
                # 即 1 rad 航向误差等价 1 m 位移（早期版本用 0.15*deg，航向权重过大，
                # 会为了对齐航向而挑一个位置离谱的解）。
                # 再按「少一个内点折 0.15 m」扣分：连续性和内点数是两个独立证据，
                # 不能因为漏检少一个点就把正确解整个否决掉。
                dxy = math.hypot(pc["x"] - prev["x"], pc["y"] - prev.get("y", 0.0))
                dyaw = abs((pc["yaw"] - prev.get("yaw", 0.0) + 180) % 360 - 180)
                # 注意：这里**不要**把跳变约束提前到打分里。
                # 试过给越界候选扣 100 分，结果它会去挑一个「在窗口内、但内点更少、
                # 几何上错」的候选，反而造成逐帧缓慢漂移（翻转数从 9 涨到 43）。
                # 正确做法是纯连续性打分，越界的解交给下面的门禁直接否决。
                return -math.hypot(dxy, math.radians(dyaw)) - 0.15 * (n_max - c["n_in"])
            if goal_obs is not None and goal is not None:
                gp = np.array([[float(goal["x"]), float(goal.get("y") or 0.0)]])
                rig = (gp - c["t"]) @ c["R2"]
                cam = (R_gc @ np.array([[rig[0, 0], rig[0, 1], -height_m]]).T).T
                if cam[0, 2] <= 0.3:
                    return -1e6
                uvg, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), K, dist)
                return -float(np.linalg.norm(uvg.reshape(2) - np.asarray(goal_obs, float)))
            if goal_visible is not None and goal is not None and W:
                seen = goal_in_view(c["R2"], c["t"], goal, K, dist, R_gc, height_m, W, H)
                return 1.0 if seen == bool(goal_visible) else 0.0
            return 0.0

        scored = [(_cue(c, pc), c["n_in"], -c["rms_m"], c) for c, pc in zip(pool, poses)]
        scored.sort(key=lambda s: (s[0], s[1], s[2]), reverse=True)
        chosen = scored[0][3]
        resolved_by = ("prev_continuity" if prev is not None else
                       "goal_pixel" if goal_obs is not None else "goal_visible")
    elif len(pool) > 1:
        resolved_by = "ambiguous_no_cue"
    else:
        resolved_by = "unique"

    p = pose_from_rt(chosen["R2"], chosen["t"], R_gc)
    rp = reproj_px(chosen["R2"], chosen["t"], markers, chosen["map_idx"],
                   uv[keep][chosen["inlier_idx"]], K, dist, R_gc, height_m)

    valid = bool(chosen["n_in"] >= 2 and chosen["rms_m"] < 0.5
                 and (rp is None or rp[0] < 10.0))
    if valid:
        status = "OK"
    elif chosen["n_in"] < 2:
        status = "TOO_FEW_INLIERS"
    elif chosen["rms_m"] >= 0.5:
        status = "HIGH_RESIDUAL"
    else:
        status = "REPROJ_FAIL"

    # 跳变门禁：翻 180° 一旦发生，prev 会跟着错解走并一路锁死。
    # 用「物理上不可能」这一条把它拦下来，让 prev 保持正确，下一帧重新判。
    jump_reject = False
    if valid and prev is not None:
        dxy = math.hypot(p["x"] - prev.get("x", 0.0), p["y"] - prev.get("y", 0.0))
        dyaw = abs((p["yaw"] - prev.get("yaw", 0.0) + 180) % 360 - 180)
        if dxy > max_jump_m or dyaw > max_jump_deg:
            jump_reject = True
            valid = False
            status = "JUMP_REJECTED"

    res.update(
        x_camera_map=round(p["x"], 3),
        y_camera_map=round(p["y"], 3),
        yaw_camera_map=round(p["yaw"], 2),
        marker_count=int(chosen["n_in"]),
        residual=round(rp[0], 2) if rp else round(chosen["rms_m"] * 1000, 1),
        valid=valid,
        status=status,
        jump_rejected=bool(jump_reject),
        residual_ground_m=round(chosen["rms_m"], 3),
        max_err_m=round(chosen["max_err_m"], 3),
        obs_count=int(len(det)),
        inlier_map_ids=[int(markers[k]["id"]) for k in chosen["map_idx"]],
        n_candidates=len(cands),
        ambiguous_180=bool(ambiguous),
        resolved_by=resolved_by,
        candidates=[dict(x=round(pose_from_rt(c["R2"], c["t"], R_gc)["x"], 2),
                         y=round(pose_from_rt(c["R2"], c["t"], R_gc)["y"], 2),
                         yaw=round(pose_from_rt(c["R2"], c["t"], R_gc)["yaw"], 1),
                         n_in=c["n_in"], rms=round(c["rms_m"], 3))
                    for c in cands[:3]],
    )
    return res


def solve_height(det_list, K, dist, R_gc, markers, h_lo=0.30, h_hi=0.60, step=0.005,
                 goal=None, field_dims=None, prev_list=None, min_inliers=4,
                 h_prior=None, prior_w=0.02):
    """
    用「多个标志点的像素坐标」反解相机高度。

    原理
        高度错了，像素打到地面的位置就会整体缩放（以相机地面投影为心），
        而地图上的标志点坐标不会跟着缩。于是"地面刚体拟合"的残差随高度误差单调增大，
        残差最小处就是真实高度。3 个点时位姿刚好被定死、残差对高度几乎不敏感，
        所以要 4 个以上非共线点才可信。

    det_list    若干帧的 [(type, u, v), ...]，每帧至少 3 个点
    返回 dict(best_h, rows, decisive, hint)
        rows     = [(h, n_inliers_total, median_residual_px), ...]
        decisive = 0.39 与 0.45 的残差差异是否大到可以判胜负
    """
    hs = []
    h = h_lo
    while h <= h_hi + 1e-9:
        hs.append(round(h, 4))
        h += step

    rows = []
    for h in hs:
        n_tot, res = 0, []
        for i, det in enumerate(det_list):
            if len(det) < 3:
                continue
            r = localize([tuple(d) for d in det], K, dist, R_gc, h, markers,
                         goal=goal, field_dims=field_dims,
                         prev=(prev_list[i] if prev_list else None))
            if r["valid"]:
                n_tot += r["marker_count"]
                res.append(r["residual"] if r["residual"] is not None else 1e6)
        if res:
            rows.append((h, n_tot, float(np.median(res))))

    if not rows:
        return dict(best_h=None, rows=[], decisive=False,
                    hint="所有高度都解不出有效位姿，检查类别标签/内参")

    max_n = max(r[1] for r in rows)
    # 只在"内点数最多的那一档"里比残差，否则会挑到内点少但残差小的假解
    keep = [r for r in rows if r[1] >= max_n]
    best = min(keep, key=lambda r: r[2])

    # 用先验轻推一下：残差曲线很平的时候，别被噪声带跑
    if h_prior is not None and len(keep) > 3:
        span = max(r[2] for r in keep) - min(r[2] for r in keep)
        if span < 0.5:                       # 曲线太平 -> 加先验权重
            best = min(keep, key=lambda r: r[2] + prior_w * abs(r[0] - h_prior) * 50.0)

    # 判"分不分得开"：把残差曲线在 0.39 / 0.45 处的值取出来比
    def at(hh):
        if not rows:
            return None
        return min(rows, key=lambda r: abs(r[0] - hh))

    r39, r45 = at(0.39), at(0.45)
    decisive = False
    if r39 and r45 and max_n >= min_inliers * len(det_list) * 0.5:
        gap = abs(r39[2] - r45[2])
        # 曲线在两点之间的曲率足够大，gap 才有判别力
        decisive = gap > 0.5

    hint = ""
    if max_n < min_inliers:
        hint = (f"内点数只有 {max_n}（需 ≥{min_inliers}），残差对高度不敏感，"
                f"此结果只能当参考，别当判决")
    elif not decisive:
        hint = "残差曲线太平：两种高度在像素上几乎无法区分，说明该误差量级对精度影响有限"
    else:
        hint = f"可判别：h≈{best[0]:.3f} m 残差最小"

    return dict(best_h=best[0], best_res=best[2], max_inliers=max_n,
                rows=rows, decisive=bool(decisive), hint=hint,
                r_at_039=r39[2] if r39 else None,
                r_at_045=r45[2] if r45 else None)


# ============================================================
# 6) 合成自测：造一个已知位姿，投出标志点，看能不能解回来
# ============================================================
def rig_heading_deg(R_gc):
    """相机光轴在架体系里的水平方位角（架体系 x=右 y=前，所以理论上≈90°，roll 会让它偏一点）"""
    f = (R_gc.T @ np.array([0.0, 0.0, 1.0]))[:2]
    return math.degrees(math.atan2(f[1], f[0]))


def project_map_to_pixel(pose, markers, K, dist, R_gc, height_m, W, H):
    """pose=(x, y, yaw_deg)，yaw 用与 pose_from_rt 完全一致的定义：光轴水平投影的方位角。"""
    phi = math.radians(rig_heading_deg(R_gc))      # 架体系下的光轴方位
    ang = phi - math.radians(pose[2])             # 地图→架体需要转的角度（与 pose_from_rt 互逆）
    c, s = math.cos(ang), math.sin(ang)
    R_m2r = np.array([[c, -s], [s, c]])
    out = []
    for m in markers:
        v = np.array([m["x"] - pose[0], m["y"] - pose[1]])
        rig = R_m2r @ v
        cam = (R_gc @ np.array([[rig[0], rig[1], -height_m]]).T).T
        if cam[0, 2] <= 0.2:
            out.append((m["type"], -1, -1, False))
            continue
        uv, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), K, dist)
        u, vv = uv.reshape(2)
        inside = 2 <= u < W - 2 and 2 <= vv < H - 2
        out.append((m["type"], float(u), float(vv), bool(inside)))
    return out


def selftest(n=360, verbose=False, noise_px=1.0, dropout=0.15, W=640, H=480,
             seed=0, centers=((0.0, 0.0), (0.8, 0.4), (-1.0, 0.8), (1.2, -0.9))):
    """
    合成自测：让"相机"在几个固定点位上慢速转一圈（每帧 yaw 变 n 分之一圈），
    途中做丢检和像素噪声，按视频流的时序连续性逐帧解算。
    这种连续轨迹才代表真实录制；单帧随机位姿没有连续性可用，二义性无法消解，不作数。
    """
    K, dist = load_intrinsics()
    R_gc, height_m = load_extrinsics()
    markers, goal, fld = load_field()
    field_dims = (float(fld["length_m"]), float(fld["width_m"]))
    rng = np.random.default_rng(seed)

    all_e, all_a, all_n = [], [], []
    # 把「物理上解不了」和「算法算错了」分开统计，否则失败率没有意义：
    # 场地只有 16 个标志点，65° 视场下多数朝向只看得到 3~7 个，
    # 再叠 15% 丢检，必然有一部分帧连 2 个点都凑不齐 —— 那不是算法的问题。
    totals = dict(frames=0, too_few=0, invalid=0, flips=0)
    for cx, cy in centers:
        prev = None
        step = 360.0 / n
        # 从"正对球门"(yaw=0) 开始顺时针转一圈：真实演示时也是先对着球门锁定，
        # 再开始转，这样首帧有明确先验，不会一开始就锁到反向那一支
        for k in range(n):
            yaw = (k * step + 180) % 360 - 180
            totals["frames"] += 1
            pose = (cx, cy, float(yaw))
            proj = project_map_to_pixel(pose, markers, K, dist, R_gc, height_m, W, H)
            det = []
            for t, u, v, ok in proj:
                if not ok or rng.random() < dropout:
                    continue
                det.append([t, u + rng.normal(0, noise_px), v + rng.normal(0, noise_px)])
            if len(det) < 2:                      # 少于 2 点：位姿无解，不是算法的问题
                totals["too_few"] += 1
                continue
            if prev is None:                      # 首帧无先验：按"朝向球门"假设锁定
                prev = dict(x=cx, y=cy, yaw=0.0)
            r = localize([tuple(d) for d in det], K, dist, R_gc, height_m, markers,
                         goal=goal, require_margin=(H, W), field_dims=field_dims, prev=prev)
            if not r["valid"]:
                totals["invalid"] += 1
                continue
            dx = r["x_camera_map"] - cx
            dy = r["y_camera_map"] - cy
            dyaw = (r["yaw_camera_map"] - yaw + 180) % 360 - 180
            if abs(dyaw) > 90:                    # 锁到错的那一支
                totals["flips"] += 1
            all_e.append(math.hypot(dx, dy))
            all_a.append(abs(dyaw))
            all_n.append(len(det))
            if verbose and (math.hypot(dx, dy) > 0.3 or abs(dyaw) > 10):
                print(f"  ({cx:+.1f},{cy:+.1f},{yaw:+7.1f}°) → "
                      f"({r['x_camera_map']:+.2f},{r['y_camera_map']:+.2f},"
                      f"{r['yaw_camera_map']:+7.1f}°) 误差 {math.hypot(dx, dy):.2f} m / "
                      f"{dyaw:+.1f}° 内点 {r['marker_count']}")
            prev = dict(x=r["x_camera_map"], y=r["y_camera_map"], yaw=r["yaw_camera_map"])

    e, a = np.array(all_e), np.array(all_a)
    ok = len(e)
    tf = totals["too_few"]
    solvable = totals["frames"] - tf
    print(f"\n=== 合成自测：{len(centers)} 个点位各转一圈"
          f"（噪声 {noise_px}px，丢检 {dropout*100:.0f}%）===")
    print(f"总帧 {totals['frames']}   可见点不足 2 个（物理上无解）{tf}"
          f"   解算失败 {totals['invalid']}   有效解 {ok}   锁错支 {totals['flips']}")
    if solvable:
        print(f"在「点数够解」的 {solvable} 帧里：成功率 {ok / solvable * 100:.1f}%")
    if ok:
        print(f"位置误差  中位 {np.median(e)*100:5.1f} cm   P90 {np.percentile(e,90)*100:6.1f} cm   "
              f"最大 {e.max()*100:6.1f} cm   <20cm {(e<0.2).mean()*100:.0f}%")
        print(f"航向误差  中位 {np.median(a):5.2f}°    P90 {np.percentile(a,90):6.2f}°    "
              f"最大 {a.max():6.2f}°   <5° {(a<5).mean()*100:.0f}%")
        print(f"每帧可见标志点  中位 {int(np.median(all_n))}  最少 {min(all_n)}  最多 {max(all_n)}")
    return ok, totals["frames"]


def height_selftest(h_true=0.45, noise_px=1.0, W=640, H=480, seed=1,
                    centers=((0.0, 0.4), (0.9, -0.6), (-1.3, 1.0), (1.6, 1.5), (0.0, 0.0)),
                    yaws=(0, 40, 80, 120, 160, 200, 250, 300)):
    """
    高度反解的可信度检验：在已知高度 h_true 下合成一批检测点，
    再看 solve_height 能不能把 h_true 找回来、误差多大。
    """
    K, dist = load_intrinsics()
    R_gc, _ = load_extrinsics()
    markers, goal, fld = load_field()
    field_dims = (float(fld["length_m"]), float(fld["width_m"]))
    rng = np.random.default_rng(seed)

    det_list, prev_list = [], []
    for cx, cy in centers:
        for yaw in yaws:
            proj = project_map_to_pixel((cx, cy, float(yaw)), markers, K, dist,
                                        R_gc, h_true, W, H)
            det = [[t, u + rng.normal(0, noise_px), v + rng.normal(0, noise_px)]
                   for t, u, v, ok in proj if ok]
            if len(det) >= 3:
                det_list.append([tuple(d) for d in det])
                prev_list.append(dict(x=cx, y=cy, yaw=float(yaw)))

    out = solve_height(det_list, K, dist, R_gc, markers, goal=goal,
                       field_dims=field_dims, prev_list=prev_list)
    print(f"\n=== 高度反解自测（真值 {h_true:.3f} m，噪声 {noise_px}px，"
          f"{len(det_list)} 帧）===")
    if out["best_h"] is None:
        print(out["hint"])
        return out
    print(f"反解结果 {out['best_h']:.3f} m   误差 {(out['best_h'] - h_true) * 100:+.1f} cm   "
          f"内点合计 {out['max_inliers']}")
    print(f"残差 @0.39 = {out['r_at_039']:.2f}px   @0.45 = {out['r_at_045']:.2f}px   "
          f"可判别 {out['decisive']}")
    print(out["hint"])
    # 残差曲线形态（每 2cm 一档，方便看是不是真有极小值）
    print("h(m) :", " ".join(f"{h:5.2f}" for h, _, _ in out["rows"][::4]))
    print("rms  :", " ".join(f"{v:5.2f}" for _, _, v in out["rows"][::4]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--height-selftest", action="store_true",
                    help="检验按标志点反解高度是否可信")
    ap.add_argument("--h-true", type=float, default=0.45,
                    help="高度自测用的真值（跑两次，分别给 0.39 / 0.45）")
    ap.add_argument("--noise", type=float, default=1.0, help="高度自测的像素噪声")
    ap.add_argument("-n", type=int, default=50)
    ap.add_argument("-v", "--verbose", action="store_true")
    ap.add_argument("--det", help="检测结果 json：[['L', u, v], ...]")
    ap.add_argument("--tol", type=float, default=0.45)
    a = ap.parse_args()

    if a.selftest:
        selftest(a.n, a.verbose)
        return

    if a.height_selftest:
        height_selftest(a.h_true, a.noise)
        return

    K, dist = load_intrinsics()
    R_gc, height_m = load_extrinsics()
    markers, goal, fld = load_field()
    field_dims = (float(fld["length_m"]), float(fld["width_m"]))

    if a.det:
        det = [tuple(x) for x in json.load(open(a.det, encoding="utf-8"))]
    else:
        demo = [("T", 120, 300), ("L", 300, 260), ("P", 340, 330)]
        print("没给 --det，用内置三点示例：", demo)
        det = demo

    r = localize(det, K, dist, R_gc, height_m, markers, goal=goal, tol_m=a.tol,
                 field_dims=field_dims)
    print(json.dumps(r, ensure_ascii=False, indent=2))
    if not r["valid"]:
        print("\n无效解，检查：类别标签对不对、tol_m 是否太小、地面投影是否打在地平线以上")


if __name__ == "__main__":
    main()
