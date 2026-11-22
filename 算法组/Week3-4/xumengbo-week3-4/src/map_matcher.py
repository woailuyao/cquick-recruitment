"""
地图匹配与位姿估计（map_matcher）
==================================
细则 2.8：不能按类别把检测框随便配到最近同类点；必须枚举同类多候选，
用整体几何残差评价候选位姿，处理场地对称歧义。

思路（骨架，SE(2) 两点定姿 + 残差评估）：
  1. 取两个观测点，分别与地图上同类候选配对；
  2. 由两对点求载体系->地图系的二维位姿 (x, y, yaw)；
  3. 把全部观测变到地图系，计算与各自类别地图点的整体残差；
  4. 枚举配对组合，保留残差最小的位姿；次优与最优接近则判 ambiguous。

状态：
  ok / insufficient_markers / ambiguous / high_residual / not_converged
"""
import math
import itertools
import numpy as np

# 残差阈值（米）：基础验收平面误差 0.5m，匹配端留余量
RESIDUAL_THRESHOLD = 0.35
# 次优/最优残差比值小于该值视为多解接近，判歧义
AMBIG_RATIO = 1.3


def _pose_from_two(p1, p2, q1, q2):
    """由载体系两点 p1,p2 与地图系对应点 q1,q2 求 SE(2) 位姿。

    返回 (x, y, yaw_deg)：把载体系变换到地图系的位姿。
    """
    vo = np.array([p2[0] - p1[0], p2[1] - p1[1]])
    vm = np.array([q2[0] - q1[0], q2[1] - q1[1]])
    ang_o = math.atan2(vo[1], vo[0])
    ang_m = math.atan2(vm[1], vm[0])
    yaw = math.degrees(ang_m - ang_o)

    a = math.radians(yaw)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    t = np.array(q1) - R @ np.array(p1)
    return float(t[0]), float(t[1]), float(yaw)


def _residual(pose, obs, field_map):
    """计算某候选位姿下全部观测的匹配残差与匹配明细。"""
    x, y, yaw = pose
    a = math.radians(yaw)
    R = np.array([[math.cos(a), -math.sin(a)], [math.sin(a), math.cos(a)]])
    t = np.array([x, y])

    dists = []
    matches = []
    for ob in obs:
        p = np.array([ob["x_b"], ob["y_b"]])
        p_map = R @ p + t
        cands = field_map.candidates(ob["type"])
        if not cands:
            continue
        # 找同类最近地图点
        best = min(cands, key=lambda c: np.hypot(c[1] - p_map[0], c[2] - p_map[1]))
        d = float(np.hypot(best[1] - p_map[0], best[2] - p_map[1]))
        dists.append(d)
        matches.append({
            "type": ob["type"], "obs_xy": [round(float(p[0]), 3), round(float(p[1]), 3)],
            "map_id": best[0], "map_xy": [best[1], best[2]], "dist": round(d, 3),
        })

    if not dists:
        return float("inf"), matches
    rms = float(np.sqrt(np.mean(np.square(dists))))
    return rms, matches


def match(obs, field_map):
    """主入口：观测地面点列表 + 地图 -> 位姿估计结果。

    obs: [{"type", "x_b", "y_b", "conf", ...}]，仅传 valid 的点。
    返回 dict：valid/status/carrier_pose/marker_count/residual/matches。
    """
    valid_obs = [o for o in obs if o.get("x_b") is not None]
    marker_count = len(valid_obs)

    # 标志不足：单点无法唯一确定位置和朝向
    if marker_count < 3:
        return {"valid": False, "status": "insufficient_markers",
                "carrier_pose": None, "marker_count": marker_count,
                "residual": None, "matches": []}

    best = {"rms": float("inf"), "pose": None, "matches": []}
    second = {"rms": float("inf")}

    # 枚举两个观测点的组合作为定姿锚点
    for i, j in itertools.combinations(range(marker_count), 2):
        oi, oj = valid_obs[i], valid_obs[j]
        cands_i = field_map.candidates(oi["type"])
        cands_j = field_map.candidates(oj["type"])
        if not cands_i or not cands_j:
            continue
        # 枚举同类候选配对
        for ci in cands_i:
            for cj in cands_j:
                if oi["type"] == oj["type"] and ci[0] == cj[0]:
                    continue  # 同类不能配到同一个地图点
                pose = _pose_from_two(
                    (oi["x_b"], oi["y_b"]), (oj["x_b"], oj["y_b"]),
                    (ci[1], ci[2]), (cj[1], cj[2]))
                rms, matches = _residual(pose, valid_obs, field_map)
                if rms < best["rms"]:
                    second = best
                    best = {"rms": rms, "pose": pose, "matches": matches}
                elif rms < second["rms"]:
                    second = {"rms": rms}

    if best["pose"] is None:
        return {"valid": False, "status": "not_converged",
                "carrier_pose": None, "marker_count": marker_count,
                "residual": None, "matches": []}

    # 歧义判断：次优与最优残差接近
    if second["rms"] < float("inf") and best["rms"] * AMBIG_RATIO > second["rms"]:
        status = "ambiguous"
        valid = False
    elif best["rms"] > RESIDUAL_THRESHOLD:
        status = "high_residual"
        valid = False
    else:
        status = "ok"
        valid = True

    return {"valid": valid, "status": status,
            "carrier_pose": best["pose"], "marker_count": marker_count,
            "residual": round(best["rms"], 4), "matches": best["matches"]}
