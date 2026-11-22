# -*- coding: utf-8 -*-
"""locator.py —— 任务二定位主脚本：标志检测 → 地图匹配 → 二维位姿

流程（与细则 2.2 / locator.cpp 的思路一致）：
  1. detect_landmarks 提取 L/T/X 交点像素坐标
  2. 用内参 + field.yaml 的已知场地坐标做 PnP-RANSAC（平面 IPPE），
     随机抽样 4 对对应点求位姿，用全部重投影内点投票选最优
  3. 位姿换算为相机光心在场地地图中的 (x, y, yaw)（+ 俯仰角 / 匹配残差）
  4. 可选：与定点拍摄记录的真值比较，输出位置误差和航向误差

约定：
  场地坐标系 field.yaml（+X 朝球门、+Y 面向 +X 左侧、Z 向上、米）
  yaw 定义：光轴在水平面的投影方向，atan2(Y, X)，逆时针为正，单位度
  （面向球门 = 0°，向左转为正）

用法：
  python locator.py --image D:/captures/_frames/M2_mid.png --truth "0,0"
  python locator.py --video D:/captures/M2.mp4 --stride 45 --truth "0,0"
"""
import argparse
import itertools
import os
import sys

import cv2
import numpy as np

from detect_landmarks import (extract_lines, line_intersections,
                              white_line_mask)


# ---------- 配置读取 ----------
def load_field(path):
    """读 field.yaml 中的 [x, y] 标志坐标（忽略标量字段）。"""
    pts = {}
    with open(path, encoding="utf-8") as f:
        for raw in f:
            line = raw.split("#")[0].strip()
            if ":" not in line:
                continue
            k, v = line.split(":", 1)
            v = v.strip()
            if not (v.startswith("[") and v.endswith("]")):
                continue
            try:
                xy = [float(t) for t in v.strip("[]").split(",")]
            except ValueError:
                continue
            if len(xy) == 2:
                pts[k.strip()] = np.array(xy, dtype=np.float64)
    return pts


def field_type(name):
    """从标志名推类型：TCross1 → T，LCross2 → L。用于匹配时的类型约束。"""
    s = name.strip().lower()
    if s.startswith("t"):
        return "T"
    if s.startswith("l"):
        return "L"
    return None


def load_intrinsics(path):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        raise RuntimeError("读不了内参文件: " + path)
    K = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("distortion_coefficients").mat()
    w = int(fs.getNode("image_width").real())
    h = int(fs.getNode("image_height").real())
    fs.release()
    return K, dist, (w, h)


# ---------- 位姿解算 ----------
def pose_from_rvec(rvec, tvec):
    R, _ = cv2.Rodrigues(rvec)
    C = (-R.T @ tvec).ravel()                 # 相机光心在世界系
    axis = R.T @ np.array([0.0, 0.0, 1.0])    # 光轴方向（世界系）
    yaw = np.degrees(np.arctan2(axis[1], axis[0]))
    pitch = np.degrees(np.arcsin(np.clip(-axis[2], -1, 1)))
    return C, yaw, pitch


def match_score(rvec, tvec, K, dist, obs, obj, tol=10.0,
                obs_types=None, obj_types=None, tol_metric=None,
                type_penalty=None):
    """把全部地图点投影到图像，与观测点贪心配对，返回 (inlier_idx_pairs, rms)。

    obs_types / obj_types 给出时启用类型约束：L 只能配 L、T 只能配 T。
    这一步很关键——场地 8 个标志在图像上排布相似，不加类型约束时
    "把 LCross2 错配成 TCross3"这种错误对应会让求解器找到几何自洽但
    完全错误的位姿。类型是免费的先验（检测结果自带），必须用上。

    type_penalty 给出时把类型约束从「禁止」改为「强烈惩罚」（距离 × 该系数）：
    检测器偶尔会把 TCross1 判成 L（远处两块标志仅相距 ~60px，纹理相同），
    硬约束下这个错误类别会把几何正确的对应直接锁死，导致位姿被拉偏；
    惩罚式则允许「类别不一致但几何极贴合」的对应胜出，同时仍强烈压制
    跨类乱配。默认 None = 硬约束（原有行为）。

    tol_metric 给出时改用米制容差：像素误差 × 深度 / 焦距 < tol_metric。
    远处标志同样大小的定位偏差对应更多像素，固定像素容差对远点过严。
    """
    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    # 相机坐标系下的深度：projectPoints 对相机后方的点同样会输出（翻转的）
    # 像素坐标，这些点是"看不见"的，若不剔除会与观测形成假对应——实测
    # M5 帧 150 因此解出 yaw=+174°（相机朝反方向）却残差 0.003 m 的假解。
    R, _ = cv2.Rodrigues(rvec)
    depth = ((R @ obj.T).T + tvec.ravel())[:, 2]
    if tol_metric is not None:
        fx = float(K[0, 0])
    pairs = []
    used = set()
    for i, p in enumerate(proj):
        if depth[i] <= 0.20:              # 相机后方或贴脸，不可见
            continue
        d = np.linalg.norm(obs - p, axis=1)
        if tol_metric is not None:
            d = d * depth[i] / fx          # 换算成该点处的米制误差
        if obs_types is not None and obj_types is not None:
            mt = obj_types[i]
            if mt is not None:
                bad = np.array([ot is not None and ot != mt for ot in obs_types])
                if type_penalty is None:
                    d = np.where(bad, np.inf, d)
                else:
                    d = np.where(bad, d * type_penalty, d)
        j = int(np.argmin(d))
        lim = tol_metric if tol_metric is not None else tol
        if d[j] < lim and j not in used:
            used.add(j)
            pairs.append((j, i, float(d[j])))
    if not pairs:
        return [], 1e9
    rms = float(np.sqrt(np.mean([p[2] ** 2 for p in pairs])))
    return pairs, rms


def refine_pose(rvec, tvec, K, dist, obs, obj, rounds=3,
                obs_types=None, obj_types=None, tol=10.0, tol_metric=None,
                type_penalty=None):
    """局部优化（LO）：用当前解的内点重解 + 重新匹配，迭代收敛。"""
    for _ in range(rounds):
        pairs, rms = match_score(rvec, tvec, K, dist, obs, obj, tol=tol,
                                 obs_types=obs_types, obj_types=obj_types,
                                 tol_metric=tol_metric,
                                 type_penalty=type_penalty)
        if len(pairs) < 4:
            return rvec, tvec, pairs, rms
        oi = np.array([p[0] for p in pairs])
        mi = np.array([p[1] for p in pairs])
        try:
            ok, r2, t2 = cv2.solvePnP(obj[mi], obs[oi].astype(np.float32),
                                      K, dist, flags=cv2.SOLVEPNP_IPPE)
            if not ok:
                break
            r2, t2 = cv2.solvePnPRefineLM(obj[mi], obs[oi].astype(np.float32),
                                          K, dist, r2, t2)
        except cv2.error:
            break
        rvec, tvec = r2, t2
    pairs, rms = match_score(rvec, tvec, K, dist, obs, obj, tol=tol,
                             obs_types=obs_types, obj_types=obj_types,
                             tol_metric=tol_metric, type_penalty=type_penalty)
    return rvec, tvec, pairs, rms


def solve_pose(obs, field_pts, K, dist, iters=8000, seed=0, obs_types=None,
               hyp_tol=0.40, final_tol=0.30, type_penalty=None):
    """平面 PnP 位姿搜索。obs: Nx2 像素；field_pts: dict 名称->[x,y]。

    采样策略：组合总数可控时（交点少，通常离线帧）穷举所有 4 对 4 组合，
    保证拿到全局最优；组合爆炸时退化为随机采样 + 局部优化。

    匹配容差用米制（像素误差×深度/焦距）：交点/检测框中心在粗白线上
    有 0.1~0.2m 的固有抖动，固定像素容差对远点过严、近点过松。
    假设阶段用 hyp_tol 宽容差避免把真解扼杀在摇篮里；终验用 final_tol
    收紧，保证输出解的重投影质量。rms 单位为米。
    """
    if len(obs) < 4:
        return []
    names_all = list(field_pts.keys())
    obj_all = np.array([[field_pts[n][0], field_pts[n][1], 0.0]
                        for n in names_all], dtype=np.float64)
    obs = np.asarray(obs, dtype=np.float64)
    obj_types = [field_type(n) for n in names_all]
    # 只保留「有明确类别」的可检测标志。field.yaml 里像 PenalityPoint
    # （点球点）这样不参与训练的键，field_type 返回 None；match_score 的
    # 类型约束在 mt is None 时会跳过，于是它可以匹配任意观测，凭空造出
    # 大量假解（实测：M5/M7 因此被拉到偏离 3m 的位姿）。
    det = [i for i, t in enumerate(obj_types) if t is not None]
    if len(det) < 4:
        return []
    names = [names_all[i] for i in det]
    obj_all = obj_all[det]
    obj_types = [obj_types[i] for i in det]
    rng = np.random.default_rng(seed)
    sols = []
    if len(names) < 4:
        return []

    combos_o = list(itertools.combinations(range(len(obs)), 4))
    # 地图侧用排列（有序 4 元组）：观测子集与地图子集各自按索引排序后
    # 位置配对会漏掉真对应——正确的对应是任意双射，必须枚举排列。
    map_perms = list(itertools.permutations(range(len(names)), 4))
    # 形状预筛：4 点的 6 个两两距离（各自按最大值归一）在投影下近似保持。
    # 预筛把 PnP 次数从 obs组合×map组合×24 降到几十次，穷举才可行。
    _pd = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)]

    def _shape(pts):
        pts = np.asarray(pts, dtype=np.float64)
        d = np.array([np.linalg.norm(pts[a] - pts[b]) for a, b in _pd])
        mx = d.max()
        return d / mx if mx > 1e-9 else d

    dmap = np.array([_shape(obj_all[list(m), :2]) for m in map_perms])
    # 全量形状掩膜（向量化，很快），再从通过预筛的候选里取用
    dobs = np.array([_shape(obs[list(oi)]) for oi in combos_o])
    okmat = np.abs(dobs[:, None, :] - dmap[None, :, :]).max(axis=2) < 0.45
    cand_all = np.argwhere(okmat)
    if len(cand_all) <= 15000:
        mode = "穷举"
        cand = [(np.array(combos_o[a]), np.array(map_perms[b]))
                for a, b in cand_all]
    else:
        mode = "采样"
        idx = rng.choice(len(cand_all), size=5000, replace=False)
        cand = [(np.array(combos_o[a]), np.array(map_perms[b]))
                for a, b in cand_all[idx]]
    print("    [解算] 交点 %d 个 × 地图 %d 个，%s模式，形状预筛后假设 %d 个"
          % (len(obs), len(names), mode, len(cand)))

    for oi, mi in cand:
        try:
            # 4 点平面位姿有镜像双解，必须两个候选都评估
            retval, rvecs, tvecs, errs = cv2.solvePnPGeneric(
                obj_all[mi], obs[oi].astype(np.float32), K, dist,
                flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:
            continue
        for rvec, tvec in zip(rvecs, tvecs):
            rvec = rvec.reshape(3, 1)
            tvec = tvec.reshape(3, 1)
            C, yaw, pitch = pose_from_rvec(rvec, tvec)
            if C[2] < 0.1 or C[2] > 2.5:          # 相机高度不合理 → 丢弃
                continue
            if abs(pitch) > 60:                   # 俯仰不合理
                continue
            pairs, rms = match_score(rvec, tvec, K, dist, obs, obj_all,
                                     tol_metric=hyp_tol,
                                     obs_types=obs_types,
                                     obj_types=obj_types,
                                     type_penalty=type_penalty)
            if len(pairs) < 4:
                continue
            # 局部优化：多数假设只有 4 个内点，重解几轮后常能收敛到 5+ 的正确解
            rvec, tvec, pairs, rms = refine_pose(
                rvec, tvec, K, dist, obs, obj_all, rounds=3,
                obs_types=obs_types, obj_types=obj_types, tol_metric=hyp_tol,
                type_penalty=type_penalty)
            if len(pairs) < 4:
                continue
            C, yaw, pitch = pose_from_rvec(rvec, tvec)
            sols.append((len(pairs), -rms, rvec.copy(), tvec.copy(), C, yaw,
                         pitch, pairs))
    if not sols:
        return []
    sols.sort(key=lambda s: (s[0], s[1]), reverse=True)

    # 用每个解的内点集做一次全局精化。终验对全部解开放：
    # 内点数多的可能是"几何自洽但位置在场外"的假对应，靠终验的门限
    # （高度/俯仰/边界）淘汰，真解即使内点少也能通过。
    out = []
    for s in sols:
        n_in, neg_rms, rvec, tvec, C, yaw, pitch, pairs = s
        oi = np.array([p[0] for p in pairs])
        mi = np.array([p[1] for p in pairs])
        try:
            ok, rvec2, tvec2 = cv2.solvePnP(
                obj_all[mi], obs[oi].astype(np.float32), K, dist,
                flags=cv2.SOLVEPNP_IPPE)
            if ok:
                rvec2, tvec2 = cv2.solvePnPRefineLM(
                    obj_all[mi], obs[oi].astype(np.float32), K, dist,
                    rvec2, tvec2)
                rvec, tvec = rvec2, tvec2
        except cv2.error:
            pass
        C, yaw, pitch = pose_from_rvec(rvec, tvec)
        pairs2, rms = match_score(rvec, tvec, K, dist, obs, obj_all,
                                  tol_metric=final_tol,
                                  type_penalty=type_penalty)
        # 精化后的合理性校验：相机高度接近实测 0.67m、俯仰接近 15°、残差小、
        # 位置必须落在场地内（含 0.5m 容差）。rms 单位：米
        if not (0.35 <= C[2] <= 1.20):
            continue
        if abs(pitch - 15.0) > 30.0:
            continue
        if rms > 0.35 or len(pairs2) < 4:
            continue
        if abs(C[0]) > 4.5 or abs(C[1]) > 3.5:
            continue
        out.append({"x": float(C[0]), "y": float(C[1]), "z": float(C[2]),
                    "yaw": float(yaw), "pitch": float(pitch),
                    "n": len(pairs2), "rms": rms,
                    "pairs": [(names[i], int(j)) for j, i, _ in pairs2]})

    # 去重：位置 0.6m + 航向 12° 内视为同一解
    final = []
    for s in sorted(out, key=lambda s: (-s["n"], s["rms"])):
        dup = False
        for f in final:
            if (abs(s["x"] - f["x"]) < 0.6 and abs(s["y"] - f["y"]) < 0.6
                    and abs((s["yaw"] - f["yaw"] + 180) % 360 - 180) < 12):
                dup = True
                break
        if not dup:
            final.append(s)
        if len(final) >= 3:
            break
    return final


def detect(bgr):
    """经典 CV 后端：白线分割 → 直线求交 → 射线分类（L/T）。"""
    mask = white_line_mask(bgr)
    lines = extract_lines(mask)
    return line_intersections(lines, mask, (bgr.shape[0], bgr.shape[1]))


_YOLO_CACHE = {}


def detect_yolo(bgr, weights, conf=0.25, imgsz=640):
    """YOLOv8 后端：返回 [(x, y, type)]，type 由类别号映射（0=L, 1=T）。"""
    try:
        from ultralytics import YOLO
    except ImportError:
        raise RuntimeError(
            "当前 Python 环境没装 ultralytics，YOLO 后端请在 WSL 训练环境里跑：\n"
            "  ~/vision_ws/k1_soccer_vision/.venv/bin/python locator.py "
            "--detector yolo --weights <best.pt> ...")
    if weights not in _YOLO_CACHE:
        _YOLO_CACHE[weights] = YOLO(weights)
    res = _YOLO_CACHE[weights].predict(bgr, imgsz=imgsz, conf=conf,
                                       verbose=False)[0]
    names = {0: "L", 1: "T"}
    cand = []
    for b in res.boxes:
        cls = int(b.cls[0])
        conf_b = float(b.conf[0])
        x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
        cand.append((conf_b, int((x1 + x2) / 2), int((y1 + y2) / 2),
                     names.get(cls, "L")))
    # 同类 35px 内的重复框去重（保留置信度最高者）
    cand.sort(reverse=True)
    out = []
    for _, x, y, t in cand:
        if not any(tt == t and (x - x2) ** 2 + (y - y2) ** 2 < 35 ** 2
                   for x2, y2, tt in out):
            out.append((x, y, t))
    return out


def report(name, sols, truth=None, truth_yaw=None):
    print("-" * 66)
    print("帧/图: %s" % name)
    if not sols:
        print("  定位状态: insufficient_markers（可解交点不足 4 个或全部被滤除）")
        return
    best = sols[0]
    print("  x_camera_map = %+.3f m   y_camera_map = %+.3f m"
          "   yaw = %+.1f°" % (best["x"], best["y"], best["yaw"]))
    print("  相机高度 = %.2f m（外参参考 0.67 m）  俯仰角 = %.1f°（参考 15°）"
          % (best["z"], best["pitch"]))
    print("  内点数 = %d / 8   重投影残差 = %.3f m   状态 = %s"
          % (best["n"], best["rms"],
             "ok" if best["n"] >= 4 and best["rms"] < 0.30
             else "high_residual"))
    print("  匹配对: " + ", ".join("%s<-%d" % (n, j) for n, j in best["pairs"]))
    if len(sols) > 1:
        print("  其他候选解（可能的多解/歧义）:")
        for s in sols[1:]:
            print("    x=%+.2f y=%+.2f yaw=%+.1f° 内点=%d 残差=%.2f"
                  % (s["x"], s["y"], s["yaw"], s["n"], s["rms"]))
    if truth is not None:
        ex = best["x"] - truth[0]
        ey = best["y"] - truth[1]
        print("  真值 (%.2f, %.2f) → 位置误差 = %.3f m %s"
              % (truth[0], truth[1], float(np.hypot(ex, ey)),
                 "✓ 达标(≤0.5m)" if np.hypot(ex, ey) <= 0.5 else "✗ 超差"))
        if truth_yaw is not None:
            dyaw = (best["yaw"] - truth_yaw + 180) % 360 - 180
            print("  航向误差 = %+.1f° %s"
                  % (dyaw, "✓ 达标(≤10°)" if abs(dyaw) <= 10 else "✗ 超差"))


def selftest(field_pts, K, dist):
    """合成数据自检：给定已知位姿，投影地图点 → 反解位姿 → 比对。
    验证 PnP-RANSAC 数学链路是否正确（与检测质量无关）。"""
    print("=" * 66)
    print("合成数据自检（已知位姿 → 投影 → 反解）")
    names = list(field_pts.keys())
    obj = np.array([[field_pts[n][0], field_pts[n][1], 0.0] for n in names])
    cases = [(0.0, 2.8, -35.0, 15.0),
             (0.0, 2.5, -30.0, 18.0),
             (0.0, 2.99, -40.0, 15.0)]
    ok_all = True
    n_tested = 0
    for tx, ty, tyaw, tpitch in cases:
        yaw = np.radians(tyaw)
        pitch = np.radians(tpitch)
        hgt = 0.67
        zc = np.array([np.cos(yaw) * np.cos(pitch),
                       np.sin(yaw) * np.cos(pitch),
                       -np.sin(pitch)])
        # OpenCV 相机系：x 向右、y 向下、z 沿光轴向前
        xc = np.cross(zc, np.array([0.0, 0.0, 1.0]))
        xc /= np.linalg.norm(xc)
        yc = np.cross(zc, xc)
        # 标准 world->cam 旋转：行 = 相机轴在世界系的方向
        R_wc = np.row_stack([xc, yc, zc])
        C = np.array([tx, ty, hgt])
        t = -R_wc @ C
        rvec, _ = cv2.Rodrigues(R_wc)
        proj, _ = cv2.projectPoints(obj, rvec, t, K, dist)
        proj = proj.reshape(-1, 2)
        in_img = [(p, i) for i, p in enumerate(proj)
                  if 0 <= p[0] < 1280 and 0 <= p[1] < 720]
        if len(in_img) < 4:
            print("  位姿 (%.1f, %.1f, %.0f°): 可见点 %d 个，跳过"
                  % (tx, ty, tyaw, len(in_img)))
            continue
        obs = np.array([p for p, _ in in_img])
        # 自检也带类型约束，和真实链路一致
        obs_types = [field_type(names[i]) for _, i in in_img]
        sols = solve_pose(obs, field_pts, K, dist, iters=1500,
                          obs_types=obs_types)
        n_tested += 1
        if not sols:
            print("  位姿 (%.1f, %.1f, %.0f°): ✗ 解算失败（可见点 %d）"
                  % (tx, ty, tyaw, len(in_img)))
            ok_all = False
            continue
        b = sols[0]
        ex = np.hypot(b["x"] - tx, b["y"] - ty)
        eyaw = (b["yaw"] - tyaw + 180) % 360 - 180
        flag = "✓" if (ex < 0.3 and abs(eyaw) < 5) else "✗"
        print("  位姿 (%.1f, %.1f, %.0f°): 解出 (%+.2f, %+.2f, %+.1f°)"
              " 误差 %.2fm/%.1f° 内点 %d  %s"
              % (tx, ty, tyaw, b["x"], b["y"], b["yaw"], ex, eyaw, b["n"],
                 flag))
        ok_all = ok_all and flag == "✓"
    if n_tested == 0:
        print("自检结果: ✗ 没有可测位姿（投影后无可见点，检查相机系构造）")
        return 1
    print("自检结果:", "通过（定位数学正确，误差来自检测）" if ok_all
          else "未通过（先用合成数据排查几何链路）")
    return 0 if ok_all else 1


def parse_truth(s):
    if not s:
        return None, None
    parts = [p for p in s.replace("，", ",").split(",") if p.strip()]
    vals = [float(p) for p in parts]
    if len(vals) == 2:
        return (vals[0], vals[1]), None
    if len(vals) == 3:
        return (vals[0], vals[1]), vals[2]
    raise ValueError("--truth 格式: x,y 或 x,y,yaw")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image")
    ap.add_argument("--video")
    ap.add_argument("--stride", type=int, default=45)
    ap.add_argument("--truth", help="真值 'x,y' 或 'x,y,yaw'")
    ap.add_argument("--field", default=r"templates/field.yaml")
    ap.add_argument("--intr", default=r"D:\captures\calib\intrinsics.yaml")
    ap.add_argument("--out-dir", default=r"D:\captures\_locate")
    ap.add_argument("--selftest", action="store_true",
                    help="合成数据自检定位数学链路")
    ap.add_argument("--detector", choices=["classic", "yolo"], default="classic",
                    help="标志检测后端：classic=直线求交，yolo=训练好的模型")
    ap.add_argument("--weights", default=r"D://captures//landmarks//weights//best_v1.pt",
                    help="YOLO 后端的权重路径")
    ap.add_argument("--conf", type=float, default=0.25, help="YOLO 置信度阈值")
    ap.add_argument("--enhance", choices=["none", "clahe"], default="none",
                    help="检测前图像增强。clahe：机位退远后球门白墙进画面，"
                         "自动曝光被拉爆、T 形标志淹没在过曝区时用 "
                         "--enhance clahe --conf 0.01（M8 实测检出 "
                         "2~3 → 5~7 个）")
    ap.add_argument("--imgsz", type=int, default=640, help="YOLO 推理尺寸")
    ap.add_argument("--refine", action="store_true",
                    help="任务书 2.3 几何细化：YOLO 框内提白线 mask，"
                         "估计真实交点像素替代 bbox 中心；不可靠时回退中心。")
    ap.add_argument("--refine-mode", choices=["ridge", "hough"], default="ridge",
                    help="细化算法：ridge=距离变换脊峰，hough=线段求交")
    ap.add_argument("--refine-shift", type=float, default=0.30,
                    help="细化点允许偏离 bbox 中心的最大框尺寸比例，越小越保守")
    ap.add_argument("--refine-blend", type=float, default=1.0,
                    help="细化点与 bbox 中心的混合权重，1.0=全用细化点，"
                         "0.3=中心占七成（远景小框更稳）")
    ap.add_argument("--type-penalty", type=float, default=None,
                    help="类型约束改为惩罚系数（默认硬约束）。检测器在远处"
                         "把 TCross 判成 L 时，硬约束会锁死几何正确的对应，"
                         "改用 8~12 的惩罚可容错。")
    args = ap.parse_args()

    base = os.path.dirname(os.path.abspath(__file__))
    field_path = args.field if os.path.isabs(args.field) \
        else os.path.join(base, args.field)
    field_pts = load_field(field_path)
    K, dist, _ = load_intrinsics(args.intr)
    truth, truth_yaw = parse_truth(args.truth)
    os.makedirs(args.out_dir, exist_ok=True)
    print("场地标志 %d 个: %s" % (len(field_pts), ", ".join(field_pts)))

    if args.selftest:
        return selftest(field_pts, K, dist)

    def run_detect(bgr):
        if args.enhance == "clahe":
            lab = cv2.cvtColor(bgr, cv2.COLOR_BGR2LAB)
            l, a, b = cv2.split(lab)
            l = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(l)
            bgr = cv2.cvtColor(cv2.merge([l, a, b]), cv2.COLOR_LAB2BGR)
        if args.detector == "yolo":
            if args.refine:
                import refine_landmarks
                return refine_landmarks.detect_yolo_refined(
                    bgr, args.weights, args.conf, args.imgsz, refine=True,
                    mode=args.refine_mode, max_shift=args.refine_shift,
                    blend=args.refine_blend)
            return detect_yolo(bgr, args.weights, args.conf, args.imgsz)
        return detect(bgr)

    if args.image:
        bgr = cv2.imread(args.image)
        pts = run_detect(bgr)
        print("检测到 %d 个交点: %s"
              % (len(pts), " ".join("%s@(%d,%d)" % (t, x, y) for x, y, t in pts)))
        sols = solve_pose(np.array([(x, y) for x, y, _ in pts], float),
                          field_pts, K, dist,
                          obs_types=[t for _, _, t in pts],
                          type_penalty=args.type_penalty)
        report(args.image, sols, truth, truth_yaw)
    elif args.video:
        cap = cv2.VideoCapture(args.video)
        k, n_ok, n_all = 0, 0, 0
        ests = []                             # 每帧成功解 (x, y, yaw, n, rms)
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if k % args.stride == 0:
                pts = run_detect(frame)
                sols = solve_pose(np.array([(x, y) for x, y, _ in pts],
                                           float), field_pts, K, dist,
                                  obs_types=[t for _, _, t in pts],
                                  type_penalty=args.type_penalty) \
                    if len(pts) >= 4 else []
                report("%s 第 %d 帧（交点 %d 个）"
                       % (os.path.basename(args.video), k, len(pts)),
                       sols, truth, truth_yaw)
                n_all += 1
                if sols and sols[0]["n"] >= 4:
                    n_ok += 1
                    b = sols[0]
                    ests.append((b["x"], b["y"], b["yaw"], b["n"], b["rms"]))
            k += 1
        cap.release()
        print("=" * 66)
        print("合计 %d 个采样帧，成功定位 %d 帧（%.0f%%）"
              % (n_all, n_ok, 100.0 * n_ok / max(1, n_all)))
        # 跨帧聚合：位置 0.8m 内的解归为一簇，取最大簇的中位数
        if ests:
            clusters = []
            for e in ests:
                hit = None
                for c in clusters:
                    if (abs(e[0] - c[0][0]) < 0.8
                            and abs(e[1] - c[0][1]) < 0.8):
                        hit = c
                        break
                if hit is None:
                    clusters.append([e])
                else:
                    hit.append(e)
            clusters.sort(key=len, reverse=True)
            best_c = clusters[0]
            xs = sorted(e[0] for e in best_c)
            ys = sorted(e[1] for e in best_c)
            yaws = sorted(e[2] for e in best_c)
            m = len(best_c) // 2
            mx, my, myaw = xs[m], ys[m], yaws[m]
            print("跨帧聚合: %d/%d 帧聚成主簇，整段估计 x=%+.3f y=%+.3f yaw=%+.1f°"
                  % (len(best_c), len(ests), mx, my, myaw))
            if truth is not None:
                err = float(np.hypot(mx - truth[0], my - truth[1]))
                print("整段位置误差 = %.3f m %s"
                      % (err, "✓ 达标(≤0.5m)" if err <= 0.5 else "✗ 超差"))
                if truth_yaw is not None:
                    dyaw = (myaw - truth_yaw + 180) % 360 - 180
                    print("整段航向误差 = %+.1f° %s"
                          % (dyaw, "✓ 达标(≤10°)" if abs(dyaw) <= 10
                             else "✗ 超差"))
    else:
        print("需要 --image 或 --video")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
