# -*- coding: utf-8 -*-
"""外参稳健求解：网格搜索给初值 + 高斯-牛顿细化 + 多帧投票。

为什么不直接用 solve_extrinsics_auto.py：
    那个工具靠「同类型一一映射」穷举对应关系，地图点一多就撞 MAX_COMBOS 上限
    （现场图有 10 个 T、12 个 L，组合数几十万级），于是绝大多数帧都被判成
    「无可信解」——不是数据不好，是枚举没走完。

这里的做法：
    1) 高度用卷尺实测值固定（相机离地 h，已知），未知数只剩 x、y、yaw、pitch；
    2) 在 (x, y, yaw, pitch) 上做粗网格，把地图上全部标志投影下去，按「类型一致 +
       像素距离」贪心匹配，用匹配质量打分；
    3) 取分数最高的若干个做高斯-牛顿细化（重投影残差最小二乘），再重新匹配、再细化；
    4) 多帧结果按位置/俯角聚类投票，票数最多的那一簇就是答案。

    因为相机离地高度已知，尺度是钉死的 —— 解出来若与地图不一致，残差会立刻变大，
    这本身就是对「现场地图」的一次独立校验。

用法（项目根目录）：
    run.bat tools\\solve_extrinsics_grid.py --video "data/extr/ext_site.mp4/clip_20261120_222251.mp4"
    run.bat tools\\solve_extrinsics_grid.py --video a.mp4 b.mp4 --frames 8 --height 1.375

输出：
    outputs/extrinsics_grid.md      投票过程与最终外参
    outputs/extrinsics_grid.png     最优帧的场地图叠影（肉眼终检）
    calibration/camera_extrinsics.grid.yaml   （加 --write 才写，不覆盖正式文件）
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import save_yaml                                  # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                     # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics                    # noqa: E402
from tools.realtime_field import (                                          # noqa: E402
    build_field_map, dedup_dets, refine_junction, white_mask,
)

TYPE_SET = ("LCross", "TCross", "XCross", "PenaltyPoint")


def rot_mat(yaw_deg: float, pitch_deg: float, roll_deg: float = 0.0) -> np.ndarray:
    """与 Extrinsics.from_params 同一套约定：光轴朝 +X，pitch 向下为正。"""
    r0 = np.array([[0.0, 0.0, 1.0],
                   [-1.0, 0.0, 0.0],
                   [0.0, -1.0, 0.0]], float)
    cy, sy = math.cos(math.radians(yaw_deg)), math.sin(math.radians(yaw_deg))
    cp, sp = math.cos(math.radians(pitch_deg)), math.sin(math.radians(pitch_deg))
    cr, sr = math.cos(math.radians(roll_deg)), math.sin(math.radians(roll_deg))
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    return Rz @ Ry @ Rx @ r0


def project_np(params, P, K, height):
    """params = (x, y, yaw, pitch) → 像素 (N,2) 与可见掩码。不含畸变（细化解码时用 cam.project）。"""
    x0, y0, yaw, pitch = params
    R = rot_mat(yaw, pitch)
    t = np.array([x0, y0, height], float)
    p_cam = (P - t) @ R
    z = p_cam[:, 2]
    ok = z > 0.05
    zz = np.where(ok, z, 1.0)
    u = K[0, 0] * p_cam[:, 0] / zz + K[0, 2]
    v = K[1, 1] * p_cam[:, 1] / zz + K[1, 2]
    return np.stack([u, v], 1), ok


def match_score(uv, ok, dets, tol_px, map_types, type_free: bool = False):
    """贪心匹配：类型一致且距离最近的点对；返回 (加权得分, 匹配对列表)。

    权重取检测框尺寸的平方：近处的标志框大、交点定位准，远处的框只有几十像素、
    中心偏差能有十几像素 —— 不加权的话远处点会靠数量把近处点淹没。

    type_free=True 时完全不看类别：低机位掠射视角下模型判不出 T/L/X（同一交点会同时
    被判成三类），这时只能靠「几何位置」匹配，类别交给后面的白线对齐去纠正。
    """
    if not dets:
        return 0.0, []
    du = np.array([d["u"] for d in dets])
    dv = np.array([d["v"] for d in dets])
    dists = np.hypot(uv[:, None, 0] - du[None, :], uv[:, None, 1] - dv[None, :])
    if type_free:
        type_ok = np.ones(dists.shape, bool)
    else:
        type_ok = np.array(map_types)[:, None] == np.array([d["type"] for d in dets])[None, :]
    valid = ok[:, None] & type_ok & (dists <= tol_px)
    if not valid.any():
        return 0.0, []
    wts = np.array([max(d.get("w", 40.0), 20.0) / 100.0 for d in dets]) ** 2
    score, pairs, used = 0.0, [], set()
    for i in range(len(dets)):
        col = dists[:, i].copy()
        col[~valid[:, i]] = np.inf
        for j in used:
            col[j] = np.inf
        j = int(np.argmin(col))
        if not np.isfinite(col[j]):
            continue
        used.add(j)
        pairs.append((j, i, float(col[j])))
        score += wts[i] * (1.0 - float(col[j]) / tol_px)
    return score, pairs


def residuals(params, P, dets, cam, height, pairs):
    """用真实相机模型（含畸变）算重投影残差；按检测框尺寸加权。"""
    x0, y0, yaw, pitch = params
    R = rot_mat(yaw, pitch)
    t = np.array([x0, y0, height], float)
    idx = [j for j, _, _ in pairs]
    p_cam = (P[idx] - t) @ R
    if np.any(p_cam[:, 2] <= 0.05):
        return None
    uv, ok = cam.project(p_cam)
    if not ok.all():
        return None
    obs = np.array([[dets[i]["u"], dets[i]["v"]] for _, i, _ in pairs])
    w = np.array([max(dets[i].get("w", 40.0), 20.0) / 100.0 for _, i, _ in pairs])
    w = np.repeat(w, 2)
    return ((uv - obs).ravel()) * w


def refine(params, P, dets, cam, height, pairs, iters=30):
    p = np.asarray(params, float).copy()
    r = residuals(p, P, dets, cam, height, pairs)
    if r is None or r.size == 0:
        return None
    cost = float(r @ r)
    for _ in range(iters):
        J = np.zeros((r.size, 4))
        for i in range(4):
            dp = p.copy()
            step = 1e-3 if i < 2 else 1e-3
            dp[i] += step
            r2 = residuals(dp, P, dets, cam, height, pairs)
            if r2 is None:
                return None
            J[:, i] = (r2 - r) / step
        try:
            delta = np.linalg.lstsq(J.T @ J + 1e-3 * np.eye(4), -J.T @ r, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        moved = False
        for k in range(6):
            pn = p + delta * (0.5 ** k)
            rn = residuals(pn, P, dets, cam, height, pairs)
            if rn is None:
                continue
            cn = float(rn @ rn)
            if cn < cost:
                p, r, cost, moved = pn, rn, cn, True
                break
        if not moved or np.linalg.norm(delta) < 1e-7:
            break
    return p, math.sqrt(cost / max(r.size / 2.0, 1.0))


def detect_frame(model, frame, conf, device):
    r = model.predict(frame, conf=conf, imgsz=640, verbose=False, device=device)[0]
    dets = []
    for b in r.boxes:
        typ = r.names[int(b.cls[0])]
        if typ not in TYPE_SET:
            continue
        box = [float(v) for v in b.xyxy[0]]
        u, v, mode, nl = refine_junction(frame, box)
        dets.append({"type": typ, "u": float(u), "v": float(v),
                     "conf": float(b.conf[0]), "mode": mode,
                     "w": float(max(box[2] - box[0], box[3] - box[1]))})
    # 低阈值下同一交点会被判成 T/L/X 多个框，位置几乎重合 —— 先按像素距离合并
    return dedup_dets(dets, radius=35.0)


def solve_frame(dets, P, types, cam, height, args):
    if len(dets) < 3:
        return []
    out = []
    yaws = list(np.arange(-12, 12.1, 3.0)) + list(np.arange(168, 192.1, 6.0))
    # 俯角下限放到 3°：相机降到 0.73 m 后，要看到远端球门线必须把视线放平，
    # 实际俯角很可能落在 6~12° 这一带，原来的 7° 起点会漏掉。
    pitches = list(np.arange(3, 34, 2.5))
    for yaw in yaws:
        for pitch in pitches:
            R = rot_mat(yaw, pitch)
            for x0 in np.arange(-8.5, 0.1, 0.4):
                for y0 in np.arange(-2.4, 2.41, 0.4):
                    p = (float(x0), float(y0), float(yaw), float(pitch))
                    uv, ok = project_np(p, P, cam.K, height)
                    s, pairs = match_score(uv, ok, dets, args.tol, types, args.type_free)
                    if len(pairs) >= args.min_markers:
                        out.append((s, p))
    out.sort(key=lambda x: -x[0])
    refined, seen = [], []
    for s, p in out[:args.top]:
        cur = p
        for _ in range(4):
            uv, ok = project_np(cur, P, cam.K, height)
            _, pairs = match_score(uv, ok, dets, args.tol, types, args.type_free)
            res = refine(cur, P, dets, cam, height, pairs)
            if res is None:
                break
            cur = res[0]
        uv, ok = project_np(cur, P, cam.K, height)
        _, pairs = match_score(uv, ok, dets, args.tol, types, args.type_free)
        res = refine(cur, P, dets, cam, height, pairs)
        if res is None:
            continue
        p2, rms = res
        if any(abs(p2[0] - q[0]) < 0.15 and abs(p2[1] - q[1]) < 0.15
               and abs(p2[2] - q[2]) < 2.0 and abs(p2[3] - q[3]) < 1.5 for q in seen):
            continue
        seen.append(p2)
        refined.append((len(pairs), rms, p2))
    refined.sort(key=lambda x: (-x[0], x[1]))
    return refined[:3]


def describe(p, height):
    R = rot_mat(p[2], p[3])
    optic = R @ np.array([0.0, 0.0, 1.0])
    return dict(x=float(p[0]), y=float(p[1]), yaw_deg=float(p[2]),
                pitch_deg=float(p[3]), height_m=height,
                yaw_axis_deg=math.degrees(math.atan2(optic[1], optic[0])),
                pitch_axis_deg=math.degrees(math.atan2(-optic[2],
                                                       math.hypot(optic[0], optic[1]))),
                R=R, t=np.array([p[0], p[1], height]))


def overlay(frame, cam, ext, fm, out_png, dets=None):
    vis = frame.copy()

    def g2p(pts):
        p_cam = ext.base_to_cam(np.asarray(pts, float).reshape(-1, 3))
        uv, ok = cam.project(p_cam)
        ok &= p_cam[:, 2] > 0.05
        return uv, ok

    def seg(a, b, color, th=2):
        uv, ok = g2p([(a[0], a[1], 0.0), (b[0], b[1], 0.0)])
        if ok.all():
            cv2.line(vis, tuple(int(v) for v in uv[0]), tuple(int(v) for v in uv[1]),
                     color, th, cv2.LINE_AA)

    g = fm.geometry
    hl, hw = g["field_length_m"] / 2.0, g["field_width_m"] / 2.0
    pa_w, pa_d = g["penalty_area_width_m"] / 2.0, g["penalty_area_length_m"]
    ga_w, ga_d = g["goal_area_width_m"] / 2.0, g["goal_area_length_m"]
    r = g["circle_radius_m"]
    blue, red, green, orange = (255, 160, 0), (0, 0, 255), (0, 220, 0), (0, 170, 255)
    for a, b in (((-hl, -hw), (hl, -hw)), ((-hl, hw), (hl, hw)),
                 ((-hl, -hw), (-hl, hw)), ((hl, -hw), (hl, hw)), ((0, -hw), (0, hw))):
        seg(a, b, blue)
    for s in (+1, -1):
        for a, b in (((s * hl, -pa_w), (s * hl, pa_w)),
                     ((s * (hl - pa_d), pa_w), (s * hl, pa_w)),
                     ((s * (hl - pa_d), -pa_w), (s * (hl - pa_d), pa_w)),
                     ((s * (hl - pa_d), -pa_w), (s * hl, -pa_w)),
                     ((s * (hl - ga_d), -ga_w), (s * (hl - ga_d), ga_w)),
                     ((s * (hl - ga_d), ga_w), (s * hl, ga_w)),
                     ((s * (hl - ga_d), -ga_w), (s * hl, -ga_w))):
            seg(a, b, red)
    th = np.linspace(0, 2 * np.pi, 160)
    uv, ok = g2p([(r * math.cos(t), r * math.sin(t), 0.0) for t in th])
    for i in range(len(uv) - 1):
        if ok[i] and ok[i + 1]:
            cv2.line(vis, tuple(int(v) for v in uv[i]), tuple(int(v) for v in uv[i + 1]),
                     green, 2, cv2.LINE_AA)
    for m in fm.markers:
        uv, ok = g2p([(m.x, m.y, 0.0)])
        if ok[0]:
            u, v = int(uv[0][0]), int(uv[0][1])
            if 0 <= u < vis.shape[1] and 0 <= v < vis.shape[0]:
                cv2.drawMarker(vis, (u, v), orange, cv2.MARKER_CROSS, 14, 2)
    if dets:
        for d in dets:
            cv2.circle(vis, (int(d["u"]), int(d["v"])), 14, (255, 0, 255), 2)
            cv2.putText(vis, d["type"][0], (int(d["u"]) + 16, int(d["v"]) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
    cv2.imwrite(str(out_png), vis)
    return out_png


def field_polyline(fm, n_arc=72):
    """把场地线画成密集采样点（世界坐标），供「与白线对齐」的细化使用。"""
    g = fm.geometry
    hl, hw = g["field_length_m"] / 2.0, g["field_width_m"] / 2.0
    pa_w, pa_d = g["penalty_area_width_m"] / 2.0, g["penalty_area_length_m"]
    ga_w, ga_d = g["goal_area_width_m"] / 2.0, g["goal_area_length_m"]
    r = g["circle_radius_m"]

    def seg(a, b, n=24):
        return [(a[0] + (b[0] - a[0]) * t / n, a[1] + (b[1] - a[1]) * t / n)
                for t in range(n + 1)]

    pts = []
    pts += seg((-hl, -hw), (hl, -hw)); pts += seg((-hl, hw), (hl, hw))
    pts += seg((-hl, -hw), (-hl, hw)); pts += seg((hl, -hw), (hl, hw))
    pts += seg((0, -hw), (0, hw))
    pts += [(r * math.cos(2 * math.pi * i / n_arc), r * math.sin(2 * math.pi * i / n_arc))
            for i in range(n_arc)]
    for s in (+1, -1):
        pts += seg((s * hl, -pa_w), (s * hl, pa_w))
        pts += seg((s * (hl - pa_d), -pa_w), (s * (hl - pa_d), pa_w))
        pts += seg((s * (hl - pa_d), -pa_w), (s * hl, -pa_w))
        pts += seg((s * (hl - pa_d), pa_w), (s * hl, pa_w))
        pts += seg((s * (hl - ga_d), -ga_w), (s * (hl - ga_d), ga_w))
        pts += seg((s * (hl - ga_d), -ga_w), (s * hl, -ga_w))
        pts += seg((s * (hl - ga_d), ga_w), (s * hl, ga_w))
    return np.array([(x, y, 0.0) for x, y in pts], float)


def line_cost(params, Pline, cam, height, dt, v_min):
    x0, y0, yaw, pitch = params
    R = rot_mat(yaw, pitch)
    t = np.array([x0, y0, height], float)
    p_cam = (Pline - t) @ R
    ok = p_cam[:, 2] > 0.1
    if ok.sum() < 40:
        return 1e9
    uv, okp = cam.project(p_cam)
    ok &= okp
    u = uv[ok, 0]; v = uv[ok, 1]
    inside = (u >= 0) & (u < dt.shape[1] - 1) & (v >= v_min) & (v < dt.shape[0] - 1)
    if inside.sum() < 40:
        return 1e9
    d = dt[v[inside].astype(int), u[inside].astype(int)]
    return float(np.minimum(d, 25.0).mean())


def refine_lines(params, Pline, cam, height, dt, v_min):
    """模式搜索：让投影出来的场地线尽量落在画好的白线上（距离变换查表）。"""
    p = np.asarray(params, float).copy()
    best = line_cost(p, Pline, cam, height, dt, v_min)
    steps = np.array([0.12, 0.12, 1.2, 0.6])
    for _ in range(60):
        improved = False
        for i in range(4):
            for sgn in (+1, -1):
                q = p.copy()
                q[i] += sgn * steps[i]
                c = line_cost(q, Pline, cam, height, dt, v_min)
                if c < best - 1e-4:
                    p, best, improved = q, c, True
        if not improved:
            steps *= 0.5
            if steps[0] < 0.002:
                break
    return p, best


def main() -> int:
    ap = argparse.ArgumentParser(description="网格初值 + GN 细化的外参求解")
    ap.add_argument("--video", nargs="+", required=True)
    ap.add_argument("--frames", type=int, default=8, help="每段取几帧")
    ap.add_argument("--height", type=float, default=None, help="镜头离地高度（默认读 field_site.yaml）")
    ap.add_argument("--weights", default="weights/best_marker_v9.pt")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--site", default="config/field_site.yaml")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--device", default="0")
    ap.add_argument("--tol", type=float, default=28.0, help="匹配容差（像素）")
    ap.add_argument("--min-markers", type=int, default=4)
    ap.add_argument("--type-free", action="store_true",
                    help="不看类别只按位置匹配（低机位下模型判不出 T/L/X 时用）")
    ap.add_argument("--top", type=int, default=40, help="每个粗解集里细化前多少个")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--out-file", default=str(CALIB_DIR / "camera_extrinsics.grid.yaml"),
                    help="--write 时写到哪里（默认 grid 文件，不碰正式外参）")
    args = ap.parse_args()

    cam = CameraModel.from_yaml(ROOT / args.intrinsics)
    fm = build_field_map(ROOT / args.field, ROOT / args.geometry, mirror=True,
                         site_path=ROOT / args.site)
    from src.common.config_io import load_yaml
    _sg = (load_yaml(ROOT / args.site).get("field_site_overrides") or {}).get("site_geometry") or {}
    height = float(args.height or _sg.get("camera_height_m") or 1.375)
    P = np.array([[m.x, m.y, 0.0] for m in fm.markers], float)
    types = [m.type for m in fm.markers]
    print(f"地图点 {len(P)}（L={fm.geometry['field_length_m']} W={fm.geometry['field_width_m']}）"
          f"｜高度 {height:.3f} m｜匹配容差 {args.tol:.0f} px")

    from ultralytics import YOLO
    model = YOLO(str(ROOT / args.weights))

    votes = []
    best_frame = None
    for vpath in args.video:
        vp = Path(vpath) if Path(vpath).is_absolute() else ROOT / vpath
        cap = cv2.VideoCapture(str(vp))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
        idxs = [int(round(i * (n - 1) / max(args.frames - 1, 1))) for i in range(args.frames)]
        print(f"\n[{rel(vp)}] {n} 帧，取 {len(idxs)} 帧")
        for fi in idxs:
            cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
            ok, frame = cap.read()
            if not ok:
                continue
            dets = detect_frame(model, frame, args.conf, args.device)
            sols = solve_frame(dets, P, types, cam, height, args)
            if not sols:
                print(f"  f{fi:04d}: 检测 {len(dets):2d} → 无解")
                continue
            nin, rms, p = sols[0]
            d = describe(p, height)
            print(f"  f{fi:04d}: 检测 {len(dets):2d} → 内点 {nin} rms={rms:.2f}px "
                  f"pos=({d['x']:+.2f},{d['y']:+.2f}) yaw={d['yaw_axis_deg']:+.1f}° "
                  f"pitch={d['pitch_axis_deg']:.1f}°")
            votes.append((nin, rms, p, vp, fi, frame, dets))
        cap.release()

    if not votes:
        print("\n没有任何可行解。检查：镜头是否朝场地、是否至少 4 个标志、高度是否填对。")
        return 1

    # 聚类投票：位置 0.3 m、俯角 2° 内算同一簇
    clusters = []
    for v in votes:
        nin, rms, p = v[0], v[1], v[2]
        placed = False
        for c in clusters:
            q = c[0][2]
            if (abs(p[0] - q[0]) < 0.3 and abs(p[1] - q[1]) < 0.3
                    and abs(p[2] - q[2]) < 3.0 and abs(p[3] - q[3]) < 2.0):
                c.append(v)
                placed = True
                break
        if not placed:
            clusters.append([v])
    clusters.sort(key=lambda c: (-len(c), sum(x[1] for x in c) / len(c)))
    best = clusters[0]
    best.sort(key=lambda x: (-x[0], x[1]))
    nin, rms, p, vp, fi, frame, dets = best[0]
    d = describe(p, height)
    print("\n" + "=" * 74)
    print(f"投票结果：{len(best)} 票一致（总帧数 {len(votes)}），簇 {len(clusters)} 个")
    print(f"外参  : x={d['x']:+.3f} m  y={d['y']:+.3f} m  h={height:.3f} m（卷尺给定）")
    print(f"姿态  : yaw={d['yaw_axis_deg']:+.2f}°  pitch={d['pitch_axis_deg']:.2f}°")
    print(f"残差  : rms={rms:.2f} px，内点 {nin} 个")
    print("=" * 74)

    ext = Extrinsics(d["R"], d["t"])
    out_png = OUTPUT_DIR / "extrinsics_grid.png"

    # ── 用「投影线与画好的白线」逐帧再细化：白线有上千像素，比几个标志点稳得多 ──
    Pline = field_polyline(fm)
    refined_rows = []
    for nin, rms_i, p_i, vp_i, fi_i, fr_i, dets_i in votes:
        mask = white_mask(fr_i)
        H, W = mask.shape[:2]
        mask[0:int(0.22 * H), int(0.22 * W):int(0.82 * W)] = 0     # 球门网也是白的，剔掉
        dt_i = cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2, 3)
        p0 = np.array([p_i[0], p_i[1], p_i[2], p_i[3]], float)
        c0 = line_cost(p0, Pline, cam, height, dt_i, 0.20 * H)
        pr, c1 = refine_lines(p0, Pline, cam, height, dt_i, 0.20 * H)
        refined_rows.append((fi_i, c0, c1, pr))
        print(f"  白线细化 f{fi_i:04d}: {c0:5.2f} → {c1:4.2f} px  "
              f"x={pr[0]:+.3f} y={pr[1]:+.3f} yaw={pr[2]:+.2f}° pitch={pr[3]:.2f}°")

    arr = np.array([r[3] for r in refined_rows])
    med = np.median(arr, axis=0)
    spread = np.std(arr, axis=0)
    cost_ref = float(np.mean([r[2] for r in refined_rows]))
    d_ref = describe(med, height)
    print(f"\n逐帧一致度：x±{spread[0]*100:.1f} cm  y±{spread[1]*100:.1f} cm  "
          f"yaw±{spread[2]:.2f}°  pitch±{spread[3]:.2f}°")
    print(f"白线对齐（各帧均值）：{cost_ref:.2f} px")
    print(f"最终外参：x={d_ref['x']:+.3f} m  y={d_ref['y']:+.3f} m  h={height:.3f} m")
    print(f"          yaw={d_ref['yaw_axis_deg']:+.2f}°  pitch={d_ref['pitch_axis_deg']:.2f}°")

    # 标志点残差（用细化后的位姿重新算一遍，供报告）
    nin, rms_mark, p_best, vp_b, fi_b, frame_b, dets_b = best[0]
    uv_all, ok_all = project_np(med, P, cam.K, height)
    _, pairs_b = match_score(uv_all, ok_all, dets_b, args.tol, types, args.type_free)
    res_mark = residuals(med, P, dets_b, cam, height, pairs_b)
    rms_after = float(np.sqrt((res_mark ** 2).mean() / 2.0)) if res_mark is not None else float("nan")
    print(f"标志点重投影（细化后）：{len(pairs_b)} 个内点，rms={rms_after:.2f} px")

    d = d_ref
    ext = Extrinsics(d["R"], d["t"])
    overlay(frame_b, cam, ext, fm, out_png, dets_b)
    md = OUTPUT_DIR / "extrinsics_grid.md"
    with md.open("w", encoding="utf-8") as f:
        f.write("# 外参求解（网格初值 + GN 细化 + 多帧投票）\n\n")
        f.write(f"- 视频：`{rel(vp)}` 第 {fi} 帧\n")
        f.write(f"- 相机高度（卷尺）：{height:.3f} m\n")
        f.write(f"- 投票：{len(best)}/{len(votes)} 帧一致，共 {len(clusters)} 个簇\n")
        f.write(f"- 重投影 RMS：{rms:.2f} px（内点 {nin} 个）\n\n")
        f.write("| 量 | 值 |\n|---|---:|\n")
        f.write(f"| x (m) | {d['x']:+.3f} |\n| y (m) | {d['y']:+.3f} |\n")
        f.write(f"| h (m) | {height:.3f}（卷尺） |\n")
        f.write(f"| yaw (°) | {d['yaw_axis_deg']:+.2f} |\n")
        f.write(f"| pitch (°) | {d['pitch_axis_deg']:.2f} |\n\n")
        f.write(f"叠图：`{rel(out_png)}`\n")
    print(f"输出: {rel(md)} / {rel(out_png)}")

    if args.write:
        out_yaml = Path(args.out_file)
        if not out_yaml.is_absolute():
            out_yaml = ROOT / out_yaml
        save_yaml(out_yaml, {"camera_extrinsics": {
            "R": d["R"].tolist(), "t": d["t"].tolist(),
            "camera_height_m": round(height, 4),
            "pitch_deg": round(d["pitch_axis_deg"], 3),
            "yaw_deg": round(d["yaw_axis_deg"], 3),
            "roll_deg": 0.0,
            "demo": False,
            "method": "网格初值+GN细化（标志点加权）+白线距离对齐；高度用卷尺固定 %.3f m" % height,
            "residual_px": round(rms_after, 3),
            "line_align_px": round(cost_ref, 3),
            "frames_agree": len(best),
            "frames_total": len(votes),
            "spread_cm": [round(float(spread[0]) * 100, 2), round(float(spread[1]) * 100, 2)],
            "spread_deg": [round(float(spread[2]), 2), round(float(spread[3]), 2)],
            "source_video": rel(vp_b),
            "source_frame": fi_b,
            "note": "地图按 config/field_site.yaml 的现场实测尺寸重建；官方 field.yaml 未改动",
        }})
        print(f"输出: {rel(out_yaml)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
