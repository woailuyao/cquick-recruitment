# -*- coding: utf-8 -*-
"""用「少量对应点 + 卷尺高度」反解相机外参（3 个点就够）。

为什么需要它：
    tools/estimate_extrinsics.py 走的是平面位姿，至少要 4 个标志；而现场能稳稳检测到的
    常常只有 3 个（例如中圈与中线的两个交点 + 一个球门角）。但卷尺已经量到镜头离地
    1.375 m，高度是已知量 —— 于是未知数少一个，3 个点（6 个方程）足以解 4 个未知数
    （前进 x、侧移 y、朝向 yaw、俯角 pitch；roll 假定 0，画面中线看着是水平的）。

    对应关系可能含糊（同一个像素既可能是 X1 也可能是 X2；那个 L 可能在四个角上的任意
    一个），所以每个像素可以给多个候选地图点，本脚本枚举笛卡尔积、逐个做高斯-牛顿，
    再用「重投影残差 + 物理约束」排序，把最好的几个打出来，并把最优解叠到图上供肉眼终检。

输入（YAML）：
    image: data/extr/ext_04.png
    height_m: 1.375
    points:
      - {u: 212, v: 295, cand: [[0.0, -0.75], [0.0, 0.75]], note: 中圈∩中线（左）}
      - {u: 881, v: 308, cand: [[0.0,  0.75], [0.0, -0.75]], note: 中圈∩中线（右）}
      - {u: 1079, v: 75, cand: [[3.80, -3.015], [3.80, 3.015]], note: 球门端角点}

输出：
    outputs/extrinsics_fixed_h.md        求解过程与候选排名
    outputs/extrinsics_fixed_h.png       最优解叠到原图上（场地线 + 圆 + 全部标志）
    calibration/camera_extrinsics.candidate.yaml   候选外参（不自动替换正式文件）

用法（项目根目录）：
    run.bat tools\\fit_extrinsics_fixed_height.py --corr .tmp\\ext04_corrs.yaml
    run.bat tools\\fit_extrinsics_fixed_height.py --corr ... --write   # 同时写 candidate.yaml
"""

from __future__ import annotations

import argparse
import itertools
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml, save_yaml                       # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                    # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics                  # noqa: E402
from tools.realtime_field import build_field_map                           # noqa: E402

MAX_PITCH, MIN_PITCH = 70.0, 2.0
MAX_DIST_XY = 8.0


def pose_to_ext(p, height: float) -> Extrinsics:
    """参数 (x, y, yaw_deg, pitch_deg) → 外参；roll 固定 0。"""
    return Extrinsics.from_params(height=height, pitch_deg=p[3], roll_deg=0.0,
                                  yaw_offset_deg=p[2], forward_m=p[0], lateral_m=p[1])


def project(p, pts3, cam, height):
    ext = pose_to_ext(p, height)
    p_cam = ext.base_to_cam(np.asarray(pts3, float).reshape(-1, 3))
    uv, ok = cam.project(p_cam)
    ok &= p_cam[:, 2] > 0.05
    return uv, ok


def residuals(p, pts3, uvs, cam, height):
    uv, ok = project(p, pts3, cam, height)
    if not ok.all():
        return None
    return (uv - np.asarray(uvs, float)).ravel()


def gauss_newton(p0, pts3, uvs, cam, height, iters=40, lam=1e-3):
    p = np.asarray(p0, float).copy()
    r = residuals(p, pts3, uvs, cam, height)
    if r is None:
        return None
    cost = float(r @ r)
    for _ in range(iters):
        J = np.zeros((r.size, p.size))
        for i in range(p.size):
            dp = p.copy()
            step = 1e-4 if i >= 2 else 1e-3
            dp[i] += step
            r2 = residuals(dp, pts3, uvs, cam, height)
            if r2 is None:
                return None
            J[:, i] = (r2 - r) / step
        try:
            delta = np.linalg.lstsq(J.T @ J + lam * np.eye(p.size), -J.T @ r, rcond=None)[0]
        except np.linalg.LinAlgError:
            return None
        # 阻尼线搜索
        improved = False
        for k in range(6):
            pn = p + delta * (0.5 ** k)
            rn = residuals(pn, pts3, uvs, cam, height)
            if rn is None:
                continue
            cn = float(rn @ rn)
            if cn < cost:
                p, r, cost, improved = pn, rn, cn, True
                break
        if not improved:
            break
        if np.linalg.norm(delta) < 1e-6:
            break
    return p, math.sqrt(cost / max(len(r), 1))


def describe(p, height):
    ext = pose_to_ext(p, height)
    optic = ext.R @ np.array([0.0, 0.0, 1.0])
    yaw = math.degrees(math.atan2(optic[1], optic[0]))
    pitch = math.degrees(math.atan2(-optic[2], math.hypot(optic[0], optic[1])))
    roll = math.degrees(math.atan2(ext.R[2, 0], ext.R[1, 0]))
    return dict(x=float(p[0]), y=float(p[1]), yaw_deg=yaw, pitch_deg=pitch,
                roll_deg=roll, height_m=height, R=ext.R, t=ext.t)


def draw_overlay(img, cam, ext, fm, out_path):
    vis = img.copy()
    green, blue, red, orange = (0, 220, 0), (255, 160, 0), (0, 0, 255), (0, 170, 255)

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

    for a, b in (((-hl, -hw), (hl, -hw)), ((-hl, hw), (hl, hw)),
                 ((-hl, -hw), (-hl, hw)), ((hl, -hw), (hl, hw)), ((0, -hw), (0, hw))):
        seg(a, b, blue)
    for s in (+1, -1):
        for a, b in (((s * hl, -pa_w), (s * hl, pa_w)),
                     (((s * (hl - pa_d)), pa_w), (s * hl, pa_w)),
                     ((s * (hl - pa_d), -pa_w), (s * (hl - pa_d), pa_w)),
                     ((s * (hl - pa_d), -pa_w), (s * hl, -pa_w)),
                     ((s * (hl - ga_d), -ga_w), (s * (hl - ga_d), ga_w)),
                     ((s * (hl - ga_d), ga_w), (s * hl, ga_w)),
                     ((s * (hl - ga_d), -ga_w), (s * hl, -ga_w))):
            seg(a, b, red)
    th = np.linspace(0, 2 * np.pi, 120)
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
                cv2.drawMarker(vis, (u, v), orange, cv2.MARKER_CROSS, 12, 2)
    cv2.imwrite(str(out_path), vis)
    return out_path


def main() -> int:
    ap = argparse.ArgumentParser(description="固定高度反解外参（3 点即可）")
    ap.add_argument("--corr", required=True, help="对应点 YAML")
    ap.add_argument("--height", type=float, default=None, help="镜头离地高度（覆盖 YAML）")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--site", default="config/field_site.yaml")
    ap.add_argument("--top", type=int, default=5, help="打印前几个候选")
    ap.add_argument("--write", action="store_true", help="写 calibration/camera_extrinsics.candidate.yaml")
    args = ap.parse_args()

    data = load_yaml(Path(args.corr) if Path(args.corr).is_absolute() else ROOT / args.corr)
    height = float(args.height or data.get("height_m") or 0.0)
    if height <= 0:
        print("缺少 height_m（镜头离地高度）")
        return 2
    img_path = ROOT / data["image"] if not Path(data["image"]).is_absolute() else Path(data["image"])
    points = data["points"]
    cam = CameraModel.from_yaml(ROOT / args.intrinsics)
    fm = build_field_map(ROOT / args.field, ROOT / args.geometry, mirror=True,
                         site_path=ROOT / args.site)

    uvs = [[float(p["u"]), float(p["v"])] for p in points]
    cand_lists = [p["cand"] for p in points]
    print(f"地图点 {len(fm.markers)}（现场尺寸 L={fm.geometry['field_length_m']} "
          f"W={fm.geometry['field_width_m']}）｜高度 {height:.3f} m｜观测 {len(points)} 个")

    starts = []
    for x0 in np.linspace(-5.5, 1.0, 7):
        for y0 in np.linspace(-2.5, 2.5, 5):
            for yaw in (-60, -30, 0, 30, 60, 90):
                for pitch in (8, 16, 24, 32, 42):
                    starts.append([x0, y0, yaw, pitch])

    results = []
    for combo in itertools.product(*cand_lists):
        pts3 = [(c[0], c[1], 0.0) for c in combo]
        best = None
        for s in starts:
            out = gauss_newton(s, pts3, uvs, cam, height)
            if out is None:
                continue
            p, rms = out
            d = describe(p, height)
            if not (MIN_PITCH <= d["pitch_deg"] <= MAX_PITCH):
                continue
            if abs(d["x"]) > MAX_DIST_XY or abs(d["y"]) > MAX_DIST_XY:
                continue
            if best is None or rms < best[2]:
                best = (p, d, rms, combo)
        if best:
            results.append(best)

    if not results:
        print("没有可行解：检查对应点或高度")
        return 1
    results.sort(key=lambda r: r[2])
    print("\n候选解（按重投影 RMS 排序）：")
    for i, (p, d, rms, combo) in enumerate(results[:args.top], 1):
        tag = " / ".join(f"({c[0]:+.2f},{c[1]:+.2f})" for c in combo)
        print(f"  #{i} rms={rms:.2f}px  pos=({d['x']:+.2f}, {d['y']:+.2f}, h={d['height_m']:.3f})"
              f"  yaw={d['yaw_deg']:+.1f}° pitch={d['pitch_deg']:.1f}° roll={d['roll_deg']:+.1f}°")
        print(f"      对应: {tag}")

    p, d, rms, combo = results[0]
    img = cv2.imread(str(img_path))
    out_png = OUTPUT_DIR / "extrinsics_fixed_h.png"
    draw_overlay(img, cam, pose_to_ext(p, height), fm, out_png)
    md = OUTPUT_DIR / "extrinsics_fixed_h.md"
    with md.open("w", encoding="utf-8") as f:
        f.write("# 固定高度反解外参\n\n")
        f.write(f"- 图像：`{rel(img_path)}`\n- 高度（卷尺）：{height:.3f} m\n")
        f.write(f"- 观测点 {len(points)} 个，枚举 {len(results)} 个可行假设\n\n")
        f.write("| 排名 | RMS(px) | x | y | yaw(°) | pitch(°) | 对应假设 |\n|---|---:|---:|---:|---:|---:|---|\n")
        for i, (pp, dd, rr, cc) in enumerate(results[:args.top], 1):
            f.write("| %d | %.2f | %+.2f | %+.2f | %+.1f | %.1f | %s |\n" % (
                i, rr, dd["x"], dd["y"], dd["yaw_deg"], dd["pitch_deg"],
                " / ".join(f"({c[0]:+.2f},{c[1]:+.2f})" for c in cc)))
        f.write("\n叠图：`%s`\n" % rel(out_png))
    print(f"\n输出: {rel(md)} / {rel(out_png)}")

    if args.write:
        out_yaml = CALIB_DIR / "camera_extrinsics.candidate.yaml"
        save_yaml(out_yaml, {"camera_extrinsics": {
            "R": d["R"].tolist(), "t": d["t"].tolist(),
            "camera_height_m": round(d["height_m"], 4),
            "pitch_deg": round(d["pitch_deg"], 3),
            "yaw_deg": round(d["yaw_deg"], 3),
            "roll_deg": round(d["roll_deg"], 3),
            "demo": False,
            "method": "固定高度反解：%.3f m（卷尺）+ %d 个检测点，roll 固定 0" % (height, len(points)),
            "residual_px": round(rms, 3),
            "source_image": rel(img_path),
        }})
        print(f"输出: {rel(out_yaml)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
