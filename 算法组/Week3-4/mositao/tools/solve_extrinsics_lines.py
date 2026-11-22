# -*- coding: utf-8 -*-
"""只用「白色场地线」反解相机外参：不依赖标志检测，类别问题直接绕开。

为什么单独写一个：
    solve_extrinsics_grid.py 先用标志点给初值、再用白线细化。低机位（0.73 m）下标志
    检测不稳定，初值一偏，白线细化就滑进局部最优 —— 实测 12 帧解出的 x 从 −2.4 m 跳到
    −9.5 m，毫无一致性。既然真正约束上千像素的是白线，就让白线自己去找位姿：

    1) 提取白线掩膜 → 距离变换（每个像素到最近白线的距离）；
    2) 在给定范围内粗扫 (x, y, yaw, pitch)，高度用卷尺值固定，代价 = 投影出来的
       场地线到最近白线的平均距离；
    3) 取代价最低的若干位姿做局部模式搜索细化；
    4) 多帧投票，输出中位位姿与逐帧散布。

    搜索范围默认假设「相机在场地外 −X 侧、基本朝 +X 看」，这是本项目的机位约定：
    这样坐标系与 demo 口径一致（相机在负 x、+X 指向场地中心方向）。

用法（项目根目录）：
    run.bat tools\\solve_extrinsics_lines.py --video data\\videos\\low_01.mp4
    run.bat tools\\solve_extrinsics_lines.py --video a.mp4 b.mp4 --frames 5 --write
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

from src.common.config_io import load_yaml, save_yaml                            # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                         # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics                        # noqa: E402
from tools.solve_extrinsics_grid import (                                       # noqa: E402
    describe, field_polyline, line_cost, overlay, refine_lines, rot_mat,
)


def build_dt(frame, net_rows=0.22, net_cols=(0.22, 0.82), field_top=0.20):
    """白线掩膜 → 距离变换，并给出「允许评估的区域」。

    white_mask 已改成亮度顶帽（不再被灰色草皮骗到），所以区域限制不必再依赖「绿色」——
    这块场地的草皮饱和度只有 9，任何按颜色判草皮的规则都会失效（实测命中 0%）。
    这里只做两件简单的事：挖掉上方球门网/墙面占比最高的那条带，以及画面最上方的 20%。
    """
    from tools.realtime_field import white_mask
    mask = white_mask(frame)
    h, w = mask.shape[:2]
    mask[0:int(net_rows * h), int(net_cols[0] * w):int(net_cols[1] * w)] = 0
    dt = cv2.distanceTransform((mask == 0).astype(np.uint8), cv2.DIST_L2, 3)
    region = np.zeros((h, w), bool)
    region[int(field_top * h):, :] = True
    return dt, region, h


def line_cost_region(params, Pline, cam, height, dt, region, min_pts=60):
    """投影出来的场地线，到最近白线的平均距离；只统计落在允许区域内的采样点。"""
    x0, y0, yaw, pitch = params
    R = rot_mat(yaw, pitch)
    t = np.array([x0, y0, height], float)
    p_cam = (Pline - t) @ R
    ok = p_cam[:, 2] > 0.1
    if ok.sum() < min_pts:
        return 1e9
    uv, okp = cam.project(p_cam)
    ok &= okp
    u = uv[:, 0]; v = uv[:, 1]
    H, W = dt.shape
    inside = (u >= 0) & (u < W - 1) & (v >= 0) & (v < H - 1) & ok
    if inside.sum() < min_pts:
        return 1e9
    ui = u[inside].astype(int); vi = v[inside].astype(int)
    keep = region[vi, ui]
    if keep.sum() < min_pts:
        return 1e9
    d = dt[vi[keep], ui[keep]]
    return float(np.minimum(d, 25.0).mean())


def solve_one(frame, cam, height, Pline, args):
    dt, region, h = build_dt(frame)
    coarse = []
    for x0 in np.arange(args.xmin, args.xmax + 1e-9, args.xstep):
        for y0 in np.arange(args.ymin, args.ymax + 1e-9, args.ystep):
            for yaw in np.arange(args.yawmin, args.yawmax + 1e-9, args.yawstep):
                for pitch in np.arange(args.pmin, args.pmax + 1e-9, args.pstep):
                    c = line_cost_region((x0, y0, yaw, pitch), Pline, cam, height, dt, region)
                    if c < 1e8:
                        coarse.append((c, (float(x0), float(y0), float(yaw), float(pitch))))
    if not coarse:
        return None
    coarse.sort(key=lambda t: t[0])
    best = None
    seen = []
    for _c, p0 in coarse[:args.top]:
        p = np.asarray(p0, float).copy()
        cost = line_cost_region(p, Pline, cam, height, dt, region)
        steps = np.array([0.12, 0.12, 1.2, 0.6])
        for _ in range(60):
            improved = False
            for i in range(4):
                for sgn in (+1, -1):
                    q = p.copy()
                    q[i] += sgn * steps[i]
                    c = line_cost_region(q, Pline, cam, height, dt, region)
                    if c < cost - 1e-4:
                        p, cost, improved = q, c, True
            if not improved:
                steps *= 0.5
                if steps[0] < 0.002:
                    break
        if any(abs(p[0] - q[0]) < 0.2 and abs(p[1] - q[1]) < 0.2
               and abs(p[2] - q[2]) < 2.0 and abs(p[3] - q[3]) < 1.5 for q in seen):
            continue
        seen.append(p)
        if best is None or cost < best[0]:
            best = (cost, p)
    if best is None:
        return None
    return best[0], best[1], len(coarse)


def main() -> int:
    ap = argparse.ArgumentParser(description="纯白线反解外参")
    ap.add_argument("--video", nargs="+", required=True)
    ap.add_argument("--frames", type=int, default=5)
    ap.add_argument("--height", type=float, default=None)
    ap.add_argument("--site", default="config/field_site.yaml")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    # 搜索范围（默认：相机在场地外 −X 侧、基本朝 +X）
    ap.add_argument("--xmin", type=float, default=-7.0)
    ap.add_argument("--xmax", type=float, default=-1.0)
    ap.add_argument("--xstep", type=float, default=0.25)
    ap.add_argument("--ymin", type=float, default=-1.5)
    ap.add_argument("--ymax", type=float, default=1.5)
    ap.add_argument("--ystep", type=float, default=0.25)
    ap.add_argument("--yawmin", type=float, default=-20.0)
    ap.add_argument("--yawmax", type=float, default=20.0)
    ap.add_argument("--yawstep", type=float, default=2.0)
    ap.add_argument("--pmin", type=float, default=4.0)
    ap.add_argument("--pmax", type=float, default=22.0)
    ap.add_argument("--pstep", type=float, default=1.0)
    ap.add_argument("--top", type=int, default=12, help="粗解里细化前多少个")
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--out-file", default=str(CALIB_DIR / "camera_extrinsics.yaml"))
    args = ap.parse_args()

    from tools.realtime_field import build_field_map
    cam = CameraModel.from_yaml(ROOT / args.intrinsics)
    fm = build_field_map(ROOT / args.field, ROOT / args.geometry, mirror=True,
                         site_path=ROOT / args.site)
    sg = (load_yaml(ROOT / args.site).get("field_site_overrides") or {}).get("site_geometry") or {}
    height = float(args.height or sg.get("camera_height_m") or 0.0)
    if height <= 0:
        print("缺少相机高度")
        return 2
    Pline = field_polyline(fm)
    print(f"地图 {len(fm.markers)} 点｜高 {height:.3f} m｜白线采样 {len(Pline)} 点｜"
          f"搜索 x[{args.xmin},{args.xmax}] y[{args.ymin},{args.ymax}] "
          f"yaw[{args.yawmin},{args.yawmax}] pitch[{args.pmin},{args.pmax}]")

    results = []
    for vp in args.video:
        p = Path(vp) if Path(vp).is_absolute() else ROOT / vp
        cap = cv2.VideoCapture(str(p))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT)) or 1
        idxs = [int(round(i * (n - 1) / max(args.frames - 1, 1))) for i in range(args.frames)]
        print(f"\n[{p.name}] {n} 帧")
        for fi in idxs:
            cap.set(cv2.CAP_PROP_POS_FRAMES, fi)
            ok, frame = cap.read()
            if not ok:
                continue
            out = solve_one(frame, cam, height, Pline, args)
            if out is None:
                print(f"  f{fi:04d}: 无解")
                continue
            cost, pr, ncoarse = out
            d = describe(pr, height)
            print(f"  f{fi:04d}: {ncoarse:6d} 个粗解 → 白线 {cost:4.2f} px  "
                  f"x={d['x']:+.2f} y={d['y']:+.2f} yaw={d['yaw_axis_deg']:+.1f}° "
                  f"pitch={d['pitch_axis_deg']:.1f}°")
            results.append((cost, pr, p, fi, frame))
        cap.release()

    if not results:
        print("没有任何可行解")
        return 1
    arr = np.array([r[1] for r in results])
    med = np.median(arr, axis=0)
    spread = np.std(arr, axis=0)
    d = describe(med, height)
    print("\n" + "=" * 74)
    print(f"逐帧散布：x±{spread[0]*100:.1f} cm  y±{spread[1]*100:.1f} cm  "
          f"yaw±{spread[2]:.2f}°  pitch±{spread[3]:.2f}°")
    print(f"中位位姿：x={d['x']:+.3f} m  y={d['y']:+.3f} m  h={height:.3f} m")
    print(f"          yaw={d['yaw_axis_deg']:+.2f}°  pitch={d['pitch_axis_deg']:.2f}°")
    print("=" * 74)

    results.sort(key=lambda r: r[0])
    cost, pr, vp, fi, frame = results[0]
    ext = Extrinsics(d["R"], d["t"])
    out_png = OUTPUT_DIR / "extrinsics_lines.png"
    overlay(frame, cam, ext, fm, out_png, None)
    md = OUTPUT_DIR / "extrinsics_lines.md"
    with md.open("w", encoding="utf-8") as f:
        f.write("# 纯白线反解外参\n\n")
        f.write(f"- 视频：`{rel(vp)}` 第 {fi} 帧（叠图取自该帧）\n")
        f.write(f"- 相机高度（卷尺）：{height:.3f} m\n")
        f.write(f"- 逐帧散布：x±{spread[0]*100:.1f} cm, y±{spread[1]*100:.1f} cm, "
                f"yaw±{spread[2]:.2f}°, pitch±{spread[3]:.2f}°\n")
        f.write(f"- 白线对齐（该帧）：{cost:.2f} px\n\n")
        f.write("| 量 | 值 |\n|---|---:|\n")
        f.write(f"| x (m) | {d['x']:+.3f} |\n| y (m) | {d['y']:+.3f} |\n")
        f.write(f"| h (m) | {height:.3f} |\n| yaw (°) | {d['yaw_axis_deg']:+.2f} |\n")
        f.write(f"| pitch (°) | {d['pitch_axis_deg']:.2f} |\n\n叠图：`{rel(out_png)}`\n")
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
            "method": "纯白线距离变换反解（不依赖标志检测）；高度用卷尺固定 %.3f m" % height,
            "line_align_px": round(cost, 3),
            "frames": len(results),
            "spread_cm": [round(float(spread[0]) * 100, 2), round(float(spread[1]) * 100, 2)],
            "spread_deg": [round(float(spread[2]), 2), round(float(spread[3]), 2)],
            "source_video": rel(vp),
            "source_frame": fi,
        }})
        print(f"输出: {rel(out_yaml)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
