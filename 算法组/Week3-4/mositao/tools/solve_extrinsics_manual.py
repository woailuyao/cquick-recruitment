# -*- coding: utf-8 -*-
"""人工点取标志 → PnP 反解外参（高度用卷尺值固定）。

为什么需要它：
    0.73 m 低机位下模型给的类别不可靠（实测 8~10 个检测却找不到任何类别一致的可行位姿），
    纯白线的全局搜索又有多个等价极小值。但只要**人眼点准 5 个交点**，位姿就是确定的：
    高度已知（少一个未知数）、每个点 2 个方程，5 点 10 个方程解 4 个未知数（x, y, yaw, pitch）。

    对应关系不用人指定：脚本按类别枚举「点击点 ↔ 地图点」的所有同类型组合，逐一 PnP，
    取重投影残差最小的那个，并把最优解叠回图上供肉眼终检。

输入：
    一张图 + 同目录同名 .txt（YOLO 格式，由 tools/annotate.py --mode click 生成，
    类别 0=L 1=T 2=X；框大小无所谓，只取框心）

用法（项目根目录）：
    run.bat tools\\solve_extrinsics_manual.py --image dataset\\manucalib\\frame_073.png
    run.bat tools\\solve_extrinsics_manual.py --image ... --write
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml, save_yaml                             # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                          # noqa: E402
from src.loc.camera_model import CameraModel, Extrinsics                         # noqa: E402
from tools.fit_extrinsics_fixed_height import (                                  # noqa: E402
    describe, draw_overlay, gauss_newton, pose_to_ext, project,
)
from tools.realtime_field import build_field_map                                 # noqa: E402

CLASSES = ["LCross", "TCross", "XCross"]


def load_clicked(img_path: Path, w: int, h: int):
    """读出 annotate.py 存下的框，取框心 + 类别。"""
    txt = img_path.with_suffix(".txt")
    if not txt.exists():
        return []
    out = []
    for line in txt.read_text(encoding="utf-8").splitlines():
        f = line.split()
        if len(f) != 5:
            continue
        cls = int(float(f[0]))
        if cls < 0 or cls >= len(CLASSES):
            continue
        cx, cy = float(f[1]) * w, float(f[2]) * h
        out.append((CLASSES[cls], cx, cy))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description="人工点取标志反解外参")
    ap.add_argument("--image", required=True)
    ap.add_argument("--height", type=float, default=None, help="镜头离地高度（默认读 field_site.yaml）")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--site", default="config/field_site.yaml")
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--best", type=int, default=5, help="打印前几个候选")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    img_path = Path(args.image) if Path(args.image).is_absolute() else ROOT / args.image
    img = cv2.imread(str(img_path))
    if img is None:
        print(f"读不出图片：{img_path}")
        return 2
    h_img, w_img = img.shape[:2]
    cam = CameraModel.from_yaml(ROOT / args.intrinsics)

    sg = (load_yaml(ROOT / args.site).get("field_site_overrides") or {}).get("site_geometry") or {}
    height = float(args.height or sg.get("camera_height_m") or 0.0)
    if height <= 0:
        print("缺少相机高度")
        return 2

    fm = build_field_map(ROOT / args.field, ROOT / args.geometry, mirror=True,
                         site_path=ROOT / args.site)
    by_type = {}
    for m in fm.markers:
        by_type.setdefault(m.type, []).append(m)

    clicks = load_clicked(img_path, w_img, h_img)
    if len(clicks) < 4:
        print(f"只读到 {len(clicks)} 个点，至少需要 4 个（建议 5~6 个，且不要都落在一条线上）")
        return 1
    cnt = {}
    for c, _u, _v in clicks:
        cnt[c] = cnt.get(c, 0) + 1
    print(f"图片 {img_path.name}（{w_img}×{h_img}）｜高度 {height:.3f} m")
    print("点击点：" + "，".join(f"{k}×{v}" for k, v in sorted(cnt.items())))

    # 按类别枚举同类型组合
    pools = []
    for t in CLASSES:
        k = cnt.get(t, 0)
        if k == 0:
            continue
        pool = by_type.get(t, [])
        if len(pool) < k:
            print(f"地图里 {t} 只有 {len(pool)} 个，不够配 {k} 个点击点")
            return 1
        pools.append((t, k, pool))

    starts = []
    for x0 in np.linspace(-6.0, 0.0, 7):
        for y0 in np.linspace(-2.0, 2.0, 5):
            for yaw in (-20, -10, 0, 10, 20, 40):
                for pitch in (5, 9, 13, 18, 24):
                    starts.append([x0, y0, yaw, pitch])

    # 退化保护：点几乎共线时 PnP 会给出 NaN 或荒谬解，必须先排掉
    pts_px = np.array([[c[1], c[2]] for c in clicks], float)
    pts_c = pts_px - pts_px.mean(0)
    sv = np.linalg.svd(pts_c, compute_uv=False)
    if sv[0] > 0 and (sv[1] / sv[0]) < 0.12:
        print(f"⚠ 这些点几乎共线（奇异值比 {sv[1] / sv[0]:.4f} < 0.12），位姿不可解。")
        print("  请在图上点『不在同一条线上』的 5~6 个交点：既有横向铺开，也要有远近两个深度。")
        return 1

    results = []
    for combo_pick in itertools.product(*[
            itertools.permutations(range(len(pool)), k) for _t, k, pool in pools]):
        pts3, uvs, tags = [], [], []
        for (t, k, pool), idxs in zip(pools, combo_pick):
            for j in idxs:
                m = pool[j]
                pts3.append((m.x, m.y, 0.0))
                tags.append(m.name)
            for c in [c for c in clicks if c[0] == t][:k]:
                uvs.append([c[1], c[2]])
        best = None
        for s in starts:
            out = gauss_newton(s, pts3, uvs, cam, height)
            if out is None:
                continue
            p, rms = out
            d = describe(p, height)
            if not (0.5 <= d["pitch_deg"] <= 60):
                continue
            if abs(d["x"]) > 9 or abs(d["y"]) > 6:
                continue
            if best is None or rms < best[2]:
                best = (p, d, rms, list(tags))
        if best:
            results.append(best)

    if not results:
        print("没有可行解：检查点的位置是否点错、类别是否标错")
        return 1
    results.sort(key=lambda r: r[2])
    print(f"\n共 {len(results)} 个可行假设，按重投影残差排序：")
    for i, (p, d, rms, tags) in enumerate(results[:args.best], 1):
        print(f"  #{i} rms={rms:5.2f} px  x={d['x']:+.2f} y={d['y']:+.2f} "
              f"yaw={d['yaw_deg']:+.1f}° pitch={d['pitch_deg']:.1f}°")
        print(f"      配到：{' / '.join(tags)}")

    p, d, rms, tags = results[0]
    out_png = OUTPUT_DIR / "extrinsics_manual.png"
    draw_overlay(img, cam, pose_to_ext(p, height), fm, out_png)
    md = OUTPUT_DIR / "extrinsics_manual.md"
    with md.open("w", encoding="utf-8") as f:
        f.write("# 人工点取反解外参\n\n")
        f.write(f"- 图片：`{rel(img_path)}`（{w_img}×{h_img}）\n")
        f.write(f"- 高度（卷尺）：{height:.3f} m\n")
        f.write(f"- 点击 {len(clicks)} 点：{'，'.join(f'{k}×{v}' for k, v in sorted(cnt.items()))}\n")
        f.write(f"- 最优对应：{' / '.join(tags)}\n")
        f.write(f"- 重投影残差：{rms:.2f} px\n\n")
        f.write("| 量 | 值 |\n|---|---:|\n")
        f.write(f"| x (m) | {d['x']:+.3f} |\n| y (m) | {d['y']:+.3f} |\n")
        f.write(f"| h (m) | {height:.3f} |\n| yaw (°) | {d['yaw_deg']:+.2f} |\n")
        f.write(f"| pitch (°) | {d['pitch_deg']:.2f} |\n\n叠图：`{rel(out_png)}`\n")
    print(f"\n最优：rms={rms:.2f} px  x={d['x']:+.3f} y={d['y']:+.3f} "
          f"yaw={d['yaw_deg']:+.2f}° pitch={d['pitch_deg']:.2f}°")
    print(f"输出: {rel(md)} / {rel(out_png)}")

    if args.write:
        out_yaml = CALIB_DIR / "camera_extrinsics.yaml"
        save_yaml(out_yaml, {"camera_extrinsics": {
            "R": d["R"].tolist(), "t": d["t"].tolist(),
            "camera_height_m": round(height, 4),
            "pitch_deg": round(d["pitch_deg"], 3),
            "yaw_deg": round(d["yaw_deg"], 3),
            "roll_deg": round(d["roll_deg"], 3),
            "demo": False,
            "method": "人工点取 %d 个交点 + 卷尺高度 %.3f m 的 PnP" % (len(clicks), height),
            "residual_px": round(rms, 3),
            "source_image": rel(img_path),
        }})
        print(f"输出: {rel(out_yaml)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
