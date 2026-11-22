# -*- coding: utf-8 -*-
"""自建场地地图：校验 config/field.yaml，派生 field_geometry.yaml、俯视图与指纹。

背景（2026-11-08 组长口径变更）：
任务二的 field.yaml **由我们自己参考官方格式定义**，不必等组织方发放。
因此本工具就是「地图的正规出处」：

    config/field.yaml            <- 唯一的可编辑真源（含 _meta 与各标志点）
        │  python tools/build_field_map.py
        ├─> config/field_geometry.yaml   派生：尺寸/原点/轴向/版本/来源
        ├─> outputs/field_topview.png    俯视图（供 UI 与 README 使用）
        └─> outputs/field_summary.txt    指纹与统计（写 README 用）

运行（项目根目录）：
    python tools/build_field_map.py
    python tools/build_field_map.py --field config/field.yaml --check-only
"""

from __future__ import annotations

import argparse
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import load_yaml, save_yaml, sha256_of          # noqa: E402
from src.common.paths import CONFIG_DIR, OUTPUT_DIR, ensure_runtime_dirs, rel  # noqa: E402
from src.loc.field_map import VALID_TYPES, FieldMap                      # noqa: E402

TYPE_COLOR = {           # BGR
    "LCross": (80, 200, 80),
    "TCross": (60, 170, 240),
    "XCross": (240, 160, 60),
    "PenaltyPoint": (200, 80, 200),
}


def render_topview(fm: FieldMap, size_m_per_px: float = 0.02, margin_m: float = 1.0,
                   goal_up: bool = False) -> np.ndarray:
    """画出场地俯视图：原点、X/Y 轴、各类标志点与类别颜色。

    goal_up=False：+X 向右、+Y 向上（与细则正文「+X 向右、+Y 向上」一致）
    goal_up=True ：+X 向上（指向球门）、+Y 向左（与组长示意图的摆法一致）
    两者是同一个右手坐标系的两种摆放，只是画面旋转 90°，数学上完全等价。
    """
    b = fm.bounds()
    if not b:
        raise ValueError("地图为空，无法绘制俯视图")
    x0, x1, y0, y1 = b
    length = fm.geometry.get("field_length_m") or (x1 - x0)
    width = fm.geometry.get("field_width_m") or (y1 - y0)
    half_x, half_y = float(length) / 2, float(width) / 2

    ppm = 1.0 / size_m_per_px
    if goal_up:
        W = int((2 * half_y + 2 * margin_m) * ppm)
        H = int((2 * half_x + 2 * margin_m) * ppm)
    else:
        W = int((2 * half_x + 2 * margin_m) * ppm)
        H = int((2 * half_y + 2 * margin_m) * ppm)
    img = np.full((H, W, 3), 245, np.uint8)

    def to_px(x, y):
        if goal_up:
            u = int((half_y + margin_m - y) * ppm)      # +Y 向左
            v = int((half_x + margin_m - x) * ppm)      # +X 向上
        else:
            u = int((x + half_x + margin_m) * ppm)
            v = int((half_y + margin_m - y) * ppm)
        return u, v

    def put(text, x, y, color, scale=0.6):
        u, v = to_px(x, y)
        cv2.putText(img, text, (u, v), cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2)

    # 场地边界与中线
    cv2.rectangle(img, to_px(-half_x, half_y), to_px(half_x, -half_y), (120, 120, 120), 2)
    cv2.line(img, to_px(0, half_y), to_px(0, -half_y), (170, 170, 170), 1)
    cv2.circle(img, to_px(0, 0), int(1.5 * ppm), (170, 170, 170), 1)

    # 坐标轴
    cv2.arrowedLine(img, to_px(0, 0), to_px(1.2, 0), (0, 0, 220), 2, tipLength=0.3)
    cv2.arrowedLine(img, to_px(0, 0), to_px(0, 1.2), (220, 0, 0), 2, tipLength=0.3)
    put("+X", 1.35, -0.35, (0, 0, 220))
    put("+Y", -0.55, 1.45, (220, 0, 0))

    for m in fm.markers:
        u, v = to_px(m.x, m.y)
        color = TYPE_COLOR.get(m.type, (0, 0, 0))
        if m.type == "LCross":
            cv2.drawMarker(img, (u, v), color, cv2.MARKER_TRIANGLE_UP, 12, 2)
        elif m.type == "TCross":
            cv2.drawMarker(img, (u, v), color, cv2.MARKER_SQUARE, 10, 2)
        elif m.type == "XCross":
            cv2.drawMarker(img, (u, v), color, cv2.MARKER_TILTED_CROSS, 12, 2)
        else:
            cv2.circle(img, (u, v), 5, color, 2)

    # 图例单独放在场地下方条带里，避免遮挡角点标志
    legend_h = 80
    canvas = np.full((H + legend_h, W, 3), 245, np.uint8)
    canvas[:H] = img
    x = 10
    for t in VALID_TYPES:
        n = sum(1 for m in fm.markers if m.type == t)
        cv2.putText(canvas, f"{t}: {n}", (x, H + 26), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    TYPE_COLOR.get(t, (0, 0, 0)), 1)
        x += 130
    cv2.putText(canvas, f"{fm.geometry.get('version', '?')}  ({length:.1f} x {width:.1f} m)"
                        + ("  +X = 球门方向  +Y = 面朝 +X 的左侧" if goal_up else ""),
                (10, H + 54), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (60, 60, 60), 1)
    return canvas


def build(field_path: Path, check_only: bool = False) -> int:
    ensure_runtime_dirs()
    fm = FieldMap.from_files(field_path)
    print("=" * 72)
    print("自建场地地图 · 校验与派生")
    print("=" * 72)
    print(fm.summary())

    b = fm.bounds()
    if b:
        print(f"标志点跨度 : x {b[1] - b[0]:.2f} m, y {b[3] - b[2]:.2f} m")
    counts = fm.counts()
    for t in VALID_TYPES:
        print(f"  {t:<13}: {counts.get(t, 0)}")
    if not fm.markers:
        print("错误：没有任何可解析的标志点")
        return 1

    # 几何一致性检查（细则 2.2：地图必须与实际场地一致，不允许为了算法改坐标）
    geo = fm.geometry
    length = geo.get("field_length_m")
    width = geo.get("field_width_m")
    problems = list(fm.warnings)
    if length and width and b:
        if b[1] - b[0] > float(length) + 0.05 or b[3] - b[2] > float(width) + 0.05:
            problems.append("标志点超出声明场地尺寸，请检查尺寸或坐标")
        if b[1] - b[0] < 0.2 * float(length):
            problems.append("标志点跨度远小于场地长度，可能只标了局部区域")
    # 共线检查：用于 PnP 的点不能共线
    pts = np.array([[m.x, m.y] for m in fm.markers])
    if len(pts) >= 3:
        centered = pts - pts.mean(axis=0)
        sv = np.linalg.svd(centered, compute_uv=False)
        if sv[0] > 0 and sv[-1] / sv[0] < 1e-3:
            problems.append("标志点近似共线，PnP 求解会退化")

    if problems:
        print("-" * 72)
        print("检查发现以下问题：")
        for p in problems:
            print("  ! " + p)
    else:
        print("检查结果   : 通过（无告警）")

    if check_only:
        print("=" * 72)
        return 1 if problems else 0

    # 派生 field_geometry.yaml
    meta = load_yaml(field_path).get("_meta", {})
    geometry = {
        "field_geometry": {
            "version": meta.get("name", "C0") + "-" + (meta.get("rev", "v1")),
            "map_version": meta.get("rev", "v1"),
            "field_length_m": float(length) if length else round(b[1] - b[0], 3),
            "field_width_m": float(width) if width else round(b[3] - b[2], 3),
            "units": "m",
            "origin": {"description": meta.get("origin", "场地中心"),
                       "x_m": 0.0, "y_m": 0.0},
            "axis": {"x": "向右为正", "y": "向上为正",
                     "yaw": "从 +X 轴逆时针为正，单位 degree"},
            "source": rel(field_path),
            "sha256": sha256_of(field_path),
            "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "verified_by_leader": bool(meta.get("verified_by_leader", False)),
        }
    }
    geo_path = CONFIG_DIR / "field_geometry.yaml"
    save_yaml(geo_path, geometry)
    print("-" * 72)
    print(f"已生成     : {rel(geo_path)}")

    img = render_topview(fm)
    img_path = OUTPUT_DIR / "field_topview.png"
    cv2.imwrite(str(img_path), img)
    print(f"俯视图     : {rel(img_path)}  ({img.shape[1]}x{img.shape[0]})")

    img2 = render_topview(fm, goal_up=True)
    img2_path = OUTPUT_DIR / "field_topview_goal_up.png"
    cv2.imwrite(str(img2_path), img2)
    print(f"俯视图(球门朝上): {rel(img2_path)}  ({img2.shape[1]}x{img2.shape[0]})")

    summary = [
        f"场地文件   : {fm.field_path}",
        f"SHA256     : {fm.field_sha256}",
        f"生成时间   : {datetime.now():%Y-%m-%d %H:%M:%S}",
        f"标志点总数 : {len(fm.markers)}",
        "类别分布   : " + ", ".join(f"{k}={v}" for k, v in sorted(counts.items())),
    ]
    if b:
        summary.append(f"坐标范围   : x[{b[0]:.2f}, {b[1]:.2f}]  y[{b[2]:.2f}, {b[3]:.2f}] m")
    if problems:
        summary.append("告警       : " + "; ".join(problems))
    out_txt = OUTPUT_DIR / "field_summary.txt"
    out_txt.write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(f"指纹摘要   : {rel(out_txt)}")
    print("=" * 72)
    return 1 if problems else 0


def main() -> int:
    ap = argparse.ArgumentParser(description="自建场地地图校验与派生")
    ap.add_argument("--field", default=str(CONFIG_DIR / "field.yaml"),
                    help="场地文件路径，默认 config/field.yaml")
    ap.add_argument("--check-only", action="store_true", help="只校验，不生成文件")
    args = ap.parse_args()
    path = Path(args.field)
    if not path.is_absolute():
        path = ROOT / path
    return build(path, args.check_only)


if __name__ == "__main__":
    raise SystemExit(main())
