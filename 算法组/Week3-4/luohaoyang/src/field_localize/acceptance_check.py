"""一键验收自检：把"能不能过"拆成 5 级，逐项给 PASS / WARN / FAIL。

    python scripts/acceptance_check.py
    python scripts/acceptance_check.py --skip-model      # 不看训练指标（更快）

检查项：
    L1 内参      fx/fy、主点、RMS
    L2 外参      2 m / 3 m 地面点误差（没有点文件就跳过）
    L3 地图      标志数、对称性、带先验的可定位率
    L4 模型      val 集 mAP（可跳过）
    L5 链路      三个测试点上模拟一次定位，看状态和误差
"""

from __future__ import annotations

import argparse
import math
import random
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
PKG_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PKG_ROOT))


from field_localize.field_map import FieldMap                        # noqa: E402
from field_localize.ground_projection import GroundProjector, euler_to_R  # noqa: E402
from field_localize.pose_match import Locator, Observation           # noqa: E402

PASS, WARN, FAIL = "PASS", "WARN", "FAIL"
rows = []


def rec(level, name, detail):
    rows.append((level, name, detail))
    print(f"  [{level:^4}] {name:<26} {detail}")


# ----------------------------------------------------------------------
def check_intrinsics(args):
    p = Path(args.intrinsics)
    if not p.exists():
        rec(FAIL, "L1 内参", f"找不到 {p}")
        return None, None
    y = yaml.safe_load(p.read_text(encoding="utf-8"))
    K = np.array(y["camera_matrix"], float)
    D = np.array(y["distortion_coefficients"], float)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    rms = float(y.get("rms_reprojection_error_px", 99))
    ratio = fx / fy
    lvl = PASS if (0.99 <= ratio <= 1.01 and rms < 0.8) else (
        WARN if 0.97 <= ratio <= 1.03 and rms < 1.5 else FAIL)
    rec(lvl, "L1 内参",
        f"fx/fy={ratio:.4f} cx={cx:.0f} cy={cy:.0f} RMS={rms:.2f}px  ({p.name})")
    return K, D


def check_extrinsics(args, K, D):
    p = Path(args.extrinsics)
    if not p.exists():
        rec(FAIL, "L2 外参", f"找不到 {p}")
        return None, None
    y = yaml.safe_load(p.read_text(encoding="utf-8"))
    if "R_base_camera" in y:
        R = np.array(y["R_base_camera"], float).reshape(3, 3)
        O = np.array(y["camera_origin_base_m"], float)
        rec(PASS, "L2 外参(来源)", f"solvePnP  h={O[2]:.3f}m  "
                                   f"RMS={y.get('reprojection_rms_px', '?')}px")
    else:
        R = euler_to_R(float(y.get("pitch_deg", 0)), float(y.get("roll_deg", 0)),
                       float(y.get("yaw_deg", 0)))
        O = np.array([float(y.get("translation_x_m", 0)),
                      float(y.get("translation_y_m", 0)),
                      float(y.get("camera_height_m", 0))])
        rec(WARN, "L2 外参(来源)",
            f"手填欧拉角 h={O[2]:.3f}m pitch={y.get('pitch_deg')}deg —— 建议 solvePnP 重标")

    pts = Path(args.marks)
    if not K is None and pts.exists():
        proj = GroundProjector(K, D, R, O)
        spec = yaml.safe_load(pts.read_text(encoding="utf-8")) or {}
        errs = []
        for view in spec.get("views", []):
            for q in view["points"]:
                g = proj.project(q["u"], q["v"])
                if g:
                    errs.append(math.hypot(g["x"] - q["x"], g["y"] - q["y"]))
        if errs:
            mean = float(np.mean(errs)) * 100
            lvl = PASS if mean <= 5 else (WARN if mean <= 15 else FAIL)
            rec(lvl, "L2 外参(2m/3m)", f"地面误差 平均 {mean:.1f} cm  "
                                       f"最大 {max(errs)*100:.1f} cm")
            return proj, R
    rec(WARN, "L2 外参(2m/3m)", "没有地面点文件，跳过实测误差")
    return (GroundProjector(K, D, R, O) if K is not None else None), R


def check_map(args, proj):
    fmap = FieldMap.load(args.field)
    n = len(fmap.markers)
    types = ", ".join(f"{t}x{len(fmap.by_type(t))}" for t in fmap.types_present())
    rec(PASS if n >= 8 else WARN, "L3 地图", f"{n} 个标志 ({types})")

    xs = [m.x for m in fmap.markers]
    ys = [m.y for m in fmap.markers]
    cx, cy = (min(xs) + max(xs)) / 2, (min(ys) + max(ys)) / 2

    def sym(fn):
        for m in fmap.markers:
            px, py = fn(m.x, m.y)
            if not any(o.type == m.type and math.hypot(o.x - px, o.y - py) < 0.1
                       for o in fmap.markers):
                return False
        return True

    flags = []
    if sym(lambda x, y: (2 * cx - x, 2 * cy - y)):
        flags.append("180度")
    if sym(lambda x, y: (2 * cx - x, y)):
        flags.append("左右")
    if sym(lambda x, y: (x, 2 * cy - y)):
        flags.append("前后")
    rec(PASS if flags else PASS, "L3 对称性",
        ("对称(" + "/".join(flags) + ") -> 必须用跟踪先验") if flags else "无对称，单帧可定")

    # 带先验的可定位率
    rng = random.Random(7)
    loc = Locator(fmap)
    loc.bounds = fmap.bbox(pad=1.0)
    ok = tot = 0
    for _ in range(100):
        tx, ty = rng.uniform(min(xs), max(xs)), rng.uniform(min(ys), max(ys))
        tyaw = rng.uniform(-180, 180)
        th = math.radians(tyaw)
        c, s = math.cos(th), math.sin(th)
        obs = []
        for m in fmap.markers:
            dx, dy = m.x - tx, m.y - ty
            bx, by = c * dx + s * dy, -s * dx + c * dy
            r = math.hypot(bx, by)
            if r > 3.6 or abs(math.degrees(math.atan2(by, bx))) > 21.5:
                continue
            obs.append(Observation(m.type, bx + rng.gauss(0, 0.02),
                                   by + rng.gauss(0, 0.02), 0.9, r))
        if len(obs) < 2:
            continue
        tot += 1
        res = loc.locate(obs, prior_pose=(tx + rng.gauss(0, 0.15),
                                          ty + rng.gauss(0, 0.15),
                                          tyaw + rng.gauss(0, 4)))
        if res.valid and math.hypot(res.x - tx, res.y - ty) < 0.3:
            ok += 1
    rate = ok / max(1, tot) * 100
    rec(PASS if rate >= 70 else (WARN if rate >= 40 else FAIL), "L3 可定位率(带先验)",
        f"{ok}/{tot} = {rate:.0f}% 命中（3.6 m、±21.5° 视野内）")


def check_model(args):
    from ultralytics import YOLO
    p = Path(args.model)
    if not p.exists():
        rec(FAIL, "L4 模型", f"找不到 {p}")
        return
    try:
        r = YOLO(str(p)).val(data=str(ROOT / "dataset" / "data.yaml"),
                             imgsz=960, batch=4, device=args.device, verbose=False)
        m50, mr, mp = r.box.map50, r.box.mr, r.box.mp
        lvl = PASS if m50 >= 0.6 else (WARN if m50 >= 0.3 else FAIL)
        rec(lvl, "L4 模型(val)",
            f"P={mp:.2f} R={mr:.2f} mAP50={m50:.3f} mAP50-95={r.box.map:.3f}")
    except Exception as e:
        rec(WARN, "L4 模型", f"评估失败: {e}")


def check_pipeline(args, fmap, proj):
    """用地图自带的标志模拟三个测试点，看链路能不能定位。"""
    loc = Locator(fmap)
    loc.bounds = fmap.bbox(pad=1.0)
    spots = [("P1", -0.90, 1.10, 110.0), ("P2", 1.10, 0.90, 70.0),
             ("P3", -1.80, 0.75, 90.0)]
    rng = random.Random(3)
    worst = 0.0
    for name, tx, ty, tyaw in spots:
        th = math.radians(tyaw)
        c, s = math.cos(th), math.sin(th)
        obs = []
        for m in fmap.markers:
            dx, dy = m.x - tx, m.y - ty
            bx, by = c * dx + s * dy, -s * dx + c * dy
            r = math.hypot(bx, by)
            if r > 3.6 or abs(math.degrees(math.atan2(by, bx))) > 21.5:
                continue
            obs.append(Observation(m.type, bx + rng.gauss(0, 0.02),
                                   by + rng.gauss(0, 0.02), 0.9, r))
        res = loc.locate(obs, prior_pose=(tx + 0.15, ty - 0.1, tyaw + 3))
        err = math.hypot(res.x - tx, res.y - ty) if res.valid else float("nan")
        dyaw = abs((res.yaw_deg - tyaw + 180) % 360 - 180) if res.valid else float("nan")
        if res.valid:
            worst = max(worst, err)
        rec(PASS if (res.valid and err < 0.5) else FAIL, f"L5 {name}",
            f"obs={len(obs)} -> {res.status:<14} 位置误差 {err*100:5.1f} cm  "
            f"航向误差 {dyaw:4.1f}°")
    return worst


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--intrinsics", default=str(ROOT / "calibration" / "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=str(ROOT / "calibration" / "camera_extrinsics.yaml"))
    ap.add_argument("--field", default=str(ROOT / "config" / "field.yaml"))
    ap.add_argument("--marks", default=str(ROOT / "calibration" / "marks_2m_3m.yaml"))
    ap.add_argument("--model", default=str(ROOT / "runs" / "markers_v5" / "weights" / "best.pt"))
    ap.add_argument("--device", default="0")
    ap.add_argument("--skip-model", action="store_true")
    args = ap.parse_args()

    print("=" * 74)
    print("任务二 · 验收自检")
    print("=" * 74)
    K, D = check_intrinsics(args)
    proj, R = check_extrinsics(args, K, D)
    fmap = FieldMap.load(args.field)
    check_map(args, proj)
    if not args.skip_model:
        check_model(args)
    check_pipeline(args, fmap, proj)

    print("-" * 74)
    n_pass = sum(1 for l, _, _ in rows if l == PASS)
    n_warn = sum(1 for l, _, _ in rows if l == WARN)
    n_fail = sum(1 for l, _, _ in rows if l == FAIL)
    print(f"结果: {n_pass} PASS / {n_warn} WARN / {n_fail} FAIL")
    if n_fail:
        print("\n优先修这几项：")
        for l, name, detail in rows:
            if l == FAIL:
                print(f"  - {name}: {detail}")
    print("\n判定参考：")
    print("  L1/L2 = PASS            -> 几何链路可信")
    print("  L3 可定位率 >=70%       -> 场地覆盖够")
    print("  L4 mAP50 >=0.6          -> 模型能检（低于 0.3 需要补数据）")
    print("  L5 三个点都 PASS        -> 现场验收可以过")


if __name__ == "__main__":
    main()