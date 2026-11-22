# -*- coding: utf-8 -*-
"""外参自动求解：从「已拍好的外参视频」直接解出相机相对底座的 R,t。

为什么需要它：任务二每一步都要外参，而 R,t 靠卷尺和量角器量不准（俯仰差 2°
就能带来数分米的地面误差）。这里改用 PnP：一帧里只要看到 4 个以上已知标志，
就能把相机的完整姿态解出来。

难点不是 PnP，而是「哪个像素对应地图上哪个点」。本工具的解法：
    1) 检测出标志 → 按像素距离去重（远景常在同一位置重复出框）；
    2) 类型已知（L/T/X/P），于是关联是一个「同类型的一一映射」；
    3) 枚举所有同类型映射，逐组做 PnP，用物理约束（高度、俯角、滚转、
       重投影误差）筛掉荒谬解；
    4) 多帧投票：外参是固定的，真解会在不同帧上反复出现，假解则散乱。

用法（项目根目录）：
    run.bat tools\\solve_extrinsics_auto.py --video data\\extr\\ext_02.mp4
    run.bat tools\\solve_extrinsics_auto.py --video data\\extr\\ext_01.mp4 data\\extr\\ext_02.mp4

输出：
    calibration/camera_extrinsics.yaml      （realtime_field.py 直接读它）
    outputs/extrinsics_report.md            （求解过程与可信度，抄进 README）
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.config_io import save_yaml                                    # noqa: E402
from src.common.paths import CALIB_DIR, OUTPUT_DIR, rel                      # noqa: E402
from src.loc.camera_model import CameraModel                                 # noqa: E402
from tools.realtime_field import build_field_map, refine_junction            # noqa: E402

DEDUP_PX = 28.0
MAX_COMBOS = 120000


def dedup(dets, radius=DEDUP_PX):
    """同一位置重复出框（远景常见）只留置信度最高的那个。"""
    kept = []
    for d in sorted(dets, key=lambda x: -x["conf"]):
        if all(math.hypot(d["u"] - k["u"], d["v"] - k["v"]) > radius for k in kept):
            kept.append(d)
    return kept


def _by_type(dets):
    """按标志类型分组（类型名排序，保证枚举顺序可复现）。"""
    groups = {}
    for d in sorted(dets, key=lambda x: x["type"]):
        groups.setdefault(d["type"], []).append(d)
    return groups


def pose_from_rvec(rvec, tvec):
    """OpenCV 的 world->camera 转成本项目的 camera->base。

    OpenCV：p_cam = R_pnp @ p_world + t_pnp
    本项目：p_base = R_ext @ p_cam + t_ext
    """
    R_pnp, _ = cv2.Rodrigues(rvec)
    R_ext = R_pnp.T
    t_ext = -R_pnp.T @ np.asarray(tvec, float).ravel()
    return R_ext, t_ext


def describe(R_ext, t_ext):
    """把 R,t 拆成人类能判断的量：高度、俯角、滚转、朝向。"""
    optic = R_ext @ np.array([0.0, 0.0, 1.0])      # 光轴方向（相机 z 朝前）
    optic /= max(np.linalg.norm(optic), 1e-9)
    height = float(t_ext[2])
    pitch = math.degrees(math.asin(max(-1.0, min(1.0, -optic[2]))))
    yaw = math.degrees(math.atan2(optic[1], optic[0]))
    xaxis = R_ext @ np.array([1.0, 0.0, 0.0])
    xaxis /= max(np.linalg.norm(xaxis), 1e-9)
    # 符号取负：与 Extrinsics.from_params 的 roll 定义（绕 x 正向）保持一致
    roll = -math.degrees(math.asin(max(-1.0, min(1.0, xaxis[2]))))
    return dict(height_m=height, pitch_deg=pitch, roll_deg=roll, yaw_deg=yaw,
                x=float(t_ext[0]), y=float(t_ext[1]))


def plausible(desc):
    """物理约束：相机架在桌面/支架上，朝下前方看场地。"""
    return (0.20 <= desc["height_m"] <= 2.50
            and 8.0 <= desc["pitch_deg"] <= 85.0
            and abs(desc["roll_deg"]) <= 20.0
            and -13.0 <= desc["x"] <= 13.0
            and -11.0 <= desc["y"] <= 11.0)


def reproj_rms(obj, img, rvec, tvec, K, dist):
    proj, _ = cv2.projectPoints(np.asarray(obj, np.float64), rvec,
                                np.asarray(tvec, np.float64), K, dist)
    d = proj.reshape(-1, 2) - np.asarray(img, np.float64).reshape(-1, 2)
    return float(np.sqrt(np.mean(np.sum(d * d, axis=1))))


def solve_frame(dets, by_type, K, dist, max_points=5, max_rms=6.0):
    """枚举同类型映射 + PnP，返回本帧最好的若干候选。"""
    dets = dedup(dets)

    # 按类型上限裁剪：地图上某类只有 N 个点，画面里却认出 N+1 个，多出来的
    # 必然是误检或同一标志重复出框。早先的做法是「整帧放弃」，结果一个低置信度
    # 的假 XCross 就能把整段视频判死（ext_05 就是这样全灭的）。这里改成保留
    # 该类置信度最高的 N 个，其余丢弃，继续用剩下的点求解。
    kept = []
    for t, ds in _by_type(dets).items():
        kept.extend(sorted(ds, key=lambda d: -d["conf"])[:len(by_type.get(t, []))])
    kept.sort(key=lambda d: -d["conf"])
    dets = kept[:max_points]
    if len(dets) < 4:
        return []

    groups = _by_type(dets)

    per_type = [list(itertools.permutations(by_type[t], len(ds)))
                for t, ds in groups.items()]
    n_total = 1
    for p in per_type:
        n_total *= len(p)
    if n_total > MAX_COMBOS:
        return []                         # 组合太多，这一帧放弃（宁缺毋假）

    type_keys = list(groups.keys())
    out = []
    for combo in itertools.product(*per_type):
        obj, img = [], []
        for tk, ms in zip(type_keys, combo):
            for d, m in zip(groups[tk], ms):
                obj.append((m.x, m.y, 0.0))
                img.append((d["u"], d["v"]))
        try:
            n_sol, rvecs, tvecs, _errs = cv2.solvePnPGeneric(
                np.asarray(obj, np.float64), np.asarray(img, np.float64),
                K, dist, flags=cv2.SOLVEPNP_IPPE)
        except cv2.error:
            continue
        if n_sol == 0:
            continue
        for i in range(n_sol):
            rms = reproj_rms(obj, img, rvecs[i], tvecs[i], K, dist)
            if rms > max_rms:
                continue
            R_ext, t_ext = pose_from_rvec(rvecs[i], tvecs[i])
            desc = describe(R_ext, t_ext)
            if not plausible(desc):
                continue
            out.append(dict(R=R_ext, t=t_ext, rms=rms, **desc,
                            obj=obj, img=img, n_points=len(obj)))
    out.sort(key=lambda r: r["rms"])
    return out


def cluster_key(c):
    return (round(c["height_m"], 1), round(c["pitch_deg"] / 3.0))


def _selftest(fm, cam, args) -> int:
    """仿真自检：已知外参 → 投影成像素 → 反解，验证「关联 + PnP」逻辑本身对不对。

    真机外参目前解不出来，是因为 ext 视频里可见的标志太少、太远（检测只给 3–5 个
    低质量点，几何近乎共线），不是算法错。这个自检把「数据质量」这一项摘掉，
    单独证明算法能还原外参；这样插上相机拍到好画面时，结果才可信。
    """
    import random

    from src.loc.camera_model import Extrinsics, ground_to_pixels, pixels_to_ground
    from src.loc.matcher import MatchParams, ObservedMarker, locate, wrap_pi

    K, dist = cam.K.copy(), np.asarray(cam.dist, float).reshape(-1, 1)
    ext_truth = Extrinsics.from_params(height=0.75, pitch_deg=26.0, roll_deg=1.2)
    pose = (9.4, -6.6, math.radians(148.0))        # 相机架在场外一角，斜向场地中心
    rng = random.Random(7)

    print("=" * 80)
    print("仿真自检：已知外参 → 像素 → 反解")
    print(f"真值：高度 0.750 m  俯角 26.00°  滚转 1.20°  位置 ({pose[0]}, {pose[1]})")
    print("=" * 80)

    fails = 0
    for trial in range(6):
        c, s = math.cos(-pose[2]), math.sin(-pose[2])
        pairs = []
        for m in fm.markers:
            dx, dy = m.x - pose[0], m.y - pose[1]
            bx, by = c * dx - s * dy, s * dx + c * dy      # 地图 -> 载体
            if not (0.8 < math.hypot(bx, by) < 14.0):
                continue
            uv, ok = ground_to_pixels(cam, ext_truth, [(bx, by, 0.0)])
            if not ok[0]:
                continue
            pairs.append((m.type, float(uv[0][0]), float(uv[0][1])))
        if len(pairs) < 4:
            print("  视场内标志不足 4 个，跳过")
            continue
        rng.shuffle(pairs)
        dets = [{"type": t, "u": u + rng.gauss(0, 1.0), "v": v + rng.gauss(0, 1.0),
                 "conf": 0.9} for t, u, v in pairs[:5]]
        cands = _solve_with_map(dets, fm, K, dist, args.max_points)
        if not cands:
            print(f"  试次 {trial}: 无解")
            fails += 1
            continue
        b = cands[0]
        dh = abs(b["height_m"] - 0.75)
        dp = abs(b["pitch_deg"] - 26.0)
        dr = abs(b["roll_deg"] - 1.2)
        # 端到端：把解出的外参照「归一化」写法重建，再跑一遍真实定位链路，
        # 看能不能把相机在场地里的位置与朝向还原出来。
        thn = math.radians(-b["yaw_deg"])
        cn, sn = math.cos(thn), math.sin(thn)
        R_norm = np.array([[cn, -sn, 0.0], [sn, cn, 0.0], [0.0, 0.0, 1.0]]) @ b["R"]
        ext_norm = Extrinsics(R_norm, [0.0, 0.0, b["height_m"]])
        uv = np.array([[d["u"], d["v"]] for d in dets], float)
        pts, okm = pixels_to_ground(cam, ext_norm, uv, max_range=16.0)
        obs = [ObservedMarker(type=dets[i]["type"], x=float(pts[i, 0]), y=float(pts[i, 1]),
                              confidence=0.9) for i in range(len(dets)) if okm[i]]
        loc = locate(obs, fm, MatchParams())
        e_xy = math.hypot(loc.x - pose[0], loc.y - pose[1]) if loc.valid else float("nan")
        e_yaw = abs(math.degrees(wrap_pi(math.radians(loc.yaw_deg) - pose[2]))) \
            if loc.valid else float("nan")
        ok = dh < 0.15 and dp < 3.0 and dr < 3.0 and loc.valid and e_xy < 0.20 and e_yaw < 3.0
        print(f"  试次 {trial}: 观测 {len(dets)} → h={b['height_m']:.3f} "
              f"pitch={b['pitch_deg']:.2f}° roll={b['roll_deg']:.2f}° rms={b['rms']:.2f}px "
              f"| 端到端 位置误差 {e_xy:.3f} m 航向误差 {e_yaw:.2f}° "
              f"[{loc.status}] {'✓' if ok else '✗'}")
        if not ok:
            fails += 1

    print("-" * 80)
    print(f"自检结论：{'通过' if fails == 0 else f'{fails} 个试次未达精度要求'}")
    print("=" * 80)
    return 0 if fails == 0 else 1


def _solve_with_map(dets, fm, K, dist, max_points):
    by_type = {}
    for m in fm.markers:
        by_type.setdefault(m.type, []).append(m)
    return solve_frame(dets, by_type, K, dist, max_points)


def main() -> int:
    ap = argparse.ArgumentParser(description="外参自动求解（PnP + 多帧投票）")
    ap.add_argument("--video", nargs="+", default=[], help="外参视频（可多个）")
    ap.add_argument("--selftest", action="store_true", help="仿真自检：验证算法本身（不需要视频）")
    ap.add_argument("--frames-per-video", type=int, default=12)
    ap.add_argument("--weights", default="weights/best_marker_v9.pt")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--intrinsics", default=str(CALIB_DIR / "camera_intrinsics.yaml"))
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--geometry", default="config/field_geometry.yaml")
    ap.add_argument("--site", default="config/field_site.yaml",
                    help="现场实测覆盖（含 site_geometry 时按现场尺寸重建地图）")
    ap.add_argument("--no-site", action="store_true", help="不叠加现场覆盖，只用官方数值")
    ap.add_argument("--max-points", type=int, default=5)
    ap.add_argument("--no-mirror", action="store_true",
                    help="只用官方 14 点，不做对称补全（官方只覆盖半个场地）")
    ap.add_argument("--out", default=str(CALIB_DIR / "camera_extrinsics.yaml"))
    ap.add_argument("--report", default=str(OUTPUT_DIR / "extrinsics_report.md"))
    ap.add_argument("--device", default="0")
    args = ap.parse_args()

    cam = CameraModel.from_yaml(ROOT / args.intrinsics)
    K = cam.K.copy()
    dist = np.asarray(cam.dist, float).reshape(-1, 1)

    fm = build_field_map(ROOT / args.field, ROOT / args.geometry, mirror=not args.no_mirror,
                         site_path=None if args.no_site else (ROOT / args.site))
    if args.selftest:
        return _selftest(fm, cam, args)
    if not args.video:
        print("需要 --video 指定外参视频，或用 --selftest 做算法自检")
        return 1
    by_type = {}
    for m in fm.markers:
        by_type.setdefault(m.type, []).append(m)

    from ultralytics import YOLO
    model = YOLO(str(ROOT / args.weights))

    print("=" * 80)
    print(f"外参自动求解｜地图点 {len(fm.markers)}｜类型 "
          + " / ".join(f"{k}={len(v)}" for k, v in sorted(by_type.items())))
    print("=" * 80)

    per_frame = []
    for vp in args.video:
        path = Path(vp)
        if not path.is_absolute():
            path = ROOT / path
        if not path.exists():
            print(f"跳过：找不到 {vp}")
            continue
        cap = cv2.VideoCapture(str(path))
        total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        want = {int(round(i * (total - 1) / max(args.frames_per_video - 1, 1)))
                for i in range(args.frames_per_video)}
        print(f"\n[{rel(path)}] {total} 帧，取 {len(want)} 帧")
        for i in range(total):
            ok, frame = cap.read()
            if not ok:
                break
            if i not in want:
                continue
            res = model.predict(frame, conf=args.conf, imgsz=640,
                                device=args.device, verbose=False)[0]
            dets = []
            if res.boxes is not None and len(res.boxes) > 0:
                for xyxy, cf, cl in zip(res.boxes.xyxy.cpu().numpy(),
                                        res.boxes.conf.cpu().numpy(),
                                        res.boxes.cls.cpu().numpy()):
                    u, v, _mode, _n = refine_junction(frame, xyxy)
                    dets.append({"type": model.names[int(cl)], "conf": float(cf),
                                 "u": float(u), "v": float(v)})
            cands = solve_frame(dets, by_type, K, dist, args.max_points)
            if not cands:
                print(f"  f{i:03d}: 检测 {len(dets)} → 无可信解")
                continue
            b = cands[0]
            per_frame.append(b)
            print(f"  f{i:03d}: 检测 {len(dets)} → 候选 {len(cands)}，"
                  f"best h={b['height_m']:.2f}m pitch={b['pitch_deg']:.1f}° "
                  f"roll={b['roll_deg']:.1f}° yaw={b['yaw_deg']:.1f}° rms={b['rms']:.2f}px")
        cap.release()

    if len(per_frame) < 2:
        print("\n可信解不足 2 帧，无法投票。")
        print("多半是检测到的标志太少或类型不可靠；换一段更近、标志更清楚的视频重试。")
        return 1

    # ---- 多帧投票 ----
    buckets = {}
    for c in per_frame:
        buckets.setdefault(cluster_key(c), []).append(c)
    best_key, best_group = max(buckets.items(), key=lambda kv: len(kv[1]))
    support = len(best_group)

    # 用簇内所有帧的所有对应点合并做一次全局 PnP，比逐帧平均更稳
    obj = [p for c in best_group for p in c["obj"]]
    img = [p for c in best_group for p in c["img"]]
    ok, rvec, tvec = cv2.solvePnP(np.asarray(obj, np.float64), np.asarray(img, np.float64),
                                  K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        print("全局 PnP 失败")
        return 1
    rms_all = reproj_rms(obj, img, rvec, tvec, K, dist)
    R_ext, t_ext = pose_from_rvec(rvec, tvec)
    desc = describe(R_ext, t_ext)

    # 外参必须写成「相机 -> 载体」，而载体取相机光心的地面投影：
    #   平移只剩高度 t=(0,0,h)，光轴的朝向由定位步骤 (x,y,yaw) 求，不塞进外参里。
    # 否则 locate() 会拿到已经在地图系里的点，位姿恒为 0，变成一个假的「完美定位」。
    th = math.radians(-desc["yaw_deg"])
    c_, s_ = math.cos(th), math.sin(th)
    R_norm = np.array([[c_, -s_, 0.0], [s_, c_, 0.0], [0.0, 0.0, 1.0]]) @ R_ext
    t_norm = np.array([0.0, 0.0, desc["height_m"]])

    print("\n" + "-" * 80)
    print(f"投票结果：{support}/{len(per_frame)} 帧支持同一物理解")
    print(f"相机位置 x={desc['x']:+.3f} m  y={desc['y']:+.3f} m  高度 {desc['height_m']:.3f} m")
    print(f"姿态      俯角 {desc['pitch_deg']:.2f}°  滚转 {desc['roll_deg']:.2f}°  "
          f"朝向 {desc['yaw_deg']:.2f}°")
    print(f"全局重投影 RMS {rms_all:.3f} px（{len(obj)} 个对应点）")

    out = Path(args.out)
    if not out.is_absolute():
        out = ROOT / out
    payload = {
        "camera_extrinsics": {
            "R": [[round(float(v), 8) for v in row] for row in R_norm],
            "t": [round(float(v), 6) for v in t_norm],
            "frame": "adult_size field frame (X 沿长边指向 +X 球门, Y 沿短边, Z 向上)",
            "camera_height_m": round(desc["height_m"], 4),
            "pitch_deg": round(desc["pitch_deg"], 3),
            "roll_deg": round(desc["roll_deg"], 3),
            "yaw_offset_deg": 0.0,
            "forward_offset_m": 0.0,
            "lateral_offset_m": 0.0,
            "map_position_x_m": round(desc["x"], 4),
            "map_position_y_m": round(desc["y"], 4),
            "method": "同类型映射枚举 + solvePnP(IPPE) + 物理约束筛选 + 多帧投票；yaw 已归零",
            "reprojection_rms_px": round(rms_all, 4),
            "points_used": len(obj),
            "frame_support": f"{support}/{len(per_frame)}",
            "source": " ".join(Path(v).name for v in args.video),
            "generated_at": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "verified_by_tape": False,
        }
    }
    save_yaml(out, payload)
    print(f"已写出外参 {rel(out)}")

    # ---- 报告 ----
    lines = [
        "# 相机外参求解报告", "",
        f"- 生成时间：{datetime.now():%Y-%m-%d %H:%M:%S}",
        f"- 视频：{' / '.join(f'`{Path(v).name}`' for v in args.video)}",
        f"- 内参：`{rel(args.intrinsics)}`",
        f"- 地图点：{len(fm.markers)} 个（官方 14 点 + 中线镜像补全）",
        f"- 方法：同类型映射枚举 → solvePnP(IPPE) → 物理约束筛选 → 多帧投票",
        "", "## 求解结果", "",
        "| 量 | 数值 |", "| --- | ---: |",
        f"| 相机高度 | {desc['height_m']:.3f} m |",
        f"| 俯角 | {desc['pitch_deg']:.2f}° |",
        f"| 滚转 | {desc['roll_deg']:.2f}° |",
        f"| 朝向 | {desc['yaw_deg']:.2f}° |",
        f"| 相机位置 x | {desc['x']:+.3f} m |",
        f"| 相机位置 y | {desc['y']:+.3f} m |",
        f"| 全局重投影 RMS | {rms_all:.3f} px |",
        f"| 用到的对应点 | {len(obj)} |",
        f"| 帧支持 | {support}/{len(per_frame)} |",
        "", "## 逐帧候选", "",
        "| 帧 | 检测数 | 候选数 | 高度 m | 俯角 ° | 滚转 ° | RMS px |",
        "| ---: | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for c in per_frame:
        lines.append(f"| {c.get('frame', '-')} | {c['n_points']} | 1 | {c['height_m']:.2f} | "
                     f"{c['pitch_deg']:.1f} | {c['roll_deg']:.1f} | {c['rms']:.2f} |")
    lines += [
        "", "## 局限（必须写进 README）", "",
        "- 外参由视频帧 PnP 反解，**不是卷尺实测量**；建议插上相机后量一次镜头离地高度做互校。",
        "- 相机一旦挪动（换支架、改俯仰、换机位），外参立即失效，必须重解。",
        "- 单帧解在共面点上存在镜像歧义，本工具靠多帧投票 + 物理约束消歧，"
        "若候选帧支持数 < 3，结果不可信。",
    ]
    rp = Path(args.report)
    if not rp.is_absolute():
        rp = ROOT / rp
    rp.parent.mkdir(parents=True, exist_ok=True)
    rp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"求解报告 {rel(rp)}")
    print("=" * 80)
    return 0 if support >= 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
