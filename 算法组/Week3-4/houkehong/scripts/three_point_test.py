# -*- coding: utf-8 -*-
"""
C0 三点定位测试（细则 2.11 要求的"多位置现场实时复测"）

为什么需要
----------
单看点 A 的输出无法判断对错 —— 位姿错一倍和全对，画面看起来一样。
但如果把三脚架**平移**一段已知距离，程序输出的位置也应该移动同样的距离。
移动量对不上，就说明外参（尤其是相机高度）有比例误差。

更有用的是：这个比例可以直接反算出正确的高度。

    计算得到的地面距离 ∝ 相机高度 h
    所以  估计位移 / 实测位移 = h_当前 / h_真实
    于是  h_真实 = h_当前 × (实测位移 / 估计位移)

用法
----
    # 跑三点测试（需要相机和场地）
    python scripts/three_point_test.py --camera 1

    # 算完如果比例不对，直接把修正后的高度写回去
    python scripts/three_point_test.py --camera 1 --write

    # 数学自检，不需要硬件
    python scripts/three_point_test.py --selftest

操作流程
--------
    1. 三脚架放位置 A，面朝球门（中圈后方 X ≤ -0.5 m），全程不要再转动云台。
    2. 按 SPACE 采集 A（约 2 秒）。
    3. 把三脚架整体**往后平移** 2.0 m（用卷尺量），保持朝向不变。
    4. 输入 "2 0"（前进 2 m、左移 0 m），按 SPACE 采集 B。
    5. 再**往左平移** 1.5 m（或任意方向），输入对应数字，采集 C。
    6. 工具输出：位移比例、修正后的高度、是否通过。
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

import numpy as np
import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from src.app import Pipeline  # noqa: E402
from src.config import (load_camera, load_extrinsics, load_field,  # noqa: E402
                        check_field)
from src.geometry import Extrinsics, wrap_deg  # noqa: E402
from src.matching import FieldMatcher  # noqa: E402

CAMERA_YAML = os.path.join(ROOT, "config", "camera.yaml")


def rot2(deg):
    r = math.radians(deg)
    return np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]])


def fit_scale(poses, moves):
    """
    poses: [(x, y, yaw), ...] 三个位置的估计位姿，poses[0] 是基准 A
    moves: [(forward, left), ...] 相对 A 的实测位移，载体坐标系（前为正、左为正）

    返回 (scale, per_move)

    比例用**位移长度之比**求，不用向量点积：
      · 点积对符号极敏感 —— 用户把"后退 2 m"填成"前进 2 m"，点积会变负，
        算出来的比例是 -0.28，直接导出负的高度，很难排查。
      · 长度之比只关心"移动了多远"，符号填反也能得到正确比例；
        方向是否一致另外用夹角检查并提示。
    """
    x0, y0, yaw0 = poses[0]
    R = rot2(yaw0)
    out = []
    wsum = 0.0
    rsum = 0.0
    for i, (fw, lf) in enumerate(moves, start=1):
        if i >= len(poses):
            break
        est = np.array([poses[i][0] - x0, poses[i][1] - y0])
        meas = R @ np.array([fw, lf])
        est_len = float(np.linalg.norm(est))
        meas_len = float(np.linalg.norm(meas))
        ratio = meas_len / est_len if est_len > 1e-6 else float("nan")

        # 方向一致性：夹角越小越好。符号填反时夹角约 180°
        cosang = float("nan")
        if est_len > 1e-6 and meas_len > 1e-6:
            cosang = float(est @ meas / (est_len * meas_len))

        out.append({"est": est, "meas": meas, "est_len": est_len,
                    "meas_len": meas_len, "ratio": ratio, "cos": cosang})

        # 按位移长度加权：移动越远，比例越可信（相对误差更小）
        if math.isfinite(ratio) and est_len > 0.2:
            w = meas_len
            rsum += ratio * w
            wsum += w

    scale = rsum / wsum if wsum > 0 else float("nan")
    return scale, out


def collect_pose(pipe, cap, cv2, n_frames=60, win="three-point", show=True):
    """
    连续取若干帧，把 valid 的位姿取平均。

    返回 (pose 或 None, n_valid, diag)
    diag 里记录失败原因的诊断信息 —— 只说"标志不足"没法排查，
    必须区分是"一个标志都没检测到"还是"检测到了但匹配不上"。
    """
    # 每次采集前清空多帧观测缓冲：上一位置的观测会误导新位置
    if hasattr(pipe, "reset_obs"):
        pipe.reset_obs()
    poses = []
    diag = {"n_read": 0, "n_valid": 0, "markers": [], "statuses": {},
            "n_ball": 0, "fps_frames": []}
    t_end = time.time() + 4.0
    while len(poses) < n_frames and time.time() < t_end:
        ok, frame = cap.read()
        if not ok:
            break
        diag["n_read"] += 1
        out = pipe.process(frame)
        st = out.status or {}
        mc = int(st.get("marker_count", 0) or 0)
        diag["markers"].append(mc)
        status = str(st.get("status", "?"))
        diag["statuses"][status] = diag["statuses"].get(status, 0) + 1
        diag["n_ball"] += sum(1 for d in out.dets if d[1] == "ball")
        if st.get("valid") and out.carrier_pose is not None:
            poses.append(out.carrier_pose)
            diag["n_valid"] += 1
        if show:
            cv2.imshow(win, pipe.render(out, fps=0.0, infer_ms=0.0))
            cv2.waitKey(1)

    if not poses:
        return None, 0, diag

    pose, in_cluster, n_clusters = robust_pose(poses)
    diag["n_cluster"] = in_cluster
    diag["n_clusters"] = n_clusters
    return pose, in_cluster, diag


def _pitch_deg(camera_yaml):
    """从 camera.yaml 读俯仰角（Extrinsics 对象本身不保存这个角度）。"""
    try:
        with open(camera_yaml, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return float(d.get("extrinsics", {}).get("pitch_deg", float("nan")))
    except Exception:
        return float("nan")


def robust_pose(poses, pos_tol=0.4, yaw_tol=10.0):
    """
    从多帧位姿里取"最大的一簇"再平均，而不是直接全部平均。

    为什么必须这样做
    ----------------
    可见标志少时，匹配器偶尔会解出**别名解**（把观测对应到地图上另一组标志），
    位姿就会跳到场地另一侧。实测同一机位两轮，B 点的 y 相差 2.7 m ——
    不是尺度误差，是少数帧解错了，却把平均值整体拉偏。

    聚类后只保留最大的那一簇，少数派（错误解）被丢弃。

    返回 (pose, 簇内帧数, 簇数)
    """
    if not poses:
        return None, 0, 0
    if len(poses) == 1:
        return poses[0], 1, 1

    def close(a, b):
        d = math.hypot(a[0] - b[0], a[1] - b[1])
        dy = abs(((a[2] - b[2] + 180.0) % 360.0) - 180.0)
        return d <= pos_tol and dy <= yaw_tol

    # 贪心聚类：按顺序把每个位姿归入第一个足够近的簇
    clusters = []          # [[pose, ...], ...]
    for p in poses:
        placed = False
        for c in clusters:
            if close(p, c[0]):
                c.append(p)
                placed = True
                break
        if not placed:
            clusters.append([p])

    clusters.sort(key=len, reverse=True)
    best = clusters[0]
    arr = np.array(best, dtype=np.float64)
    x = float(np.mean(arr[:, 0]))
    y = float(np.mean(arr[:, 1]))
    # 朝向是角度，不能直接算术平均（179° 和 -179° 会变成 0）
    s = float(np.mean(np.sin(np.radians(arr[:, 2]))))
    c = float(np.mean(np.cos(np.radians(arr[:, 2]))))
    yaw = math.degrees(math.atan2(s, c))
    return (x, y, yaw), len(best), len(clusters)


def explain_failure(tag, diag):
    """把诊断信息翻译成可执行的建议。"""
    n = max(1, diag["n_read"])
    avg_m = sum(diag["markers"]) / n if diag["markers"] else 0.0
    mx_m = max(diag["markers"]) if diag["markers"] else 0
    print(f"\n  诊断（{tag}，共取 {diag['n_read']} 帧）：")
    print(f"    平均检出标志数 {avg_m:.1f}   最多 {mx_m}")
    print(f"    状态分布: {diag['statuses']}")
    print(f"    期间检出足球 {diag['n_ball']} 次")

    if diag["n_read"] == 0:
        print("\n  → 一帧都没读到，相机可能被占用或断开了。")
        return

    if mx_m == 0:
        print("\n  → **一个标志都没检测到**。按可能性排序：")
        print("     1. 镜头没对着场地（对着墙/人/天花板？）")
        print("     2. 场地标志离得太远，在画面里太小")
        print("     3. 标志模型阈值偏高（可试 --marker-conf 0.15）")
        print("     4. 相机不在 C0 场地上")
        print(f"     建议先用实时预览确认：run.py live 里看 markers 是否 ≥2")
    elif mx_m < 2:
        print(f"\n  → 最多只看到 {mx_m} 个标志，定位需要至少 2 个。")
        print("     往后退 / 转动镜头，让画面里出现更多标志。")
    else:
        print(f"\n  → 检测到 {mx_m} 个标志但匹配失败。看状态分布：")
        if "ambiguous" in diag["statuses"]:
            print("     ambiguous = 标志分布太对称，无法唯一确定朝向。")
            print("     换一个能看到**更多**标志的机位（尤其是非对称位置的标志）。")
        if "residual_too_large" in diag["statuses"]:
            print("     residual_too_large = 观测和地图对不上。")
            print("     可能外参不对，或地图坐标有误。")
        if "insufficient_markers" in diag["statuses"] and mx_m >= 2:
            print("     insufficient_markers 但检测数 ≥2，说明检测到的标志类别")
            print("     和地图里的类别对不上（比如全是同一类且只有 1 个）。")


def selftest():
    # A 在 (-2,0) 朝球门；B 往后平移 2 m；C 往左平移 1.5 m
    # 注意符号：A→B 是 x 从 -2 变到 -4，即"后退 2 m"，所以 forward 记为 -2
    true_poses = [(-2.0, 0.0, 0.0), (-4.0, 0.0, 0.0), (-2.0, 1.5, 0.0)]
    moves = [(-2.0, 0.0), (0.0, 1.5)]
    print("真值位姿:", [tuple(round(v, 2) for v in p) for p in true_poses])
    print("实测位移(前,左):", moves)

    all_ok = True
    for h_used, h_true in ((0.67, 0.67), (0.67, 1.30), (1.30, 0.67), (0.67, 1.00)):
        k = h_used / h_true        # 估计位移相对真值的缩放
        a = true_poses[0]
        est = [a] + [(a[0] + k * (p[0] - a[0]), a[1] + k * (p[1] - a[1]), p[2])
                     for p in true_poses[1:]]
        scale, detail = fit_scale(est, moves)
        h_fix = h_used * scale if scale and math.isfinite(scale) else float("nan")
        err_cm = abs(h_fix - h_true) * 100
        all_ok &= err_cm < 1.0
        print(f"\n  h_当前={h_used:.2f}  h_真实={h_true:.2f}")
        for d in detail:
            print(f"    实测 {d['meas_len']:.2f} m  估计 {d['est_len']:.2f} m  "
                  f"比例 {d['ratio']:.3f}  方向夹角 {math.degrees(math.acos(max(-1,min(1,d['cos'])))):.1f}°")
        print(f"    综合比例 {scale:.3f}  ->  修正后高度 {h_fix:.3f} m "
              f"(误差 {err_cm:.1f} cm)")

    print("\n自检:", "通过 ✅ 修正公式能把高度还原" if all_ok else "失败 ❌")
    return 0 if all_ok else 1


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=None)
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "c0_field.yaml"))
    ap.add_argument("--geometry",
                    default=os.path.join(ROOT, "config", "c0_field_geometry.yaml"))
    ap.add_argument("--ball-weights", default=os.path.join(ROOT, "weights", "ball_best.pt"))
    ap.add_argument("--marker-weights", default=os.path.join(ROOT, "weights", "marker_best.pt"))
    # 默认值与现场实测有效的配置一致：标志在低机位下偏小，640 比 416 更稳
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--marker-every", type=int, default=2)
    ap.add_argument("--ball-conf", type=float, default=0.70)
    ap.add_argument("--marker-conf", type=float, default=0.15)
    ap.add_argument("--obs-window", type=int, default=0,
                    help="观测累积帧数。0=只用当前帧（默认，现场实测更稳）")
    ap.add_argument("--tol", type=float, default=0.10,
                    help="位移比例允许偏差，0.10 表示正负 10 个百分点")
    ap.add_argument("--save", default=os.path.join(ROOT, "snapshots", "three_point_result.txt"),
                    help="把结果写到文件（默认 snapshots/three_point_result.txt），便于写进 README")
    ap.add_argument("--write", action="store_true",
                    help="比例不对时把修正后的高度写回 camera.yaml")
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--no-gui", action="store_true")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    import cv2
    from run import open_camera

    field = load_field(args.field, args.geometry)
    check_field(field)
    cam = load_camera(CAMERA_YAML)
    extr = load_extrinsics(CAMERA_YAML)

    dev = args.camera if args.camera is not None else 1
    cap = open_camera(dev, "msmf", cam.width, cam.height, 30)
    if not cap.isOpened():
        print(f"[错误] 打不开相机 {dev}")
        return 3

    from src.app import YoloDetector
    det = YoloDetector(ball_weights=args.ball_weights,
                       marker_weights=args.marker_weights,
                       conf=0.35, imgsz=args.imgsz, device="cpu",
                       marker_every=args.marker_every,
                       ball_conf=args.ball_conf, marker_conf=args.marker_conf)
    # min_inliers=3：2 个标志的对应关系有多解，实测同一位置两次能解出差 2.7 m 的位姿。
    # 宁可报 invalid，也不要用错位姿去算位移比例。
    matcher = FieldMatcher(field.features, gate=0.6, max_residual=0.35, min_inliers=3)
    pipe = Pipeline(field, cam, extr, matcher, detector=det)
    pipe.obs_window = max(0, int(args.obs_window))

    h_used = extr.camera_height
    print("=" * 70)
    print(f"地图 {os.path.basename(args.field)}   当前相机高度 h = {h_used:.3f} m")
    print("=" * 70)
    print("\n说明：三个位置**只做平移**，朝向尽量保持一致（转一点点不影响计算，")
    print("      但『前进/左移』是相对 A 的朝向说的，转多了方向会说不清）。")
    print("      每个位置都要能看见 >=3 个场地标志。\n")

    poses = []
    moves = []
    labels = ["A（基准）", "B", "C"]
    for i in range(3):
        if i == 0:
            input(f"把三脚架放到位置 A，面朝球门，准备好后按回车开始采集 ...")
        else:
            s = input(f"把三脚架平移到位置 {labels[i]}，输入相对 A 的位移 "
                      f"「前进几米 左移几米」(例如 2 0 或 0 1.5): ").strip()
            parts = s.replace(",", " ").split()
            if len(parts) < 2:
                print("  输入无效，用 0 0 代替（只做一致性检查，不算比例）")
                fw, lf = 0.0, 0.0
            else:
                try:
                    fw, lf = float(parts[0]), float(parts[1])
                except ValueError:
                    print("  输入不是数字，用 0 0 代替")
                    fw, lf = 0.0, 0.0
            moves.append((fw, lf))
            print(f"  注意：请保持朝向与 A 一致，按回车开始采集 {labels[i]} ...")
            input()

        print(f"  采集 {labels[i]} 中（约 6 秒）...")
        pose, n, diag = collect_pose(pipe, cap, cv2, n_frames=90, show=not args.no_gui)
        if pose is None:
            print(f"  ❌ {labels[i]} 没有拿到有效位姿")
            explain_failure(labels[i], diag)
            cap.release()
            cv2.destroyAllWindows()
            return 1
        poses.append(pose)
        nc = diag.get("n_clusters", 1)
        print(f"  ✅ {labels[i]}: x={pose[0]:+.3f}  y={pose[1]:+.3f}  "
              f"yaw={pose[2]:+.2f}°   (有效 {n}/{diag['n_read']} 帧, "
              f"标志数 {min(diag['markers'])}~{max(diag['markers'])})")
        if nc > 1:
            print(f"     ⚠️ 出现过 {nc} 组不同位姿（别名解），已取最大的一簇 {n} 帧。"
                  f"建议这个位置多站一会儿或换个角度，让标志更稳定")

    cap.release()
    cv2.destroyAllWindows()

    # 转向检查：工具只比较位置，所以转一点点不影响计算；
    # 但转多了之后"往前 2 m"到底指哪个方向就说不清，比例会算歪。
    yaw0 = poses[0][2]
    drift = max(abs(wrap_deg(p[2] - yaw0)) for p in poses[1:])
    if drift > 5.0:
        print(f"\n⚠️ 三次采集之间朝向变化了 {drift:.1f}°。")
        print("   位置比较本身不受影响，但『前进/左移』是相对 A 的朝向说的，")
        print("   转动后位移方向会和输入的描述不一致，可能导致比例偏差。")
        print("   如果结果的比例明显偏离 1.000，先怀疑这一点。")

    print("\n" + "=" * 70)
    print("结果")
    print("=" * 70)
    for i, p in enumerate(poses):
        print(f"  {labels[i]}: ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.2f}°)")

    scale, detail = fit_scale(poses, moves)
    print(f"\n{'实测位移':>10}{'估计位移':>11}{'比例':>8}   判定")
    print("-" * 52)
    ok = True
    for d in detail:
        if d["meas_len"] < 0.3:
            print(f"{d['meas_len']:>9.2f}m{d['est_len']:>10.2f}m{'—':>8}   位移太小，忽略")
            continue
        good = abs(d["ratio"] - 1.0) <= args.tol
        ok &= good
        print(f"{d['meas_len']:>9.2f}m{d['est_len']:>10.2f}m{d['ratio']:>8.3f}   "
              f"{'✅' if good else '❌'}")

    # 把结果落盘，便于写进 README，也避免终端输出丢失
    try:
        os.makedirs(os.path.dirname(args.save), exist_ok=True)
        import datetime as _dt
        with open(args.save, "w", encoding="utf-8") as f:
            f.write("# 三点位移测试结果（细则 2.11 多位置现场实时复测）\n\n")
            f.write(f"时间: {_dt.datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"相机外参: h = {h_used:.3f} m, pitch = {_pitch_deg(CAMERA_YAML):.3f}°\n")
            f.write(f"地图: {os.path.basename(args.field)}\n")
            f.write(f"检测参数: imgsz={args.imgsz}, marker_conf={args.marker_conf}, "
                    f"marker_every={args.marker_every}\n\n")
            f.write("| 位置 | 估计位姿 (x, y, yaw) |\n|---|---|\n")
            for i, p in enumerate(poses):
                f.write(f"| {labels[i]} | ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.2f}°) |\n")
            f.write("\n| 实测位移 | 估计位移 | 比例 | 判定 |\n")
            f.write("|---:|---:|---:|---|\n")
            for d in detail:
                if d["meas_len"] < 0.3:
                    f.write(f"| {d['meas_len']:.2f} m | {d['est_len']:.2f} m | — | 位移太小，忽略 |\n")
                else:
                    g = abs(d["ratio"] - 1.0) <= args.tol
                    f.write(f"| {d['meas_len']:.2f} m | {d['est_len']:.2f} m | "
                            f"{d['ratio']:.3f} | {'通过' if g else '未通过'} |\n")
            f.write(f"\n综合比例: {scale:.3f}\n")
            f.write(f"结论: {'通过（位移比例在 ±%.0f%% 内）' % (args.tol*100) if ok else '未通过'}\n")
        print(f"\n结果已保存: {args.save}")
    except Exception as e:
        print(f"\n[警告] 结果保存失败: {e}")

    if not math.isfinite(scale) or not detail:
        print("\n没拿到有效位移，无法评估比例。")
        return 1

    print("-" * 52)
    print(f"综合比例 {scale:.3f}   (1.000 = 完美；偏差 >{args.tol:.2f} 说明高度需要修正)")

    if ok:
        print("\n✅ 通过：程序输出的位移与实际位移一致，外参和地图都可信。")
        print("   把这次的输出记进 README 的现场测试记录即可。")
        return 0

    h_fix = h_used * scale
    print(f"\n❌ 未通过：估计位移是实际的 {scale:.3f} 倍。")
    print(f"   → 相机高度应该修正为 h = {h_used:.3f} × {scale:.3f} = **{h_fix:.3f} m**")
    print(f"     （当前配置是 {h_used:.3f} m，差了 {abs(h_fix - h_used):.3f} m）")
    print(f"   原因：地面距离与相机高度成正比，高度量小了就会导致位移被等比压缩。")

    if args.write:
        with open(CAMERA_YAML, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        ex = data.setdefault("extrinsics", {})
        ex["camera_height_m"] = round(h_fix, 4)
        ex["verified"] = True
        ex["method"] = (f"三点位移标定修正：由 {h_used:.3f} m 修正为 {h_fix:.3f} m"
                        f"（位移比例 {scale:.3f}，实测 {len(detail)} 次平移）")
        with open(CAMERA_YAML, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        print(f"\n已把高度 {h_fix:.3f} m 写回 {CAMERA_YAML}")
        print("建议再跑一次三点测试确认比例接近 1.000。")
    else:
        print("\n加上 --write 可以直接把修正后的高度写回配置。")
    return 1


if __name__ == "__main__":
    sys.exit(main())
