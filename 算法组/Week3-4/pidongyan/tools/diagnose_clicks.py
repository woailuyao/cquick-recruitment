"""点选结果诊断：核对「点名有没有贴错」+ 消除平面 PnP 的地下镜像歧义。

适用场景
--------
人工点选最容易错的不是像素，而是**把交点认成另一个编号**（L 类 8 个点左右镜像、
场地角 vs 禁区角）。一个编号贴错，`run_task2.py clicks` 直解会给出荒谬位姿
（2026-11-20 pose_B 实测：rms 74 万 px、平面误差 473 cm、h=0.047m）。

本工具**不改像素观测**，只把每个点降级成「类别」喂给类别级 RANSAC，
让它重新认编号；再用**现场卷尺实测的相机高度 h** 做物理消歧
（平面 PnP 天然存在一个 h<0、pitch<0 的地下镜像解，位姿看起来像另一个机位）。

红线
----
- 像素观测 100% 来自本人目视，本工具一个字节都不改。
- RANSAC 只输出**点名建议**，必须本人对照 `docs/field_measured_map.png`
  目视确认后才能当作观测；候选文件是新副本（`*_candidate.yaml`），不覆盖原文件。
- 位姿真值（CSV）只用于末尾对比打分，**绝不参与生成解 / 分配**。
- h 先验是现场卷尺实测的外参高度，不是位姿真值；它只用于消镜像歧义，
  无解时**不否决**数据（见「降级」说明）。
- 候选副本在读入观测后**立即写出基线版**，任何分支提前退出都不会让
  第三条命令报 FileNotFoundError（2026-11-20 pose_C / pose_D2 两次踩坑后的结构修复）。

h 先验支无解的降级
------------------
带 prior 的 RANSAC 在「4 点 P3P 假设生成」阶段会用 pen>3.0 硬剔除与 h 偏离的
假设；5 点且含噪/含外点时，所有假设可能都被枪毙 -> 返回 None（2026-11-20
pose_D2 实测：把 h 容差放宽到 0.50、先验 h 改成实测 0.515 仍无解）。这**不是
点位错误**：若原分配直解 h>0、pitch>0（物理合法、无地下镜像），h 先验本就
不需要。此时降级用「无先验物理解」做编号核对，它同样会重新认点名，交叉
验证力仍在；且降级路径**不自动改写点名**，候选副本保持原点选，最保守。

用法
----
  python tools/diagnose_clicks.py --clicks check_out/clicks_pose_B_xxx.yaml --tag pose_B_xxx
  python tools/diagnose_clicks.py --clicks ... --tag ... --h 0.55 --h-sigma 0.12
  两种情况都会写 check_out/clicks_<tag>_candidate.yaml（编号自洽时内容与原文件
  完全一致），所以下面这条命令可以直接整行复制，不用判断要不要去掉 _candidate：
  python tools/run_task2.py clicks --clicks check_out/clicks_<tag>_candidate.yaml --tag <tag> --image <点选时同一张图>
  编号贴反时，须先对照 docs/field_measured_map.png 目视确认候选副本再跑。
"""
import argparse
import csv
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from task2 import FieldLocalizer  # noqa: E402
from task2.correspondence import ransac_pose_from_detections  # noqa: E402


def cls_of(name):
    if name.startswith(("Penal", "Penalty")):
        return "penalty_point"
    return {"L": "L_cross", "T": "T_cross", "X": "X_cross"}[name[0]]


def short(p, root):
    """尽量显示相对根目录的 posix 路径，跨盘/临时路径原样显示。"""
    try:
        return p.relative_to(root).as_posix()
    except ValueError:
        return p.as_posix()


def write_candidate(out, mapping, header):
    """写候选副本：{点名: (u,v)}。像素一律原样，只按给定点名组织。"""
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(header + yaml.safe_dump(
        {k: list(v) for k, v in mapping.items()},
        allow_unicode=True, sort_keys=False), encoding="utf-8")
    return out


def load_truth(path, tag):
    if not path.exists() or not tag:
        return None
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(ln for ln in f if not ln.startswith("#")):
            name = (row.get("视频文件") or "").rsplit(".", 1)[0]
            if name == tag:
                return {k: float(v) for k, v in row.items()
                        if k in ("x", "y", "yaw_deg", "相机高度m")}
    return None


def show_pose(tag, pose, rms_note=""):
    if pose is None:
        print(f"{tag}: 无解")
        return
    c = pose.C_F
    print(f"{tag}: rms={pose.rms_px:9.2f}px  x={c[0]:+.3f} y={c[1]:+.3f} "
          f"h={c[2]:+.3f}  yaw={pose.yaw_deg:+.1f}° pitch={pose.pitch_deg:+.1f}° {rms_note}")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--clicks", required=True, help="人工点选 yaml")
    ap.add_argument("--tag", help="位姿真值 CSV 里的视频名（不带扩展名）")
    ap.add_argument("--intrinsics", default="config/camera_intrinsics_fieldvideo.yaml")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--pose-csv", default="logs/field_pose_samples.csv")
    ap.add_argument("--h", type=float, default=0.55,
                    help="现场实测相机光心高度(m)，物理消歧用，不是位姿真值")
    ap.add_argument("--h-sigma", type=float, default=0.12)
    ap.add_argument("--px-thresh", type=float, default=30.0)
    ap.add_argument("--candidate-out", default=None,
                    help="候选修正副本，默认 check_out/clicks_<tag>_candidate.yaml")
    args = ap.parse_args()

    path = Path(args.clicks)
    if not path.is_absolute():
        path = ROOT / path
    clicks = yaml.safe_load(path.read_text(encoding="utf-8"))
    clicks = {k: tuple(v) for k, v in clicks.items()}
    print(f"点选文件: {path}   点数 {len(clicks)}")
    print("像素观测（原样，未改动）:")
    for n, uv in clicks.items():
        print(f"  {n:14s} ({uv[0]:7.1f}, {uv[1]:6.1f})")

    # 候选副本路径只在这里算一次；读入观测后立刻写「基线等价副本」。
    # 从结构上保证：下面任何分支提前 return，第三条命令都读得到文件，
    # 不再靠逐分支补写（pose_C 的「编号自洽」、pose_D2 的「h 先验无解」
    # 都是这样漏掉的）。基线副本像素/点名与原文件完全一致。
    out = (ROOT / args.candidate_out) if args.candidate_out else (
        ROOT / "check_out" / f"clicks_{args.tag or path.stem}_candidate.yaml")
    write_candidate(out, clicks,
        "# 【基线等价副本】像素观测与点名均来自本人目视点选，与原文件完全一致。\n"
        "# 在任何诊断结论之前先写出，保证「点选→诊断→解算」三步永远可整段粘贴。\n"
        "# 若诊断确认编号需修正，本文件会被「待目视确认」的修正副本覆盖。\n"
        "# 原文件未被覆盖。\n")

    third = (f"  python tools/run_task2.py clicks --clicks {short(out, ROOT)} "
             f"--tag {args.tag or '<tag>'} --image <点选时同一张图>")
    print(f"基线候选副本已写出: {out}")

    if len(clicks) < 4:
        print("\n[停止] 可见交点不足 4 个：平面 PnP 至少需 4 个不共线点，"
              "不解算、不报残差。")
        print("这是细则要求的「标志不足 → 无效状态」，不是点错；"
              "不要用理论投影或跨帧凑点。")
        print("基线等价副本已就位，直接跑第三条会输出规范的 INSUFFICIENT_MARKERS：")
        print(third)
        return 1

    loc = FieldLocalizer(ROOT / args.intrinsics, ROOT / args.field)
    print("\n=== 1. 按你贴的编号直解（clicks 现状）===")
    try:
        pose0 = loc.solve_from_clicks(clicks)
        show_pose("原分配", pose0,
                  "← 异常" if (pose0.rms_px > 100 or pose0.C_F[2] <= 0) else "← 自洽")
    except Exception as e:                                   # noqa: BLE001
        pose0 = None
        print(f"原分配求解失败: {e}")

    print("\n=== 2. 丢掉编号、只留类别，让 RANSAC 重新认点 ===")
    dets = [(cls_of(n), uv) for n, uv in clicks.items()]
    sols = {}
    for label, prior in [("无先验（会暴露镜像歧义）", None),
                         (f"带 h={args.h}m 物理先验", {"h": args.h, "h_sigma": args.h_sigma})]:
        sol = ransac_pose_from_detections(dets, loc.field_by_class,
                                          loc.K, loc.dist,
                                          px_thresh=args.px_thresh,
                                          max_combos=50000, prior=prior)
        sols[label] = sol
        if sol is None:
            print(f"{label}: 无解")
            continue
        c = sol["C_F"]
        flag = " ← 地下镜像解，物理非法" if c[2] <= 0 else " ← 物理解"
        print(f"{label}: rms={sol['rms']:.2f}px  内点 {sol['n_inlier']}/{len(dets)}  "
              f"x={c[0]:+.3f} y={c[1]:+.3f} h={c[2]:+.3f} "
              f"yaw={sol['yaw_deg']:+.1f}° pitch={sol['pitch_deg']:+.1f}°{flag}")

    physical = sols.get(f"带 h={args.h}m 物理先验")
    no_prior = sols.get("无先验（会暴露镜像歧义）")
    degraded = False
    if physical is None:
        # h 先验支无解 != 点位错。原分配已是物理合法解（h>0）时没有镜像歧义，
        # h 先验本就不需要；降级用无先验物理解核对编号，且降级不自动改点名。
        physical_ok = (no_prior is not None and no_prior["C_F"][2] > 0)
        origin_ok = (pose0 is not None and pose0.C_F[2] > 0)
        if physical_ok and origin_ok:
            degraded = True
            print("\n[降级] 带 h 先验那支无解：4 点 P3P 假设阶段被 pen>3.0 硬剔除全部枪毙")
            print("  （实测 h 容差放宽到 0.50、先验 h 改成 0.515 仍无解，与容差无关）。")
            print("  但原分配直解 h>0、pitch>0，已是物理合法解、无地下镜像歧义；")
            print("  h 先验只在撞镜像解时才必需。编号核对降级用「无先验物理解」")
            print("  （同样重新认点名，交叉验证力仍在）；降级路径不自动改写点名。")
            physical = no_prior
        else:
            print("\n[停止] 带物理先验无解，且无先验支也没有物理解 / 原分配非法：")
            print("  请检查点是否真的是白线交点，或换一帧重点。")
            print("  基线等价副本已就位，第三条仍可跑（原点选编号直解 / 无效状态）：")
            print(third)
            return 1

    names = list(physical["names"])
    print("\n=== 3. 逐点编号核对（物理解给出的建议）===")
    changed = []
    for (orig, uv), suggest, err in zip(clicks.items(), names, physical["errs"]):
        same = orig == suggest
        if not same:
            changed.append((orig, suggest))
        print(f"  你叫 {orig:14s} @({uv[0]:7.1f},{uv[1]:6.1f})  "
              f"-> 几何更像 {suggest:14s} err={err:6.1f}px  {'✓' if same else '✗ 建议改'}")

    # 只有「h 先验支正常出解 + 建议点名无重复」才自动应用编号建议；
    # 降级路径（先验支无解）一律保持原点选编号，最保守，不拿可信度较低的
    # RANSAC 建议覆盖本人目视结果。
    auto_apply = (not degraded) and (len(set(names)) == len(names))
    if degraded and changed:
        print("  [注] 降级路径发现编号建议，但未自动写入候选副本；"
              "如确认贴反请本人目视核对后手工改原点选文件。")
    if not auto_apply and not degraded:
        print("  [警告] 建议分配里出现重复点名，几何不自洽，不自动应用，保持原点选。")

    candidate = ({suggest: clicks[orig] for orig, suggest in zip(clicks, names)}
                 if auto_apply else dict(clicks))
    pose1 = loc.solve_from_clicks(candidate)
    print("\n=== 4. 最终候选重解（像素完全不变）===")
    if not auto_apply:
        print("（未自动应用 RANSAC 点名建议，候选编号 = 原点选编号）")
    show_pose("候选分配", pose1)

    truth = load_truth(ROOT / args.pose_csv, args.tag)
    if truth:
        import numpy as np
        dx, dy = pose1.C_F[0] - truth["x"], pose1.C_F[1] - truth["y"]
        print(f"  参考位姿 x={truth['x']:+.3f} y={truth['y']:+.3f} "
              f"h={truth['相机高度m']:.3f} yaw={truth['yaw_deg']:+.1f}°")
        print(f"  平面位置误差 {float(np.hypot(dx, dy))*100:.1f} cm | "
              f"航向误差 {abs(pose1.yaw_deg - truth['yaw_deg']):.1f}° | "
              f"高度误差 {abs(pose1.C_F[2]-truth['相机高度m'])*100:.1f} cm")

    print("\n=== 5. 结论 ===")
    if degraded:
        print("· 带 h 先验支无解（假设阶段 3σ 硬剔除），已降级用无先验物理解核对编号；")
        print("  原分配直解 h>0 为物理合法解，本组数据有效，不以 h 先验无解否决。")
    if not changed:
        print("· 编号全部自洽，像素观测与点名均无需修改；"
              "原分配若异常，多半是撞上地下镜像解，以物理解位姿为准。")
    else:
        print(f"· 发现 {len(changed)} 处编号可能贴反："
              f"{', '.join(f'{a}→{b}' for a,b in changed)}")
        if not auto_apply:
            print("· 未自动写入候选副本（降级 / 建议重复），请本人目视确认后手工处理。")

    # 用最终结论覆盖基线副本：等价副本像素/点名与原文件一致；
    # 修正副本只改点名、像素不变，并标注「待本人目视确认」。
    equiv = (not changed) or (not auto_apply)
    if equiv:
        basis = ("无先验物理解（h 先验支降级）" if degraded
                 else f"类别级 RANSAC + h={args.h}m 物理先验")
        header = (
            "# 【候选副本·最终】编号核对后像素观测与点名均与原文件一致，未改动。\n"
            f"# 依据：{basis}，rms={pose1.rms_px:.2f}px。\n"
            "# 本副本内容（像素 + 点名）与原点选文件完全一致，仅用于统一流程，\n"
            "# 避免「没有候选文件」导致下一条命令报 FileNotFoundError。\n"
            "# 原文件未被覆盖。\n"
        )
    else:
        header = (
            "# 【候选修正，待本人目视确认】像素观测与原文件完全一致，只改点名。\n"
            f"# 依据：类别级 RANSAC + h={args.h}m 物理先验，rms={pose1.rms_px:.2f}px。\n"
            f"# 改动：{', '.join(f'{a} -> {b}' for a, b in changed)}\n"
            "# 确认方法：对照 docs/field_measured_map.png 逐个核对交点身份。\n"
            "# 未确认前不得作为正式观测。原文件未被覆盖。\n"
        )
    write_candidate(out, candidate, header)
    print(f"候选副本已写出（原文件未动）: {out}")
    if not equiv:
        print("请对照 docs/field_measured_map.png 目视确认后，再用：")
    else:
        print("编号无需确认，可直接跑（与用原 .yaml 结果完全相同）：")
    print(third)
    print("\n红线：像素只能来自你目视点选；真值只用于打分，不参与解算；"
          "RANSAC 建议必须人工复核，不得直接当观测。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
