"""【已作废 / DEPRECATED — 2026-11-20】理论投影参考图生成器。

⚠️ 用户 2026-11-20 实测反馈：生成的红圈点「一个都不对」。扫描确认：
   换 fx（1316→2400）远处 L/T 点始终落在 v≈205~283 的墙面区下不到地面，
   换 pitch 只是整片上下平移，仍对不上。根因是 field_pose_samples.csv 的
   位姿真值本身由同一套有争议内参反解而来，正向投影把误差二次放大。
   **本脚本输出的红圈绝不可用于找点、更不可当观测值。请改用
   tools/pick_points.py 人工点选真实白线交点。** 保留仅为留档，不要引用。

以下为原始说明（历史记录，勿照做）
----
任务二 README 必填「≥3 个测试点的参考位姿 vs 估计位姿误差」（细则 634/749 行）。
`tools/run_task2.py clicks` 已能解位姿并出误差表，唯一缺的是人工在真实帧上
认出 >=4 个标志交点、给出像素坐标。盲找很费劲，尤其窄视场里 L/T/X 长得像。

本脚本用 CSV 里的**参考位姿真值** + 外参 pitch，把 field.yaml 的 15 个标志点
投影到对应 pose 帧上，画出「理论上该点大概在画面哪里」，作为找点示意。

⚠️ 学术诚信红线（务必看）
------------------------
投影值来自真值位姿，**绝不能直接拿它当点选观测喂给 clicks 求误差**——
那等于用真值自证、误差恒为 0，属于伪造定位结果，考核一票否决。
正确用法：打开参考图 → 目视找到白线真实交点 → 用那个**实际像素**填 --px。
理论点与真实交点的偏差（pose_C 备注提到远处偏左 ~150px）恰恰是要记录的
真实误差来源，不要去「对齐」它。

只写文件、不删除任何东西、不碰 GitHub、不改动 dataset/。

用法
----
  python tools/gen_pose_guides.py
  python tools/gen_pose_guides.py --pitch 10        # 全局覆盖俯仰角
  python tools/gen_pose_guides.py --frame-suffix _f000150
"""
import argparse
import csv
import re
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from task2 import FieldLocalizer, camera_R, project_field_points  # noqa: E402


def read_poses(csv_path):
    """返回 [{tag,x,y,yaw_deg,h,note}]，跳过 # 注释行。"""
    rows = []
    # 跳过 # 注释行后再交给 DictReader（表头是第一行非注释行）
    with open(csv_path, encoding="utf-8-sig") as f:
        lines = [ln for ln in f if not ln.lstrip().startswith("#")]
    for row in csv.DictReader(lines):
        tag = (row.get("视频文件") or "").rsplit(".", 1)[0]
        if not tag:
            continue
        rows.append(dict(
            tag=tag,
            x=float(row["x"]), y=float(row["y"]),
            yaw_deg=float(row["yaw_deg"]), h=float(row["相机高度m"]),
            note=row.get("备注", "") or "",
        ))
    return rows


def pitch_of(note, default):
    """备注里若写了透视反算的 pitch（如 'pitch=7.8'）优先用它，否则用外参。"""
    m = re.search(r"pitch\s*=\s*([-\d.]+)", note)
    return float(m.group(1)) if m else default


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pose-csv", default="logs/field_pose_samples.csv")
    ap.add_argument("--frames", default="frames_pool_markers")
    ap.add_argument("--out", default="check_out/pose_guides")
    ap.add_argument("--intrinsics", default="config/camera_intrinsics_fieldvideo.yaml")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--extrinsics", default="calibration/camera_extrinsics.yaml")
    ap.add_argument("--pitch", type=float, default=None,
                    help="全局覆盖 pitch（度）；不给则用备注反算值，再退回外参")
    ap.add_argument("--frame-suffix", default="_f000000",
                    help="每段 pose 选哪张帧，默认第一张")
    args = ap.parse_args()

    ext = yaml.safe_load((ROOT / args.extrinsics).read_text(encoding="utf-8"))
    pitch_default = float(ext["install"]["pitch_deg"])

    loc = FieldLocalizer(args.intrinsics, args.field)
    K, dist = loc.K, loc.dist
    poses = read_poses(ROOT / args.pose_csv)
    out_dir = ROOT / args.out
    out_dir.mkdir(parents=True, exist_ok=True)
    frames_dir = ROOT / args.frames

    print(f"内参 fx={K[0,0]:.0f}  外参 pitch={pitch_default:.2f}°  "
          f"标志点 {len(loc.field)} 个  pose 位姿 {len(poses)} 组\n")

    for p in poses:
        # 找该 pose 的代表帧
        cands = sorted(frames_dir.glob(f"{p['tag']}*{args.frame_suffix}.jpg"))
        if not cands:
            cands = sorted(frames_dir.glob(f"{p['tag']}*.jpg"))
        if not cands:
            print(f"[{p['tag']}] 找不到帧，跳过")
            continue
        img_path = cands[0]
        frame = cv2.imread(str(img_path))
        if frame is None:
            print(f"[{p['tag']}] 读不了 {img_path.name}，跳过")
            continue
        H, W = frame.shape[:2]

        pitch = args.pitch if args.pitch is not None else pitch_of(p["note"], pitch_default)
        R_CF = camera_R(p["yaw_deg"], pitch)
        C_F = np.array([p["x"], p["y"], p["h"]])
        uv_map = project_field_points(K, dist, R_CF, C_F, list(loc.field.items()))

        vis = frame.copy()
        inside = []
        for name, uv in uv_map.items():
            u, v = float(uv[0]), float(uv[1])
            if not (5 < u < W - 5 and 5 < v < H - 5):
                continue
            # 只保留相机前方的点（背后的点投影是镜像，无意义）
            P = np.array([loc.field[name][0], loc.field[name][1], 0.0])
            if float(np.dot(R_CF[2], P - C_F)) <= 0.2:
                continue
            inside.append((name, u, v))
            cv2.circle(vis, (round(u), round(v)), 9, (0, 0, 255), 2)
            cv2.circle(vis, (round(u), round(v)), 2, (0, 255, 255), -1)
            cv2.putText(vis, name, (round(u) + 12, round(v) - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)

        cv2.putText(vis, f"{p['tag']}  pitch={pitch:.1f}deg  "
                         f"RED=theoretical projection (FIND REAL INTERSECTION, do NOT copy)",
                    (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 0, 255), 1, cv2.LINE_AA)
        out_img = out_dir / f"guide_{p['tag']}.jpg"
        cv2.imwrite(str(out_img), vis)

        # 参考清单：列出理论位置，供人工对照（明确标注不可直接当观测）
        txt = out_dir / f"guide_{p['tag']}.txt"
        with open(txt, "w", encoding="utf-8") as f:
            f.write(f"# {p['tag']}  参考位姿 x={p['x']} y={p['y']} "
                    f"yaw={p['yaw_deg']} h={p['h']}  pitch={pitch}\n")
            f.write("# 下表是【真值位姿投影的理论点】，只用来告诉你要去画面哪里找白线交点。\n")
            f.write("# 严禁直接拷贝到 --px（= 用真值自证，误差恒为 0，考核一票否决）。\n")
            f.write("# 请目视确认真实交点像素，再填 run_task2.py clicks --px。\n")
            f.write(f"# 帧: {img_path.name}\n\n")
            f.write(f"{'name':16s} {'u_theory':>10s} {'v_theory':>10s}   u_real  v_real\n")
            for name, u, v in inside:
                f.write(f"{name:16s} {u:10.1f} {v:10.1f}   ____   ____\n")

        print(f"[{p['tag']}] 帧 {img_path.name}  pitch={pitch:.1f}°  "
              f"画面内理论点 {len(inside)} 个 -> {out_img.name}")
        for name, u, v in inside:
            print(f"    {name:14s} ({u:7.1f}, {v:7.1f})")
        print()

    print("下一步：打开 check_out/pose_guides/guide_*.jpg，目视找白线真实交点，")
    print("把 >=4 个点的实际像素填进：")
    print('  python tools/run_task2.py clicks --px "LCross1:u,v;TCross3:u,v;..." '
          '--tag pose_A_20261112_172518 --image frames_pool_markers/<对应帧>.jpg')
    return 0


if __name__ == "__main__":
    sys.exit(main())
