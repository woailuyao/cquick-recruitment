"""任务二定位流水线入口。

用法：
  # 1. 自检（不需要模型、不需要标注，验证几何/解算数学）
  python tools/run_task2.py selftest

  # 2. 点选模式：人工在帧上认出 >=4 个 field.yaml 标志点，直接解位姿
  python tools/run_task2.py clicks --video raw_videos/pose_A_20261112_172518.mp4 --px "LCross1:123,456;TCross3:700,200;LCross5:40,600;TCross4:900,300" --tag pose_A_20261112_172518
  #    tag 对应位姿真值 CSV 里的视频名（不带扩展名）

  # 3. 视频模式：标志模型 + 球模型 -> 逐帧位姿 + 球世界坐标 + 可视化
  python tools/run_task2.py video --video raw_videos/ball_run02_xxx.mp4 --model runs/detect/markers/weights/best.pt --ball-model runs/detect/train/weights/best.pt

  # 注：以上均为 PowerShell 单行命令，整行复制，末尾不要加反斜杠续行符。

内参默认 config/camera_intrinsics_fieldvideo.yaml（场地素材 fx=1620 族），
场地默认 config/field.yaml，位姿真值默认 logs/field_pose_samples.csv。
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from task2 import (FieldLocalizer, camera_R, load_field, load_intrinsics,  # noqa: E402
                   project_field_points)


# ----------------------------------------------------------------------
def load_pose_csv(path, tag):
    """从位姿真值 CSV 里找 tag（不含扩展名）。返回 dict 或 None。"""
    if not Path(path).exists():
        return None
    with open(path, encoding="utf-8-sig") as f:
        for row in csv.DictReader(line for line in f if not line.startswith("#")):
            name = (row.get("视频文件") or "").rsplit(".", 1)[0]
            if name == tag:
                return {k: float(v) for k, v in row.items()
                        if k in ("x", "y", "yaw_deg", "相机高度m")}
    return None


def compare_pose(pose, truth):
    """自证误差表：返回多行文本。"""
    if truth is None:
        return ["（未找到该 tag 的位姿真值，跳过对比）"]
    dx, dy = pose.C_F[0] - truth["x"], pose.C_F[1] - truth["y"]
    lines = [
        "── C0 自证误差表 ──────────────────────────────",
        f"参考位姿   x={truth['x']:+.3f}  y={truth['y']:+.3f}  h={truth['相机高度m']:.3f}  yaw={truth['yaw_deg']:+.1f}°",
        f"估计位姿   x={pose.C_F[0]:+.3f}  y={pose.C_F[1]:+.3f}  h={pose.C_F[2]:.3f}  yaw={pose.yaw_deg:+.1f}°  pitch={pose.pitch_deg:+.1f}°",
        f"平面位置误差 = {float(np.hypot(dx, dy)) * 100:.1f} cm",
        f"航向误差     = {abs(pose.yaw_deg - truth['yaw_deg']):.1f} °",
        f"高度误差     = {abs(pose.C_F[2] - truth['相机高度m']) * 100:.1f} cm",
    ]
    return lines


# ----------------------------------------------------------------------
def cmd_selftest(args):
    loc = FieldLocalizer(args.intrinsics, args.field)
    K, dist = loc.K, loc.dist
    rng = np.random.default_rng(0)

    # 用几组贴近实际机位的虚拟位姿测试（窄视场下 <4 个点属正常，跳过）
    cases = [
        dict(x=-2.225, y=1.78, h=0.55, yaw=-25.5, pitch=8),    # pose_B 同款
        dict(x=-3.085, y=-2.05, h=0.55, yaw=18.9, pitch=11.6), # pose_C 同款
        dict(x=0.055, y=-1.72, h=0.55, yaw=0.5, pitch=8),      # pose_A 同款
        dict(x=-6.5, y=0.0, h=1.2, yaw=0, pitch=15),           # 高机位全景（数学体检用）
        dict(x=5.5, y=5.0, h=1.5, yaw=215, pitch=20),           # 角外斜视
    ]
    all_pass, tested = True, 0
    for ci, c in enumerate(cases):
        R_CF = camera_R(c["yaw"], c["pitch"])
        C_F = np.array([c["x"], c["y"], c["h"]])
        uv_map = project_field_points(K, dist, R_CF, C_F, list(loc.field.items()))
        # 只保留「落在画面内 且 在相机前方」的点（背后的点投影是镜像，无意义）
        inside = {}
        for n, uv in uv_map.items():
            if not (5 < uv[0] < 1275 and 5 < uv[1] < 715):
                continue
            P = np.array([loc.field[n][0], loc.field[n][1], 0.0])
            z_cam = float(np.dot(R_CF[2], P - C_F))
            if z_cam > 0.2:
                inside[n] = uv
        if len(inside) < 4:
            print(f"[case{ci}] 画面内标志点 {len(inside)} 个 <4（窄视场正常），跳过")
            continue
        tested += 1

        # ---- 测试 A：clicks 直接 PnP（无噪声）----
        names = list(inside)[:8]
        clicks = {n: inside[n] for n in names}
        poseA = loc.solve_from_clicks(clicks)
        eA = (np.hypot(poseA.C_F[0] - c["x"], poseA.C_F[1] - c["y"]),
              abs(poseA.C_F[2] - c["h"]),
              abs(poseA.yaw_deg - c["yaw"]),
              poseA.rms_px)

        # ---- 测试 B：类别检测 + 2px 噪声走 RANSAC ----
        cls_of = {"L": "L_cross", "T": "T_cross", "X": "X_cross"}
        dets = []
        for n, uv in inside.items():
            cls_name = ("penalty_point" if n.startswith("Penal")
                        else cls_of[n[0]])
            dets.append((cls_name, (uv[0] + rng.normal(0, 2),
                                    uv[1] + rng.normal(0, 2))))
        sol = None
        from task2.correspondence import ransac_pose_from_detections
        sol = ransac_pose_from_detections(dets, loc.field_by_class,
                                          K, dist, px_thresh=args.px_thresh)
        if sol is None:
            print(f"[case{ci}] RANSAC 失败（{len(dets)} 个检测）")
            all_pass = False
            continue
        eB = (float(np.hypot(sol["C_F"][0] - c["x"], sol["C_F"][1] - c["y"])),
              abs(sol["C_F"][2] - c["h"]), abs(sol["yaw_deg"] - c["yaw"]),
              sol["rms"])

        # ---- 测试 C：地面点反投影闭环（只测投影落在画面内的点）----
        from task2.geometry import pixel_to_ground
        pts, errs = [], []
        for (gx, gy) in [(0.5, 0.3), (-1.2, 1.0), (2.0, -0.5),
                         (-0.5, -0.8), (1.5, 1.2)]:
            uv = project_field_points(K, dist, R_CF, C_F,
                                      [("t", np.array([gx, gy]))])["t"]
            if not (5 < uv[0] < 1275 and 5 < uv[1] < 715):
                continue                       # 视场外不属于反投影定义域
            w = pixel_to_ground(R_CF, C_F, K, dist, uv)
            if w is None:
                errs.append(9.99)
            else:
                errs.append(float(np.hypot(w[0] - gx, w[1] - gy)))
        if not errs:
            print(f"[case{ci}] C(反投影): 视场内无测试点，跳过")
            continue
        err_c = max(errs)

        ok = eA[0] < 0.05 and eB[0] < 0.15 and err_c < 0.02
        all_pass &= ok
        print(f"[case{ci}] 点数={len(inside):2d}  "
              f"A(点选): 位置{eA[0]*100:5.1f}cm 高{eA[1]*100:4.1f}cm "
              f"yaw{eA[2]:4.1f}° rms{eA[3]:.2f}px | "
              f"B(RANSAC): 位置{eB[0]*100:5.1f}cm 高{eB[1]*100:4.1f}cm "
              f"yaw{eB[2]:4.1f}° rms{eB[3]:.2f}px 内点{sol['n_inlier']}/{len(dets)} | "
              f"C(反投影): {err_c*100:4.1f}cm  "
              f"{'✓' if ok else '✗✗✗ FAIL'}")

    if tested == 0:
        print("没有任何 case 完成测试（不应发生）")
        return 1
    print(f"\nselftest: {tested} 组全部通过 ✅" if all_pass
          else "selftest: 存在失败项 ❌（见上）")
    return 0 if all_pass else 1


# ----------------------------------------------------------------------
def get_frame(args):
    if args.video:
        cap = cv2.VideoCapture(str(ROOT / args.video))
        n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        cap.set(cv2.CAP_PROP_POS_FRAMES, n // 2)
        ok, frame = cap.read()
        cap.release()
        if not ok:
            raise SystemExit("视频读不了帧")
        return frame
    if args.image:
        frame = cv2.imread(str(ROOT / args.image))
        if frame is None:
            raise SystemExit("图片读不了")
        return frame
    return None


def cmd_clicks(args):
    loc = FieldLocalizer(args.intrinsics, args.field, px_thresh=args.px_thresh)

    if args.clicks:
        import yaml
        clicks = yaml.safe_load(Path(args.clicks).read_text(encoding="utf-8"))
        clicks = {k: tuple(v) for k, v in clicks.items()}
    elif args.px:
        clicks = {}
        for part in args.px.split(";"):
            name, uv = part.split(":")
            u, v = uv.split(",")
            clicks[name.strip()] = (float(u), float(v))
    else:
        raise SystemExit("需要 --px \"点名:u,v;...\" 或 --clicks xxx.yaml")

    unknown = [n for n in clicks if n not in loc.field]
    if unknown:
        raise SystemExit(f"field.yaml 里没有这些点名：{unknown}")
    if len(clicks) < 4:
        # 细则 635/646 行：定位失败时正确输出无效状态，不伪造位姿。
        # 平面 PnP 至少需 4 个不共线点；不足则不解算、不报残差、不输出 x/y/yaw。
        print("── 定位状态 ──────────────────────────────")
        print(f"marker_count   : {len(clicks)}")
        print("residual       : --   （不足 4 点，不解算、不报残差）")
        print("valid          : FALSE")
        print("status         : INSUFFICIENT_MARKERS")
        print("x_camera_map   : --")
        print("y_camera_map   : --")
        print("yaw_camera_map : --")
        print("观测点（本人目视点选的真实像素，未补任何点）：")
        for n, uv in clicks.items():
            print(f"  {n:14s} ({uv[0]:7.1f}, {uv[1]:7.1f})")
        print("说明：绝不跨帧凑点、绝不猜坐标、绝不拿理论投影凑数；"
              "本组作为「标志不足 → 无效状态」证据保留。")
        return 0

    h_prior = args.h_prior if (args.h_prior and args.h_prior > 0) else None
    pose = loc.solve_from_clicks(clicks, prefer_h=h_prior)
    if h_prior is not None:
        print(f"物理消歧：prefer_h={h_prior:.2f} m（现场实测光心高度，非位姿真值）"
              f" -> 解出 h={pose.C_F[2]:+.3f} m")
        if pose.C_F[2] <= 0:
            print("[警告] 所有 IPPE 解都在地面以下，请检查点名是否贴错"
                  "（跑 tools/diagnose_clicks.py）或内参是否匹配。")
    frame = get_frame(args)
    print(f"解算点数 {len(clicks)}  重投影 rms = {pose.rms_px:.2f} px")
    print(f"相机位姿（C0）：{pose.pos_str} | {pose.ori_str}")
    print("各点误差：")
    for n, e in zip(pose.names, pose.errs_px):
        print(f"  {n:14s} {e:5.2f} px")

    truth = load_pose_csv(args.pose_csv, args.tag) if args.tag else None
    for line in compare_pose(pose, truth):
        print(line)

    if frame is not None:
        vis = loc.draw(frame, pose,
                       refined=[(n, clicks[n], "manual") for n in pose.names])
        out = ROOT / "check_out" / f"task2_clicks_{args.tag or 'noname'}.jpg"
        cv2.imwrite(str(out), vis)
        print(f"\n可视化 -> {out}")


# ----------------------------------------------------------------------
def cmd_video(args):
    try:
        from ultralytics import YOLO
    except ImportError:
        raise SystemExit("先装 ultralytics：pip install ultralytics")

    loc = FieldLocalizer(args.intrinsics, args.field, px_thresh=args.px_thresh)
    marker_model = YOLO(args.model)
    ball_model = YOLO(args.ball_model) if args.ball_model else None

    video = ROOT / args.video
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频：{video}")
    fps = cap.get(cv2.CAP_PROP_FPS) or 25
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    out_dir = ROOT / "check_out" / f"task2_{video.stem}"
    out_dir.mkdir(parents=True, exist_ok=True)
    writer = cv2.VideoWriter(str(out_dir / "overlay.mp4"),
                             cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    csv_path = out_dir / "world_coords.csv"
    csv_f = open(csv_path, "w", newline="", encoding="utf-8")
    csv_w = csv.writer(csv_f)
    csv_w.writerow(["frame", "time_s", "cam_x", "cam_y", "cam_h", "yaw_deg",
                     "pitch_deg", "rms_px", "ball_u", "ball_v",
                     "ball_world_x", "ball_world_y"])

    i, pose = -1, None
    poses_for_median = []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        i += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detections = []
        if i % args.pose_every == 0:
            for r in marker_model(frame, verbose=False, conf=args.conf)[0]:
                cls_name = marker_model.names[int(r.boxes.cls)]
                x1, y1, x2, y2 = r.boxes.xyxy[0].tolist()
                detections.append({"cls": cls_name, "box": [x1, y1, x2, y2],
                                   "conf": float(r.boxes.conf)})
            if len(detections) >= 3:
                prior = {}
                if args.h_prior:
                    prior["h"] = args.h_prior
                    prior["h_sigma"] = args.h_sigma
                if args.pitch_prior is not None:
                    prior["pitch_deg"] = args.pitch_prior
                    prior["pitch_sigma"] = args.pitch_sigma
                new_pose, refined = loc.solve_from_detections(
                    gray, detections, prior=(prior or None))
                if new_pose is not None:
                    pose = new_pose
                    poses_for_median.append(
                        [new_pose.C_F[0], new_pose.C_F[1], new_pose.C_F[2],
                         new_pose.yaw_deg, new_pose.pitch_deg])

        ball, ball_world = None, None
        if ball_model is not None and pose is not None:
            for r in ball_model(frame, verbose=False, conf=args.conf)[0]:
                x1, y1, x2, y2 = r.boxes.xyxy[0].tolist()
                ball = {"box": [x1, y1, x2, y2], "conf": float(r.boxes.conf)}
                uv = ((x1 + x2) / 2, (y1 + y2) / 2)
                ball_world = loc.pixel_to_world(pose, uv,
                                                ball_radius=args.ball_radius)
                break                                   # 只跟踪一个球

        csv_w.writerow([i, f"{i/fps:.2f}",
                        *( [f"{pose.C_F[0]:.3f}", f"{pose.C_F[1]:.3f}",
                            f"{pose.C_F[2]:.3f}", f"{pose.yaw_deg:.2f}",
                            f"{pose.pitch_deg:.2f}", f"{pose.rms_px:.2f}"]
                           if pose else [""] * 6),
                        *([f"{(ball['box'][0]+ball['box'][2])/2:.1f}",
                           f"{(ball['box'][1]+ball['box'][3])/2:.1f}"]
                          if ball else ["", ""]),
                        *([f"{ball_world[0]:.3f}", f"{ball_world[1]:.3f}"]
                          if ball_world is not None else ["", ""])])
        writer.write(loc.draw(frame, pose, None, ball, ball_world))

    cap.release()
    writer.release()
    csv_f.close()

    print(f"帧数 {i+1}  位姿成功率样本 {len(poses_for_median)} 帧")
    if poses_for_median:
        med = np.median(np.array(poses_for_median), axis=0)
        print(f"位姿中位数：x={med[0]:+.3f} y={med[1]:+.3f} h={med[2]:.3f} "
              f"yaw={med[3]:+.1f}° pitch={med[4]:+.1f}°")
        if args.tag:
            truth = load_pose_csv(args.pose_csv, args.tag)
            print("── 对比位姿真值 ──")
            class _P:
                C_F = np.array([med[0], med[1], med[2]])
                yaw_deg, pitch_deg = med[3], med[4]
            for line in compare_pose(_P, truth):
                print(line)
    print(f"\n输出：{out_dir / 'overlay.mp4'}\nCSV ：{csv_path}")


# ----------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--intrinsics", default="config/camera_intrinsics_fieldvideo.yaml")
    common.add_argument("--field", default="config/field.yaml")
    common.add_argument("--px-thresh", type=float, default=10.0)
    common.add_argument("--pose-csv", default="logs/field_pose_samples.csv")
    common.add_argument("--tag", help="位姿真值 CSV 里的视频名（不带扩展名）")

    s = sub.add_parser("selftest", parents=[common])
    c = sub.add_parser("clicks", parents=[common])
    c.add_argument("--px", help='点选串 "LCross1:123,456;TCross3:..."')
    c.add_argument("--clicks", help="点选 yaml 文件（{点名: [u,v]}）")
    c.add_argument("--video")
    c.add_argument("--image")
    c.add_argument("--h-prior", type=float, default=0.55,
                   help="现场实测相机光心高度(m)，用于消除平面 PnP 地下镜像解；"
                        "设 0 可关闭（仅调试）")

    v = sub.add_parser("video", parents=[common])
    v.add_argument("--video", required=True)
    v.add_argument("--model", required=True, help="标志检测模型 .pt")
    v.add_argument("--ball-model", help="球检测模型 .pt（可选）")
    v.add_argument("--conf", type=float, default=0.4)
    v.add_argument("--pose-every", type=int, default=10, help="每 N 帧解一次位姿")
    v.add_argument("--ball-radius", type=float, default=0.0,
                   help="球半径(m)，>0 时把球心投影换算成地面接触点")
    v.add_argument("--h-prior", type=float,
                   help="光心高度先验（米，现场卷尺量的 h）")
    v.add_argument("--h-sigma", type=float, default=0.08)
    v.add_argument("--pitch-prior", type=float,
                   help="俯仰角先验（度，可选）")
    v.add_argument("--pitch-sigma", type=float, default=8.0)

    args = ap.parse_args()
    if args.cmd == "selftest":
        sys.exit(cmd_selftest(args))
    if args.cmd == "clicks":
        cmd_clicks(args)
    elif args.cmd == "video":
        cmd_video(args)


if __name__ == "__main__":
    main()
