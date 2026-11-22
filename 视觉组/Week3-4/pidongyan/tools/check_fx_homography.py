"""fx 呼吸效应诊断工具：用场地已知共面点 + 单应/PnP 扫描求真实 fx。

背景（2026-11-12 发现的技术债）：
  近距棋盘格标定 fx=2031，远距棋盘格 fx=1888，pose_B/C 参照物反推等效 fx≈1600~1750，
  三个数互相矛盾（M12 镜头对焦呼吸）。任务二远处定位误差直接受 fx 影响，必须裁决。

原理：
  场地 15 个标志全部位于 Z=0 平面，坐标已用卷尺实测（config/field.yaml）。
  在一帧画面里点出 ≥6 个标志的像素位置后：
    已知量：物点 (x,y,0)、主点 cx/cy、畸变系数
    未知量：fx=fy=f、相机位姿 R,t
  平面单应只有 8 个自由度，f(1) + 位姿(6) = 7 个未知量 → 数据超定，
  对 f 做网格扫描，每个 f 用 solvePnP 解位姿并算重投影 RMS，
  RMS 曲线的极小值位置就是该距离下的真实 fx。

用法：
  交互点选（推荐，鼠标左键按顺序点，u 撤销，回车/esc 结束）：
    python tools/check_fx_homography.py --video raw_videos/pose_A_xxx.mp4 --points LCross1 LCross2 TCross3
  复用已保存的点选结果：
    python tools/check_fx_homography.py --clicks calibration/fx_clicks/pose_A.yaml
  直接给像素坐标（脚本/自动化用）：
    python tools/check_fx_homography.py --video xxx.mp4 --px "LCross1:640,500;TCross3:900,600"

输出：
  - f-RMS 扫描曲线（终端表格 + check_out/fx_scan_<tag>.png）
  - 最优 fx 及其在 1600/1750/1888/2031 四个候选值下的 RMS 对比
  - 最优 fx 下的相机位姿（x,y,z,yaw,pitch），与 logs/field_pose_samples.csv 真值对照
  - 重投影叠加图 check_out/fx_check_<tag>.jpg
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]

CANDIDATE_F = [1600.0, 1750.0, 1888.0, 2031.0]


def load_field_points(path):
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    # 跳过注释键、别名映射：LCross1..8 / TCross1..4 / XCross1..2 / PenalityPoint
    alias = {"PenalityPoint": "PenaltyPoint"}
    pts = {}
    for k, v in d.items():
        if k.startswith("#") or not isinstance(v, (list, tuple)) or len(v) < 2:
            continue
        pts[alias.get(k, k)] = (float(v[0]), float(v[1]))
    return pts


def load_intrinsics(path):
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    dist = np.array(d["distortion_coefficients"]["data"], dtype=np.float64).reshape(-1)
    return K, dist


def grab_frame(video, pos="mid"):
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        sys.exit(f"错误：打不开视频 {video}")
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if pos == "mid":
        cap.set(cv2.CAP_PROP_POS_FRAMES, n // 2)
    ok, f = cap.read()
    cap.release()
    if not ok:
        sys.exit("错误：取帧失败")
    return f


def click_points(frame, names):
    """交互点选：按 names 顺序点击。返回 {name: (u,v)}"""
    clicks = {}
    order = []
    idx = [0]
    disp = frame.copy()

    def redraw():
        nonlocal disp
        disp = frame.copy()
        for i, nm in enumerate(order):
            cv2.circle(disp, clicks[nm], 5, (0, 0, 255), 2)
            cv2.putText(disp, nm, (clicks[nm][0] + 8, clicks[nm][1] - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cur = names[idx[0]] if idx[0] < len(names) else "完成：回车结束"
        cv2.putText(disp, f"click: {cur}   u=undo  enter=done", (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and idx[0] < len(names):
            nm = names[idx[0]]
            clicks[nm] = (x, y)
            order.append(nm)
            idx[0] += 1
            redraw()

    redraw()
    win = "click field points"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, on_mouse)
    while True:
        cv2.imshow(win, disp)
        k = cv2.waitKey(30) & 0xFF
        if k == ord("u") and order:
            nm = order.pop()
            clicks.pop(nm)
            idx[0] -= 1
            redraw()
        elif k in (13, 27, ord("q")):
            break
    cv2.destroyWindow(win)
    return clicks


def scan_f(obj_pts, px_raw, dist, cx, cy, f_grid):
    """对每个 f：K(f) + 原始畸变像素直接 solvePnP（畸变系数按归一化坐标模型保持不变），
    重投影 RMS 最小的 f 即真实 fx。返回 [(f, rms, rvec, tvec)]"""
    obj = obj_pts.reshape(-1, 1, 3).astype(np.float64)
    img = px_raw.reshape(-1, 1, 2).astype(np.float64)
    rows = []
    for f in f_grid:
        K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            continue
        proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
        err = np.linalg.norm(proj.reshape(-1, 2) - img.reshape(-1, 2), axis=1)
        rows.append((f, float(np.sqrt((err ** 2).mean())), rvec, tvec))
    return rows


def pose_from(rvec, tvec):
    """R 为 OpenCV 相机系（X右 Y下 Z前）。返回光心位置、光轴的 yaw/pitch（场地系）"""
    R, _ = cv2.Rodrigues(rvec)
    C = -R.T @ tvec.reshape(3)                 # 相机光心在场地系（与约定无关）
    fwd = R[2, :]                              # 相机光轴在场地系方向 = R 第三行
    yaw = np.degrees(np.arctan2(fwd[1], fwd[0]))
    pitch = np.degrees(np.arcsin(-fwd[2] / np.linalg.norm(fwd)))  # 向下为正
    return C, yaw, pitch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", help="视频文件（默认取中段帧）")
    ap.add_argument("--image", help="直接用图片")
    ap.add_argument("--frame-idx", type=int, default=None, help="取指定帧（默认中间帧）")
    ap.add_argument("--points", nargs="*", help="交互模式：按此顺序点选的标志名")
    ap.add_argument("--px", help='直接指定像素："LCross1:640,500;TCross3:900,600"')
    ap.add_argument("--clicks", help="复用已保存的点选 yaml")
    ap.add_argument("--field", default=str(ROOT / "config" / "field.yaml"))
    ap.add_argument("--intrinsics", default=str(ROOT / "config" / "camera_intrinsics.yaml"))
    ap.add_argument("--truth", default=str(ROOT / "logs" / "field_pose_samples.csv"),
                    help="位姿真值 csv（用于对照，可缺省）")
    ap.add_argument("--fmin", type=float, default=1400)
    ap.add_argument("--fmax", type=float, default=2300)
    ap.add_argument("--fstep", type=float, default=10)
    ap.add_argument("--tag", default=None, help="输出文件名标签")
    args = ap.parse_args()

    field = load_field_points(args.field)
    K, dist = load_intrinsics(args.intrinsics)
    cx, cy = K[0, 2], K[1, 2]

    # ---------- 取帧 ----------
    if args.image:
        frame = cv2.imread(args.image)
        tag = args.tag or Path(args.image).stem
    elif args.video:
        frame = grab_frame(args.video, "mid" if args.frame_idx is None else None)
        tag = args.tag or Path(args.video).stem[:20]
    else:
        sys.exit("需要 --video 或 --image")

    # ---------- 收集像素点 ----------
    clicks = {}
    if args.clicks:
        d = yaml.safe_load(Path(args.clicks).read_text(encoding="utf-8"))
        clicks = {k: tuple(v) for k, v in d.get("points", {}).items()}
        tag = args.tag or Path(args.clicks).stem
    elif args.px:
        for part in args.px.split(";"):
            nm, uv = part.strip().split(":")
            u, v = uv.strip().split(",")
            clicks[nm.strip()] = (int(u), int(v))
    elif args.points:
        unknown = [p for p in args.points if p not in field]
        if unknown:
            sys.exit(f"错误：{unknown} 不在 field.yaml 中。可用：{sorted(field)}")
        clicks = click_points(frame, args.points)
    else:
        sys.exit("需要 --points / --px / --clicks 之一")

    names = [n for n in clicks if n in field]
    if len(names) < 4:
        sys.exit(f"错误：有效点不足 4 个（{len(names)}）。至少 4 个，建议 6 个以上。")
    px = np.array([clicks[n] for n in names], dtype=np.float64)
    obj = np.array([[*field[n], 0.0] for n in names], dtype=np.float64)
    print(f"\n用点 {len(names)} 个：{names}")

    # 保存点选结果供复用
    out_dir = ROOT / "calibration" / "fx_clicks"
    out_dir.mkdir(parents=True, exist_ok=True)
    (out_dir / f"{tag}.yaml").write_text(
        yaml.safe_dump({"points": {k: list(v) for k, v in clicks.items()}},
                       allow_unicode=True), encoding="utf-8")

    # ---------- 扫描（原始像素 + 逐 f 带畸变 PnP） ----------
    f_grid = np.arange(args.fmin, args.fmax + args.fstep / 2, args.fstep)
    rows = scan_f(obj, px, dist, cx, cy, f_grid)
    best = min(rows, key=lambda r: r[1])
    f_best, rms_best = best[0], best[1]

    print(f"\n=== fx 扫描结果（{args.fmin}~{args.fmax}） ===")
    print(f"最优 fx = {f_best:.0f}   重投影 RMS = {rms_best:.2f} px")
    print("\n候选值对比：")
    for f0 in CANDIDATE_F:
        near = min(rows, key=lambda r: abs(r[0] - f0))
        mark = "  <-- 最接近最优" if abs(near[0] - f_best) <= args.fstep else ""
        print(f"  fx={f0:.0f}: RMS={near[1]:6.2f} px{mark}")

    # ---------- 位姿 ----------
    C, yaw, pitch = pose_from(best[2], best[3])
    print(f"\n最优 fx 下相机位姿：x={C[0]:+.3f} m  y={C[1]:+.3f} m  z(高)={C[2]:+.3f} m  "
          f"yaw={yaw:+.1f}°  pitch={pitch:+.1f}°")

    if args.truth and Path(args.truth).exists():
        for line in Path(args.truth).read_text(encoding="utf-8-sig").splitlines():
            if line.strip() and not line.startswith("#") and tag[:8] in line:
                print(f"真值对照（{line.split(',')[0]}）：{line}")
                break

    # ---------- 图：误差曲线 + 重投影 ----------
    fs = np.array([r[0] for r in rows])
    rmss = np.array([r[1] for r in rows])
    fig = np.full((360, 720, 3), 30, np.uint8)
    cv2.putText(fig, "reprojection RMS (px) vs fx", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    x0, x1 = args.fmin, args.fmax
    ymax = min(rmss.max(), rms_best * 8)
    for p in range(len(fs) - 1):
        xa = int((fs[p] - x0) / (x1 - x0) * 680 + 20)
        xb = int((fs[p + 1] - x0) / (x1 - x0) * 680 + 20)
        ya = int(320 - min(rmss[p], ymax) / ymax * 270)
        yb = int(320 - min(rmss[p + 1], ymax) / ymax * 270)
        cv2.line(fig, (xa, ya), (xb, yb), (0, 200, 255), 2)
    for f0, col in zip(CANDIDATE_F, [(120, 120, 255)] * 4):
        xa = int((f0 - x0) / (x1 - x0) * 680 + 20)
        if x0 <= f0 <= x1:
            cv2.line(fig, (xa, 50), (xa, 330), col, 1)
            cv2.putText(fig, f"{f0:.0f}", (xa - 20, 348), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1)
    xb_ = int((f_best - x0) / (x1 - x0) * 680 + 20)
    cv2.line(fig, (xb_, 50), (xb_, 330), (0, 255, 0), 2)
    cv2.putText(fig, f"best {f_best:.0f} / {rms_best:.2f}px", (xb_ - 120, 45),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    out_scan = ROOT / "check_out" / f"fx_scan_{tag}.png"
    out_scan.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(out_scan), fig)

    # 重投影叠加
    f = f_best
    Kb = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
    _, rvec, tvec = cv2.solvePnP(obj.reshape(-1, 1, 3), px.reshape(-1, 1, 2), Kb, dist,
                                 flags=cv2.SOLVEPNP_IPPE)
    proj, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvec, tvec, Kb, dist)
    ov = frame.copy()
    for i, nm in enumerate(names):
        u, v = int(px[i][0]), int(px[i][1])
        pu, pv = int(proj[i][0][0]), int(proj[i][0][1])
        cv2.circle(ov, (u, v), 6, (0, 0, 255), 2)      # 人工点：红
        cv2.drawMarker(ov, (pu, pv), (0, 255, 0), cv2.MARKER_CROSS, 14, 2)  # 重投影：绿
        cv2.putText(ov, nm, (u + 8, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    out_chk = ROOT / "check_out" / f"fx_check_{tag}.jpg"
    cv2.imwrite(str(out_chk), ov)
    print(f"\n曲线：{out_scan}\n叠加：{out_chk}\n点选已存：{out_dir / (tag + '.yaml')}")


if __name__ == "__main__":
    main()
