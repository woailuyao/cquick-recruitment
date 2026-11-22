"""外参解算工具：用「墙脚线几何法」从画面反推相机俯仰/滚转，生成 camera_extrinsics.yaml。

为什么这样测（2026-11-10 实战验证）：
  手机水平仪贴 PCB 测 pitch 误差可达 2~3°（壳厚、板面不完全垂直光轴）；
  而俯仰角每差 1°，3 米外地面点的投影误差约 0.5 米——对任务二定位是致命的。
  用卷尺量 h（镜头离地高）和 d（镜头到墙脚水平距离），配合内参和画面里
  墙脚交界线的像素行位置，可几何反推出精确 pitch：
      pitch = atan(h/d) - atan((v_junction - cy) / fy)
  roll 由画面中真实竖直线（墙角/门框）的倾斜角直接给出（与俯仰无关）。

用法（桌边或场地通用，场地只需重量 h 和 d）：
  python tools/check_extrinsics.py --video raw_videos/xxx.mp4 --h 0.222 --d 0.63
  python tools/check_extrinsics.py --video xxx.mp4 --h 0.35 --d 1.2 --lens-above-board 0.02 --write

注意：--h 是镜头中心到「画面中那个地面/桌面」的竖直距离；--d 是镜头正下方
落点到墙脚的水平距离（用笔垂直悬在镜头下定位，别斜着量）。
"""
import argparse
import datetime as dt
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_intrinsics(path):
    d = yaml.safe_load(Path(path).read_text(encoding="utf-8"))
    K = np.array(d["camera_matrix"]["data"], dtype=np.float64).reshape(3, 3)
    return K, d.get("rms_reprojection_error"), (d["image_width"], d["image_height"])


def grab_frame(video):
    cap = cv2.VideoCapture(str(video))
    if not cap.isOpened():
        sys.exit(f"错误：打不开视频 {video}")
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    cap.set(cv2.CAP_PROP_POS_FRAMES, n // 2)
    ok, f = cap.read()
    cap.release()
    if not ok:
        sys.exit("错误：取帧失败")
    return f


def find_junction(frame):
    """画面下半部最长的近水平线 = 墙与地面/桌面的交界线。"""
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 720, threshold=120,
                            minLineLength=w // 2, maxLineGap=40)
    cand = []
    for l in (lines[:, 0] if lines is not None else []):
        x1, y1, x2, y2 = l
        ang = np.degrees(np.arctan2(y2 - y1, x2 - x1))
        if abs(ang) < 8 and y1 > h * 0.5 and y2 > h * 0.5:
            cand.append((abs(x2 - x1), x1, y1, x2, y2, ang))
    if not cand:
        sys.exit("错误：未检测到交界线。确认画面下半部能清晰看到墙与地面/桌面的交界。")
    cand.sort(reverse=True)
    _, x1, y1, x2, y2, ang = cand[0]
    cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
    return (y1 + y2) / 2, ang, frame


def find_roll(frame):
    """画面中真实竖直线（墙角/门框）的倾斜角。返回 (roll_deg, 线数)。"""
    h, _ = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 720, threshold=100,
                            minLineLength=h // 2, maxLineGap=20)
    vert = []
    for l in (lines[:, 0] if lines is not None else []):
        a, b, c, d = l
        if abs(c - a) < abs(d - b) * 0.12 and abs(d - b) > h // 2:
            t = np.degrees(np.arctan2(c - a, d - b))
            if t > 90:
                t -= 180
            if t < -90:
                t += 180
            vert.append(t)
    if not vert:
        return 0.0, 0
    return float(np.median(vert)), len(vert)


def rot_carrier_from_camera(pitch_deg, roll_deg):
    """载体系(前X/左Y/上Z, 原点光心) ← 相机系(x右/y下/z前)。

    pitch>0 = 光轴向下俯；roll>0 = 相机绕光轴顺时针（从镜头后看）。
    构造：零姿态时 camera x=(0,-1,0), y=(0,0,-1), z=(1,0,0)（载体系表示），
    再绕相机 x 轴俯仰、绕相机 z 轴滚转。
    """
    p = np.radians(pitch_deg)
    r = np.radians(roll_deg)
    R0 = np.array([[0, 0, 1],
                   [-1, 0, 0],
                   [0, -1, 0]], dtype=np.float64)
    Rx = np.array([[1, 0, 0],
                   [0, np.cos(-p), -np.sin(-p)],
                   [0, np.sin(-p), np.cos(-p)]], dtype=np.float64)
    Rz = np.array([[np.cos(-r), -np.sin(-r), 0],
                   [np.sin(-r), np.cos(-r), 0],
                   [0, 0, 1]], dtype=np.float64)
    return R0 @ Rx @ Rz


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--video", required=True, help="对着墙录的视频")
    ap.add_argument("--h", type=float, required=True, help="镜头中心到地面/桌面的竖直距离(m)")
    ap.add_argument("--d", type=float, required=True, help="镜头正下方落点到墙脚的水平距离(m)")
    ap.add_argument("--intrinsics", default="config/camera_intrinsics.yaml")
    ap.add_argument("--lens-above-board", type=float, default=None,
                    help="镜头中心到板底高度(m)，场地换算 h 用")
    ap.add_argument("--phone-pitch", type=float, default=None,
                    help="手机水平仪测的 pitch(°)，仅作对照记录")
    ap.add_argument("--write", action="store_true", help="写入 calibration/camera_extrinsics.yaml")
    args = ap.parse_args()

    K, rms, (W, H) = load_intrinsics(ROOT / args.intrinsics)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]

    video = Path(args.video)
    if not video.is_absolute():
        video = ROOT / video
    frame = grab_frame(video)
    v_j, tilt, frame = find_junction(frame)
    roll, n_vert = find_roll(frame)

    theta_j = np.degrees(np.arctan2(args.h, args.d))
    delta = np.degrees(np.arctan2(v_j - cy, fy))
    pitch = theta_j - delta

    R = rot_carrier_from_camera(pitch, roll)
    # 自检：光轴（载体系第三列）应向下
    fwd = R @ np.array([0, 0, 1.0])
    assert fwd[2] < 0, "pitch 符号错误：光轴没有朝下"
    # 自检：投影墙脚点 (d,0,0) 应落回检测到的行
    t_ground = np.array([0, 0, args.h])
    P_cam = R.T @ (np.array([args.d, 0, 0]) - t_ground)
    uvw = K @ P_cam
    v_proj = uvw[1] / uvw[2]

    out = ROOT / "check_out" / "extrinsic_check.jpg"
    out.parent.mkdir(exist_ok=True)
    cv2.imwrite(str(out), frame)

    print("=" * 62)
    print(f"内参     : fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f} (RMS {rms}px)")
    print(f"交界线   : 行 {v_j:.0f}/{H} ({v_j/H*100:.1f}%)  线倾角 {tilt:+.2f}°")
    print(f"竖直线   : {n_vert} 条 -> roll = {roll:+.2f}°")
    print(f"几何反推 : atan(h/d)={theta_j:.2f}°  画面偏角={delta:.2f}°  => pitch = {pitch:.2f}°")
    if args.phone_pitch is not None:
        print(f"对照     : 手机水平仪 {args.phone_pitch}°（差 {abs(pitch-args.phone_pitch):.1f}°，以几何法为准）")
    print(f"投影自检 : 墙脚点预测行 {v_proj:.0f} vs 实测 {v_j:.0f}（差 {abs(v_proj-v_j):.1f}px，应<2）")
    print(f"标注图   -> {out}")
    print("=" * 62)
    print("R_carrier_from_camera =")
    for row in R:
        print("  [" + "  ".join(f"{v:+.6f}" for v in row) + "]")

    if args.write:
        data = {
            "#": "相机外参（tools/check_extrinsics.py 墙脚线几何法生成，勿手改数值）",
            "carrier_definition": (
                "独立移动测试架（桌边平板），按细则以相机光心为载体参考原点："
                "x_carrier_map = x_camera_map。载体系：X前(光轴水平投影) Y左 Z上，原点=光心。"),
            "rotation_carrier_from_camera": {"rows": 3, "cols": 3,
                                             "data": [round(float(v), 8) for v in R.ravel()]},
            "translation_carrier_from_camera_m": [0.0, 0.0, 0.0],
            "install": {
                "pitch_deg": float(round(pitch, 3)),
                "pitch_method": "墙脚线几何反推 atan(h/d) - atan((v-cy)/fy)",
                "roll_deg": float(round(roll, 3)),
                "roll_method": "画面真实竖直线倾斜角（墙角线）",
                "phone_pitch_deg_ref": args.phone_pitch,
                "lens_height_above_ground_m": args.h,
                "lens_height_above_board_bottom_m": args.lens_above_board,
                "height_note": ("现场换算：h_field = 板底离地高度 + lens_height_above_board_bottom_m；"
                                "俯仰/滚转随刚性安装不变，无需重量。"),
            },
            "measurement": {
                "date": dt.datetime.now().isoformat(timespec="seconds"),
                "junction_video": str(video.name),
                "junction_row_px": float(round(v_j, 1)),
                "junction_row_percent": float(round(v_j / H * 100, 2)),
                "d_wall_m": float(args.d),
                "h_ground_m": float(args.h),
                "intrinsics_rms_px": rms,
            },
            "note": ("pitch 敏感度：每差 1°，3m 外地面点误差约 0.5m。"
                     "到场地后建议用本工具+实测 h/d 复核一次。"),
        }
        dest = ROOT / "calibration" / "camera_extrinsics.yaml"
        with open(dest, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        print(f"已写入 {dest}")


if __name__ == "__main__":
    main()
