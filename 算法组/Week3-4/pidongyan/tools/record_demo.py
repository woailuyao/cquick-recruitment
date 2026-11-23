"""现场验收录屏：左 RGB 实时检测 / 右 C0 地图分屏，连续未剪切录制。

对齐细则「视频证据要求」：姓名 + 实时时间入画、地图文件名/版本与坐标轴、
相机内外参、像素→地面坐标转换、左右分屏、有效/无效状态、ROS2 话题名展示。

红线
----
- 默认 `--source 0` 是**真实相机实时输入**；传视频文件仅供开发调试，
  画面会打 `DEBUG / REPLAY - NOT LIVE` 水印，**绝不冒充实时验收证据**。
- marker 模型未给出或位姿解算失败时，右侧显示 `valid=FALSE` 与 status，
  **不画相机位置图标、不填任何假坐标**（细则「定位失败仍伪造有效位姿 −10」）。
- 输出是单个连续 mp4，不剪切、不拼接；录屏窗口按细则保持左右分屏。
- 界面文字一律 ASCII（OpenCV putText 不支持中文，会出乱码方块）。

用法（20 号下午实验室，相机拿回后）
------------------------------------
  python tools/record_demo.py --source 0 --name pidongyan
  # 若 175 张四类已标完、marker 模型已训好：
  python tools/record_demo.py --source 0 --name pidongyan --marker-model runs/detect/markers/weights/best.pt --h-prior 0.55
  # 开发调试（视频文件，会打 DEBUG 水印，不能交）：
  python tools/record_demo.py --source raw_videos/ball_run02_xxx.mp4 --max-frames 200

快捷键：q 退出并收尾写盘，s 截图。
"""
import argparse
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

from cam_open import open_camera  # noqa: E402
from ultralytics import YOLO  # noqa: E402
from task2 import FieldLocalizer  # noqa: E402
from task2.geometry import load_field, load_intrinsics  # noqa: E402

HALF_W, HALF_H = 960, 540
INFO_W = 256                     # 右侧竖排状态栏宽度；地图占满左侧，不压线
MAP_X0, MAP_X1 = 18, HALF_W - INFO_W - 14
MAP_Y0, MAP_Y1 = 18, HALF_H - 18
GREEN = (80, 220, 80)
YELLOW = (60, 220, 230)
RED = (60, 60, 235)
WHITE = (240, 240, 240)
GREY = (150, 150, 150)
BLUE = (255, 180, 60)

FIELD_SHA = "c4b1e91302b43982..."
GEOM_SHA = "cc7927abe6210580..."


def _map_scale():
    """等比缩放系数（px/m），保证 8.35×6.0 m 场地不变形。"""
    return min((MAP_X1 - MAP_X0) / 8.35, (MAP_Y1 - MAP_Y0) / 6.00)


def w2m(x, y):
    """世界坐标 (m) -> 地图区像素，等比居中。+X 向右（球门端），+Y 向上。"""
    s = _map_scale()
    ox = MAP_X0 + ((MAP_X1 - MAP_X0) - 8.35 * s) / 2.0
    oy = MAP_Y0 + ((MAP_Y1 - MAP_Y0) - 6.0 * s) / 2.0
    return int(ox + (x + 4.175) * s), int(oy + (3.0 - y) * s)


def line_world(img, a, b, color, thick=2):
    cv2.line(img, w2m(*a), w2m(*b), color, thick, cv2.LINE_AA)


def draw_map(loc, field, pose, valid, status, marker_count, residual,
             ball_world, ball_model_name, live):
    img = np.full((HALF_H, HALF_W, 3), 28, dtype=np.uint8)
    # ---- 场地线 ----
    for a, b in [((-4.175, -3), (4.175, -3)), ((4.175, -3), (4.175, 3)),
                 ((4.175, 3), (-4.175, 3)), ((-4.175, 3), (-4.175, -3))]:
        line_world(img, a, b, WHITE, 2)
    line_world(img, (0.125, -3), (0.125, 2.92), WHITE, 2)      # 中线（实测位置）
    c = w2m(0, 0)
    r = int(0.73 * _map_scale())
    cv2.circle(img, c, r, WHITE, 2, cv2.LINE_AA)                # 中圈
    # 两个禁区框（用实测角点近似）
    line_world(img, (2.295, -1.94), (2.245, 1.95), WHITE, 2)
    line_world(img, (2.295, -1.94), (4.175, -2.00), WHITE, 2)
    line_world(img, (2.245, 1.95), (4.175, 1.95), WHITE, 2)
    line_world(img, (-2.325, -1.92), (-2.205, 1.89), WHITE, 2)
    line_world(img, (-2.325, -1.92), (-4.175, -1.92), WHITE, 2)
    line_world(img, (-2.205, 1.89), (-4.175, 1.89), WHITE, 2)

    # ---- 15 个标志点 ----
    colors = {"L": (70, 70, 235), "T": (230, 160, 60),
              "X": (70, 210, 70), "P": (200, 90, 220)}
    for n, (x, y) in field.items():
        k = n[0] if n[0] in "LTX" else "P"
        p = w2m(x, y)
        cv2.circle(img, p, 4, colors[k], -1, cv2.LINE_AA)
        cv2.putText(img, n.replace("Cross", "").replace("PenalityPoint", "P"),
                    (p[0] + 5, p[1] - 5), cv2.FONT_HERSHEY_SIMPLEX,
                    0.32, GREY, 1, cv2.LINE_AA)

    # ---- 坐标轴 ----
    o = w2m(-3.6, -2.5)
    cv2.arrowedLine(img, o, (o[0] + 62, o[1]), (60, 160, 255), 2)
    cv2.arrowedLine(img, o, (o[0], o[1] - 62), (60, 160, 255), 2)
    cv2.putText(img, "+X GOAL", (o[0] + 66, o[1] + 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 160, 255), 1, cv2.LINE_AA)
    cv2.putText(img, "+Y", (o[0] + 4, o[1] - 66),
                cv2.FONT_HERSHEY_SIMPLEX, 0.38, (60, 160, 255), 1, cv2.LINE_AA)

    # ---- 相机位姿（仅在 valid 时画，绝不伪造）----
    if valid and pose is not None:
        cp = w2m(pose.C_F[0], pose.C_F[1])
        th = np.radians(pose.yaw_deg)
        tip = (int(cp[0] + 46 * np.cos(th)), int(cp[1] - 46 * np.sin(th)))
        cv2.arrowedLine(img, cp, tip, YELLOW, 3, cv2.LINE_AA)
        cv2.circle(img, cp, 7, YELLOW, -1, cv2.LINE_AA)
        cv2.putText(img, "CAMERA", (cp[0] + 9, cp[1] + 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, YELLOW, 1, cv2.LINE_AA)
    if ball_world is not None:
        bp = w2m(ball_world[0], ball_world[1])
        cv2.circle(img, bp, 6, BLUE, -1, cv2.LINE_AA)
        cv2.putText(img, "BALL", (bp[0] + 8, bp[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, BLUE, 1, cv2.LINE_AA)

    # ---- 右侧竖排信息栏（地图占满左侧，文字不压线）----
    K, _ = load_intrinsics(ROOT / "config" / "camera_intrinsics_fieldvideo.yaml")
    info = [
        "C0 MAP  c0-self-measured-v1",
        f"sha {FIELD_SHA}",
        f"geom {GEOM_SHA}",
        "+X GOAL  +Y left  m / deg",
        f"fx={K[0,0]:.0f} cx={K[0,2]:.0f} cy={K[1,2]:.0f}",
        "k1=0.171 k2=-0.565 k3=2.888",
        "p1=-0.008 p2=-0.015",
        "pitch=14.5 roll=1.5 h=0.55",
        f"ball: {ball_model_name[:22]}",
        "/soccer/ball",
        "/localization/field_pose",
        "--- pose ---",
        f"marker_count : {marker_count}",
        f"residual : {('--' if residual is None else f'{residual:.2f}')} px",
        f"valid : {'TRUE' if valid else 'FALSE'}",
        f"status : {status[:22]}",
    ]
    if valid and pose is not None:
        info += [f"x_camera_map: {pose.C_F[0]:+.3f} m",
                 f"y_camera_map: {pose.C_F[1]:+.3f} m",
                 f"yaw_camera_map: {pose.yaw_deg:+.1f} d"]
    else:
        info += ["x_camera_map: --", "y_camera_map: --", "yaw_camera_map: --"]
    bx = HALF_W - INFO_W
    cv2.rectangle(img, (bx, 0), (HALF_W, HALF_H), (18, 18, 18), -1)
    cv2.line(img, (bx, 0), (bx, HALF_H), (90, 90, 90), 1)
    for i, s in enumerate(info):
        col = (RED if (s.startswith("valid") and not valid) else
               (GREEN if s.startswith("valid") else
                (GREY if s.startswith(("marker_count", "residual", "status",
                                       "x_camera", "y_camera", "yaw_camera",
                                       "sha ", "geom ")) else WHITE)))
        fs = 0.40
        (tw, _), _ = cv2.getTextSize(s, cv2.FONT_HERSHEY_SIMPLEX, fs, 1)
        if tw > INFO_W - 16:                       # 自动缩字号，绝不越出右边界
            fs *= (INFO_W - 16) / tw
        cv2.putText(img, s, (bx + 8, 22 + 25 * i),
                    cv2.FONT_HERSHEY_SIMPLEX, fs, col, 1, cv2.LINE_AA)
    if not live:
        cv2.rectangle(img, (MAP_X0, HALF_H - 32), (MAP_X1, HALF_H), (18, 18, 18), -1)
        cv2.putText(img, "DEBUG / REPLAY - NOT LIVE", (MAP_X0 + 8, HALF_H - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, RED, 2, cv2.LINE_AA)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="0", help="0=真实相机索引；视频路径=调试（会打 DEBUG 水印）")
    ap.add_argument("--name", default="pidongyan", help="姓名拼音，按细则要求入画")
    ap.add_argument("--model", default=str(ROOT / "best_train_v4.pt"),
                    help="正式球检测权重（双 seed 对照后确定）；回退 v3 用 --model best_train_v3.pt")
    ap.add_argument("--marker-model", default=None, help="四类标志模型 .pt；不给则 valid=FALSE")
    ap.add_argument("--out", default=None, help="输出 mp4，默认 check_out/demo_<时间>.mp4")
    ap.add_argument("--intrinsics", default="config/camera_intrinsics_fieldvideo.yaml")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--pose-every", type=int, default=5)
    ap.add_argument("--h-prior", type=float, default=0.55)
    ap.add_argument("--residual-thresh", type=float, default=15.0, help="rms 超过则判 AMBIGUOUS")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--max-frames", type=int, default=0)
    ap.add_argument("--no-record", action="store_true", help="只预览不录盘（调试）")
    args = ap.parse_args()

    model_path = Path(args.model)
    if not model_path.exists() and not model_path.is_absolute():
        model_path = ROOT / args.model
    if not model_path.exists():
        raise SystemExit(f"球模型不存在：{model_path}")
    ball_model = YOLO(str(model_path))
    marker_model = None
    if args.marker_model:
        mp = Path(args.marker_model)
        if not mp.is_absolute():
            mp = ROOT / mp
        if not mp.exists():
            raise SystemExit(f"marker 模型不存在：{mp}")
        marker_model = YOLO(str(mp))

    loc = FieldLocalizer(ROOT / args.intrinsics, ROOT / args.field)
    field = load_field(ROOT / args.field)

    live = args.source.isdigit()
    if live:
        cap, _ = open_camera(int(args.source), width=args.width, height=args.height,
                             fps=args.fps, fourcc="MJPG", backend="auto",
                             verify_fps=None, verbose=True)
    else:
        p = Path(args.source)
        if not p.is_absolute():
            p = ROOT / p
        cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        raise SystemExit(f"打不开视频源：{args.source}")

    out = (ROOT / args.out) if args.out else (
        ROOT / "check_out" / f"demo_{datetime.now():%Y%m%d_%H%M%S}.mp4")
    out.parent.mkdir(parents=True, exist_ok=True)
    writer = None
    if not args.no_record:
        writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.fps, (HALF_W * 2, HALF_H))
        if not writer.isOpened():
            raise SystemExit(f"VideoWriter 打不开：{out}")
        print(f"连续录制 -> {out}")

    print(f"source={'LIVE camera' if live else 'DEBUG replay'}  ball={model_path.name}  "
          f"marker={args.marker_model or 'none (valid=FALSE)'}")
    print("q 退出并写盘，s 截图")

    # 预热
    ball_model.predict(np.zeros((args.height, args.width, 3), np.uint8),
                       imgsz=640, verbose=False)

    pose, valid, status = None, False, "INSUFFICIENT_MARKERS (marker model not trained)"
    marker_count, residual, ball_world = 0, None, None
    frame_id, t0 = 0, time.time()
    try:
        while True:
            loop_t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                print("视频源结束或取帧失败")
                break
            if args.max_frames and frame_id >= args.max_frames:
                break

            # ---- 左：球检测 ----
            t_inf = time.time()
            r = ball_model.predict(frame, imgsz=640, conf=args.conf,
                                   verbose=False)[0]
            infer_ms = (time.time() - t_inf) * 1000
            ball_uv = None
            if r.boxes is not None and len(r.boxes):
                xyxy = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
                for (x1, y1, x2, y2), c in zip(xyxy, confs):
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)),
                                  (0, 165, 255), 2)
                    cv2.putText(frame, f"ball {c:.2f}", (int(x1), max(int(y1) - 6, 16)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
                k = int(np.argmax(confs))
                ball_uv = ((xyxy[k][0] + xyxy[k][2]) / 2,
                           (xyxy[k][1] + xyxy[k][3]) / 2)

            # ---- 位姿（仅 marker 模型可用时）----
            if marker_model is not None and frame_id % args.pose_every == 0:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                dets = []
                for rr in marker_model(frame, verbose=False, conf=args.conf)[0]:
                    cls_name = marker_model.names[int(rr.boxes.cls)]
                    dets.append({"cls": cls_name,
                                 "box": rr.boxes.xyxy[0].tolist(),
                                 "conf": float(rr.boxes.conf)})
                marker_count = len(dets)
                if len(dets) >= 4:
                    prior = {"h": args.h_prior, "h_sigma": 0.10,
                             "pitch_deg": 14.525, "pitch_sigma": 12.0}
                    new_pose, refined = loc.solve_from_detections(gray, dets, prior=prior)
                    if new_pose is not None and new_pose.C_F[2] > 0 and \
                            new_pose.rms_px < args.residual_thresh:
                        pose, valid, status = new_pose, True, "OK"
                        residual = new_pose.rms_px
                        for (cls_name, uv, method) in refined:
                            cv2.circle(frame, (int(uv[0]), int(uv[1])), 4, GREEN, -1)
                            cv2.putText(frame, cls_name, (int(uv[0]) + 6, int(uv[1]) - 6),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, GREEN, 1)
                    else:
                        pose, valid, status = None, False, "AMBIGUOUS / POSE REJECTED"
                        residual = new_pose.rms_px if new_pose is not None else None
                else:
                    pose, valid, status = None, False, "INSUFFICIENT_MARKERS"
                    residual = None

            ball_world = None
            if valid and pose is not None and ball_uv is not None:
                try:
                    ball_world = loc.pixel_to_world(pose, ball_uv)
                except Exception:                                # noqa: BLE001
                    ball_world = None

            # ---- 水印（姓名 + 实时时间 + LIVE）----
            now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            fps_now = 1.0 / max(time.time() - loop_t0, 1e-6)
            tag = "LIVE" if live else "DEBUG / REPLAY - NOT LIVE"
            head = f"{tag}   {args.name}   {now}"
            cv2.putText(frame, head, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.66, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, head, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.66, (RED if not live else GREEN), 2, cv2.LINE_AA)
            cv2.putText(frame, f"FPS {fps_now:5.1f} | infer {infer_ms:5.1f}ms | frame {frame_id}",
                        (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, WHITE, 1, cv2.LINE_AA)
            cv2.putText(frame, f"ball={0 if r.boxes is None else len(r.boxes)}  "
                               f"valid={'TRUE' if valid else 'FALSE'}  {status}",
                        (10, 74), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        GREEN if valid else YELLOW, 1, cv2.LINE_AA)

            left = cv2.resize(frame, (HALF_W, HALF_H))
            right = draw_map(loc, field, pose, valid, status, marker_count,
                             residual, ball_world, model_path.name, live)
            canvas = np.hstack([left, right])

            if writer is not None:
                writer.write(canvas)
            cv2.imshow("CQuick demo (q=quit, s=shot)", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                shot = ROOT / "check_out" / f"demo_shot_{frame_id:05d}.jpg"
                cv2.imwrite(str(shot), canvas)
                print(f"截图：{shot}")
            frame_id += 1
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        cv2.destroyAllWindows()
    el = time.time() - t0
    print(f"\n共 {frame_id} 帧 / {el:.1f}s / 平均 {frame_id/max(el,1e-6):.1f} FPS")
    if writer is not None:
        print(f"连续录屏已写盘：{out}（未剪切、未拼接）")
    print("⚠ 若 status 全程 INSUFFICIENT_MARKERS，说明 marker 模型未就绪；"
          "该段可作为细则要求的「无效状态」证据，但不得声称定位成功。")


if __name__ == "__main__":
    main()
