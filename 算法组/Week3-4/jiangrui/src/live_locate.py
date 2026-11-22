# -*- coding: utf-8 -*-
"""任务 2：单目场地自定位实时程序（左右分屏）。

左侧：相机实时画面 + L/T/X/点球点检测 + 几何细化后的 landmark 像素
右侧：field.yaml 俯视图 + 相机位置 / 朝向 + 有效标志与匹配结果
底部：x_camera_map / y_camera_map / yaw_camera_map / valid / status /
      marker_count / residual / FPS

运行前先标定外参（或运行时直接给角度）:
    python calibrate_extrinsics.py --mode manual --height 0.66 --pitch-down 13.3 --roll 0
    python live_locate.py

试运行（不写文件）:
    python live_locate.py --height 0.66 --pitch-down 13.3 --roll 0

按键:
    q 退出    s 截图    r 记录当前测试点（有 --ref 时一并算误差）

现场测试时用 --ref 输入卷尺量的参考位姿（参考值只用于算误差，不进入算法）:
    python live_locate.py --ref 1.20,-0.50,35

输出坐标系：carrier 原点 = 相机光心地面投影，因此 x_camera_map 等就是
相机光心的地图坐标；若以后把相机装到机器人底座，用外参换算即可。
"""

import argparse
import csv
import math
import os
import time

import cv2
import numpy as np

from field_map import FieldMap, PoseSolver, normalize_type
from ground_geometry import CameraModel, Extrinsics
from marker_refine import refine_pixel

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_MODEL = os.path.join(ROOT, "weights", "marker_v3_best.pt")
TYPE_COLOR = {
    "T": (0, 165, 255),
    "L": (255, 255, 0),
    "X": (255, 0, 255),
    "P": (0, 255, 255),
}


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=DEFAULT_MODEL, help="场地标志模型")
    ap.add_argument("--source", default="0", help="摄像头索引（0 通常是内置，外接 USB 多为 1）或视频/图片路径")
    ap.add_argument("--intrinsics", default=os.path.join(ROOT, "calibration", "camera_intrinsics.yaml"))
    ap.add_argument("--extrinsics", default=os.path.join(ROOT, "calibration", "camera_extrinsics.yaml"))
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field_measured_near.yaml"))
    ap.add_argument("--height", type=float, help="覆盖外参：相机光心离地高度 m")
    ap.add_argument("--pitch-down", type=float, default=0.0, help="覆盖外参：俯角，向下为正")
    ap.add_argument("--roll", type=float, default=0.0, help="覆盖外参：滚转角")
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height-px", type=int, default=720)
    ap.add_argument("--no-refine", action="store_true", help="不做事几何细化，只用框中心")
    ap.add_argument("--invert-marker", action="store_true", help="深色标线 / 浅色地面时使用")
    ap.add_argument("--min-inliers", type=int, default=3)
    ap.add_argument("--ref", default=None, help="参考位姿 x,y,yaw_deg（只用于算误差）")
    ap.add_argument("--log-dir", default=os.path.join(ROOT, "logs"))
    ap.add_argument("--no-window", action="store_true", help="不显示窗口（调试/离线跑）")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧，0=不限")
    return ap.parse_args()


def open_source(src, width, height):
    """按摄像头索引或文件打开输入。

    USB 相机在 MSMF 下有几个已知问题，按"能不能真的读到帧"逐个试：
      1. MSMF 默认分辨率（有的相机一调用 set 改分辨率就整条流崩掉）
      2. MSMF 设成 1280x720
      3. DSHOW 设成 1280x720
    只有分辨率正好等于 (width, height) 且稳定读到帧才接受；
    都拿不到目标分辨率时退而用能出图的那个，并打印警告。
    """
    if src.isdigit():
        idx = int(src)
        cap, backend = None, None

        def _usable(c):
            ok_n = 0
            for _ in range(12):
                try:
                    ok, _ = c.read()
                except Exception:
                    # OpenCV 5 在流启动失败时可能直接抛 Mat 断言，而不是返回 False
                    ok = False
                ok_n += 1 if ok else 0
            return ok_n >= 6

        def _open(name, api, set_res):
            c = cv2.VideoCapture(idx, api)
            if not c.isOpened():
                c.release()
                return None
            if set_res:
                try:
                    c.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    c.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                except Exception:
                    c.release()
                    return None
            if not _usable(c):
                c.release()
                return None
            w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (w, h) != (width, height) and not set_res:
                # 有些 UVC 驱动必须先跑起来再改分辨率，改完再验一次
                try:
                    c.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    c.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                except Exception:
                    c.release()
                    return None
                if not _usable(c):
                    c.release()
                    return None
                w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return c, name, w, h

        attempts = [("MSMF", cv2.CAP_MSMF, False),
                    ("MSMF", cv2.CAP_MSMF, True),
                    ("DSHOW", cv2.CAP_DSHOW, True)]
        fallback_cfg = None
        for name, api, set_res in attempts:
            r = _open(name, api, set_res)
            if r is None:
                time.sleep(0.8)
                continue
            c, name, w, h = r
            if (w, h) == (width, height):
                cap, backend = c, name
                break
            if fallback_cfg is None:
                fallback_cfg = (name, api, set_res)
            c.release()
            time.sleep(0.8)
        if cap is None and fallback_cfg is not None:
            name, api, set_res = fallback_cfg
            r = _open(name, api, set_res)
            if r is not None:
                cap, backend, _, _ = r
                print(f"警告: 实际分辨率 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}，"
                      f"不是 {width}x{height}，内参不匹配，定位结果会偏")
        if cap is None:
            raise SystemExit(f"摄像头 {idx} 打不开或读不到帧（MSMF/DSHOW 都失败）。"
                             "换一个索引试试：0 → 1，并确认没有别的程序占用摄像头。")
        print(f"输入源: 摄像头 {idx}（Windows 下 0 通常是内置摄像头，外接 USB 相机多从 1 开始）")
        print(f"采集后端: {backend}  "
              f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    else:
        if not os.path.exists(src):
            raise SystemExit(f"找不到输入: {src}")
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise SystemExit(f"打不开输入源: {src}")
    return cap


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


def parse_ref(text):
    if not text:
        return None
    parts = [float(v) for v in text.replace("，", ",").split(",")]
    if len(parts) != 3:
        raise SystemExit("--ref 格式应为 x,y,yaw，例如 1.20,-0.50,35")
    return parts


def build_extrinsics(args):
    if args.height is not None:
        print(f"使用命令行外参: h={args.height} m, pitch={args.pitch_down}°, roll={args.roll}°")
        return Extrinsics.from_angles(args.height, args.pitch_down, args.roll,
                                      source="命令行")
    ext = Extrinsics.from_yaml(args.extrinsics)
    if ext is None:
        raise SystemExit(
            "外参还未标定。先运行：\n"
            f"  python {os.path.join(HERE, 'calibrate_extrinsics.py')} "
            "--mode manual --height 0.66 --pitch-down 13.3 --roll 0\n"
            "或者在命令行覆盖 --height --pitch-down --roll。")
    print(f"外参: h={ext.height:.3f} m, pitch={ext.pitch_down:.2f}°, "
          f"roll={ext.roll:.2f}° ({ext.method})")
    return ext


class MapView:
    def __init__(self, field, width=640, height=720):
        self.field = field
        self.w, self.h = width, height
        xs = [m["x"] for m in field.markers]
        ys = [m["y"] for m in field.markers]
        self.xmin, self.xmax = min(xs) - 1.0, max(xs) + 1.0
        self.ymin, self.ymax = min(ys) - 1.0, max(ys) + 1.0
        self.scale = min((width - 40) / (self.xmax - self.xmin),
                         (height - 40) / (self.ymax - self.ymin))
        self.cx = width / 2.0
        self.cy = height / 2.0
        self.origin_x = (self.xmin + self.xmax) / 2.0
        self.origin_y = (self.ymin + self.ymax) / 2.0

    def to_px(self, x, y):
        return (int(round(self.cx + (x - self.origin_x) * self.scale)),
                int(round(self.cy - (y - self.origin_y) * self.scale)))

    def draw(self, canvas, result, matched_ids, cam_pose, fov_h_deg, status_text):
        cv2.rectangle(canvas, (0, 0), (self.w, self.h), (24, 24, 24), -1)

        # 1 m 网格
        for gx in range(int(math.floor(self.xmin)), int(math.ceil(self.xmax)) + 1):
            p1 = self.to_px(gx, self.ymin)
            p2 = self.to_px(gx, self.ymax)
            cv2.line(canvas, p1, p2, (40, 40, 40), 1)
        for gy in range(int(math.floor(self.ymin)), int(math.ceil(self.ymax)) + 1):
            p1 = self.to_px(self.xmin, gy)
            p2 = self.to_px(self.xmax, gy)
            cv2.line(canvas, p1, p2, (40, 40, 40), 1)

        # 场地边界（取标志点包络再外扩 0.5 m 作为示意）
        xs = [m["x"] for m in self.field.markers]
        ys = [m["y"] for m in self.field.markers]
        p1 = self.to_px(min(xs), min(ys))
        p2 = self.to_px(max(xs), max(ys))
        cv2.rectangle(canvas, p1, p2, (90, 90, 90), 1)

        # 地图标志点
        for m in self.field.markers:
            p = self.to_px(m["x"], m["y"])
            color = TYPE_COLOR.get(m["type"], (200, 200, 200))
            matched = m["id"] in matched_ids
            if matched:
                cv2.circle(canvas, p, 6, (0, 255, 0), -1)
                cv2.circle(canvas, p, 8, (255, 255, 255), 1)
            else:
                cv2.circle(canvas, p, 4, color, -1)
            cv2.putText(canvas, m["id"].replace("Cross", ""), (p[0] + 7, p[1] + 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, color, 1)

        # 相机位姿
        if cam_pose is not None:
            x, y, yaw = cam_pose
            p = self.to_px(x, y)
            ang = math.radians(yaw)
            # FOV 扇形
            half = math.radians(fov_h_deg / 2.0)
            for s in (-1, 1):
                a = ang + s * half
                q = self.to_px(x + 3.0 * math.cos(a), y + 3.0 * math.sin(a))
                cv2.line(canvas, p, q, (60, 120, 60), 1)
            tip = self.to_px(x + 0.9 * math.cos(ang), y + 0.9 * math.sin(ang))
            color = (0, 255, 0) if result.get("valid") else (0, 0, 255)
            cv2.arrowedLine(canvas, p, tip, color, 3, tipLength=0.35)
            cv2.circle(canvas, p, 7, color, -1)
            cv2.putText(canvas, "CAM", (p[0] + 9, p[1] - 9),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)

        cv2.putText(canvas, "MAP  (X right, Y up, yaw CCW from +X)", (12, 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, (180, 180, 180), 1)
        cv2.putText(canvas, status_text, (12, self.h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if result.get("valid") else (0, 120, 255), 2)


def main():
    args = parse_args()
    cam = CameraModel(args.intrinsics)
    ext = build_extrinsics(args)
    field = FieldMap(args.field)
    solver = PoseSolver(field, min_inliers=args.min_inliers)
    view = MapView(field)
    ref = parse_ref(args.ref)

    from ultralytics import YOLO
    if not os.path.exists(args.model):
        raise SystemExit(f"找不到模型: {args.model}")
    model = YOLO(args.model)

    cap = open_source(args.source, args.width, args.height_px)
    os.makedirs(args.log_dir, exist_ok=True)
    log_path = os.path.join(args.log_dir, "site_test.csv")
    print(f"模型: {args.model}")
    print(f"场地: {args.field}  标志点 {len(field.markers)} 个")
    print(f"日志: {log_path}")
    print("按键: q 退出, s 截图, r 记录测试点")

    fps_hist = []
    t_prev = time.time()
    frame_idx = 0
    last_res = None
    snapshot_dir = os.path.join(ROOT, "snapshots")
    os.makedirs(snapshot_dir, exist_ok=True)

    while True:
        ok, frame = cap.read()
        if not ok:
            print("输入结束或读帧失败")
            break
        frame_idx += 1

        r = model.predict(frame, imgsz=args.imgsz, conf=args.conf,
                          verbose=False, device=args.device)[0]
        boxes = r.boxes.xyxy.cpu().numpy() if r.boxes is not None else np.zeros((0, 4))
        confs = r.boxes.conf.cpu().numpy() if r.boxes is not None else np.zeros(0)
        clss = r.boxes.cls.cpu().numpy().astype(int) if r.boxes is not None else np.zeros(0, int)

        dets = []
        for box, cf, ci in zip(boxes, confs, clss):
            name = r.names.get(int(ci), str(ci))
            mtype = normalize_type(name)
            if mtype is None:
                continue
            if args.no_refine:
                u = float((box[0] + box[2]) / 2)
                v = float((box[1] + box[3]) / 2)
                refined, how = False, "center"
            else:
                u, v, refined, how = refine_pixel(frame, box, mtype,
                                                  invert=args.invert_marker)
            dets.append({"type": mtype, "name": name, "box": box, "conf": float(cf),
                         "u": u, "v": v, "refined": refined, "how": how})

        uvs = np.array([[d["u"], d["v"]] for d in dets], dtype=float).reshape(-1, 2)
        proj = ext.project_to_ground(uvs, cam) if len(dets) else {
            "points": np.zeros((0, 2)), "valid": np.zeros(0, bool), "dist": np.zeros(0)}

        solve_idx = [i for i in range(len(dets)) if proj["valid"][i]]
        solve_types = [dets[i]["type"] for i in solve_idx]
        solve_xy = proj["points"][solve_idx] if solve_idx else np.zeros((0, 2))
        result = solver.solve(solve_types, solve_xy)
        last_res = result

        # 匹配结果映射回检测索引
        matched_ids, det_match = set(), {}
        for sub_i, info in result.get("matches", {}).items():
            if sub_i < len(solve_idx):
                di = solve_idx[sub_i]
                det_match[di] = info
                matched_ids.add(info["id"])

        # ---------------- 左图（先在原分辨率画，再缩放显示） ----------------
        canvas = np.zeros((720, 1280, 3), np.uint8)
        for i, d in enumerate(dets):
            x1, y1, x2, y2 = [int(v) for v in d["box"]]
            color = TYPE_COLOR.get(d["type"], (200, 200, 200))
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"{d['type']} {d['conf']:.2f}",
                        (x1, max(14, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            bcv = (int((x1 + x2) / 2), int((y1 + y2) / 2))
            cv2.circle(frame, bcv, 3, (255, 0, 0), -1)
            px = (int(round(d["u"])), int(round(d["v"])))
            pcolor = (0, 255, 0) if i in det_match else (0, 0, 255)
            cv2.circle(frame, px, 6, pcolor, -1)
            if i in det_match:
                cv2.putText(frame, det_match[i]["id"], (px[0] + 8, px[1] - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)
        canvas[0:360, 0:640] = cv2.resize(frame, (640, 360))

        # ---------------- 右侧地图 ----------------
        cam_pose = None
        if result.get("valid"):
            th = math.radians(result["yaw"])
            yaw_cam = result["yaw"] + ext.camera_yaw_offset_deg()
            x_cam = (result["x"] + math.cos(th) * ext.t_bc[0]
                     - math.sin(th) * ext.t_bc[1])
            y_cam = (result["y"] + math.sin(th) * ext.t_bc[0]
                     + math.cos(th) * ext.t_bc[1])
            cam_pose = (x_cam, y_cam, yaw_cam)
        status_text = f"valid={result['valid']}  status={result['status']}"
        view.draw(canvas[0:720, 640:1280], result, matched_ids, cam_pose,
                  44.502, status_text)

        # ---------------- 文本面板 ----------------
        panel = canvas[360:720, 0:640]
        panel[:] = (18, 18, 18)
        lines = [
            ("TASK 2  MONOCULAR LOCALIZATION", (200, 200, 200)),
            (f"x_camera_map   : {result['x']: .3f} m" if result.get("valid")
             else "x_camera_map   : --", (0, 255, 0) if result.get("valid") else (0, 120, 255)),
        ]
        if result.get("valid"):
            th = math.radians(result["yaw"])
            x_cam = result["x"] + math.cos(th) * ext.t_bc[0] - math.sin(th) * ext.t_bc[1]
            y_cam = result["y"] + math.sin(th) * ext.t_bc[0] + math.cos(th) * ext.t_bc[1]
            lines += [
                (f"y_camera_map   : {y_cam: .3f} m", (0, 255, 0)),
                (f"yaw_camera_map : {wrap_deg(result['yaw'] + ext.camera_yaw_offset_deg()): .1f} deg", (0, 255, 0)),
                (f"valid / status : True / {result['status']}", (0, 255, 0)),
                (f"marker_count   : {result['marker_count']}", (220, 220, 220)),
                (f"residual       : {result['residual']:.3f} m", (220, 220, 220)),
            ]
        else:
            lines += [
                ("y_camera_map   : --", (0, 120, 255)),
                ("yaw_camera_map : --", (0, 120, 255)),
                (f"valid / status : False / {result['status']}", (0, 120, 255)),
                (f"marker_count   : {result['marker_count']}", (220, 220, 220)),
                (f"residual       : {result['residual']:.3f} m"
                 if math.isfinite(result.get("residual", float("nan")))
                 else "residual       : --", (220, 220, 220)),
            ]
        n_valid = int(sum(proj["valid"])) if len(dets) else 0
        n_ref = sum(1 for d in dets if d["refined"])
        infer = r.speed.get("inference", 0.0) if hasattr(r, "speed") else 0.0
        lines += [
            (f"det {len(dets)}   refined {n_ref}   ground_ok {n_valid}", (180, 180, 180)),
            (f"FPS {sum(fps_hist) / max(len(fps_hist), 1):5.1f}   infer {infer:4.1f} ms", (180, 180, 180)),
            (f"rig h={ext.height:.2f}m pitch={ext.pitch_down:.1f} roll={ext.roll:.1f}", (150, 150, 150)),
        ]
        if ref is not None:
            if result.get("valid"):
                ex = result["x"] - ref[0]
                ey = result["y"] - ref[1]
                eyaw = wrap_deg(result["yaw"] - ref[2])
                lines.append((f"ref {ref[0]:.2f},{ref[1]:.2f},{ref[2]:.1f}  "
                              f"err {math.hypot(ex, ey):.3f} m / {eyaw:+.1f} deg",
                              (255, 220, 150)))
            else:
                lines.append((f"ref {ref[0]:.2f},{ref[1]:.2f},{ref[2]:.1f}  err --",
                              (255, 220, 150)))
        lines.append(("q quit   s snapshot   r record test point", (120, 120, 120)))

        for k, (text, color) in enumerate(lines):
            cv2.putText(panel, text, (14, 34 + k * 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 1)

        now = time.time()
        fps_hist.append(1.0 / max(now - t_prev, 1e-6))
        t_prev = now
        fps_hist = fps_hist[-30:]

        if frame_idx == 1 or frame_idx % 30 == 0:
            if result.get("valid"):
                print(f"[{frame_idx}] ok x={result['x']:.2f} y={result['y']:.2f} "
                      f"yaw={result['yaw']:.1f} n={result['marker_count']} "
                      f"res={result['residual']:.3f}")
            else:
                print(f"[{frame_idx}] {result['status']} det={len(dets)} "
                      f"ground_ok={n_valid}")

        if not args.no_window:
            cv2.imshow("task2 live locate", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                path = os.path.join(snapshot_dir, time.strftime("locate_%Y%m%d_%H%M%S.jpg"))
                cv2.imwrite(path, canvas)
                print("截图:", path)
            if key == ord("r"):
                row = {
                    "time": time.strftime("%Y-%m-%d %H:%M:%S"),
                    "ref_x": ref[0] if ref else "",
                    "ref_y": ref[1] if ref else "",
                    "ref_yaw": ref[2] if ref else "",
                    "est_x": round(result["x"], 3) if result.get("valid") else "",
                    "est_y": round(result["y"], 3) if result.get("valid") else "",
                    "est_yaw": round(result["yaw"], 2) if result.get("valid") else "",
                    "err_xy_m": (round(math.hypot(result["x"] - ref[0],
                                                  result["y"] - ref[1]), 3)
                                 if ref and result.get("valid") else ""),
                    "err_yaw_deg": (round(wrap_deg(result["yaw"] - ref[2]), 2)
                                    if ref and result.get("valid") else ""),
                    "marker_count": result["marker_count"],
                    "residual": round(result["residual"], 3) if result.get("residual") == result.get("residual") else "",
                    "status": result["status"],
                }
                new_file = not os.path.exists(log_path)
                with open(log_path, "a", newline="", encoding="utf-8-sig") as f:
                    writer = csv.DictWriter(f, fieldnames=list(row.keys()))
                    if new_file:
                        writer.writeheader()
                    writer.writerow(row)
                print("已记录测试点:", row)

        if args.max_frames and frame_idx >= args.max_frames:
            break

    cap.release()
    if not args.no_window:
        cv2.destroyAllWindows()
    if last_res is not None:
        print(f"结束: 最后一帧 status={last_res['status']} "
              f"valid={last_res['valid']}")


if __name__ == "__main__":
    main()