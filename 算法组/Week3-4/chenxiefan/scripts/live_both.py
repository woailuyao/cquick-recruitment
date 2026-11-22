#!/usr/bin/env python3

import argparse
import math
import os
import time
from collections import deque

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
import sys
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import locate_field as lf

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="http://192.168.56.1:8080/video")
    ap.add_argument("--ball-model", default=os.path.join(ROOT, "models", "ball_best.pt"))
    ap.add_argument("--marker-model", default=os.path.join(ROOT, "models", "marker_best.pt"))
    ap.add_argument("--ball-conf", type=float, default=0.4)
    ap.add_argument("--marker-conf", type=float, default=0.25)
    ap.add_argument("--marker-imgsz", type=int, default=960)
    ap.add_argument("--camera", default=os.path.join(ROOT, "config", "camera.yaml"))
    ap.add_argument("--extrinsics", default=os.path.join(ROOT, "config", "camera_extrinsics.yaml"))
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--seconds", type=float, default=0.0, help="0 表示一直跑")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--save", default="")
    return ap.parse_args()

def main():
    args = parse_args()
    from ultralytics import YOLO

    cam = lf.load_yaml(args.camera) or {}
    extr = lf.load_yaml(args.extrinsics) or {}
    field = lf.load_yaml(args.field) or {}
    markers = lf.load_field_markers(field)
    K = np.array(cam["camera_matrix"], float)
    dist = np.array(cam["dist_coeffs"], float)
    R = np.array(extr["R_base_camera"], float)
    t = np.array(extr["t_base_camera"], float)

    ball_model = YOLO(args.ball_model)
    marker_model = YOLO(args.marker_model)

    frames = lf.Frames(args.source)
    if not frames.cap.isOpened():
        print("[错误] 打不开视频源", args.source)
        return 1

    xs = [m["x"] for m in markers]
    ys = [m["y"] for m in markers]
    span = max(max(xs) - min(xs), max(ys) - min(ys), 1.0)
    scale = 560.0 / span
    field_w = (max(xs) - min(xs)) * scale
    origin = ((640 - field_w) / 2 - min(xs) * scale, 600 + min(ys) * scale)

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 15.0, (1280, 720))

    q_valid = deque(maxlen=200)
    q_markers = deque(maxlen=200)
    q_balls = deque(maxlen=200)
    tracker = lf.MarkerTracker()
    smoother = lf.PoseSmoother()
    frame_id = 0
    t0 = time.time()
    t_prev = t0
    last_report = t0
    fps = 0.0
    pose = None
    n_ball = 0
    n_marker = 0

    while True:
        ok, frame = frames.read()
        if not ok:
            break
        frame_id += 1
        now = time.time()
        fps = 1.0 / max(now - t_prev, 1e-6)
        t_prev = now

        res_b = ball_model.predict(frame, conf=args.ball_conf, verbose=False)
        n_ball = 0
        for b in res_b[0].boxes:
            name = res_b[0].names[int(b.cls[0])]
            if name != "ball":
                continue
            n_ball += 1
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            u, v = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
            cv2.putText(frame, "ball %.2f" % float(b.conf[0]), (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        res_m = marker_model.predict(frame, conf=args.marker_conf,
                                     imgsz=args.marker_imgsz, verbose=False)
        obs = []
        raw = []
        for b in res_m[0].boxes:
            name = res_m[0].names[int(b.cls[0])]
            if name not in lf.CLASSES:
                continue
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            cx0, cy0 = (x1 + x2) / 2.0, (y1 + y2) / 2.0
            prior = None
            for trc in tracker.tracks:
                if trc["type"] != name:
                    continue
                tx1, ty1, tx2, ty2 = trc["box"]
                tcx, tcy = (tx1 + tx2) / 2.0, (ty1 + ty2) / 2.0
                if abs(tcx - cx0) < 110 and abs(tcy - cy0) < 110:
                    prior = (trc["u"] + (cx0 - tcx), trc["v"] + (cy0 - tcy))
                    break
            u, v = lf.refine_landmark(frame, name, (x1, y1, x2, y2), prior=prior)
            raw.append({"type": name, "conf": float(b.conf[0]), "u": u, "v": v,
                        "box": (x1, y1, x2, y2)})
        raw = lf.dedupe_detections(raw)
        confirmed = tracker.update(raw)
        n_marker = len(confirmed)
        for tr in confirmed:
            name = tr["type"]
            u, v = tr["u"], tr["v"]
            x1, y1, x2, y2 = [int(z) for z in tr["box"]]
            ground = lf.pixel_to_ground(u, v, K, dist, R, t)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 128, 0), 2)
            cv2.circle(frame, (int(u), int(v)), 3, (0, 0, 255), -1)
            cv2.putText(frame, "%s #%d" % (name, tr["id"]), (x1, max(y1 - 6, 14)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 128, 0), 1)
            if ground is not None:
                obs.append({"type": name, "x": ground[0], "y": ground[1],
                            "conf": tr["conf"]})

        obs = lf.add_virtual_center(obs)
        pose = smoother.update(lf.solve_pose(obs, markers, prior=smoother.prev))

        q_valid.append(1 if pose.get("valid") else 0)
        q_markers.append(n_marker)
        q_balls.append(n_ball)

        cv2.putText(frame, "fps=%.1f ball=%d marker=%d" % (fps, n_ball, n_marker),
                    (10, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(frame, "pose %s markers=%d resid=%.2f" % (
            pose.get("status", ""), pose.get("marker_count", 0), pose.get("residual", 0.0)),
                    (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 1)

        if frame_id % 15 == 0 or args.no_show:
            if now - last_report >= 1.0:
                print("t=%5.1fs frames=%4d fps=%4.1f ball=%d marker=%d status=%s" % (
                    now - t0, frame_id, fps, n_ball, n_marker, pose.get("status", "")))
                last_report = now

        view = None
        if not args.no_show or writer is not None:
            canvas = lf.draw_map(640, markers, pose, scale, origin)
            left = cv2.resize(frame, (576, 324))
            panel = np.zeros((324, 576, 3), np.uint8)
            lines = [
                "frame %d   fps %.1f" % (frame_id, fps),
                "ball %d   marker %d" % (n_ball, n_marker),
                "valid %s   %s" % (pose.get("valid"), pose.get("status", "")),
                "x %.3f  y %.3f  yaw %.1f" % (pose.get("x", 0.0), pose.get("y", 0.0),
                                              pose.get("yaw_deg", 0.0)),
                "markers %d   resid %.3f" % (pose.get("marker_count", 0),
                                             pose.get("residual", 0.0)),
            ]
            for i, txt in enumerate(lines):
                cv2.putText(panel, txt, (12, 48 + i * 54), cv2.FONT_HERSHEY_SIMPLEX,
                            0.8, (0, 255, 255), 2)
            left_col = np.vstack([left, panel])
            right = np.zeros((648, 576, 3), np.uint8)
            right[36:612, :] = cv2.resize(canvas, (576, 576))
            view = np.hstack([left_col, right])
        if writer is not None and view is not None:
            writer.write(view)
        if not args.no_show and view is not None:
            lf.show_window("live both (left: camera, right: map)", view)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        if args.seconds > 0 and now - t0 >= args.seconds:
            break

    frames.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()

    total = max(len(q_valid), 1)
    print("---- 统计 ----")
    print("帧数 %d，耗时 %.1f s，平均 %.1f fps" % (frame_id, time.time() - t0, frame_id / max(time.time() - t0, 1e-6)))
    print("任务一：平均每帧 %.2f 个球，有球帧占比 %.0f%%" % (
        np.mean(q_balls) if q_balls else 0,
        100.0 * sum(1 for x in q_balls if x > 0) / total))
    print("任务二：平均 %.2f 个标志，定位有效率 %.0f%%" % (
        np.mean(q_markers) if q_markers else 0,
        100.0 * sum(q_valid) / total))
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
