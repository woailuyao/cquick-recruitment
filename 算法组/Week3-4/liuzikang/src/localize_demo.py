#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""任务 2：单目场地定位演示（左右分屏）。"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from field_map import FieldMap, match_pose          # noqa: E402
from ground_projection import GroundProjection      # noqa: E402

CLASS_NAMES = ["LCross", "TCross", "XCross"]
COLORS = {"LCross": (60, 200, 90), "TCross": (90, 150, 255), "XCross": (80, 80, 240)}

def load_intrinsics(path: Path):
    K = np.zeros((3, 3), np.float64)
    dist = np.zeros(5, np.float64)
    rows = 0
    for line in path.read_text(encoding="utf-8").splitlines():
        s = line.strip()
        if s.startswith("[") and rows < 3:
            vals = [float(v) for v in s.strip("[],").split(",")]
            if len(vals) == 3:
                K[rows] = vals
                rows += 1
        elif s.startswith("dist_coeffs:"):
            txt = s.split(":", 1)[1].strip().strip("[]")
            vals = [float(v) for v in txt.split(",") if v.strip()]
            dist[: len(vals)] = vals[:5]
    return K, dist


def draw_field_panel(fmap: FieldMap, pose, size=(640, 720), trail=None):
    """画右半屏：场地俯视图 + 相机位姿。"""
    W, H = size
    panel = np.full((H, W, 3), 24, np.uint8)
    xs = [p["x"] for p in fmap.points]
    ys = [p["y"] for p in fmap.points]
    x0, x1 = min(xs) - 1.0, max(xs) + 1.0
    y0, y1 = min(ys) - 1.0, max(ys) + 1.0
    scale = min(W / (y1 - y0), H / (x1 - x0)) * 0.86

    def P(x, y):
        cx = W / 2 + (0 - (y - (y0 + y1) / 2)) * scale
        cy = H / 2 - (x - (x0 + x1) / 2) * scale
        return int(cx), int(cy)

    cv2.rectangle(panel, P(x0 + 1.0, y1 - 1.0), P(x1 - 1.0, y0 + 1.0), (70, 110, 70), 2)

    for p in fmap.points:
        c = COLORS.get(p["type"], (200, 200, 200))
        cv2.circle(panel, P(p["x"], p["y"]), 6, c, -1)
        cv2.putText(panel, p["name"], (P(p["x"], p["y"])[0] + 8, P(p["x"], p["y"])[1] + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (200, 200, 200), 1, cv2.LINE_AA)

    if pose is not None:
        px, py = P(pose.x, pose.y)
        col = (60, 220, 120) if pose.valid else (80, 80, 230)
        cv2.circle(panel, (px, py), 9, col, -1)
        a = math.radians(pose.yaw_deg)
        dx = -math.sin(a) * 60
        dy = -math.cos(a) * 60
        cv2.arrowedLine(panel, (px, py), (int(px + dx), int(py + dy)), col, 3, tipLength=0.25)

    return panel


def main() -> int:
    ap = argparse.ArgumentParser(description="单目场地定位演示")
    ap.add_argument("--model", default="runs/detect/markers_v2/weights/best.pt")
    ap.add_argument("--intrinsics", default="calibration/camera_intrinsics.yaml")
    ap.add_argument("--field", default="config/field.yaml")
    ap.add_argument("--height", type=float, default=1.10, help="相机光心离地高度（米）")
    ap.add_argument("--pitch", type=float, default=15.0, help="俯仰角（度，镜头向下为正）")
    ap.add_argument("--roll", type=float, default=0.0, help="滚转角（度）")
    ap.add_argument("--side", type=int, default=1, choices=[-1, 0, 1],
                    help="相机在场地哪一侧：+1 表示 +Y 侧，-1 表示 -Y 侧，0 未知")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--video", default="", help="用视频代替相机")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.35)
    ap.add_argument("--tol", type=float, default=0.45, help="匹配内点阈值（米）")
    args = ap.parse_args()

    from ultralytics import YOLO

    if not Path(args.model).exists():
        print(f"找不到模型：{args.model}")
        return 1
    K, dist = load_intrinsics(Path(args.intrinsics))
    fm = FieldMap.load(args.field)
    gp = GroundProjection(K=K, dist=dist, height=args.height,
                          pitch_deg=args.pitch, roll_deg=args.roll)
    model = YOLO(args.model)
    print(f"地图 {len(fm.points)} 个标志 | 相机高度 {args.height} m | 俯仰 {args.pitch}°")

    if args.video:
        cap = cv2.VideoCapture(args.video)
    else:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_DSHOW if sys.platform.startswith("win") else cv2.CAP_V4L2)
    if not cap.isOpened():
        print("打不开视频源")
        return 1

    last_pose = None
    frame_id = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_id += 1

        res = model.predict(frame, imgsz=args.imgsz, conf=args.conf,
                            verbose=False, device="cpu")[0]
        left = frame.copy()
        observed = []
        for b in res.boxes:
            x1, y1, x2, y2 = [float(v) for v in b.xyxy[0]]
            cf = float(b.conf[0])
            name = model.names[int(b.cls[0])]
            u, v = (x1 + x2) / 2, (y1 + y2) / 2
            g = gp.pixel_to_ground(u, v)
            col = COLORS.get(name, (200, 200, 200))
            cv2.rectangle(left, (int(x1), int(y1)), (int(x2), int(y2)), col, 2)
            cv2.circle(left, (int(u), int(v)), 4, (0, 0, 255), -1)
            tag = f"{name} {cf:.2f}" + (f" -> ({g[0]:.2f},{g[1]:.2f})m" if g else " -> inv")
            cv2.putText(left, tag, (int(x1), max(16, int(y1) - 6)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2, cv2.LINE_AA)
            if g is not None:
                observed.append({"type": name, "x": g[0], "y": g[1], "conf": cf})

        pose = match_pose(observed, fm, tol=args.tol, side_hint=args.side,
                          near_pose=(last_pose.x, last_pose.y, last_pose.yaw_deg)
                          if last_pose and last_pose.valid else None)
        if pose.valid:
            last_pose = pose

        right = draw_field_panel(fm, pose)
        status = (f"valid={pose.valid}  status={pose.status}  "
                  f"markers={pose.marker_count}  inliers={pose.inliers}  "
                  f"res={pose.residual:.2f}m")
        cv2.putText(right, status, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                    (60, 220, 120) if pose.valid else (80, 80, 230), 1, cv2.LINE_AA)
        pose_txt = (f"x={pose.x:+.2f} m   y={pose.y:+.2f} m   yaw={pose.yaw_deg:+.1f} deg"
                    if pose.valid else "POSE INVALID (not enough / ambiguous markers)")
        cv2.putText(right, pose_txt, (10, 52), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                    (255, 255, 255), 2, cv2.LINE_AA)

        h = min(left.shape[0], right.shape[0])
        left = cv2.resize(left, (int(left.shape[1] * h / left.shape[0]), h))
        right = cv2.resize(right, (right.shape[1], h))
        view = np.hstack([left, right])
        cv2.imshow("field localization (left: camera | right: map)", view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            fn = f"localize_{frame_id:05d}.jpg"
            cv2.imwrite(fn, view)
            print("截图", fn)
        if key == ord("p"):
            print(f"[{frame_id}] valid={pose.valid} x={pose.x:.3f} y={pose.y:.3f} "
                  f"yaw={pose.yaw_deg:.2f} markers={pose.marker_count} "
                  f"inliers={pose.inliers} residual={pose.residual:.3f}")

    cap.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
