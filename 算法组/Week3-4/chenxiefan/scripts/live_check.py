#!/usr/bin/env python3

import argparse
import math
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import locate_field as lf

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="http://192.168.56.1:8080/video")
    ap.add_argument("--model", default=os.path.join(ROOT, "models", "marker_best.pt"))
    ap.add_argument("--camera", default=os.path.join(ROOT, "config", "camera.yaml"))
    ap.add_argument("--extrinsics", default=os.path.join(ROOT, "config", "camera_extrinsics.yaml"))
    ap.add_argument("--field", default=os.path.join(ROOT, "config", "field.yaml"))
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    from ultralytics import YOLO

    cam = lf.load_yaml(args.camera) or {}
    extr = lf.load_yaml(args.extrinsics) or {}
    field = lf.load_yaml(args.field) or {}
    markers = lf.load_field_markers(field)
    K = np.array(cam["camera_matrix"], float)
    dist = np.array(cam["dist_coeffs"], float)
    R = np.array(extr["R_base_camera"], float)
    t = np.array(extr["t_base_camera"], float)
    model = YOLO(args.model)

    if args.source.startswith("http"):
        cap = cv2.VideoCapture(args.source, cv2.CAP_FFMPEG)
    elif args.source.startswith("camera:"):
        cap = cv2.VideoCapture(int(args.source.split(":")[1] or 0), cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    else:
        cap = cv2.VideoCapture(args.source)
    if not cap.isOpened():
        print("[错误] 打不开视频源", args.source)
        return 1

    total = valid = 0
    marker_sum = 0
    t0 = time.time()
    last_report = t0
    while time.time() - t0 < args.seconds:
        ok, frame = cap.read()
        if not ok:
            continue
        total += 1
        res = model.predict(frame, conf=args.conf, imgsz=args.imgsz, verbose=False)
        obs = []
        for b in res[0].boxes:
            name = res[0].names[int(b.cls[0])]
            if name not in lf.CLASSES:
                continue
            u, v = lf.refine_landmark(frame, name, b.xyxy[0].tolist())
            g = lf.pixel_to_ground(u, v, K, dist, R, t)
            if g is not None:
                obs.append({"type": name, "x": g[0], "y": g[1], "conf": float(b.conf[0])})
        obs = lf.add_virtual_center(obs)
        pose = lf.solve_pose(obs, markers)
        marker_sum += pose.get("marker_count", 0)
        if pose.get("valid"):
            valid += 1
        now = time.time()
        if now - last_report >= 1.0:
            print("t=%4.1fs  frames=%3d  valid=%3d  last_status=%s"
                  % (now - t0, total, valid, pose.get("status", "")))
            last_report = now

    cap.release()
    rate = valid / max(total, 1) * 100.0
    print("----")
    print("总帧 %d，有效 %d，有效率 %.0f%%，平均标志数 %.1f"
          % (total, valid, rate, marker_sum / max(total, 1)))
    if rate >= 70:
        print("结论：可以录，有效率足够。")
    elif rate >= 40:
        print("结论：勉强能录，建议换个能看到更多标志的位置。")
    else:
        print("结论：不建议录，画面里标志不足或检测不稳定。")
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
