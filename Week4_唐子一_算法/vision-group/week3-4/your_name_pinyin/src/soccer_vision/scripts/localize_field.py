"""任务2：单目场地自定位（纯 Python 版，无 ROS2 也可跑通）。

流程：加载 field.yaml + 相机内外参 -> YOLO 检测 L/T/X/点球点 ->
      几何细化 -> 像素投影到地面(Z=0) -> 地图匹配求 SE(2) 位姿 ->
      左右分屏可视化。

用法：
    python localize_field.py \
        --field config/field.yaml --geometry config/field_geometry.yaml \
        --intrinsics calibration/camera_intrinsics.yaml \
        --extrinsics calibration/camera_extrinsics.yaml \
        --marker_model best_marker.pt --device /dev/video0
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from soccer_vision.camera_model import CameraGeometry, Extrinsics, Intrinsics  # noqa: E402
from soccer_vision.field_map import FieldMap  # noqa: E402
from soccer_vision.landmark_refine import refine  # noqa: E402
from soccer_vision.localizer import Localizer, Observation  # noqa: E402
from soccer_vision.visualizer import draw_map, draw_detections, combine  # noqa: E402
from ultralytics import YOLO  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--field", required=True)
    ap.add_argument("--geometry", required=True)
    ap.add_argument("--intrinsics", required=True)
    ap.add_argument("--extrinsics", required=True)
    ap.add_argument("--marker_model", default="best_marker.pt")
    ap.add_argument("--device", default="/dev/video0")
    ap.add_argument("--conf", type=float, default=0.5)
    args = ap.parse_args()

    # 加载地图 + 相机
    field = FieldMap(args.field, args.geometry)
    intr = Intrinsics.from_yaml(args.intrinsics)
    extr = Extrinsics.from_yaml(args.extrinsics)
    cam = CameraGeometry(intr, extr)
    localizer = Localizer(
        field,
        approx_x=-2.0,
        approx_y=0.0,
        approx_radius=2.0,
        inlier_dist=0.6,
        max_residual=1.2,
    )


    model = YOLO(args.marker_model)

    cap = cv2.VideoCapture(args.device)
    if not cap.isOpened():
        print("[FAIL] 无法打开相机")
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, intr.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, intr.height)

    print(f"加载 field.yaml v{field.version}, 标志点 {len(field.markers)} 个")
    print("开始实时定位，按 q 退出")

    frame_id = 0
    fps = 0.0
    t0 = time.time()
    last_results = None
    last_gray = None

    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame_id += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

                # 隔帧检测：每 3 帧跑一次 YOLO，其余帧复用上一次结果，大幅降低 CPU 负担
        if frame_id % 3 == 1:
            results = model(frame, conf=args.conf, imgsz=320, verbose=False)
            last_results = results
            last_gray = gray
        else:
            results = last_results
            gray = last_gray
        boxes = results[0].boxes


        # 收集观测
        observations = []
        det_boxes, det_names, det_confs, det_centers = [], [], [], []
        for box in boxes:
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            conf = float(box.conf[0])
            cls = int(box.cls[0])
            name = results[0].names[cls]
            lm = refine(gray, name, (x1, y1, x2, y2), conf)
            det_boxes.append((x1, y1, x2, y2))
            det_names.append(name)
            det_confs.append(conf)
            if lm is not None:
                det_centers.append((lm.u, lm.v))
                # 地面投影
                P = cam.project_to_ground(lm.u, lm.v)
                if P is not None:
                    observations.append(
                        Observation(type=lm.type, x=float(P[0]), y=float(P[1]), confidence=conf)
                    )
            else:
                det_centers.append(((x1 + x2) / 2, (y1 + y2) / 2))

        # 定位
                # 调试：打印每个观测的类型和地图坐标

        pose = localizer.localize(observations)

        # 终端输出
        print(
            f"[frame {frame_id}] valid={pose.valid} n={pose.marker_count} "
            f"res={pose.residual:.3f} status={pose.status} "
            f"x={pose.x:.3f} y={pose.y:.3f} yaw={pose.yaw_deg:.2f}"
        )

        # 可视化
        left = draw_detections(
            frame, det_boxes, det_names, det_confs, det_centers, fps, (intr.width, intr.height)
        )
        right = draw_map(field, pose)
        combined = combine(left, right)
        cv2.imshow("field localization", combined)

        if frame_id % 30 == 0:
            dt = time.time() - t0
            fps = 30 / dt
            t0 = time.time()

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
