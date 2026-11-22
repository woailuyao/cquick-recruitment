#!/usr/bin/env python3
"""
realtime_detect.py — CQuick 第 3-4 周 任务 1：实时足球检测
加载自己训练的 YOLO 权重，对相机实时流检测，输出细则要求的全部字段：

  类别名 / 置信度 / bbox / 像素中心 (u,v) / 处理 FPS /
  单帧推理时间 (ms) / 相机标称帧率 / 相机实测采集帧率

HUD 分多行常驻画面顶部与底部，便于录屏展示。

用法（WSL，需 WSLg）：
  cd ~/vision_ws/k1_soccer_vision && source .venv/bin/activate
  python realtime_detect_v2.py --url http://127.0.0.1:8090 \
      --weights runs/ball_yolov8n_aug2/weights/best.pt --conf 0.25 \
      --csv runs/realtime_log.csv --save runs/realtime_demo.mp4
"""
import argparse
import csv
import os
import sys
import time

import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser(description="YOLO 实时足球检测（任务 1）")
    p.add_argument("--weights", default="runs/ball_yolov8n_aug2/weights/best.pt")
    p.add_argument("--device", default="/dev/video0")
    p.add_argument("--url", default=None, help="网络相机地址，优先于 --device")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--target-class", default="ball")
    p.add_argument("--camera-fps", type=float, default=0.0,
                   help="相机标称帧率；0=自动读取")
    p.add_argument("--save", default=None, help="保存标注视频")
    p.add_argument("--csv", default=None, help="逐帧检测记录 CSV")
    return p.parse_args()


def read_bridge_fps(base_url):
    """从 Windows 摄像头桥 /status 读实测采集帧率；失败返回 None。"""
    try:
        import requests
        st = requests.get(base_url.rstrip("/") + "/status", timeout=2).json()
        return float(st.get("measured_fps", 0.0)) or None
    except Exception:
        return None


def put_lines(img, lines, x, y, color, scale=0.55, thick=1, gap=19):
    for i, text in enumerate(lines):
        cv2.putText(img, text, (x, y + i * gap), cv2.FONT_HERSHEY_SIMPLEX,
                    scale, color, thick, cv2.LINE_AA)


def main():
    args = parse_args()
    from ultralytics import YOLO
    model = YOLO(args.weights)
    names = model.names

    bridge_url = args.url
    if args.url:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from net_camera import NetCamera
        cap = NetCamera(args.url)
        src_desc = "network " + args.url
    else:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"M", "J", "P", "G"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        src_desc = args.device
    if not cap.isOpened():
        raise SystemExit("无法打开相机 " + src_desc)

    # 标称采集帧率：网络模式取桥参数，本地取 CAP_PROP_FPS
    nominal_fps = args.camera_fps
    if nominal_fps <= 0:
        if args.url:
            nominal_fps = read_bridge_fps(bridge_url) or 0.0
        else:
            nominal_fps = cap.get(cv2.CAP_PROP_FPS) or 0.0

    csv_fh = csv_wr = None
    if args.csv:
        os.makedirs(os.path.dirname(args.csv) or ".", exist_ok=True)
        csv_fh = open(args.csv, "w", newline="", encoding="utf-8")
        csv_wr = csv.writer(csv_fh)
        csv_wr.writerow(["frame", "timestamp_s", "ball_count",
                         "class_name", "conf", "x1", "y1", "x2", "y2",
                         "center_u", "center_v", "proc_fps", "infer_ms",
                         "camera_fps_nominal", "camera_fps_measured"])

    print("相机:", src_desc, "标称帧率:", round(nominal_fps, 1))
    print("模型:", args.weights, "类别:", names)
    print("按 q 退出，按 s 暂停/继续")

    frame_id = 0
    proc_fps = 0.0
    infer_ms = 0.0
    measured_cam_fps = 0.0
    t_prev = time.time()
    t_last_status = 0.0
    cam_timestamps = []
    paused = False
    grab_fail = 0
    writer = None

    try:
        while True:
            if not paused:
                ok, frame = cap.read()
                if not ok:
                    grab_fail += 1
                    print("[warn] 取帧失败 %d" % grab_fail)
                    if grab_fail >= 30:
                        raise SystemExit("连续取帧失败，退出")
                    time.sleep(0.1)
                    continue
                grab_fail = 0
                frame_id += 1

                # 相机实测采集帧率：按帧到达时间间隔统计（滑动 30 帧）
                now_cam = time.time()
                cam_timestamps.append(now_cam)
                if len(cam_timestamps) > 31:
                    cam_timestamps.pop(0)
                if len(cam_timestamps) >= 5:
                    span = cam_timestamps[-1] - cam_timestamps[0]
                    measured_cam_fps = (len(cam_timestamps) - 1) / span if span > 0 else 0.0

                # ---- 推理（单独计时）----
                t_infer0 = time.perf_counter()
                results = model.predict(frame, conf=args.conf, iou=args.iou,
                                        verbose=False)
                infer_ms = (time.perf_counter() - t_infer0) * 1000.0

                now = time.time()
                dt = now - t_prev
                proc_fps = 1.0 / dt if dt > 0 else 0.0
                t_prev = now

                # 每 3 秒从桥刷新一次相机实测帧率（更权威）
                if args.url and now - t_last_status > 3.0:
                    bf = read_bridge_fps(bridge_url)
                    if bf:
                        measured_cam_fps = bf
                    t_last_status = now

                r = results[0]
                annot = r.plot()
                h, w = annot.shape[:2]

                det_lines = []
                valid_boxes = []
                for box in r.boxes:
                    cid = int(box.cls[0])
                    cname = names[cid]
                    if args.target_class and cname != args.target_class:
                        continue
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    det_lines.append("%s %.2f (u=%d,v=%d)" % (cname, conf, u, v))
                    valid_boxes.append((cname, conf, x1, y1, x2, y2, u, v))
                    # 框上标签：类别名 + 置信度 + 中心
                    label = "%s %.2f c=(%d,%d)" % (cname, conf, u, v)
                    cv2.putText(annot, label, (x1, max(y1 - 6, 16)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                                cv2.LINE_AA)
                # 每一帧都记录性能；检测框逐行跟随（无框则写一行空检测）
                if csv_wr:
                    perf = ["%.2f" % proc_fps, "%.2f" % infer_ms,
                            "%.2f" % nominal_fps, "%.2f" % measured_cam_fps]
                    if not valid_boxes:
                        csv_wr.writerow([frame_id, "%.3f" % now, 0, "", "", "",
                                         "", "", "", "", ""] + perf)
                    else:
                        for cname, conf, x1, y1, x2, y2, u, v in valid_boxes:
                            csv_wr.writerow([frame_id, "%.3f" % now,
                                             len(valid_boxes), cname, "%.3f" % conf,
                                             x1, y1, x2, y2, u, v] + perf)

                # ---- HUD：顶部模型信息，底部性能与结果 ----
                cv2.rectangle(annot, (0, 0), (w, 46), (20, 20, 20), -1)
                put_lines(annot, [
                    "MODEL: %s  |  classes: %s" % (os.path.basename(args.weights),
                                                   ",".join(names.values())),
                    "camera: %s  |  %dx%d" % ("HTTP bridge" if args.url else args.device, w, h),
                ], 10, 17, (255, 255, 255), scale=0.5, gap=17)

                cv2.rectangle(annot, (0, h - 74), (w, h), (20, 20, 20), -1)
                put_lines(annot, [
                    "FPS(proc) %.1f  |  infer %.1f ms  |  camera fps: nominal %.0f / measured %.1f"
                    % (proc_fps, infer_ms, nominal_fps, measured_cam_fps),
                    "frame %d  |  ball count %d" % (frame_id, len(det_lines)),
                    (" | ".join(det_lines[:2]) if det_lines else "no ball detected"),
                ], 10, h - 58, (0, 255, 0), scale=0.5, gap=17)

                if writer is None and args.save:
                    os.makedirs(os.path.dirname(args.save) or ".", exist_ok=True)
                    fourcc = cv2.VideoWriter_fourcc(*"m", "p", "4", "v")
                    writer = cv2.VideoWriter(args.save, fourcc,
                                             measured_cam_fps if 5 < measured_cam_fps < 61
                                             else 30.0, (w, h))
                if writer is not None:
                    writer.write(annot)

                cv2.imshow("task1 realtime detect (q=quit s=pause)", annot)
            else:
                cv2.imshow("task1 realtime detect (q=quit s=pause)", annot)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                paused = not paused
                print("已暂停" if paused else "继续")
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if csv_fh is not None:
            csv_fh.close()
        cv2.destroyAllWindows()
        print("结束：共 %d 帧，处理 FPS %.1f，推理 %.1f ms，相机实测 %.1f fps"
              % (frame_id, proc_fps, infer_ms, measured_cam_fps))
        for pth in (args.csv, args.save):
            if pth:
                print("已保存:", pth)


if __name__ == "__main__":
    main()
