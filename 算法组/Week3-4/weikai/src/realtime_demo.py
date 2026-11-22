#!/usr/bin/env python3
"""
CQuick 第3-4周 任务1：实时足球检测 —— 录屏演示窗口（一屏全覆盖）

左侧：真实相机连续实时画面 + YOLO 检测框 / 类别 / 置信度 / 中心像素 / FPS / 推理耗时 / 相机帧率
右侧：姓名+时间自证 / 模型信息 / 独立测试集指标 / 基线对比 / 误检漏检分析

用法（WSL，需 WSLg）：
  source .venv/bin/activate
  python realtime_demo.py --url http://127.0.0.1:8090 \
      --weights runs/ball_yolov8n_aug2/weights/best.pt --conf 0.25
  按 q 退出。
录屏请用 Windows 侧 ffmpeg gdigrab 录整个桌面（见 record_screen.bat）。
"""
import argparse
import os
import sys
import time
from datetime import datetime

import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--weights", default="runs/ball_yolov8n_aug2/weights/best.pt")
    p.add_argument("--url", default="http://127.0.0.1:8090")
    p.add_argument("--conf", type=float, default=0.25)
    p.add_argument("--iou", type=float, default=0.45)
    p.add_argument("--target-class", default="ball")
    return p.parse_args()


def read_bridge_fps(base_url):
    try:
        import requests
        st = requests.get(base_url.rstrip("/") + "/status", timeout=2).json()
        return float(st.get("measured_fps", 0.0)) or None
    except Exception:
        return None


def txt(img, x, y, s, scale=0.42, color=(230, 230, 230), thick=1):
    cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX,
                scale, color, thick, cv2.LINE_AA)


def draw_panel(panel, now_str, live):
    """右侧信息面板：自证 + 模型 + 指标 + 误检分析 + 实时状态。"""
    panel[:] = (24, 24, 26)
    W = panel.shape[1]
    y = 26
    # 自证
    txt(panel, 12, y, "=== SELF-ID ===", 0.5, (0, 200, 255), 1); y += 24
    txt(panel, 12, y, "Name : weikai", 0.48, (255, 255, 255), 1); y += 22
    txt(panel, 12, y, "Time : %s" % now_str, 0.45, (255, 255, 255), 1); y += 30
    # 模型
    txt(panel, 12, y, "=== MODEL ===", 0.5, (0, 200, 255), 1); y += 24
    txt(panel, 12, y, "YOLOv8n  |  best.pt", 0.45); y += 21
    txt(panel, 12, y, "dual-distractor augmented", 0.42, (170, 170, 170)); y += 21
    txt(panel, 12, y, "classes: ball (nc=1)", 0.45); y += 21
    txt(panel, 12, y, "train: 118 imgs (neg pasted)", 0.42, (170, 170, 170)); y += 30
    # 指标
    txt(panel, 12, y, "=== TEST (V5, 60 imgs) ===", 0.5, (0, 200, 255), 1); y += 24
    for k, v in [("Precision", "0.760"), ("Recall", "0.929"),
                 ("mAP50", "0.925"), ("mAP50-95", "0.849")]:
        txt(panel, 12, y, "  %-10s %s" % (k, v), 0.46, (120, 255, 120), 1); y += 21
    y += 8
    txt(panel, 12, y, "mAP50  0.822 -> 0.925", 0.42, (170, 170, 170)); y += 20
    txt(panel, 12, y, "Recall 0.724 -> 0.929", 0.42, (170, 170, 170)); y += 30
    # 误检/漏检
    txt(panel, 12, y, "=== ERROR CASES ===", 0.5, (0, 200, 255), 1); y += 24
    txt(panel, 12, y, "FP green/red-blue ball:", 0.42, (255, 200, 120)); y += 20
    txt(panel, 12, y, "  copy-paste negatives added", 0.40, (170, 170, 170)); y += 19
    txt(panel, 12, y, "  green 47->0  redblue 8->1", 0.42, (120, 255, 120)); y += 24
    txt(panel, 12, y, "FN small/blurred ball:", 0.42, (255, 200, 120)); y += 20
    txt(panel, 12, y, "  re-annotated 5 frames", 0.40, (170, 170, 170)); y += 19
    txt(panel, 12, y, "  Recall 0.724->0.929", 0.42, (120, 255, 120)); y += 30
    # 实时状态
    txt(panel, 12, y, "=== LIVE ===", 0.5, (0, 200, 255), 1); y += 24
    for line in live:
        txt(panel, 12, y, line, 0.42, (255, 255, 255)); y += 20
    cv2.rectangle(panel, (0, 0), (W - 1, panel.shape[0] - 1), (80, 80, 80), 1)


def main():
    args = parse_args()
    from ultralytics import YOLO
    model = YOLO(args.weights)
    names = model.names

    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from net_camera import NetCamera
    cap = NetCamera(args.url)
    if not cap.isOpened():
        raise SystemExit("无法打开相机 " + args.url)

    nominal = read_bridge_fps(args.url) or 0.0
    print("相机:", args.url, "标称帧率:", round(nominal, 1))
    print("模型:", args.weights, names)
    print("按 q 退出；录屏请用 Windows ffmpeg gdigrab 录整个桌面")

    PANEL_W = 360
    frame_id = 0
    proc_fps = infer_ms = meas_cam = 0.0
    t_prev = time.time()
    cam_ts = []
    live = ["ball count: 0", "FPS: 0.0  infer: 0 ms",
            "cam fps: n%.0f / m%.1f" % (nominal, 0.0)]

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            frame_id += 1
            now_cam = time.time()
            cam_ts.append(now_cam)
            if len(cam_ts) > 31:
                cam_ts.pop(0)
            if len(cam_ts) >= 5:
                span = cam_ts[-1] - cam_ts[0]
                meas_cam = (len(cam_ts) - 1) / span if span > 0 else 0.0

            t0 = time.perf_counter()
            r = model.predict(frame, conf=args.conf, iou=args.iou, verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000.0
            now = time.time()
            dt = now - t_prev
            proc_fps = 1.0 / dt if dt > 0 else 0.0
            t_prev = now

            annot = r.plot(labels=False, conf=False)  # 只画框，标签由下方自定义（信息更全）
            h, w = annot.shape[:2]
            det = []
            for box in r.boxes:
                cid = int(box.cls[0]); cname = names[cid]
                if args.target_class and cname != args.target_class:
                    continue
                conf = float(box.conf[0])
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                u, v = (x1 + x2) // 2, (y1 + y2) // 2
                det.append((cname, conf, x1, y1, x2, y2, u, v))
                # 框上方：类别 / 置信度 / bbox 四坐标 / 中心像素（任务 1.7 全部字段）
                label = "%s %.2f bbox=(%d,%d,%d,%d) c=(%d,%d)" % (
                    cname, conf, x1, y1, x2, y2, u, v)
                cv2.putText(annot, label, (x1, max(y1 - 8, 20)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
                            cv2.LINE_AA)

            # 底部 HUD（实时性能 + 检测结果）
            cv2.rectangle(annot, (0, h - 74), (w, h), (20, 20, 20), -1)
            txt(annot, 10, h - 56,
                "FPS %.1f | infer %.1f ms | cam fps nominal %.0f / measured %.1f"
                % (proc_fps, infer_ms, nominal, meas_cam), 0.5, (0, 255, 0), 1)
            txt(annot, 10, h - 36, "frame %d | ball count %d" % (frame_id, len(det)),
                0.5, (255, 255, 255), 1)
            if det:
                d = det[0]
                detline = "%s conf=%.2f center=(%d,%d) bbox=(%d,%d,%d,%d)" % (
                    d[0], d[1], d[6], d[7], d[2], d[3], d[4], d[5])
            else:
                detline = "no ball detected"
            txt(annot, 10, h - 16, detline, 0.46, (0, 255, 255), 1)

            live = ["ball count: %d" % len(det),
                    "FPS: %.1f  infer: %.1f ms" % (proc_fps, infer_ms),
                    "cam fps: n%.0f / m%.1f" % (nominal, meas_cam),
                    "conf: %s" % (("%.2f" % det[0][1]) if det else "-")]
            if det:
                live.append("bbox=(%d,%d,%d,%d)" % (det[0][2], det[0][3],
                                                     det[0][4], det[0][5]))
            panel = np.zeros((h, PANEL_W, 3), np.uint8)
            draw_panel(panel, datetime.now().strftime("%Y-%m-%d %H:%M:%S"), live)

            canvas = np.hstack([annot, panel])

            # ---- 顶部自证标题栏：姓名 + 实时时间（录屏自证，全程常驻）----
            cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 40), (20, 20, 20), -1)
            txt(canvas, 14, 27,
                "CQuick W3-4 Task1 | weikai | %s | q=quit" %
                datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
                0.6, (255, 255, 255), 1)

            cv2.imshow("TASK1 realtime demo - q=quit", canvas)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("结束：%d 帧，FPS %.1f，infer %.1f ms" % (frame_id, proc_fps, infer_ms))


if __name__ == "__main__":
    main()
