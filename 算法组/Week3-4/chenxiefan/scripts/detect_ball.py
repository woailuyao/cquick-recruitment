#!/usr/bin/env python3

import argparse
import os
import time

import cv2

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT_DIR = os.path.join(ROOT, "data", "output")

def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(ROOT, "models", "ball_best.pt"))
    ap.add_argument("--source", default="camera:0",
                    help="camera:0 / video:路径 / image:路径 / http 地址")
    ap.add_argument("--device", type=int, default=None,
                    help="兼容旧用法，等价于 --source camera:N")
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--save", action="store_true")
    ap.add_argument("--no-show", action="store_true")
    ap.add_argument("--realtime", action="store_true",
                    help="按视频原始帧率播放（演示/彩排用）")
    return ap.parse_args()

def draw_box(frame, xyxy, conf, label="ball"):
    x1, y1, x2, y2 = [int(v) for v in xyxy]
    u, v = (x1 + x2) // 2, (y1 + y2) // 2
    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.circle(frame, (u, v), 4, (0, 0, 255), -1)
    cv2.putText(frame, f"{label} {conf:.2f}", (x1, max(y1 - 8, 16)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
    cv2.putText(frame, f"u={u} v={v}", (x1, min(y2 + 20, frame.shape[0] - 8)),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
    return u, v

def main():
    args = parse_args()
    if not os.path.exists(args.model):
        print(f"[错误] 找不到模型 {args.model}，先训练或拷 best.pt 过来")
        return 1

    try:
        from ultralytics import YOLO
    except ImportError:
        print("[错误] 没装 ultralytics，执行 pip install -r requirements.txt")
        return 1

    model = YOLO(args.model)

    spec = args.source if args.device is None else f"camera:{args.device}"
    single = None
    src_fps = 30.0
    src_label = spec
    if spec.startswith("http"):
        cap = cv2.VideoCapture(spec, cv2.CAP_FFMPEG)
    else:
        kind, _, arg = spec.partition(":")
        if kind == "camera":
            cap = cv2.VideoCapture(int(arg or 0), cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
            src_label = "CAMERA"
        elif kind == "video":
            cap = cv2.VideoCapture(arg)
            src_fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
            src_label = "VIDEO FILE (彩排)"
        elif kind == "image":
            single = cv2.imread(arg)
            cap = None
            if single is None:
                print(f"[错误] 读不了图片 {arg}")
                return 1
        else:
            print(f"[错误] 不认识的 --source：{spec}")
            return 1

    if cap is not None and not cap.isOpened():
        print(f"[错误] 打不开 {spec}")
        return 1

    writer = None

    frame_id = 0
    win_ready = False
    t_prev = time.time()
    while True:
        if single is not None:
            ok, frame = True, single.copy()
        else:
            ok, frame = cap.read()
        if not ok:
            if frame_id == 0:
                print("[错误] 取帧失败")
            break
        frame_id += 1

        t0 = time.time()
        results = model.predict(frame, conf=args.conf, verbose=False)
        infer_ms = (time.time() - t0) * 1000

        for box in results[0].boxes:
            cls_id = int(box.cls[0])
            name = results[0].names.get(cls_id, str(cls_id))
            if name != "ball":
                continue
            draw_box(frame, box.xyxy[0].tolist(), float(box.conf[0]), name)

        now = time.time()
        fps = 1.0 / max(now - t_prev, 1e-6)
        t_prev = now
        cv2.putText(frame, f"frame={frame_id} fps={fps:.1f} infer={infer_ms:.1f}ms",
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(frame, src_label, (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if args.save:
            if writer is None:
                os.makedirs(OUT_DIR, exist_ok=True)
                h, w = frame.shape[:2]
                writer = cv2.VideoWriter(os.path.join(OUT_DIR, "ball_demo.mp4"),
                                         cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (w, h))
            writer.write(frame)
        if not args.no_show:
            if not win_ready:
                cv2.namedWindow("ball", cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
                cv2.moveWindow("ball", 0, 0)
                win_ready = True
            cv2.imshow("ball", cv2.resize(frame, (1152, 648)))
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
        elif single is not None:
            break

        if args.realtime and single is None:
            remain = 1.0 / max(src_fps, 1.0) - (time.time() - now)
            if remain > 0:
                time.sleep(remain)

    if cap is not None:
        cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    return 0

if __name__ == "__main__":
    raise SystemExit(main())
