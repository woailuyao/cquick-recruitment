"""实时足球检测 v2：加了两项抗抖动处理。

1. --min-hits：同一目标要连续 N 帧都被检测到才画框，抑制闪动；
2. 框坐标做指数平滑，画面更稳；
3. 默认 --conf 提到 0.45，减少把场地弧线误判成球。

用法：
    python detect2.py --weights runs/detect/runs/ball_v8n/weights/best.pt
    python detect2.py --weights ... --conf 0.5 --min-hits 3 --save demo.avi --csv outputs/demo.csv

按键：q 退出，g 切换中心点显示。
"""
import argparse
import csv
import os
import time

import cv2

import camera


def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    if iw <= 0 or ih <= 0:
        return 0.0
    inter = iw * ih
    return inter / ((ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter)


class Track:
    __slots__ = ("box", "conf", "hits", "miss")

    def __init__(self, box, conf):
        self.box = box
        self.conf = conf
        self.hits = 1
        self.miss = 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", default="runs/ball_v8n/weights/best.pt")
    ap.add_argument("--conf", type=float, default=0.45)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--min-hits", type=int, default=3, help="连续命中几帧才显示")
    ap.add_argument("--smooth", type=float, default=0.6, help="旧框权重，越大越平滑")
    ap.add_argument("--ignore-top", type=float, default=0.0, help="丢弃中心在画面上部这个比例以内的检测（0~1）")
    ap.add_argument("--max-aspect", type=float, default=0.0, help="框的宽高比上限，超过则丢弃（0 表示不过滤）")
    ap.add_argument("--save", default=None)
    ap.add_argument("--csv", default=None)
    args = ap.parse_args()

    if not os.path.exists(args.weights):
        print(f"找不到权重 {args.weights}")
        return 1

    from ultralytics import YOLO

    model = YOLO(args.weights)
    cap, frame, info = camera.auto_open()
    real_fps = camera.measure_fps(cap, 15)
    h, w = frame.shape[:2]
    print(f"相机 {w}x{h} 实测 {real_fps} fps | 模型 {args.weights}")
    print(f"conf={args.conf} min_hits={args.min_hits}")

    writer = None
    if args.save:
        writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"MJPG"),
                                 real_fps or 30, (w, h))
        print(f"录屏 -> {args.save}")

    csv_file = csv_writer = None
    if args.csv:
        csv_file = open(args.csv, "w", newline="", encoding="utf-8-sig")
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow(["frame", "timestamp", "cls", "conf", "x1", "y1", "x2", "y2",
                             "center_u", "center_v", "infer_ms", "fps"])

    tracks = []
    show_center = True
    fps_ema = 0.0
    frame_idx = 0
    t_start = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            print("取帧失败，结束")
            break

        t0 = time.perf_counter()
        # 用很低的阈值推理，才能看到「低于阈值的检测」——用于讲解漏检
        res = model.predict(frame, conf=0.03, iou=args.iou, imgsz=args.imgsz,
                            verbose=False)[0]
        infer_ms = (time.perf_counter() - t0) * 1000
        dt = time.perf_counter() - t0
        fps_ema = (1.0 / dt) if fps_ema == 0 else 0.9 * fps_ema + 0.1 * (1.0 / dt)

        dets = []
        raw_max = 0.0
        if res.boxes is not None and len(res.boxes):
            for xyxy, conf, cls in zip(res.boxes.xyxy.tolist(), res.boxes.conf.tolist(),
                                       res.boxes.cls.tolist()):
                if float(conf) > raw_max:
                    raw_max = float(conf)
                if float(conf) >= args.conf:
                    dets.append((list(xyxy), float(conf), int(cls)))

        # 简单关联：按 IoU 把检测分配给最近的轨迹
        for t in tracks:
            t.miss += 1
        for box, conf, cls in dets:
            best, best_iou = None, 0.3
            for t in tracks:
                s = iou(box, t.box)
                if s > best_iou:
                    best, best_iou = t, s
            if best is None:
                tracks.append(Track(box, conf))
            else:
                a = args.smooth
                best.box = [a * ob + (1 - a) * nb for ob, nb in zip(best.box, box)]
                best.conf = conf
                best.hits += 1
                best.miss = 0
        tracks = [t for t in tracks if t.miss <= 5]

        canvas = frame
        shown = 0
        for t in tracks:
            if t.hits < args.min_hits or t.miss > 0:
                continue
            bx1, by1, bx2, by2 = t.box
            cy_norm = (by1 + by2) / 2 / h
            if args.ignore_top > 0 and cy_norm < args.ignore_top:
                continue
            bw = max(1e-6, bx2 - bx1)
            bh = max(1e-6, by2 - by1)
            if args.max_aspect > 0 and max(bw / bh, bh / bw) > args.max_aspect:
                continue
            x1, y1, x2, y2 = [int(v) for v in t.box]
            u, v = (x1 + x2) / 2, (y1 + y2) / 2
            cv2.rectangle(canvas, (x1, y1), (x2, y2), (0, 200, 0), 2)
            cv2.putText(canvas, f"ball {t.conf:.2f} ({int(u)},{int(v)})",
                        (x1, max(18, y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)
            if show_center:
                cv2.circle(canvas, (int(u), int(v)), 4, (0, 0, 255), -1)
            shown += 1
            if csv_writer:
                csv_writer.writerow([frame_idx, time.strftime("%H:%M:%S"), "ball",
                                     f"{t.conf:.4f}", x1, y1, x2, y2,
                                     f"{u:.1f}", f"{v:.1f}", f"{infer_ms:.1f}", f"{fps_ema:.1f}"])

        hud = (f"{w}x{h} {real_fps:.0f}fps | infer {infer_ms:.1f}ms | loop {fps_ema:.1f}fps"
               f" | conf={args.conf} raw={raw_max:.2f} | ball {shown} | frame {frame_idx}")
        cv2.rectangle(canvas, (0, 0), (w, 30), (0, 0, 0), -1)
        cv2.putText(canvas, hud, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        if writer:
            writer.write(canvas)
        cv2.imshow("ball detect", canvas)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        if key == ord("g"):
            show_center = not show_center
        frame_idx += 1

    total = time.time() - t_start
    cap.release()
    if writer:
        writer.release()
    if csv_file:
        csv_file.close()
    cv2.destroyAllWindows()
    print(f"处理 {frame_idx} 帧，平均 {frame_idx / total:.1f} fps")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

