"""任务一：足球实时检测

加载自己训练的 ball 模型，对相机（或视频文件）逐帧推理，
输出检测框、类别、置信度、像素中心、帧号与实时 FPS。

用法：
    python src/ball/detect_ball.py --weights ball_best.pt
    python src/ball/detect_ball.py --weights ball_best.pt --source 0
    python src/ball/detect_ball.py --weights ball_best.pt --source test.mp4 --save task1_demo.mp4
    python src/ball/detect_ball.py --weights ball_best.pt --headless --source test.mp4   # 不弹窗

类别固定为 ball（class_id = 0）。
"""

import argparse
import os
import sys
import time

import cv2

try:
    from ultralytics import YOLO
except ImportError:  # pragma: no cover
    print("需要先安装 ultralytics：pip install ultralytics")
    raise

WINDOW = "CQuick Football Detection"


def open_source(source):
    """source 为数字时打开摄像头，否则当视频路径。"""
    if isinstance(source, str) and source.isdigit():
        source = int(source)
    cap = cv2.VideoCapture(source)
    return cap


def main():
    ap = argparse.ArgumentParser(description="足球实时检测")
    ap.add_argument("--weights", default="ball_best.pt", help="训练得到的权重")
    ap.add_argument("--source", default="0", help="摄像头编号或视频路径")
    ap.add_argument("--conf", type=float, default=0.45, help="置信度阈值")
    ap.add_argument("--iou", type=float, default=0.5, help="NMS IoU 阈值")
    ap.add_argument("--imgsz", type=int, default=640, help="推理尺寸")
    ap.add_argument("--save", default="", help="把结果写成视频（可选）")
    ap.add_argument("--headless", action="store_true", help="不开窗口（配合 --save）")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理多少帧（0=全部）")
    args = ap.parse_args()

    if not os.path.exists(args.weights):
        print("[错误] 找不到权重：%s" % args.weights)
        return 1

    model = YOLO(args.weights)
    cap = open_source(args.source)
    if not cap.isOpened():
        print("[错误] 打不开相机或视频：%s" % args.source)
        return 1
    if args.source in ("0", 0):
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    writer = None
    frame_id = 0
    fps = 0.0
    t_start = time.time()
    t_last = t_start
    print("[信息] 模型：%s" % os.path.basename(args.weights))
    print("[信息] 按 q 退出")

    while True:
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        frame_id += 1

        t0 = time.time()
        results = model.predict(frame, conf=args.conf, iou=args.iou, imgsz=args.imgsz, verbose=False)
        infer_ms = (time.time() - t0) * 1000.0

        # 一帧一结果
        r = results[0]
        for box in r.boxes:
            x1, y1, x2, y2 = [float(v) for v in box.xyxy[0].tolist()]
            conf = float(box.conf[0])
            cls_name = r.names.get(int(box.cls[0]), "ball")
            u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0  # 检测框中心像素

            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.circle(frame, (int(u), int(v)), 4, (0, 0, 255), -1)
            cv2.putText(frame, "%s %.2f" % (cls_name, conf), (int(x1), max(20, int(y1) - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, "(%d,%d)" % (int(u), int(v)), (int(u) + 8, int(v) + 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        # 每 0.5 秒按累计帧数刷新一次 FPS（含读取+推理+绘制）
        now = time.time()
        if now - t_last >= 0.5:
            fps = frame_id / max(now - t_start, 1e-9)
            t_last = now

        hud = "frame %d  %dx%d  FPS %.1f  infer %.1f ms  balls %d" % (
            frame_id, frame.shape[1], frame.shape[0], fps, infer_ms, len(r.boxes))
        cv2.rectangle(frame, (0, 0), (frame.shape[1], 32), (0, 0, 0), -1)
        cv2.putText(frame, hud, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        if writer is None and args.save:
            writer = cv2.VideoWriter(args.save, cv2.VideoWriter_fourcc(*"mp4v"), 25.0,
                                     (frame.shape[1], frame.shape[0]))
        if writer is not None:
            writer.write(frame)

        if args.max_frames and frame_id >= args.max_frames:
            break
        if not args.headless:
            cv2.imshow(WINDOW, frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    if writer is not None:
        writer.release()
    cv2.destroyAllWindows()
    print("[信息] 结束：%d 帧，平均 %.1f FPS" % (frame_id, fps))
    return 0


if __name__ == "__main__":
    sys.exit(main())
