#!/usr/bin/env python3
"""
realtime_detect.py — CQuick 第 3-4 周 任务 1：实时足球检测
加载 YOLO 模型对相机实时流做检测，输出细则要求的全部字段：
  类别名、confidence、bbox (x1,y1,x2,y2)、像素中心 (u,v)、帧号/时间戳、实时 FPS

用法（WSL，需 GUI 支持 WSLg，Ubuntu-22.04 自带）：
  cd ~/vision_ws/k1_soccer_vision && source .venv/bin/activate
  # 冒烟测试：用预训练权重（此时检测的是 COCO 80 类，仅验证链路通）
  python realtime_detect.py --weights yolov8n.pt --device /dev/video0
  # 正式演示：训练完用自己的 best.pt
  python realtime_detect.py --weights runs/detect/train/weights/best.pt \
      --device /dev/video0 --conf 0.25 --save runs/demo.mp4

注意：
  - 只输出像素级位置（bbox/中心点/朝向），不算距离或三维球心（细则明确不做）。
  - 每次开机先做：usbipd attach --wsl --busid 1-9（Windows 管理员 PowerShell）
    和 sudo chmod 666 /dev/video*（WSL）。
"""

import argparse
import csv
import os
import sys
import time

import cv2


def parse_args():
    p = argparse.ArgumentParser(description="YOLO 实时检测（任务 1）")
    p.add_argument("--weights", default="yolov8n.pt", help="模型权重路径")
    p.add_argument("--device", default="/dev/video0", help="本地相机设备节点")
    p.add_argument("--url", default=None,
                   help="网络相机地址(如 http://172.x.x.x:8090)，优先于 --device")
    p.add_argument("--width", type=int, default=1280)
    p.add_argument("--height", type=int, default=720)
    p.add_argument("--conf", type=float, default=0.25, help="置信度阈值")
    p.add_argument("--iou", type=float, default=0.45, help="NMS IoU 阈值")
    p.add_argument("--target-class", default=None,
                   help="只显示该类别名（如 ball）；不填则显示全部类别")
    p.add_argument("--save", default=None, help="保存标注视频的输出路径（可选）")
    p.add_argument("--csv", default=None, help="逐帧检测记录 CSV 输出路径（可选）")
    return p.parse_args()


def main():
    args = parse_args()

    # ---- 模型（延迟导入，让 --help 不依赖 ultralytics）----
    from ultralytics import YOLO
    model = YOLO(args.weights)
    names = model.names  # {class_id: class_name}

    # ---- 相机：本地 V4L2 或网络 MJPEG ----
    if args.url:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from net_camera import NetCamera
        cap = NetCamera(args.url)
        got_w, got_h = args.width, args.height
        print(f"相机: 网络模式 {args.url} @ {got_w}x{got_h}")
    else:
        cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        got_w, got_h = int(cap.get(3)), int(cap.get(4))
        print(f"相机: {args.device} @ {got_w}x{got_h}（申请 {args.width}x{args.height}）")
    if not cap.isOpened():
        raise SystemExit(f"无法打开相机 {args.url or args.device}")
    writer = None
    csv_fh, csv_wr = None, None
    if args.csv:
        csv_fh = open(args.csv, "w", newline="", encoding="utf-8")
        csv_wr = csv.writer(csv_fh)
        csv_wr.writerow(["frame", "timestamp_s", "class_id", "class_name",
                         "conf", "x1", "y1", "x2", "y2", "center_u", "center_v",
                         "fps"])

    print(f"模型: {args.weights}，类别: {names}")
    print("按 q 退出，按 s 暂停/继续")

    frame_id = 0
    fps = 0.0
    t_prev = time.time()
    paused = False
    grab_fail = 0

    try:
        while True:
            if not paused:
                ok, frame = cap.read()
                if not ok:
                    grab_fail += 1
                    print(f"[warn] 取帧失败（第 {grab_fail} 次）——设备被占用或断开？")
                    if grab_fail >= 30:
                        raise SystemExit("连续取帧失败，退出")
                    time.sleep(0.1)
                    continue
                grab_fail = 0
                frame_id += 1

                # ---- 推理 ----
                results = model.predict(frame, conf=args.conf, iou=args.iou,
                                        verbose=False)
                now = time.time()
                dt = now - t_prev
                fps = 1.0 / dt if dt > 0 else 0.0
                t_prev = now

                r = results[0]
                annot = r.plot()  # ultralytics 自带画框
                for box in r.boxes:
                    cid = int(box.cls[0])
                    cname = names[cid]
                    if args.target_class and cname != args.target_class:
                        continue
                    conf = float(box.conf[0])
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                    u, v = (x1 + x2) // 2, (y1 + y2) // 2
                    # 细则要求的核心数值，左上角逐条打出
                    info = f"{cname} {conf:.2f} bbox=({x1},{y1},{x2},{y2}) c=({u},{v})"
                    cv2.putText(annot, info, (x1, max(y1 - 8, 14)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    if csv_wr:
                        csv_wr.writerow([frame_id, f"{now:.3f}", cid, cname,
                                         f"{conf:.3f}", x1, y1, x2, y2, u, v,
                                         f"{fps:.2f}"])

                hud = f"frame {frame_id}  fps {fps:.1f}  {os.path.basename(args.weights)}"
                cv2.putText(annot, hud, (10, annot.shape[0] - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                if writer is None and args.save:
                    os.makedirs(os.path.dirname(args.save) or ".", exist_ok=True)
                    writer = cv2.VideoWriter(
                        args.save, cv2.VideoWriter_fourcc(*"mp4v"),
                        fps if fps > 1 else 30.0, (annot.shape[1], annot.shape[0]))
                if writer is not None:
                    writer.write(annot)

                cv2.imshow("task1 realtime detect (q=quit s=pause)", annot)
            else:
                cv2.imshow("task1 realtime detect (q=quit s=pause)",
                           annot if frame_id else frame)

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
        print(f"结束：共 {frame_id} 帧，最后 fps {fps:.1f}")
        if args.csv:
            print(f"检测记录已保存: {args.csv}")
        if args.save:
            print(f"标注视频已保存: {args.save}")


if __name__ == "__main__":
    main()
