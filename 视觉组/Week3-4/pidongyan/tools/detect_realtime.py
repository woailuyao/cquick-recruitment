"""实时足球检测：摄像头 / 视频文件 → YOLO → 检测框、置信度、像素中心、FPS。

这是任务一的最终交付程序。输出字段完全对齐考核要求：
  类别名、置信度、检测框 x1,y1,x2,y2、中心 center_u,center_v、帧号/时间戳、FPS 与推理耗时

用法：
  # 用摄像头（考核现场就是这样跑；默认模型已是 best_train_v4.pt）
  python tools/detect_realtime.py --source 0

  # 用视频文件（开发调试用；不能作为验收证据）
  python tools/detect_realtime.py --source raw_videos/ball_run01_xxx.mp4 --save

  # 无显示器时只打印，不开窗口
  python tools/detect_realtime.py --source 0 --no-show --print-every 1

快捷键：q 退出，s 截图保存当前帧
"""
import argparse
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

from cam_open import open_camera

ROOT = Path(__file__).resolve().parents[1]


def resolve_device(device):
    if device == "auto":
        try:
            import torch
            return 0 if torch.cuda.is_available() else "cpu"
        except Exception:
            return "cpu"
    return device


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--source", default="0", help="0=摄像头索引，或视频文件路径")
    ap.add_argument("--model", default=str(ROOT / "best_train_v4.pt"),
                    help="默认用正式权重 best_train_v4.pt（双 seed 对照后确定）；"
                         "回退 v3 用 --model best_train_v3.pt；yolov8n.pt 是 COCO 预训练模型，"
                         "没有 ball 类，不能用于验收")
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou", type=float, default=0.45)
    ap.add_argument("--device", default="auto", help="auto / 0 / cpu")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--cam-fps", type=float, default=30.0)
    ap.add_argument("--fourcc", default="MJPG",
                    help="摄像头像素格式，默认 MJPG（720p 用 YUY2 在 USB2 上只有 10fps）")
    ap.add_argument("--cam-backend", default="auto", choices=["auto", "dshow", "msmf", "v4l2", "any"])
    ap.add_argument("--save", action="store_true", help="把带框视频存到 runs/detect_out/")
    ap.add_argument("--no-show", action="store_true", help="不开预览窗口")
    ap.add_argument("--print-every", type=int, default=15, help="每 N 帧打印一次检测结果")
    ap.add_argument("--max-frames", type=int, default=0, help="最多处理 N 帧后退出（0=不限，调试用）")
    args = ap.parse_args()

    device = resolve_device(args.device)
    model_path = Path(args.model)
    if not model_path.exists():
        raise SystemExit(f"模型不存在：{model_path}\n先用工具训练，或指定 --model yolov8n.pt")

    model = YOLO(str(model_path))
    names = model.names

    src = args.source
    is_camera = src.isdigit()
    if is_camera:
        fourcc = None if str(args.fourcc).lower() in ("none", "default", "") else args.fourcc
        cap, cinfo = open_camera(int(src), width=args.width, height=args.height,
                                 fps=args.cam_fps, fourcc=fourcc,
                                 backend=args.cam_backend, verify_fps=None, verbose=True)
    else:
        if not Path(src).exists():
            raise SystemExit(f"视频文件不存在：{src}")
        cap = cv2.VideoCapture(src)

    if not cap.isOpened():
        raise SystemExit(f"打不开视频源：{src}（摄像头被占用？索引不对？）")

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = None
    if args.save:
        out_dir = ROOT / "runs" / "detect_out"
        out_dir.mkdir(parents=True, exist_ok=True)
        out_path = out_dir / f"{Path(src).stem if not is_camera else 'camera'}_{time.strftime('%Y%m%d_%H%M%S')}.mp4"
        writer = cv2.VideoWriter(str(out_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                 args.cam_fps, (aw, ah))
        print(f"录制输出：{out_path}")

    print(f"模型 {model_path.name} | device={device} | imgsz={args.imgsz} | "
          f"conf={args.conf} | 输入 {aw}x{ah}")
    print("按 q 退出" if not args.no_show else "无窗口模式，Ctrl+C 退出")

    # 预热：首帧要加载 CUDA kernel，不预热会让 FPS 统计偏低
    model.predict(np.zeros((ah, aw, 3), np.uint8), imgsz=args.imgsz,
                  device=device, verbose=False)

    fps_win = deque(maxlen=30)
    frame_id = 0
    t_start = time.time()

    try:
        while True:
            loop_t0 = time.time()
            ok, frame = cap.read()
            if not ok:
                print("视频源结束或取帧失败")
                break
            if args.max_frames and frame_id >= args.max_frames:
                break

            t_infer = time.time()
            results = model.predict(
                frame, imgsz=args.imgsz, conf=args.conf, iou=args.iou,
                device=device, verbose=False,
            )
            infer_ms = (time.time() - t_infer) * 1000
            r = results[0]

            dets = []
            if r.boxes is not None and len(r.boxes) > 0:
                xyxy = r.boxes.xyxy.cpu().numpy()
                confs = r.boxes.conf.cpu().numpy()
                clss = r.boxes.cls.cpu().numpy().astype(int)
                for (x1, y1, x2, y2), c, k in zip(xyxy, confs, clss):
                    u = (x1 + x2) / 2.0
                    v = (y1 + y2) / 2.0
                    dets.append((int(x1), int(y1), int(x2), int(y2), float(c),
                                 names.get(int(k), str(k)), u, v))

            dets.sort(key=lambda d: -d[4])

            for x1, y1, x2, y2, c, name, u, v in dets:
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2)
                cv2.drawMarker(frame, (int(u), int(v)), (0, 255, 0),
                               cv2.MARKER_CROSS, 14, 2)
                label = f"{name} {c:.2f}"
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
                cv2.rectangle(frame, (x1, max(y1 - th - 8, 0)),
                              (x1 + tw + 6, max(y1, th + 8)), (0, 165, 255), -1)
                cv2.putText(frame, label, (x1 + 3, max(y1 - 5, th + 2)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2, cv2.LINE_AA)

            fps_win.append(1.0 / max(time.time() - loop_t0, 1e-6))
            avg_fps = sum(fps_win) / len(fps_win)
            ts = time.strftime("%H:%M:%S")
            head = f"FPS {avg_fps:5.1f} | infer {infer_ms:5.1f}ms | frame {frame_id} | {ts}"
            cv2.putText(frame, head, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(frame, head, (10, 26), cv2.FONT_HERSHEY_SIMPLEX,
                        0.62, (0, 255, 0), 1, cv2.LINE_AA)
            cv2.putText(frame, f"ball: {len(dets)}", (10, 52),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.62, (0, 255, 0), 2, cv2.LINE_AA)

            if args.print_every > 0 and frame_id % args.print_every == 0:
                if dets:
                    x1, y1, x2, y2, c, name, u, v = dets[0]
                    print(f"frame={frame_id:05d} ts={ts} {name} conf={c:.2f} "
                          f"bbox=({x1},{y1},{x2},{y2}) center=({u:.1f},{v:.1f}) "
                          f"infer={infer_ms:.1f}ms fps={avg_fps:.1f} n={len(dets)}")
                else:
                    print(f"frame={frame_id:05d} ts={ts} 无检测  infer={infer_ms:.1f}ms "
                          f"fps={avg_fps:.1f}")

            if writer is not None:
                writer.write(frame)

            if not args.no_show:
                cv2.imshow("ball detect  (q=quit, s=shot)", frame)
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                if key == ord("s"):
                    shot = ROOT / "check_out" / f"shot_{frame_id:05d}.jpg"
                    shot.parent.mkdir(parents=True, exist_ok=True)
                    cv2.imwrite(str(shot), frame)
                    print(f"截图：{shot}")

            frame_id += 1
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        cap.release()
        if writer is not None:
            writer.release()
        if not args.no_show:
            cv2.destroyAllWindows()

    elapsed = time.time() - t_start
    print(f"\n共 {frame_id} 帧，{elapsed:.1f}s，平均 {frame_id / max(elapsed, 1e-6):.1f} FPS")


if __name__ == "__main__":
    main()
