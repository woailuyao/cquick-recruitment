# -*- coding: utf-8 -*-
"""任务一实时检测：单目相机画面里检测足球并输出全部要求字段。

细则 1.7 要求至少输出：
    类别名 ball、置信度、检测框 x1,y1,x2,y2、框中心 (u,v)、帧号/时间戳、实时 FPS 与推理耗时。

本程序另做两件便于验收的事：
    · 屏幕左上角常驻显示设备/分辨率/实测 FPS/平均推理耗时；
    · 每帧在终端打印一行结构化数据（可截图为证据，也可被其他程序解析）。

用法（项目根目录）：
    .\\run.bat tools\\realtime_ball.py
    .\\run.bat tools\\realtime_ball.py --conf 0.60 --record outputs/realtime/demo_ball.mp4

按键：q 退出；s 存当前帧到 outputs/；空格 暂停/继续。
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, WEIGHTS_DIR, rel          # noqa: E402

try:
    cv2.utils.logging.setLogLevel(cv2.utils.logging.LOG_LEVEL_SILENT)
except Exception:                                                  # noqa: BLE001
    pass

BACKENDS = [("MSMF", cv2.CAP_MSMF), ("DSHOW", cv2.CAP_DSHOW), ("ANY", cv2.CAP_ANY)]
W, H = 1280, 720


def open_camera(index: int):
    for name, api in BACKENDS:
        cap = cv2.VideoCapture(index, api)
        if not cap.isOpened():
            cap.release()
            continue
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
        cap.set(cv2.CAP_PROP_FPS, 30)
        return cap, name
    return None, ""


def main() -> int:
    ap = argparse.ArgumentParser(description="实时足球检测")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_ball.pt"))
    ap.add_argument("--conf", type=float, default=0.50,
                    help="置信度阈值；现场演示建议 0.6 以上（见 README 3.5）")
    ap.add_argument("--device", default="0")
    ap.add_argument("--record", default="", help="可选：把带标注的画面录成视频")
    ap.add_argument("--print-every", type=int, default=15, help="每 N 帧打印一行结构化数据")
    ap.add_argument("--video", default="", help="可选：用视频文件代替相机（快速自检）")
    ap.add_argument("--max-frames", type=int, default=0, help="处理 N 帧后自动退出，0=不限")
    ap.add_argument("--no-show", action="store_true", help="不开窗口（无人值守自检）")
    args = ap.parse_args()

    wpath = Path(args.weights)
    if not wpath.is_absolute():
        wpath = ROOT / wpath
    if not wpath.exists():
        print(f"没找到权重 {rel(wpath)}")
        print("请先训练（tools/train.py），或用 --weights 指定其他权重")
        return 1

    from ultralytics import YOLO
    model = YOLO(str(wpath))

    if args.video:
        vp = Path(args.video)
        if not vp.is_absolute():
            vp = ROOT / vp
        if not vp.exists():
            print(f"没找到视频 {rel(vp)}")
            return 1
        cap = cv2.VideoCapture(str(vp))
        backend = "FILE"
        if not cap.isOpened():
            print(f"打不开视频 {rel(vp)}")
            return 1
    else:
        cap, backend = open_camera(args.index)
        if cap is None:
            print(f"打不开相机 index={args.index}")
            return 1
    got_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    got_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    writer = None
    if args.record:
        rec = Path(args.record)
        if not rec.is_absolute():
            rec = ROOT / rec
        rec.parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(rec), cv2.VideoWriter_fourcc(*"mp4v"), 30.0, (got_w, got_h))
        print(f"录制到 {rel(rec)}")

    print("=" * 78)
    print(f"实时足球检测｜权重 {rel(wpath)}｜conf≥{args.conf}｜设备 index={args.index} "
          f"({backend})｜{got_w}x{got_h}")
    print("按键：q 退出 / s 存图 / 空格 暂停")
    print("=" * 78)

    fps_win = deque(maxlen=30)
    infer_win = deque(maxlen=30)
    frame_idx = 0
    paused = False
    out_dir = OUTPUT_DIR / "realtime"
    out_dir.mkdir(parents=True, exist_ok=True)

    # 预热：首帧要建 CUDA 上下文并分配显存，耗时可达数秒。
    # 若把它算进统计，画面会闪出 "FPS 0.4 / infer 2386ms" 这种误导数字（录屏时尤其难看）。
    print("预热中…", end="", flush=True)
    for _ in range(5):
        ok, frame = cap.read()
        if not ok or frame is None:
            break
        model.predict(frame, conf=args.conf, imgsz=640, device=args.device, verbose=False)
    print("完成")
    t_prev = time.perf_counter()

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("取帧失败：相机可能被占用或已断开")
                break
            frame_idx += 1

            t0 = time.perf_counter()
            res = model.predict(frame, conf=args.conf, imgsz=640, device=args.device,
                                verbose=False)[0]
            infer_ms = (time.perf_counter() - t0) * 1000
            infer_win.append(infer_ms)

            now = time.perf_counter()
            fps_win.append(1.0 / max(now - t_prev, 1e-9))
            t_prev = now
            fps_now = float(np.mean(fps_win)) if fps_win else 0.0
            infer_avg = float(np.mean(infer_win)) if infer_win else 0.0

            dets = []
            if res.boxes is not None and len(res.boxes) > 0:
                for xyxy, cf, cls in zip(res.boxes.xyxy.cpu().numpy(),
                                         res.boxes.conf.cpu().numpy(),
                                         res.boxes.cls.cpu().numpy()):
                    x1, y1, x2, y2 = [float(v) for v in xyxy]
                    u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
                    dets.append({"cls": int(cls), "conf": float(cf),
                                 "x1": x1, "y1": y1, "x2": x2, "y2": y2, "u": u, "v": v})
                    cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 220, 0), 2)
                    cv2.circle(frame, (int(u), int(v)), 3, (0, 0, 255), -1)
                    cv2.putText(frame, f"ball {cf:.2f}", (int(x1), max(int(y1) - 6, 14)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 220, 0), 2)

            # 每 N 帧打一行，无球的帧也打——既能证明「没有球时不会硬凑框」，也便于录屏取证
            if args.print_every > 0 and frame_idx % args.print_every == 0:
                line = (f"frame={frame_idx:6d} ts={datetime.now():%H:%M:%S.%f}"[:-3]
                        + f"  fps={fps_now:5.1f} infer={infer_ms:5.1f}ms balls={len(dets)}")
                if dets:
                    d = dets[0]
                    line += (f"  ball conf={d['conf']:.3f} bbox=({d['x1']:.0f},{d['y1']:.0f},"
                             f"{d['x2']:.0f},{d['y2']:.0f}) center=({d['u']:.0f},{d['v']:.0f})")
                print(line)

            # HUD 垫一层半透明黑底：纯白字压在亮背景（白墙、天窗）上会看不见，
            # 录屏取证时"设备信息"整行会消失——必须垫底。
            overlay = frame.copy()
            cv2.rectangle(overlay, (0, 0), (820, 68), (0, 0, 0), -1)
            cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
            cv2.putText(frame, f"device idx={args.index} {backend} {got_w}x{got_h}", (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            cv2.putText(frame, f"FPS {fps_now:5.1f}  infer {infer_avg:5.1f} ms  "
                               f"balls {len(dets)}  frame {frame_idx}",
                        (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            if paused:
                cv2.putText(frame, "PAUSED", (10, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 0, 255), 2)

            if writer is not None:
                writer.write(frame)
            if not args.no_show:
                cv2.imshow("realtime ball detection", frame)

        if args.no_show:
            if args.max_frames and frame_idx >= args.max_frames:
                break
            continue

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" "):
            paused = not paused
        if key == ord("s"):
            p = out_dir / f"snapshot_{datetime.now():%H%M%S}.png"
            cv2.imwrite(str(p), frame)
            print(f"已存图：{rel(p)}")

        if args.max_frames and frame_idx >= args.max_frames:
            break

    cap.release()
    if writer is not None:
        writer.release()
        print("录制已保存")
    if not args.no_show:
        cv2.destroyAllWindows()
    print(f"共处理 {frame_idx} 帧")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
