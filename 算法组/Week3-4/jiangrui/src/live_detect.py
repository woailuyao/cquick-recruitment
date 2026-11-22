# -*- coding: utf-8 -*-
"""实时摄像头 / 视频检测程序。

用途：接单目相机做连续检测，画面上显示检测框、类别、置信度、像素中心、FPS、推理耗时。
录屏演示时直接运行本程序即可。

用法:
    python live_detect.py                                  # 默认相机 0，用足球模型
    python live_detect.py --model D:/.../best.pt --source 1
    python live_detect.py --source D:/vision_data/ball_images/v5_frame_0002.jpg   # 单张图自检
    python live_detect.py --source D:/some/video.mp4                              # 离线视频

按键:
    q  退出
    s  保存当前画面截图到 ./snapshots/
    p  暂停 / 继续
"""

import argparse
import os
import time

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))


def parse_args():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default=os.path.join(ROOT, "weights", "ball_v2_best.pt"),
                    help="模型权重路径（默认 weights/ball_v2_best.pt，下载方式见 README）")
    ap.add_argument("--source", default="0",
                    help="摄像头索引（0/1/...；0 通常是笔记本内置摄像头，外接 USB 相机多为 1）或视频/图片路径")
    ap.add_argument("--imgsz", type=int, default=640, help="推理输入尺寸")
    ap.add_argument("--conf", type=float, default=0.40, help="置信度阈值")
    ap.add_argument("--device", default="0", help="0=GPU, cpu=CPU")
    ap.add_argument("--width", type=int, default=1280, help="摄像头采集宽度")
    ap.add_argument("--height", type=int, default=720, help="摄像头采集高度")
    ap.add_argument("--no-window", action="store_true", help="不显示窗口，只打印统计")
    return ap.parse_args()


def open_source(src, width, height):
    """source 是纯数字则按摄像头打开，否则按文件路径打开。

    USB 相机在 MSMF 下有几个已知问题，这里按"能不能真的读到帧"逐个试：
      1. MSMF 默认分辨率（有的相机一调用 set 改分辨率就整条流崩掉）
      2. MSMF 设成 1280x720
      3. DSHOW 设成 1280x720
    只有分辨率正好等于 (width, height) 且能稳定读到帧才接受；
    都拿不到目标分辨率时，退而用能出图的那个，并明确打印警告。
    """
    if src.isdigit():
        idx = int(src)
        cap, backend = None, None

        def _usable(c):
            ok_n = 0
            for _ in range(12):
                try:
                    ok, _ = c.read()
                except Exception:
                    # OpenCV 5 在流启动失败时可能直接抛 Mat 断言，而不是返回 False
                    ok = False
                ok_n += 1 if ok else 0
            return ok_n >= 6

        def _open(name, api, set_res):
            c = cv2.VideoCapture(idx, api)
            if not c.isOpened():
                c.release()
                return None
            if set_res:
                try:
                    c.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    c.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                except Exception:
                    c.release()
                    return None
            if not _usable(c):
                c.release()
                return None
            w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
            h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
            if (w, h) != (width, height) and not set_res:
                # 有些 UVC 驱动必须先跑起来再改分辨率，改完再验一次
                try:
                    c.set(cv2.CAP_PROP_FRAME_WIDTH, width)
                    c.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
                except Exception:
                    c.release()
                    return None
                if not _usable(c):
                    c.release()
                    return None
                w = int(c.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(c.get(cv2.CAP_PROP_FRAME_HEIGHT))
            return c, name, w, h

        attempts = [("MSMF", cv2.CAP_MSMF, False),
                    ("MSMF", cv2.CAP_MSMF, True),
                    ("DSHOW", cv2.CAP_DSHOW, True)]
        fallback_cfg = None
        for name, api, set_res in attempts:
            r = _open(name, api, set_res)
            if r is None:
                time.sleep(0.8)
                continue
            c, name, w, h = r
            if (w, h) == (width, height):
                cap, backend = c, name
                break
            if fallback_cfg is None:
                fallback_cfg = (name, api, set_res)
            c.release()
            time.sleep(0.8)
        if cap is None and fallback_cfg is not None:
            name, api, set_res = fallback_cfg
            r = _open(name, api, set_res)
            if r is not None:
                cap, backend, _, _ = r
                print(f"警告: 实际分辨率 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
                      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}，"
                      f"不是 {width}x{height}，内参不匹配，定位结果会偏")
        if cap is None:
            raise SystemExit(f"摄像头 {idx} 打不开或读不到帧（MSMF/DSHOW 都失败）。"
                             "换一个索引试试：0 → 1，并确认没有别的程序占用摄像头。")
        print(f"输入源: 摄像头 {idx}（Windows 下 0 通常是内置摄像头，外接 USB 相机多从 1 开始）")
        print(f"采集后端: {backend}  "
              f"{int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
              f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
    else:
        if not os.path.exists(src):
            raise SystemExit(f"找不到输入: {src}")
        cap = cv2.VideoCapture(src)
        if not cap.isOpened():
            raise SystemExit(f"打不开输入源: {src}")
    return cap


def draw_centers(img, boxes, names, confs):
    """在检测框中心画点并标注像素坐标 (u, v)。"""
    for (x1, y1, x2, y2), name, cf in zip(boxes, names, confs):
        u = int(round((x1 + x2) / 2.0))
        v = int(round((y1 + y2) / 2.0))
        cv2.circle(img, (u, v), 5, (0, 0, 255), -1)
        cv2.circle(img, (u, v), 7, (255, 255, 255), 1)
        cv2.putText(img, f"({u},{v})", (u + 9, v - 9),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2)
    return img


def main():
    args = parse_args()

    from ultralytics import YOLO
    if not os.path.exists(args.model):
        raise SystemExit(f"找不到模型: {args.model}")
    model = YOLO(args.model)

    cap = open_source(args.source, args.width, args.height)
    src_fps = cap.get(cv2.CAP_PROP_FPS)
    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    snap_dir = "snapshots"
    os.makedirs(snap_dir, exist_ok=True)

    fps_hist = []
    t_prev = time.time()
    frame_idx = 0
    paused = False
    n_det = 0

    print(f"模型: {args.model}")
    print(f"输入: {args.source}  分辨率: {src_w}x{src_h}  采集帧率: {src_fps:.1f}")
    print("按 q 退出，s 截图，p 暂停")

    while True:
        if not paused:
            ok, frame = cap.read()
            if not ok:
                print("输入结束或读帧失败")
                break
            frame_idx += 1

            r = model.predict(frame, imgsz=args.imgsz, conf=args.conf,
                              verbose=False, device=args.device)[0]

            boxes = r.boxes.xyxy.tolist() if r.boxes is not None else []
            confs = r.boxes.conf.tolist() if r.boxes is not None else []
            cls_ids = r.boxes.cls.tolist() if r.boxes is not None else []
            names = [r.names.get(int(c), str(int(c))) for c in cls_ids]

            img = r.plot() if boxes else frame.copy()

            # 必须在 plot() 之后再画中心点，否则会被 plot 的结果覆盖
            img = draw_centers(img, boxes, names, confs)

            now = time.time()
            fps_hist.append(1.0 / max(now - t_prev, 1e-6))
            t_prev = now
            fps_hist = fps_hist[-30:]
            fps = sum(fps_hist) / len(fps_hist)
            infer_ms = r.speed.get("inference", 0.0) if hasattr(r, "speed") else 0.0
            n_det += len(boxes)

            cv2.rectangle(img, (0, 0), (img.shape[1], 58), (0, 0, 0), -1)
            cv2.putText(img,
                        f"FPS {fps:5.1f}   infer {infer_ms:4.1f}ms   "
                        f"objs {len(boxes)}   frame {frame_idx}",
                        (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(img, f"model {os.path.basename(args.model)}   "
                             f"conf>={args.conf}   imgsz {args.imgsz}",
                        (12, 48), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)
        else:
            img = last_img

        last_img = img

        if not args.no_window:
            cv2.imshow("live_detect", img)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                p = os.path.join(snap_dir, time.strftime("snap_%Y%m%d_%H%M%S.jpg"))
                cv2.imwrite(p, img)
                print("截图已保存:", p)
            elif key == ord("p"):
                paused = not paused
                print("暂停" if paused else "继续")

    if fps_hist:
        avg = sum(fps_hist) / len(fps_hist)
        print(f"共处理 {frame_idx} 帧，累计检出 {n_det} 个目标，"
              f"最近 30 帧平均 {avg:.1f} FPS")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()