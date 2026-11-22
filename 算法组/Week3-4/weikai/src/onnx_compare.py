# -*- coding: utf-8 -*-
"""onnx_compare.py —— PyTorch (.pt) 与 ONNX Runtime (.onnx) 同段推理对比

加分项：导出 ONNX 后，在同一测试视频的同一批帧上分别用 PyTorch 与
ONNX Runtime 跑同一模型，对比检测一致性与延迟 / FPS。

对比口径：
  * 两者均在 CPU 上推理（onnxruntime 在本机 WSL 内无 CUDA EP，
    PyTorch 也强制 device=cpu，保证同设备公平对比）；
  * 前后处理均走 ultralytics 的同一套 predict 链路（AutoBackend），
    letterbox / NMS 完全一致，差异只来自算子实现；
  * 前 warmup 帧不计入延迟统计。

用法：
  python onnx_compare.py --pt <weights.pt> --onnx <weights.onnx> \
      --video /mnt/d/captures/M10.mp4 --conf 0.05 --max-frames 100
"""
import argparse
import statistics
import time

import cv2


def boxes_of(res):
    out = []
    for bx in res.boxes:
        x1, y1, x2, y2 = [float(v) for v in bx.xyxy[0]]
        out.append((int(bx.cls[0]), float(bx.conf[0]),
                    (x1 + x2) / 2, (y1 + y2) / 2, x1, y1, x2, y2))
    return out


def iou(a, b):
    ax1, ay1, ax2, ay2 = a[4:8]
    bx1, by1, bx2, by2 = b[4:8]
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def stat(xs):
    xs = sorted(xs)
    n = len(xs)
    return (statistics.mean(xs),
            xs[n // 2],
            xs[min(n - 1, int(n * 0.95))])


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--pt", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--video", required=True)
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--stride", type=int, default=1)
    ap.add_argument("--max-frames", type=int, default=100)
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--iou-thr", type=float, default=0.5)
    args = ap.parse_args()

    from ultralytics import YOLO
    m_pt = YOLO(args.pt)
    m_on = YOLO(args.onnx)          # AutoBackend → onnxruntime CPU

    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit("打不开视频: " + args.video)

    lat_pt, lat_on = [], []
    n_both = 0
    ious, c_dists = [], []
    fi = n_proc = 0
    while n_proc - args.warmup < args.max_frames:
        ok, frame = cap.read()
        if not ok:
            break
        fi += 1
        if (fi - 1) % args.stride:
            continue
        n_proc += 1
        warm = n_proc <= args.warmup

        t0 = time.perf_counter()
        rp = m_pt.predict(frame, imgsz=args.imgsz, conf=args.conf,
                          device="cpu", verbose=False)[0]
        t1 = time.perf_counter()
        ro = m_on.predict(frame, imgsz=args.imgsz, conf=args.conf,
                          verbose=False)[0]
        t2 = time.perf_counter()
        if not warm:
            lat_pt.append(t1 - t0)
            lat_on.append(t2 - t1)
            bp, bo = boxes_of(rp), boxes_of(ro)
            if len(bp) == len(bo):
                n_both += 1
            # 最优框（最高置信度）对齐比较
            if bp and bo:
                bpa = max(bp, key=lambda b: b[1])
                boa = max(bo, key=lambda b: b[1])
                ious.append(iou(bpa, boa))
                c_dists.append(((bpa[2] - boa[2]) ** 2
                                + (bpa[3] - boa[3]) ** 2) ** 0.5)
    cap.release()

    n_infer = len(lat_pt)
    if not n_infer:
        raise SystemExit("没有统计到推理帧")

    def report(tag, lat):
        if not lat:
            return
        mean, p50, p95 = stat(lat)
        print("  %s: mean %6.1f ms | p50 %6.1f ms | p95 %6.1f ms | "
              "FPS %.1f" % (tag, mean * 1e3, p50 * 1e3, p95 * 1e3, 1 / mean))

    print("=" * 66)
    print("视频 %s（推理帧 %d, conf=%.2f, imgsz=%d, 均为 CPU 推理）"
          % (args.video, n_infer, args.conf, args.imgsz))
    report("PyTorch (.pt)      ", lat_pt)
    report("ONNX Runtime (.onnx)", lat_on)
    if ious:
        print("-" * 66)
        print("检测一致性（同一帧逐框对比）:")
        print("  检出框数完全一致: %d/%d (%.1f%%)"
              % (n_both, n_infer, 100 * n_both / n_infer))
        print("  最优框 IoU: mean %.4f | min %.4f | IoU>0.9 比例 %.1f%%"
              % (statistics.mean(ious), min(ious),
                 100 * sum(v > 0.9 for v in ious) / len(ious)))
        print("  最优框中心偏差: mean %.2f px | max %.2f px"
              % (statistics.mean(c_dists), max(c_dists)))


if __name__ == "__main__":
    main()
