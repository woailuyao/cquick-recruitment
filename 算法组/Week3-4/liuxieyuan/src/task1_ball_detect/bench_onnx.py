# -*- coding: utf-8 -*-
"""
ONNX 导出与推理对比（加分项）

用法：
    python src/bench_onnx.py --model weights/best.pt --image dataset/images/xxx.jpg

输出：PyTorch 与 ONNX Runtime 的端到端耗时与纯前向耗时，用于 README 中的对比表。
依赖：ultralytics, onnx, onnxruntime, opencv-python, torch
"""

import argparse
import glob
import statistics
import time

import cv2
import numpy as np
import onnxruntime as ort
import torch
from ultralytics import YOLO


def letterbox(img, size=640):
    h0, w0 = img.shape[:2]
    r = min(size / h0, size / w0)
    nh, nw = int(round(h0 * r)), int(round(w0 * r))
    canvas = np.full((size, size, 3), 114, np.uint8)
    canvas[:nh, :nw] = cv2.resize(img, (nw, nh))
    return canvas[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="weights/best.pt")
    ap.add_argument("--image", default="")
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--imgsz", type=int, default=640)
    a = ap.parse_args()

    img_path = a.image or glob.glob("dataset/images/**/*.jpg", recursive=True)[0]
    img = cv2.imread(img_path)
    print(f"测试图: {img_path}  {img.shape}")

    # 1) 导出 ONNX
    yolo = YOLO(a.model)
    onnx_path = str(yolo.export(format="onnx", imgsz=a.imgsz, device="cpu", opset=12))
    print(f"ONNX 已导出: {onnx_path}")

    # 2) 端到端（含前后处理）
    def e2e(model_path):
        m = YOLO(model_path)
        m.predict(img, imgsz=a.imgsz, conf=0.25, verbose=False, device="cpu")
        ts, boxes = [], None
        for _ in range(a.iters):
            t0 = time.perf_counter()
            r = m.predict(img, imgsz=a.imgsz, conf=0.25, verbose=False, device="cpu")[0]
            ts.append((time.perf_counter() - t0) * 1000)
            if boxes is None:
                boxes = [(round(float(b.conf[0]), 3), [round(float(v), 1) for v in b.xyxy[0]])
                         for b in r.boxes]
        return statistics.mean(ts), boxes

    pt_ms, pt_boxes = e2e(a.model)
    on_ms, on_boxes = e2e(onnx_path)

    # 3) 纯前向（不含前后处理）
    inp = letterbox(img, a.imgsz)
    model = yolo.model.eval()
    t = torch.from_numpy(inp)
    with torch.no_grad():
        model(t)
        ts = []
        for _ in range(a.iters):
            t0 = time.perf_counter()
            model(t)
            ts.append((time.perf_counter() - t0) * 1000)
    pt_fwd = statistics.mean(ts)

    so = ort.SessionOptions()
    so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
    so.intra_op_num_threads = 4
    sess = ort.InferenceSession(onnx_path, sess_options=so, providers=["CPUExecutionProvider"])
    name = sess.get_inputs()[0].name
    sess.run(None, {name: inp})
    ts = []
    for _ in range(a.iters):
        t0 = time.perf_counter()
        sess.run(None, {name: inp})
        ts.append((time.perf_counter() - t0) * 1000)
    on_fwd = statistics.mean(ts)

    print("\n| 指标 | PyTorch (best.pt) | ONNX Runtime (best.onnx) |")
    print("| --- | --- | --- |")
    print(f"| 端到端 predict（含前后处理） | {pt_ms:.1f} ms / {1000/pt_ms:.1f} FPS | "
          f"{on_ms:.1f} ms / {1000/on_ms:.1f} FPS |")
    print(f"| 纯前向 forward | {pt_fwd:.1f} ms / {1000/pt_fwd:.1f} FPS | "
          f"{on_fwd:.1f} ms / {1000/on_fwd:.1f} FPS |")
    print(f"| 检测结果一致 | {pt_boxes} | {on_boxes} |")


if __name__ == "__main__":
    main()
