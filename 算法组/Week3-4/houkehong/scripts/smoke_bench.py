# -*- coding: utf-8 -*-
"""
推理性能基准（用来判断这台机器实时检测到底能跑多少 FPS）

对比 4 种组合：
  PyTorch  @640 / @416
  ONNX     @640 / @416

并顺手验证 ONNX 导出链路（细则加分项 +3 需要）。

用法：
  D:\\vision_ws\\k1_soccer_vision\\.venv\\Scripts\\python.exe scripts\\smoke_bench.py
"""

import os
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEIGHTS = os.path.join(ROOT, "weights", "yolov8n.pt")
OUT = os.path.join(ROOT, "snapshots")


def make_synthetic_frame(h=720, w=1280):
    """造一张有球、有场地线的合成图，只用于跑通链路和计时，不代表真实数据。"""
    img = np.full((h, w, 3), 40, np.uint8)
    cv2.rectangle(img, (0, int(h * 0.55)), (w, h), (60, 120, 60), -1)
    for x in range(0, w, 160):
        cv2.line(img, (x, int(h * 0.55)), (int(w / 2 + (x - w / 2) * 1.6), h), (240, 240, 240), 3)
    cv2.circle(img, (640, 520), 34, (245, 245, 245), -1)
    cv2.circle(img, (640, 520), 34, (30, 30, 30), 2)
    return img


def bench_torch(imgsz, runs=8):
    from ultralytics import YOLO
    model = YOLO(WEIGHTS)
    frame = make_synthetic_frame()
    # 预热
    for _ in range(2):
        model.predict(frame, imgsz=imgsz, verbose=False, device="cpu")
    t0 = time.perf_counter()
    for _ in range(runs):
        model.predict(frame, imgsz=imgsz, verbose=False, device="cpu")
    dt = time.perf_counter() - t0
    return dt / runs * 1000.0, 1.0 / (dt / runs)


def bench_onnx(imgsz, runs=8):
    import onnxruntime as ort
    onnx_path = os.path.join(ROOT, "weights", f"yolov8n_{imgsz}.onnx")
    if not os.path.exists(onnx_path):
        from ultralytics import YOLO
        model = YOLO(WEIGHTS)
        model.export(format="onnx", imgsz=imgsz, dynamic=False, simplify=False)
        produced = os.path.join(ROOT, "weights", "yolov8n.onnx")
        if os.path.exists(produced):
            os.replace(produced, onnx_path)

    sess = ort.InferenceSession(onnx_path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    name = inp.name
    shape = [1, 3, imgsz, imgsz]
    dummy = np.random.rand(*shape).astype(np.float32)

    for _ in range(2):
        sess.run(None, {name: dummy})
    t0 = time.perf_counter()
    for _ in range(runs):
        sess.run(None, {name: dummy})
    dt = time.perf_counter() - t0
    return dt / runs * 1000.0, 1.0 / (dt / runs)


def main():
    os.makedirs(OUT, exist_ok=True)
    print("=" * 66)
    print("YOLOv8n 推理基准（CPU: Ryzen 5 4600U, 6C/12T）")
    print("=" * 66)
    print(f"{'backend':<10}{'imgsz':>7}{'ms/frame':>12}{'FPS':>10}")
    print("-" * 66)

    rows = []
    for imgsz in (640, 416):
        try:
            ms, fps = bench_torch(imgsz)
            rows.append(("PyTorch", imgsz, ms, fps))
            print(f"{'PyTorch':<10}{imgsz:>7}{ms:>12.1f}{fps:>10.1f}")
        except Exception as e:
            print(f"{'PyTorch':<10}{imgsz:>7}   FAILED: {e}")

    for imgsz in (640, 416):
        try:
            ms, fps = bench_onnx(imgsz)
            rows.append(("ONNX", imgsz, ms, fps))
            print(f"{'ONNX':<10}{imgsz:>7}{ms:>12.1f}{fps:>10.1f}")
        except Exception as e:
            print(f"{'ONNX':<10}{imgsz:>7}   FAILED: {e}")

    print("-" * 66)
    print("说明：单模型单帧耗时。任务 1+2 要同时跑两个模型，")
    print("      实际整帧耗时要按两倍估算，除非合并成一个多类别模型。")

    # 存一张带检测框的结果图，确认整条链路真的能出图
    try:
        from ultralytics import YOLO
        model = YOLO(WEIGHTS)
        frame = make_synthetic_frame()
        res = model.predict(frame, imgsz=640, verbose=False, device="cpu")[0]
        vis = res.plot()
        path = os.path.join(OUT, "yolov8n_synthetic_result.jpg")
        cv2.imwrite(path, vis)
        print(f"结果图 -> {path}")
    except Exception as e:
        print(f"可视化失败: {e}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
