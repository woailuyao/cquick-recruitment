# -*- coding: utf-8 -*-
"""
加分项 1：PyTorch vs ONNX 推理对比（细则 +3 分）

三项对比，同一测试片段、同一预处理、同一线程数：
  1) 纯模型前向延迟（mean / p50 / p95）与 FPS
  2) 检测结果一致性（框数、类别、IoU≥0.5 匹配率）
  3) 独立测试集 mAP 是否等价
"""
import os
import platform
import statistics
import sys
import time

os.chdir(r"D:\vision_ws\k1_soccer_vision")

import cv2
import numpy as np
import onnxruntime as ort
import torch
from ultralytics import YOLO

THREADS = 4
N_WARMUP = 20
N_RUN = 120

torch.set_num_threads(THREADS)


def letterbox(img, imgsz):
    h, w = img.shape[:2]
    r = min(imgsz / h, imgsz / w)
    nw, nh = int(round(w * r)), int(round(h * r))
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    canvas = np.full((imgsz, imgsz, 3), 114, np.uint8)
    top, left = (imgsz - nh) // 2, (imgsz - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    return canvas


def to_tensor(canvas):
    x = canvas[:, :, ::-1].transpose(2, 0, 1)[None]      # BGR->RGB, HWC->CHW
    return np.ascontiguousarray(x, dtype=np.float32) / 255.0


def bench(fn):
    for _ in range(N_WARMUP):
        fn()
    ts = []
    for _ in range(N_RUN):
        t0 = time.perf_counter()
        fn()
        ts.append((time.perf_counter() - t0) * 1000.0)
    ts.sort()
    return {"mean": statistics.mean(ts), "p50": ts[len(ts) // 2],
            "p95": ts[int(len(ts) * 0.95)], "fps": 1000.0 / statistics.mean(ts)}


def iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    ua = (a[2]-a[0])*(a[3]-a[1]) + (b[2]-b[0])*(b[3]-b[1]) - inter
    return inter / ua if ua > 0 else 0


def load_frames(clip, n_want=60, step=4):
    cap = cv2.VideoCapture(clip)
    out, idx = [], 0
    while len(out) < n_want:
        ok, f = cap.read()
        if not ok:
            break
        if idx % step == 0:
            out.append(f)
        idx += 1
    cap.release()
    return out


def main():
    clip = r"recordings\ball_20261118_221812.mp4"
    frames = load_frames(clip)
    print(f"测试片段 {os.path.basename(clip)}：{len(frames)} 帧\n")

    for tag, pt_path, onnx_path, imgsz, data_yaml in (
        ("任务1 ball", "weights/ball_best.pt", "weights/ball_best.onnx", 416,
         r"datasets/ball/data.yaml"),
        ("任务2 marker", "weights/marker_best.pt", "weights/marker_best.onnx", 640,
         r"datasets/marker_v6/data.yaml"),
    ):
        print("=" * 80)
        print(f"### {tag}   imgsz={imgsz}   线程={THREADS}")
        print("-" * 80)

        net = YOLO(pt_path).model.eval()
        canvas = letterbox(frames[0], imgsz)
        x = torch.from_numpy(to_tensor(canvas))
        with torch.no_grad():
            pt = bench(lambda: net(x))

        so = ort.SessionOptions()
        so.intra_op_num_threads = THREADS
        so.inter_op_num_threads = 1
        sess = ort.InferenceSession(onnx_path, sess_options=so,
                                    providers=["CPUExecutionProvider"])
        iname = sess.get_inputs()[0].name
        xn = to_tensor(canvas)
        on = bench(lambda: sess.run(None, {iname: xn}))

        print(f"{'实现':<16}{'mean':>10}{'p50':>10}{'p95':>10}{'FPS':>9}")
        for name, r in (("PyTorch", pt), ("ONNX Runtime", on)):
            print(f"{name:<16}{r['mean']:>9.1f}ms{r['p50']:>9.1f}ms"
                  f"{r['p95']:>9.1f}ms{r['fps']:>8.1f}")
        print(f"→ ONNX 延迟降低 {(pt['mean']-on['mean'])/pt['mean']*100:.1f}%，"
              f"加速 {pt['mean']/on['mean']:.2f}×")

        # 检测一致性
        ptm, onm = YOLO(pt_path), YOLO(onnx_path)
        conf = 0.25
        same = tot = 0
        cp = co = 0
        ious = []
        for f in frames:
            rp = ptm.predict(f, imgsz=imgsz, conf=conf, device="cpu", verbose=False)[0]
            ro = onm.predict(f, imgsz=imgsz, conf=conf, device="cpu", verbose=False)[0]
            bp = [(int(rp.boxes.cls[i].item()), rp.boxes.xyxy[i].tolist()) for i in range(len(rp.boxes))]
            bo = [(int(ro.boxes.cls[i].item()), ro.boxes.xyxy[i].tolist()) for i in range(len(ro.boxes))]
            cp += len(bp); co += len(bo)
            used = set()
            for c, a in bp:
                best, bi = 0.0, -1
                for k, (c2, b) in enumerate(bo):
                    if k in used or c2 != c:
                        continue
                    v = iou(a, b)
                    if v > best:
                        best, bi = v, k
                if best >= 0.5:
                    used.add(bi); same += 1; ious.append(best)
                tot += 1
        print(f"\n检测一致性（conf={conf}，{len(frames)} 帧）")
        print(f"  PyTorch {cp} 框 / ONNX {co} 框")
        print(f"  同类 IoU≥0.5 匹配率 {same/max(1,tot)*100:.1f}%  "
              f"平均 IoU {np.mean(ious) if ious else 0:.3f}")

        # mAP 等价性
        print("\n独立测试集 mAP 对比（ulytics val）")
        for name, mp in (("PyTorch ", pt_path), ("ONNX    ", onnx_path)):
            try:
                r = YOLO(mp).val(data=data_yaml, split="test", imgsz=imgsz,
                                 device="cpu", verbose=False, plots=False)
                print(f"  {name}  mAP50 {r.box.map50:.3f}   mAP50-95 {r.box.map:.3f}")
            except Exception as e:
                print(f"  {name}  val 失败: {type(e).__name__}: {e}")
        print()

    print("=" * 80)
    print("环境:", platform.platform())
    print("torch", torch.__version__, "| onnxruntime", ort.__version__,
          "| onnx", __import__("onnx").__version__)


if __name__ == "__main__":
    main()
