# PyTorch 与 ONNX 推理对比：延迟、FPS、检测框一致性
import argparse
import time

import cv2
import numpy as np
import onnxruntime as ort
from ultralytics import YOLO


def letterbox(img, size):
    h, w = img.shape[:2]
    r = min(size / h, size / w)
    nh, nw = int(round(h * r)), int(round(w * r))
    resized = cv2.resize(img, (nw, nh))
    canvas = np.full((size, size, 3), 114, np.uint8)
    top = (size - nh) // 2
    left = (size - nw) // 2
    canvas[top:top + nh, left:left + nw] = resized
    return canvas, r, left, top


def onnx_boxes(sess, frame, conf, size, names):
    img, r, dx, dy = letterbox(frame, size)
    blob = img[:, :, ::-1].transpose(2, 0, 1)[None].astype(np.float32) / 255.0
    inp = sess.get_inputs()[0].name
    out = sess.run(None, {inp: blob})[0]
    pred = out[0].T if out.shape[1] < out.shape[2] else out[0]
    scores = pred[:, 4:]
    cls = scores.argmax(1)
    confs = scores.max(1)
    keep = confs >= conf
    pred, cls, confs = pred[keep], cls[keep], confs[keep]
    if len(pred) == 0:
        return []
    cx, cy, bw, bh = pred[:, 0], pred[:, 1], pred[:, 2], pred[:, 3]
    x1 = (cx - bw / 2 - dx) / r
    y1 = (cy - bh / 2 - dy) / r
    x2 = (cx + bw / 2 - dx) / r
    y2 = (cy + bh / 2 - dy) / r
    idx = cv2.dnn.NMSBoxes(np.stack([x1, y1, x2 - x1, y2 - y1], 1).tolist(),
                           confs.tolist(), conf, 0.45)
    res = []
    for i in np.array(idx).flatten().tolist():
        res.append([int(cls[i]), float(confs[i]), x1[i], y1[i], x2[i], y2[i]])
    return res


def iou(a, b):
    x1, y1 = max(a[2], b[2]), max(a[3], b[3])
    x2, y2 = min(a[4], b[4]), min(a[5], b[5])
    w, h = max(0.0, x2 - x1), max(0.0, y2 - y1)
    inter = w * h
    area_a = (a[4] - a[2]) * (a[5] - a[3])
    area_b = (b[4] - b[2]) * (b[5] - b[3])
    return inter / (area_a + area_b - inter + 1e-9)


def match(pt, on, thr=0.5):
    used = set()
    hit = 0
    shifts = []
    for p in pt:
        best, bi = 0.0, -1
        for j, o in enumerate(on):
            if j in used or o[0] != p[0]:
                continue
            v = iou(p, o)
            if v > best:
                best, bi = v, j
        if best >= thr:
            hit += 1
            used.add(bi)
            shifts.append(abs((p[2] + p[4]) / 2 - (on[bi][2] + on[bi][4]) / 2))
    miss = len(pt) - hit
    extra = len(on) - hit
    return hit, miss, extra, (float(np.mean(shifts)) if shifts else 0.0)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("video")
    ap.add_argument("--pt", required=True)
    ap.add_argument("--onnx", required=True)
    ap.add_argument("--conf", type=float, default=0.4)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="cpu")
    ap.add_argument("--max-frames", type=int, default=300)
    args = ap.parse_args()

    model = YOLO(args.pt)
    sess = ort.InferenceSession(args.onnx, providers=["CPUExecutionProvider"])
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"打不开视频 {args.video}")

    t_pt = t_on = 0.0
    n = hit_all = miss_all = extra_all = 0
    shift_all = []

    for _ in range(args.max_frames):
        ok, frame = cap.read()
        if not ok:
            break
        if frame.shape[1] != 1280:
            frame = cv2.resize(frame, (1280, 720))

        t0 = time.perf_counter()
        r = model(frame, conf=args.conf, imgsz=args.imgsz, device=args.device, verbose=False)[0]
        t_pt += time.perf_counter() - t0
        pt = []
        for b in r.boxes:
            x1, y1, x2, y2 = b.xyxy[0].tolist()
            pt.append([int(b.cls[0]), float(b.conf[0]), x1, y1, x2, y2])

        t0 = time.perf_counter()
        on = onnx_boxes(sess, frame, args.conf, args.imgsz, model.names)
        t_on += time.perf_counter() - t0

        h, m, e, s = match(pt, on)
        hit_all += h
        miss_all += m
        extra_all += e
        shift_all.append(s)
        n += 1

    cap.release()
    if n == 0:
        raise SystemExit("没读到帧")

    print(f"帧数 {n}")
    print(f"PyTorch({args.device})  平均 {t_pt / n * 1000:7.2f} ms   {n / t_pt:6.1f} FPS")
    print(f"ONNX(cpu)         平均 {t_on / n * 1000:7.2f} ms   {n / t_on:6.1f} FPS")
    print(f"加速比 {t_pt / t_on:.2f}x")
    print(f"框一致(IoU>=0.5) {hit_all / max(hit_all + miss_all, 1) * 100:.1f}%  "
          f"漏 {miss_all} 多 {extra_all}")
    print(f"中心平均偏移 {np.mean(shift_all):.2f} px")


if __name__ == "__main__":
    main()
