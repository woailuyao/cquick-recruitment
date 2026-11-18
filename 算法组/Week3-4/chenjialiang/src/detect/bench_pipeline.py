
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
from ultralytics import YOLO

SRC = sys.argv[1] if len(sys.argv) > 1 else "http://localhost:8000/stream"
MODEL = os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..",
    "runs", "detect", "train", "weights", "best.pt")

cap = cv2.VideoCapture(SRC)
if not cap.isOpened():
    print("打不开", SRC)
    sys.exit(1)
cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"输入 {SRC}  {w}x{h}")

model = YOLO(MODEL)

for _ in range(5):
    ok, f = cap.read()
    if ok:
        model(f, conf=0.25, verbose=False)

N = 60
t_read = t_infer = t_check = t_enc = 0.0
n_ok = 0
for _ in range(N):
    t0 = time.perf_counter()
    ok, frame = cap.read()
    t1 = time.perf_counter()
    if not ok:
        continue
    n_ok += 1

    res = model(frame, conf=0.25, verbose=False)[0]
    t2 = time.perf_counter()

    for b in res.boxes:
        x1, y1, x2, y2 = map(int, b.xyxy[0].tolist())
        patch = frame[y1:y2, x1:x2]
        if patch.size:
            cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)
    t3 = time.perf_counter()

    cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 60])
    t4 = time.perf_counter()

    t_read += t1 - t0
    t_infer += t2 - t1
    t_check += t3 - t2
    t_enc += t4 - t3

if n_ok == 0:
    print("一帧都没读到")
    sys.exit(1)

ms = lambda x: x / n_ok * 1000
total = t_read + t_infer + t_check + t_enc
print()
print(f"样本 {n_ok} 帧")
print(f"  读帧(含等新帧)  {ms(t_read):7.1f} ms")
print(f"  YOLO 推理       {ms(t_infer):7.1f} ms")
print(f"  几何判据        {ms(t_check):7.1f} ms")
print(f"  JPEG 编码       {ms(t_enc):7.1f} ms")
print(f"  ----------------------------")
print(f"  单帧合计        {ms(total):7.1f} ms   ->  {1000 / ms(total):.1f} FPS")
print()
print("哪一项占了 80% 以上，就是它拖慢的。")
