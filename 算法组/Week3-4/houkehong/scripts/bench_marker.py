import os, statistics, time
os.chdir(r"D:\vision_ws\k1_soccer_vision")
import cv2, numpy as np, onnxruntime as ort, torch
from ultralytics import YOLO

T = 4
torch.set_num_threads(T)

def lb(img, s):
    h, w = img.shape[:2]; r = min(s/h, s/w)
    nw, nh = int(round(w*r)), int(round(h*r))
    c = np.full((s, s, 3), 114, np.uint8)
    t, l = (s-nh)//2, (s-nw)//2
    c[t:t+nh, l:l+nw] = cv2.resize(img, (nw, nh))
    return c

def tt(c):
    return np.ascontiguousarray(c[:, :, ::-1].transpose(2,0,1)[None], dtype=np.float32)/255.0

def bench(fn, warm=30, n=200):
    for _ in range(warm): fn()
    ts = []
    for _ in range(n):
        t0=time.perf_counter(); fn(); ts.append((time.perf_counter()-t0)*1000)
    ts.sort()
    return dict(p50=ts[len(ts)//2], p05=ts[int(len(ts)*0.05)], p95=ts[int(len(ts)*0.95)],
                mean=statistics.mean(ts), trimmed=statistics.mean(ts[:int(len(ts)*0.9)]))

cap = cv2.VideoCapture(r"recordings\ball_20261118_221812.mp4")
ok, frame = cap.read(); cap.release()

net = YOLO(r"weights\marker_best.pt").model.eval()
c = lb(frame, 640); x = torch.from_numpy(tt(c))
with torch.no_grad():
    pt = bench(lambda: net(x))

so = ort.SessionOptions(); so.intra_op_num_threads=T; so.inter_op_num_threads=1
sess = ort.InferenceSession(r"weights\marker_best.onnx", sess_options=so,
                            providers=["CPUExecutionProvider"])
xn = tt(c); iname = sess.get_inputs()[0].name
on = bench(lambda: sess.run(None, {iname: xn}))

print("marker @640，200 次，去掉最慢 10% 再平均")
print(f"{'实现':<14}{'p50':>9}{'p05':>9}{'p95':>9}{'mean':>9}{'trimmed':>10}")
for n, r in (("PyTorch", pt), ("ONNX", on)):
    print(f"{n:<14}{r['p50']:>8.1f}{r['p05']:>8.1f}{r['p95']:>8.1f}{r['mean']:>8.1f}{r['trimmed']:>9.1f}")
print(f"\n按 p50 比: ONNX {pt['p50']/on['p50']:.2f}×  按 trimmed 比: {pt['trimmed']/on['trimmed']:.2f}×")
