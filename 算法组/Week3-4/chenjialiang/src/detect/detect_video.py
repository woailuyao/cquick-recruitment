
import os
import sys

import cv2
from ultralytics import YOLO

CONF = 0.25
MAX_AREA = 150000
DARK_LEVEL = 150
BRIGHT_LEVEL = 170
MIN_BRIGHT, MAX_BRIGHT = 0.28, 0.78
MIN_DARK, MAX_DARK = 0.15, 0.60
RATIO = (0.6, 1.7)

IOU_TH = 0.3
MAX_MISS = 3
MIN_AGE = 2
MAX_MISS_OUT = 2

SRC = sys.argv[1] if len(sys.argv) > 1 else "/mnt/e/CQuick/ws34/tools/records/rec_20261108_162049.mp4"
MODEL = sys.argv[2] if len(sys.argv) > 2 else "runs/detect/train/weights/best.pt"
OUT_DIR = "/mnt/e/CQuick/ws34/outputs/detect_out"
os.makedirs(OUT_DIR, exist_ok=True)

stem = os.path.splitext(os.path.basename(SRC))[0]
OUT = os.path.join(OUT_DIR, f"{stem}_tracked.mp4")

def check(frame, x1, y1, x2, y2):
    w, h = x2 - x1, y2 - y1
    if w <= 0 or h <= 0:
        return False, "bad_box"
    if w * h > MAX_AREA:
        return False, "too_big"
    if not (RATIO[0] < w / h < RATIO[1]):
        return False, "bad_ratio"

    patch = frame[y1:y2, x1:x2]
    if patch.size == 0:
        return False, "empty"
    gray = cv2.cvtColor(patch, cv2.COLOR_BGR2GRAY)

    bright = float((gray > BRIGHT_LEVEL).mean())
    dark = float((gray < DARK_LEVEL).mean())

    if not (MIN_BRIGHT <= bright <= MAX_BRIGHT):
        return False, "bright_out"
    if not (MIN_DARK <= dark <= MAX_DARK):
        return False, "dark_out"

    return True, "ok"

def iou(a, b):
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0, ix2 - ix1), max(0, iy2 - iy1)
    inter = iw * ih
    if inter == 0:
        return 0.0
    ua = (ax2 - ax1) * (ay2 - ay1) + (bx2 - bx1) * (by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0

def center_dist(a, b):
    ax, ay = (a[0] + a[2]) / 2, (a[1] + a[3]) / 2
    bx, by = (b[0] + b[2]) / 2, (b[1] + b[3]) / 2
    return ((ax - bx) ** 2 + (ay - by) ** 2) ** 0.5

class Track:

    __slots__ = ("box", "conf", "age", "miss")

    def __init__(self, box, conf):
        self.box = box
        self.conf = conf
        self.age = 1
        self.miss = 0

    def update(self, box, conf):
        self.box = box
        self.conf = conf
        self.age += 1
        self.miss = 0

    def size(self):
        return max(self.box[2] - self.box[0], self.box[3] - self.box[1])

    def matched_with(self, box):
        if iou(self.box, box) > IOU_TH:
            return True
        return center_dist(self.box, box) < max(60.0, 2.0 * self.size())

model = YOLO(MODEL)
cap = cv2.VideoCapture(SRC)
if not cap.isOpened():
    print(f"打不开视频 {SRC}")
    sys.exit(1)

W = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
H = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
FPS = cap.get(cv2.CAP_PROP_FPS) or 30
N = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (W, H))

print(f"输入 {SRC}")
print(f"输出 {OUT}")
print(f"{W}x{H}  {FPS:.1f}fps  {N}帧")

tracks = []
raw_count = 0
passed = 0
output_count = 0
filled = 0
dropped = {"too_big": 0, "bad_ratio": 0, "bright_out": 0, "dark_out": 0,
           "bad_box": 0, "empty": 0}
idx = 0

while True:
    ok, frame = cap.read()
    if not ok:
        break
    idx += 1

    res = model(frame, conf=CONF, verbose=False)[0]

    dets = []
    for box in res.boxes:
        x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
        conf_v = float(box.conf[0])
        raw_count += 1

        keep, reason = check(frame, x1, y1, x2, y2)
        if keep:
            dets.append((x1, y1, x2, y2, conf_v))
            passed += 1
        else:
            dropped[reason] = dropped.get(reason, 0) + 1

    used = [False] * len(dets)
    dead = []
    for t in tracks:
        best_i, best_v = -1, -1.0
        for i, d in enumerate(dets):
            if used[i]:
                continue
            if t.matched_with(d[:4]):
                v = iou(t.box, d[:4])
                if v > best_v:
                    best_v, best_i = v, i
        if best_i >= 0:
            t.update(dets[best_i][:4], dets[best_i][4])
            used[best_i] = True
        else:
            t.miss += 1
            if t.miss > MAX_MISS:
                dead.append(t)

    for t in dead:
        tracks.remove(t)

    for i, d in enumerate(dets):
        if not used[i]:
            tracks.append(Track(d[:4], d[4]))

    for t in tracks:
        if t.age < MIN_AGE or t.miss > MAX_MISS_OUT:
            continue
        x1, y1, x2, y2 = t.box
        output_count += 1
        if t.miss > 0:
            filled += 1
            color = (0, 200, 255)
            label = f"ball {t.conf:.2f} fill"
        else:
            color = (0, 255, 0)
            label = f"ball {t.conf:.2f}"
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, max(y1 - 6, 14)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

    cv2.putText(frame, f"out {output_count}  fill {filled}", (10, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    writer.write(frame)

    if idx % 100 == 0:
        print(f"  {idx}/{N} 帧  输出 {output_count}  补帧 {filled}")

cap.release()
writer.release()

print("-" * 52)
print(f"原始检测框 {raw_count}")
print(f"通过几何判据 {passed}，被挡 {raw_count - passed}")
for k, v in dropped.items():
    if v:
        print(f"  {k}: {v}")
print(f"最终输出 {output_count} 个框，其中补帧 {filled} 个")
print(f"输出视频 {OUT}")
