# 任务1 实时检测：左侧画框，右下角显示 FPS 与像素中心
import os
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cv2
from ultralytics import YOLO

from stream_server import StreamServer

OWNER = "chenjialiang"
# 30fps 下漏一帧有跟踪兜底，阈值可抬高压误检
CONF = 0.38

MAX_AREA = 150000

BRIGHT_LEVEL = 170
DARK_LEVEL = 150

MIN_BRIGHT, MAX_BRIGHT = 0.22, 0.86
MIN_DARK, MAX_DARK = 0.14, 0.70

RATIO = (0.50, 2.0)

IOU_TH = 0.3
MAX_MISS = 6
MIN_AGE = 2
MAX_MISS_OUT = 4

_ARGS = [a for a in sys.argv[1:] if not a.startswith("--")]
SRC = _ARGS[0] if _ARGS else "http://localhost:8000/stream"
SAVE = "--save" in sys.argv
NO_CHECK = "--no-check" in sys.argv
_HERE = os.path.dirname(os.path.abspath(__file__))
_ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))
MODEL = os.path.join(_ROOT, "runs", "detect", "train4", "weights", "best.pt")
PORT = 8001
OUT_VIDEO = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "outputs", "detect_live.mp4"))

def check(frame, x1, y1, x2, y2):
    if NO_CHECK:
        return True, "skip"
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
        return center_dist(self.box, box) < max(120.0, 3.0 * self.size())

class LatestFrame:

    def __init__(self, cap):
        self.cap = cap
        self.frame = None
        self.seq = 0
        self.running = True
        self.lock = threading.Lock()
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.running:
            ok, f = self.cap.read()
            if not ok:
                time.sleep(0.005)
                continue
            with self.lock:
                self.frame = f
                self.seq += 1

    def read(self):
        with self.lock:
            return self.frame, self.seq

    def stop(self):
        self.running = False

def main():
    model = YOLO(MODEL)

    cap = cv2.VideoCapture(SRC)
    if not cap.isOpened():
        print(f"打不开输入源 {SRC}")
        sys.exit(1)

    is_stream = SRC.startswith("http")
    if is_stream:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    reader = LatestFrame(cap) if is_stream else None

    srv = StreamServer(port=PORT, quality=60).start()

    writer = None
    if SAVE:
        os.makedirs(os.path.dirname(OUT_VIDEO), exist_ok=True)
        fw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 960
        fh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 540
        writer = cv2.VideoWriter(OUT_VIDEO, cv2.VideoWriter_fourcc(*"mp4v"), 30, (fw, fh))
        print(f"录制到 {OUT_VIDEO}  ({fw}x{fh})")

    tracks = []
    fps = 0.0
    infer_ms = 0.0
    t_prev = time.time()
    last_seq = -1
    n_new = 0
    t_fps = time.time()

    try:
        while True:
            if reader is not None:
                frame, seq = reader.read()
                if frame is None:
                    time.sleep(0.005)
                    continue
                if seq != last_seq:
                    last_seq = seq
                    n_new += 1
            else:
                ok, frame = cap.read()
                if not ok:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    continue

            t_inf = time.time()
            res = model(frame, conf=CONF, verbose=False)[0]
            infer_ms = (time.time() - t_inf) * 1000.0

            dets = []
            for box in res.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
                conf_v = float(box.conf[0])
                keep, _ = check(frame, x1, y1, x2, y2)
                if keep:
                    dets.append((x1, y1, x2, y2, conf_v))

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

            shown = 0
            for t in tracks:
                if t.age < MIN_AGE or t.miss > MAX_MISS_OUT:
                    continue

                x1, y1, x2, y2 = t.box
                shown += 1

                if t.miss > 0:
                    color = (0, 200, 255)
                    label = f"ball {t.conf:.2f} fill"
                else:
                    color = (0, 255, 0)
                    label = f"ball {t.conf:.2f}"

                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                ccx, ccy = (x1 + x2) // 2, (y1 + y2) // 2
                cv2.drawMarker(frame, (ccx, ccy), color, cv2.MARKER_CROSS, 12, 2)
                label = f"{label} ({ccx},{ccy})"
                cv2.putText(frame, label, (x1, max(y1 - 6, 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)

            now = time.time()
            if reader is not None:
                if now - t_fps >= 0.5:
                    fps = n_new / (now - t_fps)
                    n_new = 0
                    t_fps = now
            else:
                dt = now - t_prev
                if dt > 0:
                    fps = 0.9 * fps + 0.1 * (1.0 / dt)
                t_prev = now

            cv2.putText(frame, f"FPS {fps:.1f}  infer {infer_ms:.1f} ms  balls {shown}",
                        (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)

            ts = time.strftime("%Y-%m-%d %H:%M:%S")
            hud2 = f"{OWNER}   {ts}"
            (tw, _), _ = cv2.getTextSize(hud2, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
            cv2.putText(frame, hud2, (frame.shape[1] - tw - 12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            srv.push(frame)

            if writer is not None:
                writer.write(frame)

    except KeyboardInterrupt:
        pass
    finally:
        if reader is not None:
            reader.stop()
        cap.release()
        if writer is not None:
            writer.release()
            print(f"录制完成 {OUT_VIDEO}")
        srv.stop()

if __name__ == "__main__":
    main()
