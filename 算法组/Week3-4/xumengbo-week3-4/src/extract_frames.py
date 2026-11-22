import os
import cv2
import numpy as np

RAW = r"D:\W34_dataset\raw"
ROOT = r"D:\W34_dataset\ball"
SAMPLE_FPS = 4.0
DIFF_THRESH = 2.0
DIFF_BY_VIDEO = {"clip_04": 0.15}

SPLIT = {
    "clip_01": "train",
    "clip_03": "train",
    "clip_04": "train",
    "clip_06": "train",
    "clip_07": "train",
    "clip_05": "val",
    "clip_02": "test",
    "clip_08": "test",
}

for sub in ("train", "val", "test"):
    d = os.path.join(ROOT, "images", sub)
    os.makedirs(d, exist_ok=True)
    for fn in os.listdir(d):
        if fn.lower().endswith(".jpg"):
            os.remove(os.path.join(d, fn))
    os.makedirs(os.path.join(ROOT, "labels", sub), exist_ok=True)


def thumb(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.resize(gray, (64, 64), interpolation=cv2.INTER_AREA).astype(np.float32)


total = {}
for stem, split in SPLIT.items():
    path = os.path.join(RAW, stem + ".avi")
    if not os.path.exists(path):
        print(f"{stem}: 文件不存在，跳过")
        continue

    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"{stem}: 打不开")
        continue

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    step = max(1, int(round(fps / SAMPLE_FPS)))
    diff_thresh = DIFF_BY_VIDEO.get(stem, DIFF_THRESH)
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    kept = 0
    last_thumb = None
    idx = 0
    while idx < total_frames:
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            break
        t = thumb(frame)
        if last_thumb is not None:
            diff = float(np.abs(t - last_thumb).mean())
            if diff < diff_thresh:
                idx += step
                continue
        name = f"{stem}_f{idx:06d}.jpg"
        ok2, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 92])
        if ok2:
            with open(os.path.join(ROOT, "images", split, name), "wb") as f:
                f.write(buf.tobytes())
            kept += 1
            last_thumb = t
        idx += step

    cap.release()
    total[split] = total.get(split, 0) + kept
    print(f"{stem} -> {split}: 抽 {kept} 张（视频共 {total_frames} 帧，间隔 {step} 帧）")

print("--- 汇总 ---")
alln = 0
for s in ("train", "val", "test"):
    n = len(os.listdir(os.path.join(ROOT, "images", s)))
    alln += n
    print(f"{s}: {n} 张")
print(f"合计: {alln} 张")
