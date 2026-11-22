import os
import cv2

RAW = r"D:\W34_dataset\raw"
OUT = r"D:\W34_dataset\check"
os.makedirs(OUT, exist_ok=True)

for name in sorted(os.listdir(RAW)):
    if not name.lower().endswith((".avi", ".mp4")):
        continue
    path = os.path.join(RAW, name)
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"{name}: 打不开")
        continue

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    dur = n / fps if fps else 0
    print(f"{name}: {w}x{h}  fps={fps:.1f}  frames={n}  dur={dur:.1f}s")

    stem = os.path.splitext(name)[0]
    for tag, ratio in (("a", 0.15), ("b", 0.5), ("c", 0.85)):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * ratio))
        ok, frame = cap.read()
        if not ok:
            continue
        ok2, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if ok2:
            with open(os.path.join(OUT, f"{stem}_{tag}.jpg"), "wb") as f:
                f.write(buf.tobytes())
    cap.release()

print("DONE")
