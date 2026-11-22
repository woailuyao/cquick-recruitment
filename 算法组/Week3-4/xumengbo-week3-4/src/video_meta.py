import os, cv2
RAW = r"D:\W34_dataset\raw"
for fn in sorted(os.listdir(RAW)):
    if not fn.lower().endswith(".avi"):
        continue
    p = os.path.join(RAW, fn)
    cap = cv2.VideoCapture(p)
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    dur = n / fps if fps else 0
    print(f"{fn}: {w}x{h}  fps={fps:.1f}  frames={n}  dur={dur:.1f}s  size={os.path.getsize(p)/1024/1024:.1f}MB")
    cap.release()
