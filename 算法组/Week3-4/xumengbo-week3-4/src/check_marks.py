import os
import cv2

SRC = r"D:\W34_dataset\marks"
OUT = r"D:\W34_dataset\check_marks"
os.makedirs(OUT, exist_ok=True)

for f in sorted(os.listdir(SRC)):
    if not f.lower().endswith(".avi"):
        continue
    p = os.path.join(SRC, f)
    cap = cv2.VideoCapture(p)
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    dur = (n / fps) if fps else 0
    print("%s: %dx%d  %d frames  %.1f fps  %.1f s" % (f, w, h, n, fps, dur))
    for tag, pos in (("a", 0.15), ("b", 0.5), ("c", 0.85)):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(n * pos))
        ok, frame = cap.read()
        if ok:
            cv2.imwrite(os.path.join(OUT, "%s_%s.jpg" % (f[:-4], tag)), frame)
    cap.release()
print("done")
