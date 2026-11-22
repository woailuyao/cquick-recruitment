"""抽取最新两段录屏的元数据与均匀帧。"""
import os, cv2

SRC_DIR = r"D:\obs_recordings"
OUT_ROOT = r"D:\W34_dataset\check\screen_frames"
TARGETS = ["2026-11-14 17-39-44.mp4", "2026-11-14 17-41-03.mp4"]
N_FRAMES = 10

for fn in TARGETS:
    path = os.path.join(SRC_DIR, fn)
    cap = cv2.VideoCapture(path)
    if not cap.isOpened():
        print(f"[打不开] {fn}")
        continue
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    dur = n / fps if fps else 0.0
    print(f"\n=== {fn} ===")
    print(f"  {w}x{h} fps={fps:.2f} frames={n} dur={dur:.1f}s size={os.path.getsize(path)/1024/1024:.1f}MB")

    stem = os.path.splitext(fn)[0]
    out_dir = os.path.join(OUT_ROOT, stem)
    os.makedirs(out_dir, exist_ok=True)
    for i in range(N_FRAMES):
        idx = int(n * (i + 0.5) / N_FRAMES)
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            continue
        scale = 900.0 / frame.shape[1]
        if scale < 1.0:
            frame = cv2.resize(frame, (900, int(frame.shape[0] * scale)))
        cv2.imwrite(os.path.join(out_dir, f"f{i:02d}_{idx:06d}.jpg"), frame,
                    [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    cap.release()
    print(f"  目录: {out_dir}")
