"""
录屏检查：读取 obs_recordings 下 mp4 的元数据，并均匀抽帧存到 check/screen_frames。
用法（Python 3.10 全路径）：
  python analyze_screen.py
"""
import os
import cv2

SRC_DIR = r"D:\obs_recordings"
OUT_ROOT = r"D:\W34_dataset\check\screen_frames"
N_FRAMES = 8  # 每个视频均匀抽多少帧

os.makedirs(OUT_ROOT, exist_ok=True)

for fn in sorted(os.listdir(SRC_DIR)):
    if not fn.lower().endswith(".mp4"):
        continue
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
    size_mb = os.path.getsize(path) / 1024 / 1024
    print(f"\n=== {fn} ===")
    print(f"  分辨率 {w}x{h}  fps={fps:.2f}  帧数={n}  时长={dur:.1f}s  大小={size_mb:.1f}MB")

    stem = os.path.splitext(fn)[0]
    out_dir = os.path.join(OUT_ROOT, stem)
    os.makedirs(out_dir, exist_ok=True)

    if n <= 0:
        cap.release()
        continue

    for i in range(N_FRAMES):
        # 按时间均匀取帧位置（避开第 0 帧和最后一帧）
        idx = int(n * (i + 0.5) / N_FRAMES)
        cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
        ok, frame = cap.read()
        if not ok:
            continue
        # 缩到宽 900，便于快速辨认
        scale = 900.0 / frame.shape[1]
        if scale < 1.0:
            frame = cv2.resize(frame, (900, int(frame.shape[0] * scale)))
        out = os.path.join(out_dir, f"f{i:02d}_{idx:06d}.jpg")
        cv2.imwrite(out, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
    cap.release()
    print(f"  抽帧目录: {out_dir}")
