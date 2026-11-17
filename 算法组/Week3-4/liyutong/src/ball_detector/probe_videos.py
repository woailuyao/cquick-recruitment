# -*- coding: utf-8 -*-
"""体检：读 videos/ 下每段视频的分辨率/帧率/帧数/时长，并各抽 1 帧预览到 _probe/ 供人眼确认。"""
import cv2, os

VIDEO_DIR = "videos"
OUT_DIR = "_probe"
os.makedirs(OUT_DIR, exist_ok=True)


def imwrite_safe(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok


vids = sorted(f for f in os.listdir(VIDEO_DIR) if f.lower().endswith(".mp4"))
print(f"{'文件':<14}{'分辨率':<12}{'FPS':<8}{'总帧数':<9}{'时长(s)':<9}{'码率(Mbps)'}")
print("-" * 66)
for v in vids:
    p = os.path.join(VIDEO_DIR, v)
    cap = cv2.VideoCapture(p)
    if not cap.isOpened():
        print(f"{v:<14}打不开")
        continue
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS) or 0
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    size_mb = os.path.getsize(p) / 1024 / 1024
    dur = n / fps if fps else 0
    bitrate = size_mb * 8 / dur if dur else 0
    print(f"{v:<14}{w}x{h:<8}{fps:<8.1f}{n:<9}{dur:<9.1f}{bitrate:.1f}")

    # 抽中间一帧做预览
    cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, n // 2))
    ok, frame = cap.read()
    if ok:
        imwrite_safe(os.path.join(OUT_DIR, f"{os.path.splitext(v)[0]}_mid.jpg"), frame)
    # 再抽 1/4 处
    cap.set(cv2.CAP_PROP_POS_FRAMES, max(0, n // 4))
    ok, frame = cap.read()
    if ok:
        imwrite_safe(os.path.join(OUT_DIR, f"{os.path.splitext(v)[0]}_q1.jpg"), frame)
    cap.release()

print(f"\n预览帧已存到 {OUT_DIR}/")
