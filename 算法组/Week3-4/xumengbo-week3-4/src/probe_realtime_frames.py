"""在最新主片实时段（帧 1500~3100）加密抽帧，查找遮挡/滚动/出画。"""
import os, cv2

PATH = r"D:\obs_recordings\2026-11-14 17-41-03.mp4"
OUT = r"D:\W34_dataset\check\screen_frames\realtime_check"
os.makedirs(OUT, exist_ok=True)

cap = cv2.VideoCapture(PATH)
for idx in range(1500, 3100, 100):
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, frame = cap.read()
    if not ok:
        continue
    scale = 900.0 / frame.shape[1]
    frame = cv2.resize(frame, (900, int(frame.shape[0] * scale)))
    cv2.imwrite(os.path.join(OUT, f"r_{idx:06d}.jpg"), frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), 88])
cap.release()
print("done", len(os.listdir(OUT)))
