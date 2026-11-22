"""补抽 17-30 录屏 show_metrics 时段（帧 800~1500），查找 P/R/mAP 指标输出。"""
import os, cv2

PATH = r"D:\obs_recordings\2026-11-14 17-30-55.mp4"
OUT = r"D:\W34_dataset\check\screen_frames\metrics_check"
os.makedirs(OUT, exist_ok=True)

cap = cv2.VideoCapture(PATH)
for idx in range(800, 1500, 50):
    cap.set(cv2.CAP_PROP_POS_FRAMES, idx)
    ok, frame = cap.read()
    if not ok:
        continue
    scale = 900.0 / frame.shape[1]
    frame = cv2.resize(frame, (900, int(frame.shape[0] * scale)))
    cv2.imwrite(os.path.join(OUT, f"m_{idx:06d}.jpg"), frame, [int(cv2.IMWRITE_JPEG_QUALITY), 88])
cap.release()
print("done", len(os.listdir(OUT)))
