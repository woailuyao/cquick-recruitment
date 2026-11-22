import os
import cv2

OUT = r"D:\W34_dataset\raw\test_record.avi"
os.makedirs(os.path.dirname(OUT), exist_ok=True)

cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FPS, 30)

w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"分辨率 {w}x{h}")

writer = cv2.VideoWriter(OUT, cv2.VideoWriter_fourcc(*"MJPG"), 30, (w, h))
frames = 0
for _ in range(90):
    ok, frame = cap.read()
    if not ok:
        break
    writer.write(frame)
    frames += 1

writer.release()
cap.release()

print(f"写入帧数: {frames}")
if os.path.exists(OUT):
    print(f"文件大小: {os.path.getsize(OUT)} bytes")
else:
    print("文件未生成")
