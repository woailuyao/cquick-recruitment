import cv2
import os

# 保存图片的文件夹
save_dir = "field_mark_frames"
os.makedirs(save_dir, exist_ok=True)

# 5个cheese录制的场地视频
video_list = [
"~/Videos/Webcam/2026-11-15-181310.webm",
"~/Videos/Webcam/2026-11-15-185921.webm",
"~/Videos/Webcam/2026-11-15-190050.webm",
"~/Videos/Webcam/2026-11-15-190208.webm",
"~/Videos/Webcam/2026-11-15-191138.webm",
]

# 每隔15帧抽一张
frame_interval = 5
count = 0
for vid_name in video_list:
    cap = cv2.VideoCapture(os.path.expanduser(vid_name))
    if not cap.isOpened():
        print(f"无法打开视频 {vid_name}")
        continue
    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % frame_interval == 0:
            img_path = os.path.join(save_dir, f"img_{count:04d}.jpg")
            cv2.imwrite(img_path, frame)
            count += 1
        frame_idx += 1
    cap.release()

print(f"场地标线抽帧完成！一共提取 {count} 张图片，保存在 {save_dir}")
