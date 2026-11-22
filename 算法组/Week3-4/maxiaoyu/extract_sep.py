import cv2
import os

video_list = [
    "football_1.avi",
    "football_2.avi",
    "football_3.avi",
    "football_4.avi",
    "football_5.avi"
]
frame_interval = 15

for vid_name in video_list:
    # 每个视频单独建文件夹
    base_name = vid_name.split(".")[0]
    save_dir = base_name
    os.makedirs(save_dir, exist_ok=True)

    cap = cv2.VideoCapture(vid_name)
    if not cap.isOpened():
        print(f"无法打开视频 {vid_name}")
        continue
    frame_idx = 0
    count = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if frame_idx % frame_interval == 0:
            img_path = os.path.join(save_dir, f"{base_name}_img_{count:04d}.jpg")
            cv2.imwrite(img_path, frame)
            count += 1
        frame_idx += 1
    cap.release()
    print(f"{vid_name} 抽帧完成，共 {count} 张，保存在 {save_dir}")

