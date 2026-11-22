import cv2 as cv
import os

source_dir = 'task1_qipaige'
target_split = 'qipaige'
save_dir = f'calib_imgs/{target_split}'
os.makedirs(save_dir, exist_ok = True)

#遍历source_dir里面的MP4，并存入一个列表当中
video_files = [f for f in os.listdir(source_dir) if f.endswith('.mp4')]

print(f"在{source_dir}中找到了{len(video_files)}个视频")

count = 0

for video_name in video_files:
    video_path = os.path.join(source_dir, video_name)

    cap = cv.VideoCapture(video_path)

    fps = int(cap.get(cv.CAP_PROP_FPS))
    interval = fps if fps > 0 else 30

    frame_id = 0

    while True:

        ret, frame = cap.read()
        if not ret:
            break

        if frame_id%interval == 0:
            img_name = f"{target_split}_{count}.jpg"
            save_path = os.path.join(save_dir, img_name)
            cv.imwrite(save_path, frame)

            count += 1

        frame_id += 1

    cap.release()

print("ok!")
