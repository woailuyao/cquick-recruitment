import cv2
import os
import glob

video_files = glob.glob('data/field_videos/*.mp4') + glob.glob('data/field_videos/*.avi')
output_dir = 'dataset/images/train' # 场地图先全放 train
os.makedirs(output_dir, exist_ok=True)

total = 0
for video_path in video_files:
    print(f"正在处理: {video_path}")
    cap = cv2.VideoCapture(video_path)
    count = 0
    while True:
        ret, frame = cap.read()
        if not ret: break
        if count % 15 == 0:
            frame = cv2.resize(frame, (640, 480)) 
            cv2.imwrite(f"{output_dir}/field_{total:04d}.jpg", frame)
            total += 1
        count += 1
    cap.release()
print(f"✅ 场地抽帧完成！共 {total} 张图。")