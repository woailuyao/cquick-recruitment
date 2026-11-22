import cv2
import os
import glob

video_files = glob.glob('data/*.mp4') + glob.glob('data/*.avi')
output_dir = 'dataset_ball/images/raw'
os.makedirs(output_dir, exist_ok=True)

total = 0
for video_path in video_files:
    print(f"正在处理: {video_path}")
    cap = cv2.VideoCapture(video_path)
    count = 0
    while True:
        ret, frame = cap.read()
        if not ret: break
        # 每10帧抽1张
        if count % 10 == 0:
            frame = cv2.resize(frame, (640, 480)) 
            cv2.imwrite(f"{output_dir}/ball_{total:04d}.jpg", frame)
            total += 1
        count += 1
    cap.release()
print(f"✅ 抽帧完成！共提取 {total} 张图，保存在 dataset_ball/images/raw 里。")