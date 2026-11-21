
import cv2
import os
import glob


video_files = glob.glob("*.mp4")

if not video_files:
    print("当前目录下没有找到 .mp4 视频文件！")
else:
    print(f"共找到 {len(video_files)} 个视频，开始抽帧...")

for video_name in video_files:
    
    folder_name = video_name.split('.')[0]
    if not os.path.exists(folder_name):
        os.makedirs(folder_name)
        
    cap = cv2.VideoCapture(video_name)
    frame_count = 0
    save_count = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
            
       
        if frame_count % 15 == 0:
            img_path = os.path.join(folder_name, f"{folder_name}_img_{save_count:04d}.jpg")
            cv2.imwrite(img_path, frame)
            save_count += 1
            
        frame_count += 1
        
    cap.release()
    print(f"{video_name} 抽帧完成！共抽取了 {save_count} 张图片。")

print("全部处理完毕！")