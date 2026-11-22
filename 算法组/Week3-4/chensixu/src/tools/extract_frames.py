import cv2
import os
import glob

# 匹配 Camera_Data 文件夹下所有的 mp4 和 avi 视频
video_files = glob.glob('*.mp4') + glob.glob('*.avi')

if not video_files:
    print("❌ 没找到视频文件！请检查视频是否放在当前文件夹，且后缀是 mp4 或 avi。")
    exit()

output_dir = 'field_images'
os.makedirs(output_dir, exist_ok=True)

total_saved = 0

for video_path in video_files:
    print(f"正在处理视频: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"❌ 无法打开视频: {video_path}")
        continue

    count = 0
    saved_count = 0
    frame_interval = 10 # 每 10 帧抽 1 张

    while True:
        ret, frame = cap.read()
        if not ret:
            break
        if count % frame_interval == 0:
            # 文件名加上视频前缀，防止不同视频的图片重名覆盖
            video_name = os.path.splitext(video_path)[0]
            save_path = f"{output_dir}/{video_name}_{saved_count:04d}.jpg"
            cv2.imwrite(save_path, frame)
            saved_count += 1
            total_saved += 1
        count += 1
    cap.release()
    print(f"✅ {video_path} 抽取完成，共抽出 {saved_count} 张。")

print(f"\n🎉 全部完成！5 个视频总共抽取了 {total_saved} 张图片，全部保存在 {output_dir} 文件夹中。")