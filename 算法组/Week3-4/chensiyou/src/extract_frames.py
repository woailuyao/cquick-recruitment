import cv2
import os
import glob
VIDEO_DIR = r"D:\Marker_Dataset\videos"
OUTPUT_DIR = r"D:\Marker_Dataset"
FRAME_INTERVAL = 15

def extract_frames():
    # 检查视频文件夹是否存在
    if not os.path.exists(VIDEO_DIR):
        print(f" 错误：找不到视频文件夹 {VIDEO_DIR}，请检查路径！")
        return

    # 自动寻找该文件夹下所有的 .mp4 文件
    video_files = glob.glob(os.path.join(VIDEO_DIR, "*.mp4"))

    if not video_files:
        print(" 视频文件夹里没有找到 .mp4 文件！")
        return

    print(f" 共找到 {len(video_files)} 个视频，开始抽帧...\n")

    for video_path in video_files:
        # 获取视频文件名（不带路径和后缀），比如 marker1
        video_name = os.path.basename(video_path).split('.')[0]

        # 为该视频创建独立的图片保存文件夹
        save_folder = os.path.join(OUTPUT_DIR, video_name)
        if not os.path.exists(save_folder):
            os.makedirs(save_folder)

        # 打开视频
        cap = cv2.VideoCapture(video_path)
        frame_count = 0
        save_count = 0

        while True:
            ret, frame = cap.read()
            if not ret:
                break  # 视频读取完毕

            # 达到间隔帧数，保存图片
            if frame_count % FRAME_INTERVAL == 0:
                # 图片命名格式：marker1_img_0000.jpg
                img_name = f"{video_name}_img_{save_count:04d}.jpg"
                img_path = os.path.join(save_folder, img_name)

                # 保存图片
                cv2.imwrite(img_path, frame)
                save_count += 1

            frame_count += 1

        cap.release()
        print(f" {video_name}.mp4 抽帧完成！共抽取了 {save_count} 张图片。存放在: {save_folder}")




if __name__ == '__main__':
    extract_frames()