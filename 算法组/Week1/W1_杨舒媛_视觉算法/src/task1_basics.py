import cv2
import os

image_paths = ['../data/image/1.jpg', '../data/image/2.jpg']

for i, path in enumerate(image_paths):
    print(f"\n===== 开始处理第 {i + 1} 张图片: {path} =====")

    img = cv2.imread(path)
    if img is None:
        print(f"❌ 还是找不到！请确认路径 {path}")
        continue

    height, width, channels = img.shape
    print(f"✅ 读取成功。尺寸: 宽 {width}, 高 {height}, 通道数 {channels}")

    os.makedirs('../outputs/task1', exist_ok=True)

    # 处理图片
    resized_img = cv2.resize(img, (width // 2, height // 2))
    cropped_img = img[height // 4: height * 3 // 4, width // 4: width * 3 // 4]
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 保存图片
    cv2.imwrite(f'../outputs/task1/resized_{i + 1}.jpg', resized_img)
    cv2.imwrite(f'../outputs/task1/cropped_{i + 1}.jpg', cropped_img)
    cv2.imwrite(f'../outputs/task1/gray_{i + 1}.jpg', gray_img)
    print(f"✅ 文件已保存到 outputs/task1/")

    print("⚠️ 请看弹出的三个窗口，按键盘上【任意键】继续处理下一张图...")

    cv2.imshow(f'Pic {i + 1}: Resized (Half Size)', resized_img)
    cv2.imshow(f'Pic {i + 1}: Cropped (Middle)', cropped_img)
    cv2.imshow(f'Pic {i + 1}: Gray (Black & White)', gray_img)

    # 等按键
    cv2.waitKey(0)
    # 按键后，把这3个窗口全关了，准备弹下一张图的
    cv2.destroyAllWindows()

print("\n===== 任务一全部运行完毕！=====")