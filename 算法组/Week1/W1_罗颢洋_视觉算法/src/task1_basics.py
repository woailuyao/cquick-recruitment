import cv2
import os


def task1_basics(image_path,nu, output_dir="../outputs/task1"):
    # 1. 确保输出目录存在
    os.makedirs(output_dir, exist_ok=True)

    # 2. 读取图片并检查是否成功
    img = cv2.imread(image_path)
    img = cv2.resize(img, (0,0),fx = 0.3,fy = 0.3)
    if img is None:
        print(f"图片读取失败了，检查一下路径：{image_path}")
        return

    # 打印宽度、高度和通道数
    height, width, channels = img.shape
    print(f"图片读取成功！宽: {width}, 高: {height}, 通道数: {channels}")

    # 3. 分别完成缩放、裁剪和灰度转换
    # 缩放 (统一缩小一半)
    resized = cv2.resize(img, (width // 2, height // 2))

    # 裁剪 (截取中心区域)
    crop_y, crop_x = height // 4, width // 4
    cropped = img[crop_y:crop_y + height // 2, crop_x:crop_x + width // 2]

    # 灰度转换
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 4. 保存结果 (原图不覆盖)
    cv2.imwrite(os.path.join(output_dir, f"resized_{nu}.jpg"), resized)
    cv2.imwrite(os.path.join(output_dir, f"cropped_{nu}.jpg"), cropped)
    cv2.imwrite(os.path.join(output_dir, f"gray_{nu}.jpg"), gray)

    print(f"图片都存到 {output_dir} 了。")

    # 显示结果 (按任意键关闭窗口)
    cv2.imshow("Original", img)
    cv2.imshow("Resized", resized)
    cv2.imshow("Cropped", cropped)
    cv2.imshow("Gray", gray)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

task1_basics('../data/images/01.jpg',1)
task1_basics('../data/images/02.jpg',2)