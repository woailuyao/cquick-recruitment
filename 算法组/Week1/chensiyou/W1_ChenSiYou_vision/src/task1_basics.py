# task1_basics.py
import cv2
import os

# 图片路径
img_path1 = "../data/images/img1.jpg"
img_path2 = "../data/images/img2.jpg"
img_path3 = "../data/images/img3.jpg"


def basic_image_ops(img_path):
    # 读取图像，OpenCV默认读取为BGR格式
    img = cv2.imread(img_path)
    if img is None:
        print(f"图片 {img_path} 读取失败！检查文件路径")
        return

    print(f"\n========== 处理图片：{img_path} ==========")
    # 获取图像尺寸、通道
    height, width, channel = img.shape
    print(f"图像高度: {height}")
    print(f"图像宽度: {width}")
    print(f"通道数: {channel}")
    print("说明：OpenCV读取图片默认是BGR，不是RGB")

    # 转为灰度图
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 图像缩放，等比例缩放到宽度400
    scale_ratio = 400 / width
    img_resized = cv2.resize(img, (int(width * scale_ratio), int(height * scale_ratio)))

    # 图像裁剪，y范围在前，x在后
    crop_img = img[100:500, 100:500]

    # 保存处理后的图片到output文件夹
    base_name = os.path.basename(img_path)
    cv2.imwrite(f"../outputs/gray_{base_name}", gray_img)
    cv2.imwrite(f"../outputs/resize_{base_name}", img_resized)
    cv2.imwrite(f"../outputs/crop_{base_name}", crop_img)
    print("已保存灰度图、缩放图、裁剪图到outputs文件夹")

    # ===== 新增：验证保存后的图片可以重新读取 =====
    saved_gray_path = f"../outputs/gray_{base_name}"
    check_img = cv2.imread(saved_gray_path)
    if check_img is not None:
        print(f"验证成功：{saved_gray_path} 可以重新读取")
    else:
        print(f"验证失败：{saved_gray_path} 读取失败")

    # 显示图片
    cv2.imshow("Original", img_resized)
    cv2.imshow("Gray Image", gray_img)
    print("按任意按键关闭当前图片窗口，继续下一张")
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    basic_image_ops(img_path1)
    basic_image_ops(img_path2)
    basic_image_ops(img_path3)