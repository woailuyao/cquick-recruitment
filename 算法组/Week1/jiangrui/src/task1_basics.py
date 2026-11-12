import cv2
import os
import numpy as np
#第一张图片
#配置路径
input_path1 =os.path.join('算法组/Week1/jiangrui/data/images', 'img1.jpg')
output_dir = os.path.join('算法组/Week1/jiangrui/outputs','task1')
#读取原图并校验
img1 = cv2.imread(input_path1)
if img1 is None:
    print(f"错误：无法读取图片，请检查路径是否正确：{input_path1}")
    exit()
#打印高，宽和通道数
height,width,channels = img1.shape
print(f"图像读取成功！高度{height},宽度{width},通道数{channels}")
#图像缩放
resized_img1 = cv2.resize(img1,(200,300))
resized_save_path = os.path.join(output_dir,'img1_resized.jpg')
cv2.imwrite(resized_save_path,resized_img1)
#校验保存后的图片是否能重新读取
check_resized = cv2.imread(resized_save_path)
if check_resized is not None:
    print(f"缩放图保存并二次读取成功,路径为：{resized_save_path}")
else:
    print("保存失败，无法重新读取")
#图像裁剪
cropped_img1 = img1[100:200,200:500]
cv2.imwrite(os.path.join(output_dir,'img1_cropped.jpg') ,cropped_img1)
#灰度转换
gray_img1 = cv2.cvtColor(img1,cv2.COLOR_BGR2GRAY)
cv2.imwrite(os.path.join(output_dir,'img1_gray.jpg') ,gray_img1)

#第二张图片缩小后灰度转换
input_path2 = os.path.join('算法组/Week1/jiangrui/data/images', 'img2.jpg')
img2 = cv2.imread(input_path2)
if img2 is None:
    print(f"错误：无法读取图片，请检查路径是否正确：{input_path2}")
    exit()
resized_img2 = cv2.resize(img2,(200,300))
cv2.imwrite(os.path.join(output_dir,'img2_resized.jpg') ,resized_img2)
gray_resized_img2 = cv2.cvtColor(resized_img2, cv2.COLOR_BGR2GRAY)
cv2.imwrite(os.path.join(output_dir, 'img2_resized_gray.jpg'), gray_resized_img2)
#图片显示
cv2.imshow("Original",img1)
cv2.waitKey(1000)
cv2.imshow("Resized",resized_img1)
cv2.waitKey(1000)
cv2.imshow("Cropped",cropped_img1)
cv2.waitKey(1000)
cv2.imshow("Grayscale",gray_img1)
cv2.waitKey(1000)
cv2.imshow("Grayscale2",gray_resized_img2)
cv2.waitKey(0)
cv2.destroyAllWindows()