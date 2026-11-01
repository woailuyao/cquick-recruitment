import cv2 as cv
import os

img1 = cv.imread('data/images/task1_basics.py_image1.jpg')
img2 = cv.imread('data/images/task1_basics.py_image2.jpg')

#1. 读取图片并检查是否读取成功，打印图像宽、高和通道数。
if img1 is None:
    sys.exit("无法读取图片，请检查路径是否正确！")
else:
    h1, w1, c1 = img1.shape
    print("=" * 30)
    print(f"图片读取成功！")
    print(f"宽度:{w1}, 高度:{h1}, 通道数:{c1}")
    print("=" * 30)


if img2 is None:
    sys.exit("无法读取图片，请检查路径是否正确！")
else:
    h2, w2, c2 = img2.shape
    print("=" * 30)
    print(f"图片读取成功！")
    print(f"宽度:{w2}, 高度:{h2}, 通道数:{c2}")
    print("=" * 30)

#2. 分别完成缩放、指定区域裁剪和灰度转换；显示或保存处理结果。
resized_img1 = cv.resize(img1,( w1 // 2, h1 // 2))
resized_img2 = cv.resize(img2, (w2 // 2, h2 // 2))

cropped_img1 = img1[0:300, 0:300]
cropped_img2 = img2[0:300, 0:300]

gray_img1 = cv.cvtColor(img1, cv.COLOR_BGR2GRAY)
gray_img2 = cv.cvtColor(img2, cv.COLOR_BGR2GRAY)

images1 = [('reimg1', resized_img1),('croimg1', cropped_img1), ('grayimg1', gray_img1)]
for name1, pic1 in images1:
    cv.imshow(name1,pic1)
    cv.waitKey(0) #按任意键继续操作
    cv.destroyWindow(name1)

images2 = [('reimg2', resized_img2),('croimg2', cropped_img2), ('grayimg2', gray_img2)]
for name2, pic2 in images2:
    cv.imshow(name2, pic2)
    cv.waitKey(0) #按任意键继续操作
    cv.destroyWindow(name2)


#3. 将处理后的图片写入文件，检查保存文件是否能够重新读取。原始图片不得被覆盖。
save_dir = 'outputs/task1'
os.makedirs(save_dir, exist_ok=True)

cv.imwrite(f'{save_dir}/img1_01_resized.jpg', resized_img1)
cv.imwrite(f'{save_dir}/img2_01_resized.jpg', resized_img2)

cv.imwrite(f'{save_dir}/img1_02_cropped.jpg', cropped_img1)
cv.imwrite(f'{save_dir}/img2_02_cropped.jpg', cropped_img2)

cv.imwrite(f'{save_dir}/img1_03_gray.jpg', gray_img1)
cv.imwrite(f'{save_dir}/img2_03_gray.jpg', gray_img2)
print(f"处理完成! 六张图片已保存至:task1 文件夹。")

print("开始检查保存文件是否能够重新读取")
files_to_check = [
    f'{save_dir}/img1_01_resized.jpg',
    f'{save_dir}/img2_01_resized.jpg',
    f'{save_dir}/img1_02_cropped.jpg',
    f'{save_dir}/img2_02_cropped.jpg',
    f'{save_dir}/img1_03_gray.jpg',
    f'{save_dir}/img2_03_gray.jpg'
]

for file_path in files_to_check:
    check_img = cv.imread(file_path)

    if check_img is not None:
        print("读取成功！")
    else:
        print(f"读取失败，文件损坏或不存在:{file_path}")

#检查原始图片
check_original1 = cv.imread('data/images/task1_basics.py_image1.jpg')
check_original2 = cv.imread('data/images/task1_basics.py_image2.jpg')
if check_original1 is not None :
        print("原图片1仍在原路径!图片1没有被覆盖!")
        
if check_original2 is not None:
        print("原图片2仍在原路径!图片2没有被覆盖!")



