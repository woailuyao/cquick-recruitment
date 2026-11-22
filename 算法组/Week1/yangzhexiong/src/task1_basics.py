import cv2
import os
input_folder='../data/images'
output_folder='../outputs/task1'
image_list=['test1.jpg','test2.jpg']
print("---开始执行任务---")
for img_name in image_list:
    print(f"\n正在处理图片:{img_name}")
    img_path=os.path.join(input_folder,img_name)
    img = cv2.imread(img_path)
    if img is None:
        print(f"错误:无法读取{img_name},请检查图片是否在 images 文件夹中")
        continue
    height, width, channels = img.shape
    print(f"读取成功 尺寸：宽={width}，高={height}，通道数={channels}")
    file_prefix = os.path.splitext(img_name)[0]
    new_w = width // 2
    new_h = height // 2
    resized_img = cv2.resize(img,(new_w,new_h))
    resized_path = os.path.join(output_folder,f"{file_prefix}_resized.jpg")
    cv2.imwrite(resized_path,resized_img)
    print(f"已生成缩放图:{resized_path}")
    crop_img = img[height//4 : height*3//4,width//4 : width*3//4]
    crop_path = os.path.join(output_folder,f"{file_prefix}_crop.jpg")
    cv2.imwrite(crop_path,crop_img)
    print(f"已生成裁剪图:{crop_path}")
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    gray_path = os.path.join(output_folder,f"{file_prefix}_gray.jpg")
    cv2.imwrite(gray_path,gray_img)
    print(f"已生成灰度图:{gray_path}")
    check_img = cv2.imread(gray_path)
    if check_img is None:
        print(f"验证通过：保存的图片可以重新读取")
    else:
        print(f"警告保存的图片无法读取！")
print("\n---任务一全部执行完毕 请去 outputs/task1 文件夹查看结果---")