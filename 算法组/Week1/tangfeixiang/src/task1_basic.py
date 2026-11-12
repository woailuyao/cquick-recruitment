from distutils.command import check
from email.mime import image

import cv2
import os

current_script_path=os.path.abspath(__file__)
src_dir=os.path.dirname(current_script_path)
project_root=os.path.dirname(src_dir)
image_path=os.path.join(project_root, "../data", "images", "test1.jpg")
output_dir = os.path.join(project_root, "../outputs", "task1")


from matplotlib.pyplot import gray
from numpy.ma.core import resize

image_path= "../data/images/test1.jpg"
img=cv2.imread(image_path)
if img is None:
    print("图片读取失败")
else:
    print("图片读取成功")
    h,w,c=img.shape
    print(f"宽:{w},高:{h},长:{c}")
    resized_img=cv2.resize(img,(100,100))
    cv2.imwrite("../outputs/task1/resized.jpg", resized_img)
    crop_img=img[100:200,100:200]
    cv2.imwrite("../outputs/task1/crop.jpg", crop_img)
    gray_img=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    cv2.imwrite("../outputs/task1/gray.jpg", gray_img)
    print("成功运行")
    check_img=cv2.imread("../outputs/task1/gray.jpg")
    if check_img is None:
        print("失败，未成功保存")
    else:
        print("成功保存")
