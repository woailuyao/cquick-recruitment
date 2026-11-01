import numpy as np#矩阵运算
import matplotlib.pyplot as plt#绘图    
import cv2
import os
plt.rcParams['font.sans-serif'] = ['SimHei']#中文字体   
plt.rcParams['axes.unicode_minus'] = False#负号显示

# 1. 读取图片并检查是否读取成功，打印图像宽、高和通道数。
# 2. 分别完成缩放、指定区域裁剪和灰度转换；显示或保存处理结果。
# 3. 将处理后的图片写入文件，检查保存文件是否能够重新读取。原始图片不得被覆盖。
#完成标准：代码可独立运行；每种操作至少有一张输出图片；路径采用项目内相对路径或在 README 中说明
IMG_PATH={"img1":r"data\images\img1.jpg",
          "img2":r"data\images\img2.jpg",
          "img3":r"data\images\img3.jpg"}
OUTPUT=r"outputs\the_first_task"


def img_read(path):
    data=np.fromfile(path,dtype=np.uint8)#先读取为数组
    img=cv2.imdecode(data,cv2.IMREAD_COLOR)#解压缩成BGR（高，宽，3通道）（蓝，绿，红）
    return img
#输入图片路径，返回BGR图像

def img_write(path,img,name):
    ext=os.path.splitext(path)[1]#读后缀
    name=name+ext#输出文件名
    out_path=os.path.join(OUTPUT,name)#输出路径
    ok,buf=cv2.imencode(ext,img)#得图片字节
    if ok==False:
        raise IOError(f"编码失败：{out_path}")
    buf.tofile(out_path)#写入outputs
#输入图片，与原图路径。

#支持中文路径
for i_img in IMG_PATH:
    img=img_read(IMG_PATH[i_img])#读图
    h,w,c=img.shape#高，宽，通道数
    print(f"{i_img}的高为{h}，宽为{w}，通道数为{c}")
#1.读取图片并检查是否读取成功，打印图像宽、高和通道数。


for i_img in IMG_PATH:
    img=img_read(IMG_PATH[i_img])#读图
    resized=cv2.resize(img,None,fx=0.5,fy=0.5,interpolation=cv2.INTER_AREA)#缩放
    img_write(IMG_PATH[i_img],resized,i_img+"_resized_small")#写入
    print(f"{i_img}缩小成功")

for i_img in IMG_PATH:
    img=img_read(IMG_PATH[i_img])#读图
    resized=cv2.resize(img,None,fx=2.0,fy=2.0,interpolation=cv2.INTER_CUBIC)#缩放
    img_write(IMG_PATH[i_img],resized,i_img+"_resized_large")#写入
    print(f"{i_img}放大成功")
#缩放成功


for i_img in IMG_PATH:
    img=img_read(IMG_PATH[i_img])#读图
    roi = img[h//3:h*2//3, w//3:w*2//3].copy()    
    img_write(IMG_PATH[i_img],roi,i_img+"_roi")#写入
    print(f"{i_img}区域裁剪成功")
#区域裁剪成功


for i_img in IMG_PATH:
    img=img_read(IMG_PATH[i_img])#读图
    gray=cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)#灰度转换
    img_write(IMG_PATH[i_img],gray,i_img+"_gray")#写入
    print(f"{i_img}灰度转换成功")
#灰度转换成功   


"""尝试读取保存的文件"""
files=os.listdir(OUTPUT)
for file in files:
    file_path=os.path.join(OUTPUT,file)
    try:
        img=img_read(file_path)
        print(f"{file}读取成功")
    except:
        print(f"{file}读取失败")



print("任务一完成，过程图片保存至任务1,文件夹")
    
