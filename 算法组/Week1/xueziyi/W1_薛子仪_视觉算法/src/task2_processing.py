import numpy as np#矩阵运算
import matplotlib.pyplot as plt#绘图    
import cv2
import os
plt.rcParams['font.sans-serif'] = ['SimHei']#中文字体   
plt.rcParams['axes.unicode_minus'] = False#负号显示


OUTPUT=r"outputs\task2"


def img_write(path,img,name):
    ext=os.path.splitext(path)[1]#读后缀
    name=name+ext#输出文件名
    out_path=os.path.join(OUTPUT,name)#输出路径
    ok,buf=cv2.imencode(ext,img)#得图片字节
    if ok==False:
        raise IOError(f"编码失败：{out_path}")
    buf.tofile(out_path)#写入outputs
#输入图片，与原图路径。
#将图片存入output下并同名
#支持中文路径


def img_read(path):
    data=np.fromfile(path,dtype=np.uint8)#先读取为数组
    img=cv2.imdecode(data,cv2.IMREAD_COLOR)#解压缩成BGR（高，宽，3通道）（蓝，绿，红）
    return img

def ch_pare(img,blur):
    """
    对图像进行前后对比
    
    参数：
        img:原图
        blur:后图
        
    打印两图对比
    
    返回：
        无"""
    blur_rgb=cv2.cvtColor(blur,cv2.COLOR_BGR2RGB)
    img_rgb=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    plt.subplot(1,2,1)
    plt.imshow(img_rgb,cmap='gray',vmin=0,vmax=255)
    plt.axis("off")
    plt.subplot(1,2,2)#一行，二列，的第二格  从左到右，从上到下
    plt.imshow(blur_rgb,cmap='gray',vmin=0,vmax=255)
    plt.axis("off")
    plt.show()
"""用于直观对比前后变化"""


"""高斯滤波"""#在给定范围，加权平均数，越近越大。细节保留多于平均
def gaussian_blur(img, kernel_size=(5, 5), sigma=0):
    """
    对图像进行高斯滤波。
    
    参数：
        img: 输入图像（BGR格式）。
        kernel_size: 高斯核的大小，必须为奇数。
        sigma: 高斯核的标准差，0表示根据kernel_size自动计算。
    
    返回：
        滤波后的图像。
    """
    return cv2.GaussianBlur(img, kernel_size, sigma)



def main(path):

    name = os.path.basename(path)
    stem = os.path.splitext(name)[0]

    img=img_read(path)#读图
    gray=cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ch_pare(img,gray)
    img_rgb=cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
    img_write(path,gray,stem+"灰度图")

    retval,thresh1=cv2.threshold(gray,127,255,cv2.THRESH_BINARY)#返回retval(实际阈值——自适应有用)
    #thresh1是二值化后的图像，127是阈值，255是最大值，cv2.THRESH_BINARY是二值化类型白白黑黑
    retval,thresh2=cv2.threshold(gray,87,255,cv2.THRESH_BINARY)
    retval,thresh3=cv2.threshold(gray,167,255,cv2.THRESH_BINARY)
    plt.subplot(2,2,1);plt.imshow(img_rgb);plt.axis("off")
    plt.subplot(2,2,2);plt.imshow(thresh1,'gray',vmin=0,vmax=255);plt.axis("off")#显示二值化图
    plt.subplot(2,2,3);plt.imshow(thresh2,'gray',vmin=0,vmax=255);plt.axis("off")
    plt.subplot(2,2,4);plt.imshow(thresh3,'gray',vmin=0,vmax=255);plt.axis("off")
    plt.show()
    """进行不同阈值分割对比
        2*2的图像对比"""
    img_write(path,thresh1,stem+"二值图1")
    img_write(path,thresh2,stem+"二值图2")
    img_write(path,thresh3,stem+"二值图3")

    blur=gaussian_blur(img, (5, 5), 0)
    ch_pare(img,blur)#进行高斯滤波对比
    img_write(path,blur,stem+"高斯滤波图")


    edges=cv2.Canny(blur,100,180)#定上下限
    ch_pare(blur,edges)#边缘图像，一般先高斯滤波，再找边缘
    img_write(path,edges,stem+"边缘图")


    """进行图像形态变换,以边源Dilation图为基础
            适用与二值图，若使用其他图像，要进行额外操作

            本质是使用结构元素进行阈值变化"""

    kernel=np.ones((5,5),np.uint8)
    dilation=cv2.dilate(edges,kernel,iterations=1)
    #输入图像，结构元素，迭代次数
    #如果一个像素元素至少有一个像素在核下是“1”，
    
    closing = cv2.morphologyEx(dilation, cv2.MORPH_CLOSE, kernel)#闭运算
    
    opening=cv2.morphologyEx(dilation,cv2.MORPH_OPEN,kernel)
    
    blackhat = cv2.morphologyEx(dilation, cv2.MORPH_BLACKHAT, kernel)#闭运算前后区别
    
    img_write(path,closing,stem+"闭运算图")
    img_write(path,opening,stem+"开运算图")


main(r"data\images\img21.jpg")
main(r"data\images\img31.jpg")