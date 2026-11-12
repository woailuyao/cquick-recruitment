# ======== 这里写你的代码（任务二：图像处理）========
#
# 留 言：
#   1. 本文件一行代码都没有，从当前这行之后开始写就行。
#   2. 运行：选中这个文件按 F5，或终端 python src/task2_processing.py
#   3. 输入 "data/images/"，输出 "outputs/"——工作目录已设为项目根。
#
# 建议覆盖：高斯／中值滤波、边缘检测（Sobel／Canny）、阈值分割（Otsu）、
#           形态学（开运算／闭运算）、处理前后对比图拼接。
#
# —— 中间结果记得都存到 outputs/，README 里要写参数。(￣ω￣)
# ===================================================
import cv2
import numpy as np


path = "data/images/sample_scene.png"#设置路径
img = cv2.imread(path)#根据path读取进img


gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) #生成灰度图
cv2.imwrite("outputs/task2/sample2_gray.jpg",gray)#保存gray


#打开和关闭窗口
cv2.imshow("gray",gray)
cv2.waitKey(0)
cv2.destroyAllWindows()


gaus = cv2.GaussianBlur(img,(5,5),0)#高斯滤波
cv2.imwrite("outputs/task2/sample2_gaus.jpg",gaus)#保存gaus


#打开和关闭窗口
cv2.imshow("gaus",gaus)
cv2.waitKey(0)
cv2.destroyAllWindows()

gaus2 = cv2.GaussianBlur(img,(21,21),0)#高斯滤波
cv2.imwrite("outputs/task2/sample2_gaus2.jpg",gaus2)#保存gaus


#打开和关闭窗口
cv2.imshow("gaus2",gaus2)
cv2.waitKey(0)
cv2.destroyAllWindows()


_, thres = cv2.threshold(img, 130, 255, cv2.THRESH_BINARY)#阈值分割
cv2.imwrite("outputs/task2/sample2_thres.jpg",thres)#保存thres


#打开和关闭窗口
cv2.imshow("thres",thres)
cv2.waitKey(0)
cv2.destroyAllWindows()


can = cv2.Canny(img,50,150)#canny边缘图
cv2.imwrite("outputs/task2/sample2_can.jpg",can)#保存can


#打开和关闭窗口
cv2.imshow("can",can)
cv2.waitKey(0)
cv2.destroyAllWindows()


kernel = np.ones((3, 3), np.uint8)#调用numpy，构建一个3*3的核
mpeopen= cv2.morphologyEx(img,cv2.MORPH_OPEN,kernel)#形态学处理，消除图片中小于对应值的白点
mpeclose= cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel)#形态学处理，消除图片中小于对应值的黑点
cv2.imwrite("outputs/task2/sample2_mpeopen.jpg",mpeopen)#保存mpe
cv2.imwrite("outputs/task2/sample2_mpeclose.jpg",mpeclose)#保存mpe


#打开和关闭窗口
cv2.imshow("mpe",mpeopen)
cv2.waitKey(0)
cv2.destroyAllWindows()
#打开和关闭窗口
cv2.imshow("mpe",mpeclose)
cv2.waitKey(0)
cv2.destroyAllWindows()

kernel2 = np.ones((21, 21), np.uint8)#调用numpy，构建一个3*3的核
mpeopen2= cv2.morphologyEx(img,cv2.MORPH_OPEN,kernel2)#形态学处理，消除图片中小于对应值的白点
mpeclose2= cv2.morphologyEx(img,cv2.MORPH_CLOSE,kernel2)#形态学处理，消除图片中小于对应值的黑点
cv2.imwrite("outputs/task2/sample2_mpeopen2.jpg",mpeopen2)#保存mpe
cv2.imwrite("outputs/task2/sample2_mpeclose2.jpg",mpeclose2)#保存mpe


#打开和关闭窗口
cv2.imshow("mpe2",mpeopen2)
cv2.waitKey(0)
cv2.destroyAllWindows()
#打开和关闭窗口
cv2.imshow("mpe2",mpeclose2)
cv2.waitKey(0)
cv2.destroyAllWindows()