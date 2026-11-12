# ======== 这里写你的代码（任务三：颜色识别与检测）========
#
# 留 言：
#   1. 本文件一行代码都没有，从当前这行之后开始写就行。
#   2. 运行：选中这个文件按 F5，或终端 python src/task3_color_detect.py
#   3. 输入 "data/images/"，掩膜图、标注图、对比图都存 "outputs/"。
#
# 建议覆盖：BGR→HSV 转换、inRange 取色、形态学去噪、findContours 找轮廓、
#           面积／外接矩形／中心点，结果画回原图并标注。
#           sample_scene.png 里有蓝菱形、绿圆、橙三角，正好分三组阈值。
#
# —— 参数改一次存一张图，README 的参数表才有东西写。(๑•̀ㅁ•́๑)✧
# ======================================================
import os
import cv2
import glob
import numpy as np 

path = "data/images/sample_scene.png"#设置路径
img = cv2.imread(path)#根据path读取进img


#原图
cv2.imshow("img",img)
cv2.waitKey(0)


hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#BGR 2 HSV
maska = cv2.inRange(hsv,np.array([100,120,70]),np.array([130,255,255]))#掩膜，仅保留HSV中记录为蓝色区域内的色块，识别出蓝色
cv2.imwrite("outputs/task3/sample3_hsv.jpg",hsv)#保存hsv
cv2.imwrite("outputs/task3/sample3_maska.jpg",maska)#保存maska

#打开和关闭窗口
cv2.imshow("hsv",hsv)
cv2.waitKey(0)
cv2.imshow("maska",maska)
cv2.waitKey(0)
cv2.destroyAllWindows()


#原图
cv2.imshow("img",img)
cv2.waitKey(0)


core = np.ones((5,5),np.uint8)#生成一个核结构元素，大于噪点但小于目标
maskaOpen = cv2.morphologyEx(maska,cv2.MORPH_OPEN,core)#进行一次开运算
maskaClose = cv2.morphologyEx(maska,cv2.MORPH_CLOSE,core)#进行一次闭运算，分别储存
cv2.imwrite("outputs/task3/sample3_maskaOpen.jpg",maskaOpen)#保存maskaOpen
cv2.imwrite("outputs/task3/sample3_maskaClose.jpg",maskaClose)#保存maskaClose

#打开和关闭窗口
cv2.imshow("maskaOpen",maskaOpen)
cv2.waitKey(0)
cv2.imshow("maskaClose",maskaClose)
cv2.waitKey(0)
cv2.destroyAllWindows()


Howshape,_ = cv2.findContours(maska,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#统计轮廓的拐点
Howshape = [c for c in Howshape if cv2.contourArea(c)>=300]#筛选，保留面积大于300的轮廓
print("How many left?",len(Howshape))#输出保留的轮廓数
print(Howshape)#输出保留下来的轮廓长啥样


result = img.copy()#被标记图
for c in Howshape:#遍历
    area = cv2.contourArea(c)#筛选，记录面积
    cnt1,cnt2,cnt3,cnt4 = cv2.boundingRect(c)#记录外接矩形，用四个角坐标表示
    M1 = cv2.moments(c)#创建一个一阶矩，等会算重心
    if M1["m00"] == 0:
        continue       #防止零做除数
    cx = int(M1["m10"]/M1["m00"])
    cy = int(M1["m01"]/M1["m00"])
    #记录重心
    cv2.rectangle(result,(cnt1,cnt2),(cnt1+cnt3,cnt2+cnt4),(0,255,0),2)#画外接矩形
    cv2.circle(result,(cx,cy),4,(0,0,255),-1)#画中心点
    print("The center:",(cx,cy),"The area:",area)
cv2.imwrite("outputs/task3/sample3_result.jpg",result)#保存result


#打开和关闭窗口
cv2.imshow("result",result)
cv2.waitKey(0)
cv2.destroyAllWindows()


# ================================================================ #

for f in glob.glob("data/images/A*.png"):#循环，扫描路径下A开头的所有文件，带进来遍历
    An= cv2.imread(f)#存进An
    if An is None:#跳过无法读出来的
        continue
    hsvAn = cv2.cvtColor(An,cv2.COLOR_BGR2HSV)#BGR 2 HSV
    maskAn = cv2.inRange(hsvAn,(0,0,0),(180,90,100))#识别暗色区间
    maskAn = cv2.morphologyEx(maskAn, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))#用开运算填补
    cntAn,_ = cv2.findContours(maskAn,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#找相应轮廓
    for i in cntAn:
        cntAn1,cntAn2,cntAn3,cntAn4 = cv2.boundingRect(i)
        if cntAn3*cntAn4>20000 and cntAn4>=cntAn3:#要有一定大小、还要高大于宽
            cv2.rectangle(An,(cntAn1,cntAn2),(cntAn1+cntAn3,cntAn2+cntAn4),(0,255,0),2)#Zywoo插U盘了
    Namecheck = os.path.basename(f)
    check = cv2.imwrite("outputs/task3/The Zywoo_"+Namecheck,An)#保存Zywoo视角
    print(Namecheck,f)

# ======================================================================= 
#
# 上面这一段是视觉识别的应用，用 5张不同背景或光照条件的图片进行测试            
# 可以看到在A1、A2、A5这种比较亮的地方的识别性会好一些                        
# A3是因为开镜导致识别了大量的暗色区域，这个比较容易优化，即只取中间圆形识别即可 
# A4是因为人物在A门内测阴影处，导致明暗差距较小，经过掩膜后过于模糊，于是识别了匪门上的弹孔作为识别结果，这个需要更精细的识别程序去优化     
# 总之这个识别方法还是有很大局限性，但也有很多优化的方案，希望早日Zywoo play吧   
#
# ======================================================================= 