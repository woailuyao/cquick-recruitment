# ======== 任务一：HSV 颜色分割 + 中心点（40 分）========
#
# 留 言：
#   1. 本文件一行代码都没有，从当前这行之后开始写就行。
#   2. 运行：选中这个文件按 F5（配置已备好），或终端 python src/task1_color_detect.py
#   3. 读图写 "data/images/xxx"，存图写 "out/task1/xxx"——工作目录已设为项目根。
#   4. data/images/ 里放了 7 张练手图（sample_scene.png 是蓝菱形）。正式交作业要换成
#      自己拍的 3 张：目标清晰 / 目标较小 / 背景有相近颜色。
#
# 建议覆盖：读图并检查是否成功、BGR→HSV、inRange 取色、开/闭运算去噪、
#           findContours 找轮廓、按面积筛、算重心 (u, v)、画框画点存图、
#           还有「未检测到目标」分支（少了这条扣 8 分）。
#           阈值写成参数、能从命令行改，加分 +3。
#
# —— 卡住了把报错原样贴给我。(๑•̀ㅂ•́)و✧
# ======================================================
import json
import sys
import cv2
import numpy as np


Image_Path = "data/ball/sample_7.png" #设置统一路径，方便后续代码修改
HSV_Lower = (36,85,60)
HSV_Upper = (60,255,255)
#这里是经过几遍试了过后的hsv，36-60是颜色在黄绿色段，>85是保证灰度不太灰，>60是保证亮度，总体上救能识别到亮绿色

Min_Area = 300 #最小面积排除噪点，200-700好像基本都行，因为这几场图里的球够大

Core_Kernel = 5 #设置开闭核

Show_Window = "--no-window" not in sys.argv #防止被waitkey卡住弹窗



def detect_color(img,lower=HSV_Lower,upper=HSV_Upper,min_area=Min_Area,cores=Core_Kernel):#写成函数
    
    hsv = cv2.cvtColor(img,cv2.COLOR_BGR2HSV)#BGR2HSV
    
    mask = cv2.inRange(hsv,np.array(lower,np.uint8),np.array(upper,np.uint8))#掩膜
    
    kernel = np.ones((cores,cores),np.uint8)#去噪
    
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)#开运算删白点
    mask =cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)#闭运算填黑点
    
    contours,_ = cv2.findContours(mask,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)#找轮廓，取拐点

    #挑选
    best,best_area=None,0.0 #o.O,best是图片类型，bastarea是整型数据
    for c in contours:
        area = cv2.contourArea(c)#轮廓面积
        if area < min_area:#跳过较小的，当作噪点处理
            continue
        if area > best_area:#挑最大，如果有更大的就替换，一直遍历到循环结束
            best,best_area = c,area


    if best is None:
            return None,mask#对没结果的图片有个返回值


    #标重心
    M = cv2.moments(best)#创建一个一阶矩
    if M["m00"] == 0:#防止除零，防止RE
         return None,mask
    cx = int(M["m10"]/M["m00"])
    cy = int(M["m01"]/M["m00"])
    #算重心


    cnt1,cnt2,cnt3,cnt4 = cv2.boundingRect(best)#外接矩阵

    return {
        "detected": True,
        "cnt3": cx, "cnt4": cy,
        "area": int(best_area),
        "bbox": [cnt1,cnt2,cnt3,cnt4],
    }, mask
    #返回一些关键值，然后通过mask输出图片，可视化 



#画检测结果
def draw_result(img,result,mask):
     
    vis = img.copy()#取副本
    mask_BGR = cv2.cvtColor(mask,cv2.COLOR_GRAY2BGR)#转成彩色便于拼接

    if result is None:#判空
        cv2.putText(vis,"Fail to target",(20,45),cv2.FONT_HERSHEY_SIMPLEX,1.2,(0,0,255),3)#提示红字
        return vis,mask_BGR
    #画矩阵
    cnt1,cnt2,cnt3,cnt4 = result["bbox"]
    cv2.rectangle(vis,(cnt1,cnt2),(cnt1+cnt3,cnt2+cnt4),(0,255,0),2)
    cv2.circle(vis,(result["cnt3"],result["cnt4"]),4,(0,0,255),-1)
    cv2.putText(vis,f"({result['cnt3']},{result['cnt4']})",(cnt1,cnt2-8),cv2.FONT_HERSHEY_SIMPLEX,0.6,(0,255,0),2)
    return vis,mask_BGR;


#主函数
def main():
    img = cv2.imread(Image_Path)
    if img is None:#判空
        print("Fail to read,please check the path:",Image_Path)
        sys.exit(1)
    print("high,weigh,tongdao:",img.shape)

    result,mask = detect_color(img)#调用detect函数，最终返回值到reault和mask中

    if result is None:
        print(f"Fail to target,the BGR from {HSV_Lower} to {HSV_Upper},the min area{Min_Area}")
    else:
        print(f"detected={result['detected']} u={result['cnt3']} v={result['cnt4']} "
              f"area={result['area']} bbox={result['bbox']}")

    vis,mask_BGR = draw_result(img,result,mask)#调用draw画框

    #保存
    cv2.imwrite("out/task1/result.png", vis)
    cv2.imwrite("out/task1/mask.png", mask)
    cv2.imwrite("out/task1/side_by_side.png", np.hstack([vis, mask_BGR]))

    with open("out/task1/target.json","w",encoding="utf-8") as f:
        json.dump(result or {"detected":False},f,ensure_ascii=False,indent=2)#存根，方便ros2查阅

    if Show_Window:
        cv2.imshow("mask", mask)
        cv2.imshow("result", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()