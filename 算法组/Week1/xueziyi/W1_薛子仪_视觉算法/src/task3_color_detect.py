import numpy as np#矩阵运算
import matplotlib.pyplot as plt#绘图    
import cv2
import os
plt.rcParams['font.sans-serif'] = ['SimHei']#中文字体   
plt.rcParams['axes.unicode_minus'] = False#负号显示

CANDIDATES = {
    "blue":   ((100, 120, 60), (130, 255, 255)),
    "green":  ((40, 80, 50),  (85, 255, 255)),
    "orange": ((8, 120, 80),  (22, 255, 255)),
     # 红色在 HSV 里跨 0 点，额外设置
    "yellow": ((22, 100, 100),(35, 255, 255)),
}

def color_choose(hsv):
    """
    输入：
        hsv
    输出：
        最适合的掩膜,
        并且已经初步去噪"""
    scores={}
    masks={}
    
    for color,(lower,upper) in CANDIDATES.items():
        mask=cv2.inRange(hsv,lower,upper)
        kernel = np.ones((5,5),np.uint8)
        mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
        mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
        contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        scores[color] = max([cv2.contourArea(c) for c in contours], default=0.0)
        masks[color]=mask
#红色
    lower1 = np.array([0, 100, 100])#h o,s 100,v 100
    upper1 = np.array([10, 255, 255])
    mask1 = cv2.inRange(hsv, lower1, upper1)
    lower2 = np.array([170, 100, 100])
    upper2 = np.array([180, 255, 255])
    mask2 = cv2.inRange(hsv, lower2, upper2)
    mask = cv2.bitwise_or(mask1, mask2)    
    contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    scores["red"] = max([cv2.contourArea(c) for c in contours], default=0.0)
    masks["red"]=mask
    best=max(scores,key=scores.get)
    best_mask=masks[best]

    return best_mask




def img_read(path):
    data=np.fromfile(path,dtype=np.uint8)#先读取为数组
    img=cv2.imdecode(data,cv2.IMREAD_COLOR)#解压缩成BGR（高，宽，3通道）（蓝，绿，红）
    return img
#输入图片路径，返回BGR图像

def img_write(path,img,name):
    ext=os.path.splitext(path)[1]#读后缀
    name=name+".png"#输出文件名
    out_path=os.path.join(r"outputs\task3",name)#输出路径
    ok,buf=cv2.imencode(".png",img)#得图片字节
    if ok==False:
        raise IOError(f"编码失败：{out_path}")
    buf.tofile(out_path)#写入outputs
#输入图片，与原图路径。

def main(path):
    img=img_read(path)
    name = os.path.splitext(os.path.basename(path))[0]
    hsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    #h,是色调，红  → 0 附近 和 179 附近
    # lower1 = np.array([0, 100, 100])#h o,s 100,v 100
    # upper1 = np.array([10, 255, 255])
    # mask1 = cv2.inRange(hsv, lower1, upper1)
    # lower2 = np.array([170, 100, 100])
    # upper2 = np.array([180, 255, 255])
    # mask2 = cv2.inRange(hsv, lower2, upper2)
    # mask = cv2.bitwise_or(mask1, mask2)



    mask=color_choose(hsv)
    """掩膜生成中"""
    print("掩膜生成完成")
    img_write(path,mask,name+"mask")



    # kernel = np.ones((5,5),np.uint8)
    # mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,kernel)
    # mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,kernel)
    # #通过开闭运算进行去噪


    #识别轮廓
    contours,hierarchy=cv2.findContours(mask,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
    #输入二值图，轮廓检索模式，轮廓近似方法
    #输出轮廓，轮廓层级
    #然后要绘制轮廓
    """
    cv.drawContours

    输入：
        图像，
        轮廓，
        轮廓索引（-1指全部，从0开始），
        颜色（_,_,_)BGR,
        线宽（-1 or cv2.FILLED指填充）
    """
    contour=[i for i in contours if cv2.contourArea(i)>=1600]
    """按面积筛选"""
    print(f"原生成{len(contours)}个轮廓\n经面积筛选后和剩{len(contour)}个")

    """绘制外界接矩形与中心点"""
    
    cv2.drawContours(img, contour, -1, (255,255,255), 2) # 这时才是红色
    # 给颜色是按通道排序给，与图像通道排序相同，先化为rgb再给颜色
    
    contour_center=[]
    print(f"{name}中")
    for cnt in contour:
        area=cv2.contourArea(cnt)
        rect = cv2.minAreaRect(cnt)      # 1. 求最小外接旋转矩形
        box  = cv2.boxPoints(rect)       # 2. 转成 4 个角点
        box  = np.round(box).astype(np.int32)             # 3. 角点转整数
        cv2.drawContours(img, [box], 0, (0,0,0), 2)   # 4. 画出来
        #wai'jei矩形搞定
        
       
        M = cv2.moments(cnt)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            contour_center.append((cx, cy))
            cv2.circle(img, (cx, cy), 5, (255,0,0), -1)   
        else:
            print("过滤面积设置过大")
        print(f"轮廓{len(contour_center)}面积为{cv2.contourArea(cnt)}\n中心点为({cx},{cy})\n")
    print("\n")
    img_write(path,img,name+"result")

main(r"data\images\img31.jpg")
main(r"data\images\img21.jpg")
main(r"data\images\img32.jpg")

