from __future__ import annotations
import math
from typing import Optional, Tuple

import cv2
import numpy as np

CANNY_LOW = 40#Canny边缘检测的低阈值：越小保留越多噪声
CANNY_HIGH = 120#Canny的高阈值：强边缘的判定
MIN_LINE_LEN_RATIO = 0.35#相对bbox短边的最小线段长度
MAX_LINE_GAP = 6#同一条线上的断口小于6像素就连起来
ANGLE_SPLIT_DEG = 25.0#两组主方向至少要差25度

def _roi(frame:np.ndarray,box:Tuple[float,float,float,float],pad_ratio:float = 0.25):#沿着框裁剪下来，方便后面算质心
    h,w = frame.shape[:2]#【：2】只取高和宽
    x1,y1,x2,y2 = box#解包
    bw,bh = x2-x1,y2-y1#求框的高和宽
    px,py = bw*pad_ratio,bh*pad_ratio#求向外扩张的像素，扩张是为了保证边缘完整
    rx1,ry1 = int(max(0,x1-px)),int(max(0,y1-py))#max(0,...)/min(w,...)保证坐标不出图像边界，int()把小数转成整数像素下标
    rx2,ry2 = int(min(w,x2+px)),int(min(h,y2+py))

    if rx2-rx1<8 or ry2-ry1<8:#区域太小没法找线
        return None
    return rx1,ry1,rx2,ry2#返回(左, 上, 右, 下)


def _line_angle(p1:np.ndarray,p2:np.ndarray) -> float:#将线段相对水平的角度折算会0-180
    return math.degrees(math.atan2(p2[1]-p1[1],p2[0]-p1[0]))%180.0


def _intersect(l1:np.ndarray,l2:np.ndarray) -> Optional[np.ndarray]:#求两条线段延长线的焦点，平行或近乎平行返回none
    x1,y1,x2,y2 = l1#第一条线段两端
    x3,y3,x4,y4 = l2#第二条线段两端
    den =(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)#用两点式直线的行列式公式，den是分母

    if abs(den)<1e-6:#平行
        return None

    px = ((x1*y2-y1*x2)*(x3-x4)-(x1-x2)*(x3*y4-y3*x4))/den
    py = ((x1*y2-y1*x2)*(y3-y4)-(y1-y2)*(x3*y4-y3*x4))/den

    return np.array([px,py],dtype=float)#交点坐标


#ROI = Region Of Interest
def refine_cross(frame:np.ndarray,box:Tuple[float,float,float,float]) -> Tuple[Optional[tuple],str]:#利用检测框算出一个精确交点cross，即在ROI内找两组主方向线段，返回的是全图坐标
    roi_box = _roi(frame,box)#先扩出ROI
    if roi_box is None:
        return None,"bbox_too_small"#第二个返回值是方法名/失败原因，方便排查

    rx1,ry1,rx2,ry2 = roi_box#解包
    patch = frame[ry1:ry2,rx1:rx2]#把这一小块裁下来，numpy的切片是左闭右开，ry1:ry2高，rx1:rx2取宽
    gray = cv2.cvtColor(patch,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(3,3),0)#高斯去噪，core=3*3

    bw,bh = box[2]-box[0],box[3]-box[1]#原检测框的宽高
    min_len = max(8,int(min(bw,bh)*MIN_LINE_LEN_RATIO))#最短线段长度
    edges = cv2.Canny(gray,CANNY_LOW,CANNY_HIGH)#边缘图，白线表示边缘
    lines = cv2.HoughLinesP(#霍夫变换，可从边缘里找出直线段
        edges,1,np.pi/180.0,threshold=max(12,min_len//2),#edges图、分辨率=1、theta角度分辨率、threshold边缘点最少数量
        minLineLength=min_len,maxLineGap=MAX_LINE_GAP,#长度下线、断裂容忍
    )

    if lines is None:
        return None,"no_lines"
    segments = np.asarray(lines,dtype=float).reshape(-1,4)#每行放两个端点的四个坐标
    if len(segments)<2:
        return None,"no_lines"
    lengths = np.linalg.norm(segments[:,2:]-segments[:,:2],axis=1)
    la = segments[lengths.argmax()]
    angle = _line_angle(la[:2],la[2:])
    #按长度从大到小找另一条线，反向的平行线仍算同一方向
    for idx in np.argsort(lengths)[::-1]:
        lb = segments[idx]
        delta = abs(angle-_line_angle(lb[:2],lb[2:]))
        if min(delta,180.0-delta)>=ANGLE_SPLIT_DEG:
            break
    else:
        return None,"one_direction_only"

    point = _intersect(la,lb)#求交点
    if point is None:
        return None,"parallel"

    u,v = point[0]+rx1,point[1]+ry1#ROI内坐标要加回ROI的偏移才是整幅图里的坐标

    if not (rx1-bw<=u<=rx2+bw and ry1-bh<=v<=ry2+bh):#交点跑出ROI太远说明配错线
        return None,"intersection_out_of_roi"#报错

    return (float(u),float(v)),"hough_intersection"#成功就返回坐标和方法名


def refine_penalty_point(frame:np.ndarray,box:Tuple[float,float,float,float]) -> Tuple[Optional[tuple],str]:#ROI 内做阈值分割，取最大连通块的质心作为点球点
    roi_box = _roi(frame,box,pad_ratio=0.15) #点球点是圆点，外扩小一点就够
    if roi_box is None:
        return None,"bbox_too_small"

    rx1,ry1,rx2,ry2 = roi_box
    patch = frame[ry1:ry2,rx1:rx2]#抠图
    gray = cv2.cvtColor(patch,cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray,(5, 5),0)

    _,mask = cv2.threshold(gray,0,255,cv2.THRESH_BINARY+cv2.THRESH_OTSU)#筛白点，点球点是百点

    if mask.mean()>127:#白色像素平均超过一半说明前景反了
        mask = cv2.bitwise_not(mask) #黑白翻转

    kernel = np.ones((3,3),np.uint8)#3×3的核
    mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN, kernel)#开运算


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)#找轮廓
    if not contours:#空列表
        return None, "no_blob"

    biggest = max(contours,key=cv2.contourArea)#取面积最大轮廓

    if cv2.contourArea(biggest)<12: #太小弃之
        return None,"blob_too_small"

    m = cv2.moments(biggest)#算图像矩求重心

    if abs(m["m00"])<1e-6:
        return None,"zero_moment"
    u = m["m10"]/m["m00"]+rx1
    v = m["m01"]/m["m00"]+ry1

    return (float(u),float(v)),"threshold_centroid"


def refine_marker(frame:np.ndarray,box:Tuple[float,float,float,float],marker_type:str) -> Tuple[tuple,str]:#按标志类型选择细化方式，返回(像素点, 方法说明)，失败时退回bbox中心
    marker_type = str(marker_type)#统一转成字符串

    if marker_type in ("LCross","TCross","XCross","L","T","X"):
        point,method = refine_cross(frame,box)#交叉型标志走找交点
    elif marker_type in ("PenaltyPoint","PenalityPoint"):#兼容旧拼写
        point,method = refine_penalty_point(frame, box) #圆点走质心
    else:
        point,method = None,"unsupported_type" #unkwon类型

    if point is None:#细化失败，退回检测框的中心点
        cx = (box[0]+box[2])/2.0
        cy = (box[1]+box[3])/2.0

        return (cx,cy), f"bbox_center_fallback({method})"

    return point,method#成功
