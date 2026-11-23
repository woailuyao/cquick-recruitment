from __future__ import annotations
from dataclasses import dataclass
from typing import Optional,Sequence,Tuple

import math
import cv2
import numpy as np


#编写一个角度转换函数，将任意角度都折算回-180 到 +180
def wrap_deg(angle:float) -> float:#输入一个float值angle，返回函数值->为float
    a = (float(angle)+180.0)%360.0 - 180.0#折算公式
    return 180.0 if a == -180.0 else a#保证是-180到180的左开右闭区间


#生成一个二维旋转矩阵，把角度转theta
def rot2d(theta_deg:float) -> np.ndarray:
    th = math.radians(theta_deg) #把角度从度改成弧度制，方便后面cossin计算
    c,s = math.cos(th),math.sin(th)
    return np.array([[c,-s],[s,c]],dtype=float) #标准二维旋转矩阵,通过角度旋转返回两个坐标数组，数组中都是角度处理后的值，这样能快速旋转——1_Rod2d


@dataclass #调用工具，自动补全__init__类，直接写可调用为__init__(self,x,y,yaw_deg)

#声明一个类，用来记录机器人在场上位置
class Pose2D:
    x:float #地图坐标x
    y:float #地图坐标y
    yaw_deg:float #摄像机方向角

    #把上面的三个量整合到一个元组里
    def as_tuple(self) -> Tuple[float,float,float]:
        return (self.x,self.y,self.yaw_deg)

    #将采集到的点与地图上坐标相匹配
    def apply(self,points_xy:np.ndarray) -> np.ndarray:
        pts = np.asarray(points_xy,dtype=float).reshape(-1,2)#把输入进来的数组统一格式，防止后面矩阵乘法爆似
        return pts @ rot2d(self.yaw_deg).T + np.array([self.x,self.y])#用.T转置矩阵，然后进行矩阵乘法，再平移

    #将自己的坐标变成视觉采集点（和上面的apply互为逆运算）
    def inverse_apply(self,point_xy:np.ndarray) -> np.ndarray:
        pts = np.asarray(point_xy,dtype=float).reshape(-1,2)
        return (pts - np.array([self.x,self.y])) @ rot2d(self.yaw_deg)#逆变换，其中不用.T，和上面旋转矩阵运算互逆

    #相机防抖
    def lerp(self,other:"Pose2D",alpha:float) -> "Pose2D":
        alpha = float(min(max(alpha,0.0),1.0))#将alpha限制到0-1区间内
        dyaw = wrap_deg(other.yaw_deg - self.yaw_deg)#计算旋转角度，通过刚刚写的wrap_deg转化为180内的角
        return Pose2D(#返回一个pose2d类
            #位置按照换算后的比例靠向other
            self.x+alpha*(other.x-self.x),
            self.y+alpha*(other.y-self.y),
            wrap_deg(self.yaw_deg+alpha*dyaw)#角度同时转，折算进180内
        )


#记录某个像素点数据：投影点、载体坐标
@dataclass
class GroundPoint:
    #原来的像素坐标，即该像素点的绝对位置
    u:float
    v:float
    #载体的像素坐标，即从摄像头到该像素点的相对位置
    x:float
    y:float
    range_m:float#勾股算水平距离
    lam:float#射线参数

#将识别到的点投到地面上
class GroundProjector:

    #记录参数
    def __init__(self,K,dist,R,t,min_range_m:float=0.3,max_range_m:float=15.0):
        self.K = np.asarray(K,dtype=float).reshape(3,3)#存储内参矩阵，全部reshape成3*3的形式
        self.dist = np.asarray(dist,dtype=float).reshape(-1)#用一个一维数组记录畸变系数，几遍系数一般是五个数[k1,k2,p1,p2,k3]
        self.R = np.asarray(R,dtype=float).reshape(3,3)#记录外参旋转R
        self.t = np.asarray(t,dtype=float).reshape(3)#记录外参平移t
        self.min_range_m = float(min_range_m)#过近的点由于dist不可靠，这里选择舍弃
        self.max_range_m = float(max_range_m)#过远的点误差较大
        self._K_inv = np.linalg.inv(self.K)#把内参矩阵求逆矩阵后存储，避免每次投影都重复运算，降低时间复杂度

    #记录屏幕上的某个点，判断出相机是从什么方向看它的
    def pixel_to_ray_base(self,u:float,v:float) -> np.ndarray:
        pts = np.array([[[float(u),float(v)]]],dtype=np.float64)#用三层方括号把u，v转成（N，1，2）形式，方便后面undistortpoints
        und = cv2.undistortPoints(pts,self.K,self.dist)#输入畸变像素，去畸变，将边缘的弯曲线拉直
        ray_cam = np.array([und[0,0,0],und[0,0,1],1.0],dtype=float)#输出相机偏角向量，即目标点与相机的相对位置
        ray_cam /= np.linalg.norm(ray_cam)#归一化cam,除以长度，用norm函数求向量长度，把这个ray变成只标识方向的一个单位向量
        ray_base = self.R @ ray_cam #通过矩阵乘法把相机坐标转换到载体坐标上
        norm = np.linalg.norm(ray_base)#归一化base
        return ray_base/norm if norm>1e-9 else ray_base #返回最后判断出来的方向向量

    #输入一个方向向量，求摄像头根据这个方向向量到地面上的投影
    def project_pixel(self,u:float,v:float) -> Optional[GroundPoint]:#函数返回类型为classGroundPiont or None
        d = self.pixel_to_ray_base(u,v)#通过ray函数得到一个从相机出发的单位向量
        if abs(d[2])<1e-6:#射线趋于平行地面，则舍弃
            return None
        lam = -self.t[2]/d[2] #用平移量t除以单位向量，求射线与平面0的焦点，相机位置在z，方向为d，故解方程t[2]+lam*d[2]=0得到这个式子
        if not np.isfinite(lam) or lam<=0.0:#lam无限大或者小于零（小于零就是在相机后方），则不算有效
            return None
        point = self.t+lam*d #算交点坐标=相机位置+距离*单位向量指向的方向
        rng = float(math.hypot(point[0],point[1]))#通过hypot函数求斜边（水平平面上的距离
        if rng<self.min_range_m or rng>self.max_range_m:#丢掉过近点和过远点
            return None
        return GroundPoint(u=float(u),v=float(v),x=float(point[0]),y=float(point[1]),range_m=rng,lam=float(lam))#按照class类打包返回u

    #批量调用
    def project_pixels(self,uv:Sequence[Sequence[float]]):
        return [self.project_pixel(p[0],p[1])for p in uv]

#将地面上的点投到摄像头捕捉的画面上，逆向验证
def project_ground_to_pixel(K,dist,R,t,points_base:np.ndarray) -> np.ndarray:
    K = np.asarray(K,dtype=float).reshape(3,3)
    R = np.asarray(R,dtype=float).reshape(3,3)
    t = np.asarray(t,dtype=float).reshape(3)
    pts = np.asarray(points_base,dtype=float).reshape(-1,3)
    pts_cam = (pts - t) @ R
    rvec = np.zeros(3)
    tvec = np.zeros(3)
    uv,_ = cv2.projectPoints(pts_cam,rvec,tvec,K,np.asarray(dist,dtype=float).reshape(-1))#相机坐标要算上畸变才能把地面坐标完全投影法上来
    return uv.reshape(-1,2)

#通过相机安装的高度角度来简介计算R、t，构造外参，要识别的参数有安装高度、俯仰角、滚转角、偏转角
def build_extrinsics(height_m:float,pitch_deg:float,roll_deg:float,yaw_deg:float=0.0) ->Tuple[np.ndarray,np.ndarray]:
    R0 = np.array(
        [
            [0.0,0.0,1.0],#相机向前
            [-1.0,0.0,0.0],#相机向右
            [0.0,-1.0,0.0]#相机向下
        ]
    )

    rx, ry, rz = math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)#安装角转成弧度制
    Rx = np.array([[1, 0, 0], [0, math.cos(rx), -math.sin(rx)], [0, math.sin(rx), math.cos(rx)]])#绕X轴转rx度
    Ry = np.array([[math.cos(ry), 0, math.sin(ry)], [0, 1, 0], [-math.sin(ry), 0, math.cos(ry)]])#Y轴转ry度
    Rz = np.array([[math.cos(rz), -math.sin(rz), 0], [math.sin(rz), math.cos(rz), 0], [0, 0, 1]])#Z轴转rz度
    R = Rz @ Ry @ Rx @ R0#摆正R0，依次滚转、俯仰、偏转；@矩阵乘法，连乘顺序是旋转施加顺序
    t = np.array([0.0, 0.0, float(height_m)]) #平移分量：相机比地面高height_m
    return R, t #返回numpy数组
