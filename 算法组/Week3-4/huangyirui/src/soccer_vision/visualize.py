#用OpenCV画左右两块画面，左边是相机，右边是场地俯视图
#左边画球框、标志框和细化后的像素点，同时显示FPS和推理耗时
#右边画地图特征点和匹配连线，用圆点和箭头表示相机位置和朝向
#地图y轴向上，图像y轴向下，算像素位置时要翻转

#兼容开关：类型注解当文字看
from __future__ import annotations

import math#算三角函数，用来放置朝向箭头
from typing import Optional#类型提示：Optional[X] 表示是 X 或者 None

import cv2#绘图 + 文字
import numpy as np#造画布、拼图

from .detect import Detection#一个检测框（这里只用来做类型提示）
from .field_map import FieldMap, MatchResult#场地地图、匹配结果
from .geometry import Pose2D#二维位姿

#---- 颜色常量（注意：OpenCV 用的是 BGR 顺序，不是 RGB！(255,170,60) 其实是偏蓝的橙色）----
COLOR_BALL = (60,200,255)#足球框的颜色：橙黄
COLOR_OK = (80,220,120)#定位成功：绿色
COLOR_BAD = (80,80,240)#定位失败：红色
COLOR_MARKER = {#每种标志一个颜色，用字典按类型查
    "LCross": (255,170,60),
    "TCross": (255,120,200),
    "XCross": (120,220,255),
    "PenaltyPoint": (160,255,160),
}
FONT = cv2.FONT_HERSHEY_SIMPLEX#OpenCV 自带的一种简洁无衬线字体


def _put(img,text,org,scale=0.55,color=(240,240,240),thick=1):
    #先写黑色描边再写彩色字，背景亮的时候也看得清
    #先画粗一点的黑色字（thick + 2）当描边，org 是文字左下角的位置
    cv2.putText(img,text,org,FONT,scale,(0,0,0),thick + 2,cv2.LINE_AA)
    #再画细一点的彩色字盖在上面；LINE_AA 表示抗锯齿，字边缘更平滑
    cv2.putText(img,text,org,FONT,scale,color,thick,cv2.LINE_AA)


#--------------------------------------------------------------------------- #
#左侧：RGB 画面
#--------------------------------------------------------------------------- #
def draw_left(
    frame: np.ndarray,#原始画面
    balls: list,#足球检测框列表
    markers: list,#场地标志信息列表（soccer_vision.localizer.MarkerInfo）
    fps: float,#当前帧率
    infer_ms: float,#推理耗时（毫秒）
    match: Optional[MatchResult] = None,#匹配结果，可能为空
) -> np.ndarray:
    canvas = frame.copy()#复制一份再画，绝不在原图上涂改（投流时要保持干净）

    for det in balls:#遍历每个足球
        #列表推导式转成整数像素坐标：OpenCV 只接受整数
        x1,y1,x2,y2 = [int(v) for v in det.xyxy]
        cv2.rectangle(canvas,(x1,y1),(x2,y2),COLOR_BALL,2)#画框，线宽 2
        cu,cv = det.center#框的中心
        #radius=4、线宽 -1 表示实心圆点
        cv2.circle(canvas,(int(cu),int(cv)),4,COLOR_BALL,-1)
        #max(14, y1 - 8)：标签放在框上方 8 像素，但不能顶出画面（至少留 14）
        _put(canvas,f"{det.name} {det.conf:.2f} (u={cu:.0f},v={cv:.0f})",(x1,max(14,y1 - 8)),0.5,COLOR_BALL)

    for info in markers:#遍历每个场地标志
        x1,y1,x2,y2 = [int(v) for v in info.box]
        #dict.get(键, 默认值)：找不到这个类型就用灰色，不会报 KeyError
        color = COLOR_MARKER.get(info.marker_type,(200,200,200))
        cv2.rectangle(canvas,(x1,y1),(x2,y2),color,2)
        pu,pv = int(info.pixel[0]),int(info.pixel[1])#细化后的精确像素点
        #drawMarker 画一个十字星，尺寸 12，线宽 2
        cv2.drawMarker(canvas,(pu,pv),color,cv2.MARKER_CROSS,12,2)
        label = f"{info.marker_type} {info.conf:.2f}"
        if info.ground is not None:#成功投到地面了
            label += f" ({info.ground.x:.2f},{info.ground.y:.2f})m"#把地面坐标也写上去
        else:
            label += " proj:fail"#投影失败就标出来
        _put(canvas,label,(x1,max(14,y1 - 8)),0.46,color)

    h,w = canvas.shape[:2]#画面尺寸：高、宽
    #左下角写上分辨率、帧率、推理耗时
    _put(canvas,f"{w}x{h}  FPS {fps:5.1f}  infer {infer_ms:5.1f} ms",(10,h - 12),0.55)
    return canvas


#--------------------------------------------------------------------------- #
#右侧：地图
#--------------------------------------------------------------------------- #
class MapCanvas:
    #把场地里的米换成画布上的像素，用来画俯视图

    def __init__(self,field_map: FieldMap,width: int = 640,height: int = 720,margin: int = 46):
        self.field_map = field_map#地图数据
        self.width = int(width)#画布宽（像素）
        self.height = int(height)#画布高（像素）
        self.margin = int(margin)#四周边距
        x_min,x_max,y_min,y_max = field_map.bounds()#场地特征点的范围
        pad = 0.6#上下左右再留 0.6 米的余量
        self.x_min,self.x_max = x_min - pad,x_max + pad
        self.y_min,self.y_max = y_min - pad,y_max + pad
        #分别算横向每米几像素和纵向每米几像素，取小的那个保证不拉伸变形
        sx = (self.width - 2 * self.margin) / max(self.x_max - self.x_min,1e-6)
        sy = (self.height - 2 * self.margin) / max(self.y_max - self.y_min,1e-6)
        self.scale = min(sx,sy)

    #地图用米表示距离，窗口用像素指定落笔位置
    #假设每米画 50 像素，地面相距 2 米的两个点就该画得相隔 100 像素
    #还要考虑屏幕上的行号是向下增大的
    def to_px(self,x: float,y: float) -> tuple:
        #把场地坐标换成画布上的像素坐标
        px = self.margin + (x - self.x_min) * self.scale
        #地图 +Y 向上，图像 +Y 向下 -> 翻转
        py = self.height - self.margin - (y - self.y_min) * self.scale
        return int(px),int(py)#OpenCV 要求整数

    def _base(self) -> np.ndarray:
        #先画深色背景、每米一格的网格和坐标轴，再画地图特征点
        #np.full(形状, 填充值, 类型)：造一块 22 灰度的深色画布
        img = np.full((self.height,self.width,3),22,dtype=np.uint8)
        #网格（1 米一格）
        #floor 向下取整、ceil 向上取整，range 左闭右开，所以 +1 保证画到边
        for gx in range(int(math.floor(self.x_min)),int(math.ceil(self.x_max)) + 1):
            p1,p2 = self.to_px(gx,self.y_min),self.to_px(gx,self.y_max)
            cv2.line(img,p1,p2,(40,44,50),1)#竖线
        for gy in range(int(math.floor(self.y_min)),int(math.ceil(self.y_max)) + 1):
            p1,p2 = self.to_px(self.x_min,gy),self.to_px(self.x_max,gy)
            cv2.line(img,p1,p2,(40,44,50),1)#横线
        #坐标轴：+X 向右（红），+Y 向上（绿）
        ox,oy = self.to_px(0,0)#原点在画布上的位置
        cv2.arrowedLine(img,(ox,oy),(ox + 40,oy),(90,90,220),2,tipLength=0.25)
        cv2.arrowedLine(img,(ox,oy),(ox,oy - 40),(90,200,90),2,tipLength=0.25)
        _put(img,"+X",(ox + 44,oy + 6),0.45,(140,140,240))
        _put(img,"+Y",(ox + 6,oy - 44),0.45,(140,240,140))
        #地图特征点
        for f in self.field_map.features:#每个地图上的标志
            color = COLOR_MARKER.get(f.type,(200,200,200))
            px,py = self.to_px(f.x,f.y)#换成画布坐标
            cv2.circle(img,(px,py),4,color,-1)#实心小圆点
            #replace("Cross", "")：把名字里的 Cross 去掉，标签更短
            cv2.putText(img,f.name.replace("Cross",""),(px + 6,py - 6),FONT,0.36,color,1,cv2.LINE_AA)
        #统计每种标志各有多少个，拼成 "LCross:4  TCross:2" 这样的字符串
        counts = "  ".join(f"{t}:{n}" for t,n in sorted(self.field_map.counts().items()))
        _put(img,f"field.yaml  {len(self.field_map)} pts  [{counts}]",(12,20),0.46,(200,200,200))
        return img

    def draw(
        self,
        pose: Optional[Pose2D],#当前位姿，可能没有（定位失败时）
        match: Optional[MatchResult],#匹配结果
        fps: float = 0.0,
        loc_hz: float = 0.0,
    ) -> np.ndarray:
        img = self._base()#先画底图

        if pose is not None:#有位姿才画相机
            px,py = self.to_px(pose.x,pose.y)
            #朝向箭头：yaw 从 +X 逆时针，图像 y 向下 -> 取负
            th = math.radians(pose.yaw_deg)#角度转弧度
            #cos 管 x 方向、sin 管 y 方向；箭头长 46 像素，y 要用减号才是逆时针
            tip = (int(px + 46 * math.cos(th)),int(py - 46 * math.sin(th)))
            cv2.arrowedLine(img,(px,py),tip,COLOR_OK,3,tipLength=0.3)
            cv2.circle(img,(px,py),7,COLOR_OK,-1)#相机位置画个实心圆
            _put(img,"camera",(px + 10,py + 22),0.46,COLOR_OK)

            if match is not None and match.pairs:
                #观测点 -> 匹配到的地图点，画细线看匹配关系
                for tag,name,_res in match.pairs:#元组解包，_res 不用
                    #next((...), None)：从生成器里取第一个满足条件的元素，取不到给 None
                    feat = next((f for f in self.field_map.features if f.name == name),None)
                    if feat is None:
                        continue#地图里没这个名字，跳过
                    qx,qy = self.to_px(feat.x,feat.y)
                    cv2.line(img,(px,py),(qx,qy),(90,160,120),1,cv2.LINE_AA)

        #下面几行是为了没有 match 时也能正常显示，统一取默认值
        valid = match.valid if match is not None else False
        status = match.status if match is not None else "-"
        residual = match.residual if match is not None else float("nan")#nan = 非数字
        count = match.marker_count if match is not None else 0
        total = match.total_observations if match is not None else 0

        lines = []#先把要写的几行文字攒起来
        if pose is not None:
            #:+.3f 表示带正负号、保留 3 位小数，+: 是即使正数也显示加号
            lines.append(f"x_camera_map = {pose.x:+.3f} m")
            lines.append(f"y_camera_map = {pose.y:+.3f} m")
            lines.append(f"yaw_camera_map = {pose.yaw_deg:+.2f} deg")
        else:
            lines.append("x_camera_map = --")#没有数值就写两个横线
            lines.append("y_camera_map = --")
            lines.append("yaw_camera_map = --")
        lines.append(f"marker_count = {count}/{total}")
        #residual == residual 是判断不是 nan的常用技巧（nan 永远不等于自己）
        lines.append(f"residual = {residual:.3f} m" if residual == residual else "residual = --")
        lines.append(f"valid = {str(valid).lower()}   status = {status}")#.lower() 变成 true/false
        lines.append(f"FPS {fps:5.1f}   loc {loc_hz:5.1f} Hz")

        y0 = self.height - self.margin + 4#文字区从画布底部往上排
        color = COLOR_OK if valid else COLOR_BAD#有效绿、无效红
        for i,text in enumerate(lines):#enumerate 同时给下标和数据
            #往前三行是坐标数字，用白色；后面的状态行用绿/红
            _put(img,text,(12,y0 - (len(lines) - 1 - i) * 16 + 12),0.46,color if i >= 3 else (235,235,235))
        return img


def compose(left: np.ndarray,right: np.ndarray) -> np.ndarray:
    #先对齐左右图的高度，再横向拼成一张图
    h = max(left.shape[0],right.shape[0])#目标高度取两者较高者
    if left.shape[0] != h:#高度不一样就缩放，缩放时保持宽高比
        left = cv2.resize(left,(int(left.shape[1] * h / left.shape[0]),h))
    if right.shape[0] != h:
        right = cv2.resize(right,(int(right.shape[1] * h / right.shape[0]),h))
    #造一条 4 像素宽的灰色竖线当分隔条
    sep = np.full((h,4,3),60,dtype=np.uint8)
    #hstack = horizontal stack，横向堆放：左图 + 分隔条 + 右图
    return np.hstack([left,sep,right])
