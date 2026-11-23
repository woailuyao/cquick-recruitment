#每帧先筛出足球，再从标志框里找特征点，把像素坐标投到地面
#用投影成功的点匹配地图，求出位置和朝向，再用上一帧位姿做平滑
#保存每一步的结果，方便查是哪一步定位出了问题

#兼容开关：类型注解当文字看
from __future__ import annotations

from dataclasses import dataclass, field#dataclass 自动生成构造函数；field 给字段设默认值
from typing import List, Optional#类型提示

import numpy as np#numpy：这一帧图像就是 numpy 数组

#从同一个包（文件夹）里的其他模块导入需要的零件，点号表示同一目录
from .detect import Detection#一个检测框
from .field_map import FieldMap, MatchResult, Observation, estimate_pose#地图、匹配结果、观测点、匹配算法
from .geometry import GroundProjector, Pose2D#地面投影器、二维位姿
from .refine import refine_marker#把检测框细化成更精确的像素点


@dataclass#自动生成构造函数
class MarkerInfo:
    #存一个标志的检测框、细化后的像素点和投影得到的地面点，画图和查错时用

    marker_type: str#标志类型，如 LCross
    conf: float#检测置信度
    box: tuple#检测框 (x1,y1,x2,y2)
    pixel: tuple#细化后的精确像素点 (u,v)
    refine_method: str#用了哪种细化方法，如 hough / otsu
    ground: Optional[object]#geometry.GroundPoint 或 None    # 投到地面后的三维点
    observation: Optional[Observation] = None#转换成的观测对象，可能为空


@dataclass
class LocalizerOutput:#每一帧的完整输出，交给画图和打印
    balls: List[Detection] = field(default_factory=list)#这一帧检测到的球
    markers: List[MarkerInfo] = field(default_factory=list)#这一帧的场地标志
    observations: List[Observation] = field(default_factory=list)#成功投到地面的观测
    match: Optional[MatchResult] = None#匹配结果
    pose_raw: Optional[Pose2D] = None#滤波前的位姿
    pose: Optional[Pose2D] = None#滤波后的位姿，画面上显示这个
    dropped: int = 0#投影失败的标志数


class Localizer:#定位器：把一系列步骤串起来
    def __init__(#构造函数，注入它需要的所有零件
        self,
        projector: GroundProjector,#地面投影器（负责像素 -> 地面点）
        field_map: FieldMap,#场地地图（标志点的真实坐标）
        match_cfg: dict,#匹配参数
        filter_cfg: dict,#滤波参数
        ball_names=("ball",),#哪些类别算球，默认只有 ball
    ):
        self.projector = projector#存下来，每帧要用
        self.field_map = field_map
        self.match_cfg = dict(match_cfg)#dict() 复制一份，避免外面改动影响这里
        self.filter_cfg = dict(filter_cfg)#同上
        self.ball_names = set(ball_names)#转成集合（set），后面判断在不在里面更快

        self._prev_pose: Optional[Pose2D] = None#上一帧的位姿，滤波要用
        self._hold = 0#已经连续保持了几帧
        self._last_match: Optional[MatchResult] = None#最近一次匹配结果，调试时方便查

    #------------------------------------------------------------------ #
    def _pick_balls(self,detections: List[Detection]) -> List[Detection]:
        #如果 ball_names 是空的，说明不筛选，所有检测结果都算球
        if not self.ball_names:
            return list(detections)#list() 复制一份，避免外面改动原列表
        #列表推导式 + 条件：只保留名字在 ball_names 集合里的检测框
        return [d for d in detections if d.name in self.ball_names]

    #以 T 字白线为例，检测框圈出的是一小片草地，地图标出的却是两条白线相交的点
    #这里要先在框里找交点，再把它从图像坐标换算成地面坐标
    def _build_markers(self,frame,detections: List[Detection]) -> List[MarkerInfo]:
        infos: List[MarkerInfo] = []#准备空列表装结果
        for det in detections:#逐个场地标志检测框处理
            #第一步：把粗略的检测框细化成更准的一个像素点（交线交点 / 质心等）
            pixel,method = refine_marker(frame,det.xyxy,det.name)
            #第二步：把这个像素点投到地面，得到地图平面上的三维点（z=0）
            ground = self.projector.project_pixel(pixel[0],pixel[1])
            info = MarkerInfo(#打包这一条的中间结果
                marker_type=det.name,#类型
                conf=det.conf,#置信度
                box=det.xyxy,#原来的框
                pixel=pixel,#细化后的点
                refine_method=method,#用了什么方法
                ground=ground,#地面点（可能为 None）
            )
            if ground is not None:#只有真投到地面了，才能当观测用
                info.observation = Observation(#转成匹配算法要的观测格式
                    type=det.name,#标志类型
                    x=ground.x,#地图系 x
                    y=ground.y,#地图系 y
                    conf=det.conf,#置信度
                    tag=f"{det.name}@({pixel[0]:.0f},{pixel[1]:.0f})",#给人看的标签
                    pixel=pixel,#像素坐标，画图时用
                )
            infos.append(info)#加进结果列表
        return infos#返回这一帧所有标志的信息

    #------------------------------------------------------------------ #
    def update(self,frame: np.ndarray,ball_dets: List[Detection],marker_dets: List[Detection]) -> LocalizerOutput:
        #处理当前帧，返回球框、标志点、匹配结果和位姿
        out = LocalizerOutput()#新建一个输出对象
        out.balls = self._pick_balls(ball_dets)#挑出球
        out.markers = self._build_markers(frame,marker_dets)#处理场地标志
        #只保留有观测的那些（ground 为 None 的丢掉）
        out.observations = [m.observation for m in out.markers if m.observation is not None]
        out.dropped = len(out.markers) - len(out.observations)#投影失败的个数

        #---- 地图匹配 ---- #
        cfg = self.match_cfg#拿到参数
        match = estimate_pose(#用观测点去和地图里的点匹配，解出位姿
            out.observations,#这一帧的观测
            self.field_map,#场地地图
            #cfg.get("键", 默认值)：有就用配置里的，没有就用默认值，避免 KeyError
            negative_half_only=cfg.get("negative_half_only",False),
            min_markers=int(cfg.get("min_markers",2)),#少于 2 个点解不出来
            max_markers=int(cfg.get("max_markers",8)),#最多用 8 个点
            max_residual_m=float(cfg.get("max_residual_m",0.45)),
            ambiguity_ratio=float(cfg.get("ambiguity_ratio",0.85)),
        )
        out.match = match#记录匹配结果
        self._last_match = match
        out.pose_raw = match.pose if match.valid else None#匹配成功才有位姿

        #---- 时间滤波 ---- #
        fcfg = self.filter_cfg
        alpha = float(fcfg.get("alpha",0.45))#平滑系数：0 等于完全不动，1 等于完全相信新值
        enabled = bool(fcfg.get("enabled",True))#滤波总开关
        max_hold = int(fcfg.get("max_hold_frames",0))#失败后最多保持几帧旧值，0 表示不保持

        if match.valid and match.pose is not None:#这一帧匹配成功
            raw = match.pose#原始结果
            if enabled and self._prev_pose is not None:#滤波开着，且上一帧有位姿
                out.pose = self._prev_pose.lerp(raw,alpha)#在旧值和新值之间插值，画面更稳
            else:
                out.pose = raw#直接用新值
            self._prev_pose = out.pose#更新上一帧
            self._hold = 0#保持计数清零
        else:
            #定位失败：默认立即置无效，不伪造连续输出
            if max_hold > 0 and self._prev_pose is not None and self._hold < max_hold:
                out.pose = self._prev_pose#短暂沿用上一次的位姿
                self._hold += 1#保持计数加一
            else:
                out.pose = None#真的丢了
                self._prev_pose = None
                self._hold = 0
        return out#交给上层去画图 / 打印
