from __future__ import annotations
from dataclasses import dataclass,field
from typing import Dict,List,Optional
from .geometry import Pose2D,rot2d,wrap_deg

import itertools
import math
import numpy as np


#字典键: 值一一对应，做别名 -> 标准名翻译表
TYPE_ALIASES = {
    "PenalityPoint": "PenaltyPoint",
    "PenaltyPoint": "PenaltyPoint",
    "L": "LCross",
    "LCross": "LCross",
    "T": "TCross",
    "TCross": "TCross",
    "X": "XCross",
    "XCross": "XCross",
}

#将上述的写法全部根据字典Type_aliases换成标准的类别名
def canon_type(raw:str) -> str:
    return TYPE_ALIASES.get(str(raw).strip(),str(raw).strip())#用strip去空格

@dataclass(frozen=True)#创建制度记录，冻结修改权限
class FieldFeature:#记录地图上的一个标记点
    name:str#原始名
    type:str#标准名
    #x，y坐标
    x:float
    y:float

@dataclass
class Observation:#记录一个观测到的场地标志
    type:str
    x:float
    y:float
    conf:float=1.0#置信度，挑出最可信的点，将可信度比较高的点优先读取并计入计算，可信度的低的点排到后面
    tag:str = ""
    pixel:tuple = (0.0,0.0)#原来的像素坐标，这里单独提出来以后画图用

    def __post_init__(self) -> None:
        self.type = canon_type(self.type)#类型名规范化

@dataclass
class MatchResult:#将算出来的位姿和置信度打包放进类里
    pose:Optional[Pose2D]#接出来的位姿，若解析哦失败就是None
    valid:bool#判断可用性
    status:str#文字状态
    residual:float #残差
    marker_count:int #参与到计算中的标志数
    total_observations:int#某一帧观测到的标志总数
    candidates:int =0#记录试过多少候选位姿
    pairs:List[tuple] = field(default_factory=list)#记录配对，哪个观测点和哪个地面点配对

    #转到可以写进json的普通字典里
    def to_dict(self) -> dict:

        return {
            "valid": self.valid,
            "status": self.status,
            "residual": round(self.residual, 4), #round四舍五入到4位小数
            "marker_count": self.marker_count,
            "total_observations": self.total_observations,
            "candidates": self.candidates,
            "pose": None if self.pose is None else {#没有位姿就写 None有就转成小字典
                "x": round(self.pose.x, 4),
                "y": round(self.pose.y, 4),
                "yaw_deg": round(self.pose.yaw_deg, 3),
            },#把每个 (标签, 名字, 残差) 变成一个字典
            "pairs": [
                {"obs": tag, "map": name, "residual": round(res, 4)} for tag, name, res in self.pairs
            ],
        }


#记录场地地图
class FieldMap:
    def __init__(self,features:List[FieldFeature],source:str=""):
        self.features = list(features)#利用list复制一份fuatures，防止函数里面导致外面改动
        self.source = source#记录来自哪个文件
        self._by_type:Dict[str,List[FieldFeature]]={}#把场地上采集到的features整合list全部放进一个索引表
        for f in self.features:#遍历self.features
            self._by_type.setdefault(f.type,[]).append(f)#setdefault(f.type,[])查字典，利用f.type取f类型，再利用append将f加入list类型变量self._by_type的末尾，即入典

    @classmethod
    def load(cls,path:str) -> "FieldMap":#记录field路径，并且整合fieldmap对象
        from .configio import load_yaml #延迟导入，避免循环导入爆似

        data = load_yaml(path)#读字典
        features:List[FieldFeature] = []#依旧copy一个下来,不动原来的
        for key,value in data.items():#利用.items（）同时获取data的键和值
            if key.startswith("_"):#读取键的开头，如果开头是下划线代表其为自定义数据，直接跳过
                continue
            if not isinstance(value,(list,tuple)) or len(value)<2:#长度值小于等于2也跳过
                continue
            raw_type = "".join(ch for ch in str(key) if ch.isalpha())#删掉key上面的数字后缀再存入raw
            features.append(
                FieldFeature(name=str(key),type=canon_type(raw_type),x=float(value[0]),y=float(value[1]))
            )
        if not features:#一个点都没有，文件格式不对
            raise ValueError(f"field.yaml is None:{path}")
        return cls(features,source = path)#返回一个与def类型相同的cls类函数(FieldMap)

    def by_type(self,marker_type:str) -> List[FieldFeature]:#查询，从某个类型的所有地图点里遍历
        return self._by_type.get(canon_type(marker_type),[])#没有就会直接返回一个全是[]的空列表

    @property
    def types(self) -> List[str]:#将所有类型名存储并用sort排序
        return sorted(self._by_type.keys())#利用.keys()取所有值

    def bounds(self) -> tuple:#标定场地范围
        xs = [f.x for f in self.features]
        ys = [f.y for f in self.features]
        return min(xs),max(xs),min(ys),max(ys)

    def counts(self) -> Dict[str,int]:#统计每种类型各有多少个点
            return {t:len(v) for t,v in self._by_type.items()}#字典推导式：{类型：列表}

    def __len__(self) -> int:#python的特殊处理
        return len(self.features)


#匹配地图与观测
def _pose_from_pair(p1:np.ndarray,p2:np.ndarray,m1:np.ndarray,m2:np.ndarray) -> Optional[Pose2D]:#由两个观测点与两个地图点闭式解se2

    dp,dm = p2-p1,m2-m1#通过坐标相减得到两个方向向量，一个是观测的，一个是地图里的

    if np.linalg.norm(dp)<1e-6 or np.linalg.norm(dm)<1e-6:#两点重合
        return None

    yaw = math.degrees(math.atan2(dm[1],dm[0])-math.atan2(dp[1],dp[0]))#通过atan2求方向角弧度，再相减得到旋转量
    R = rot2d(yaw)#转成旋转矩阵
    t = m1 - R@p1#平移量=地图点-转过后的观测点

    return Pose2D(float(t[0]),float(t[1]),wrap_deg(yaw))#包成位姿对象


def _evaluate(pose:Pose2D,observations:List[Observation],field_map:FieldMap,inlier_thresh:float):#把观测搬到地图上

    pts= np.array([[o.x,o.y] for o in observations],dtype=float)#N×2的观测坐标矩阵
    moved = pose.apply(pts)#把N个点搬到地图坐标系
    edges = []
    for idx,(obs,point) in enumerate(zip(observations,moved)):
        for feature in field_map.by_type(obs.type):
            distance = math.hypot(point[0]-feature.x,point[1]-feature.y)
            if distance<=inlier_thresh:
                edges.append((distance,idx,feature.name))
    used_names = set()
    assignment = {}
    for distance,idx,name in sorted(edges):
        if idx not in assignment and name not in used_names:
            assignment[idx] = (name,distance)
            used_names.add(name)
    key = tuple((idx,assignment[idx][0]) for idx in sorted(assignment))
    pairs = [(observations[idx].tag or observations[idx].type,name,assignment[idx][1]) for idx,name in key]
    rmse = math.sqrt(sum(d*d for _,_,d in pairs)/len(pairs)) if pairs else math.inf
    return len(pairs),rmse,pairs,key


def estimate_pose(
    observations:List[Observation],#帧观测
    field_map:FieldMap,#地图
    min_markers:int = 2,#最少几个标志才能解
    max_markers:int = 8,#最多用几个(优化枚举时间复杂度)
    max_residual_m:float = 0.45,#残差上限
    ambiguity_ratio:float = 0.85,#歧义判定阈值
    inlier_thresh_m:float = 0.35,#判定命中的距离阈值
    negative_half_only:bool = False,
) -> MatchResult:
    total = len(observations)#观测总数
    obs = sorted(observations,key=lambda o:-o.conf)[: max(2,max_markers)] if observations else []#按置信度从高到低排序，最多取max_markers个，至少留2个
    if len(obs)<2 or total==0: #MatchResult的参数顺序pose,valid,status,residual,marker_count,total_observations
        return MatchResult(None,False,"insufficient_markers",float("inf"),0,total)
    if len(obs)<min_markers:
        return MatchResult(None,False,"insufficient_markers",float("inf"),0,total)

    obs = [o for o in obs if field_map.by_type(o.type)]#只用地图里真有同类点的观测
    if len(obs)<min_markers:
        return MatchResult(None,False,"unknown_marker_type",float("inf"),0,total)


    candidates = set()#按观测编号和地图点名字去重

    #下面的循环运算量极大，比较影响时间复杂度，也是有优化空间的地方
    #总体是将所有观测点两队匹配假设，再全场评分选最优匹配点
    #其中利用记录key进行排除重复，用到了部分记忆化搜索的思想


    for i,j in itertools.combinations(range(len(obs)), 2):#combinations(range(n),2)生成所有两两组合的下标对
        oi,oj = obs[i],obs[j]
        p1 = np.array([oi.x,oi.y])
        p2 = np.array([oj.x,oj.y])
        maps_i = field_map.by_type(oi.type)
        maps_j = field_map.by_type(oj.type)

        if oi.type==oj.type:
            map_pairs = list(itertools.permutations(maps_i,2))
        else:
            map_pairs = [(a, b) for a in maps_i for b in maps_j]

        for mi, mj in map_pairs:
            pose = _pose_from_pair(p1,p2,np.array([mi.x,mi.y]),np.array([mj.x,mj.y]))
            if pose is None:
                continue
            inliers,rmse,pairs,key = _evaluate(pose,obs,field_map,inlier_thresh_m)
            if inliers>=2:
                candidates.add(key)

    if not candidates:
        return MatchResult(None,False,"insufficient_markers",math.inf,0,total)

    #同一组配对只拟合一次，距离较近的点受像素噪声影响更小
    features = {f.name:f for f in field_map.features}
    scored = []
    if negative_half_only:
        xmin,_,ymin,ymax = field_map.bounds()
    for key in sorted(candidates):
        src = np.array([(obs[i].x,obs[i].y) for i,_ in key])
        dst = np.array([(features[n].x,features[n].y) for _,n in key])
        weights = 1.0/np.maximum(1.0,np.sum(src*src,axis=1))**2
        center_src = np.average(src,axis=0,weights=weights)
        center_dst = np.average(dst,axis=0,weights=weights)
        a,b = src-center_src,dst-center_dst
        dot = np.sum(weights*(a[:,0]*b[:,0]+a[:,1]*b[:,1]))
        cross = np.sum(weights*(a[:,0]*b[:,1]-a[:,1]*b[:,0]))
        if math.hypot(dot,cross)<1e-12:
            continue
        yaw = math.degrees(math.atan2(cross,dot))
        x,y = center_dst-rot2d(yaw)@center_src
        pose = Pose2D(float(x),float(y),yaw)
        count,rmse,pairs,assignment = _evaluate(pose,obs,field_map,inlier_thresh_m)
        # 本次机位都在无球门半场，观测目标仍可跨中线
        if negative_half_only and not (xmin<=x<=0 and ymin<=y<=ymax):
            continue
        scored.append((count,rmse,pose,pairs,assignment))

    if not scored:
        return MatchResult(None,False,"not_converged",math.inf,0,total)
    scored.sort(key=lambda item: (-item[0],item[1]))#内点优先，残差次之，排序键(-内点数,残差)内点数取反 =从多到少，残差从小到大。
    best = scored[0]#最优点


    #歧义判定：内点数相同、地图点集合不同、残差很接近
    second = None

    for item in scored[1:]:#[1:]表示从第二个开始
        if item[0]==best[0] and item[4] != best[4]:
            second = item
            break#找到一个就停

    ambiguous = False

    if second is not None:
        #保留厘米级噪声余量，两个近似等好的配对判为歧义
        threshold = ambiguity_ratio if ambiguity_ratio<=1 else 1/ambiguity_ratio
        #远处点投影到地面后误差更大，两点解缺第三个标志交叉验证
        tolerance = 0.025
        if best[0]==2:
            distances = [math.hypot(obs[i].x,obs[i].y) for i,_ in best[4]]
            tolerance = max(tolerance,0.005*max(distances)**2)
        ambiguous = best[1]+tolerance>=threshold*second[1]

    inliers,rmse,pose,pairs, _ = best#元组解包


    if inliers<min_markers:
        return MatchResult(pose,False,"insufficient_markers",rmse,inliers,total,len(scored), pairs)
    if not math.isfinite(rmse) or rmse>max_residual_m:
        return MatchResult(pose,False,"high_residual",rmse,inliers,total,len(scored), pairs)
    if ambiguous:
        return MatchResult(pose,False,"ambiguous",rmse,inliers,total,len(scored),pairs)
    return MatchResult(pose,True,"ok",rmse,inliers,total,len(scored),pairs)
