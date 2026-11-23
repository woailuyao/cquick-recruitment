from __future__ import annotations
import os
from dataclasses import dataclass
from typing import List, Optional, Sequence
import numpy as np


@dataclass
class Detection:#检测框数据结构
    cls_id: int#类别编号
    name: str#类别名字
    conf: float#置信度0~1，越大表示模型越有把握
    x1: float#框左上角的横坐标
    y1: float#框左上角的纵坐标
    x2: float#框右下角的横坐标
    y2: float#框右下角的纵坐标

    @property
    def xyxy(self) -> tuple:#返回四个坐标组成的元组
        return(self.x1,self.y1,self.x2,self.y2)#按OpenCV习惯的顺序打包

    @property
    def center(self) -> tuple:#检测框中心
        return((self.x1+self.x2)/2.0,(self.y1+self.y2)/2.0)#横坐标中点=（边界+右边界）/2

    @property
    def width(self) -> float:#框的宽度
        return self.x2-self.x1
    @property
    def height(self) -> float:#框的高度
        return self.y2-self.y1

class YoloDetector:#封装一个权重文件对应的YOLO检测器
    def __init__(#构造函数
        self,#self指这个对象自身
        weights:str,#权重文件路径
        conf:float = 0.35,#置信度阈值，低于此值的框舍弃
        imgsz:int = 640,#推理前把图缩放像素
        device:str = "cpu",#cpu或0
        expected_names:Optional[Sequence[str]] = None,#期望类别名
    ):
        self.weights = weights#把参数存到对象上
        self.conf = float(conf)#转浮点数
        self.imgsz = int(imgsz)#存成整数
        self.device = device
        self.expected_names = list(expected_names) if expected_names else None#给了期望类别名就转成列表，没给保持None
        self.available = False#先假设不可用，下面检查后可True
        self.reason = ""#不可用报错
        self.names: dict = {}#类别编号 -> 类别名字
        self._model = None#YOLO模型对象留空

        if not weights:#权重路径是空字符串
            self.reason = "未配置权重路径"
            return
        if not os.path.isfile(weights):#有路径但文件不存在
            self.reason = f"权重文件不存在：{weights}（训练完成后放到 weights/ 目录）"
            return
        try:#尝试导入ultralytics
            from ultralytics import YOLO#延迟导入
        except Exception as exc:
            self.reason = f"未安装 ultralytics：{exc}"#记下失败原因
            return

        try:#加载权重
            self._model = YOLO(weights)#读入.pt文件，得到模型对象
            self.names = dict(getattr(self._model,"names",{}) or {})#getattr取模型names属性，默认空字典，用dict()复制
            self.available = True#到这里说明一没出错，此时就可以标记可用，在上面47行默认false
        except Exception as exc:
            self.reason = f"加载权重失败：{exc}"


    def infer(self,frame_bgr:np.ndarray) -> List[Detection]:#对一帧图做检测
        if not self.available or frame_bgr is None:#模型没准备好或图是None
            return []#返回空列表

        results = self._model.predict(#调用模型推理
            frame_bgr,imgsz=self.imgsz,conf=self.conf,device=self.device,verbose=False
        )

        detections:List[Detection] = []#准备空列表，后续装ans

        for res in results:#一帧图可能返回多个结果对象
            boxes = getattr(res,"boxes",None)#取出boxes检测框集合
            if boxes is None:#boxes空
                continue

            for box in boxes:#一个box对应一整个检测框
                xyxy = box.xyxy[0].tolist()#xyxy是形状[1,4]的二维数组，[0] 取第一行，用tolist()把这个东西变成普通Python列表
                cls_id = int(box.cls[0].item())#类别编号
                conf = float(box.conf[0].item())# 置信度
                name = self.names.get(cls_id, str(cls_id))#查表

                detections.append(#结果加进列表
                    Detection(#新建Detection
                        cls_id=cls_id,#编号
                        name=str(name),#名字
                        conf=conf,#置信度
                        x1=float(xyxy[0]),#左上 x
                        y1=float(xyxy[1]),#左上 y
                        x2=float(xyxy[2]),#右下 x
                        y2=float(xyxy[3]),#右下 y
                    )
                )


        return sorted(detections, key=lambda d: -d.conf)#按置信度从大到小sorted


    def describe(self) -> str:#返回本检测器状态
        if self.available:#可用
            return f"YOLO[{'/'.join(self.names.values())}] <- {os.path.basename(self.weights)}"
        return f"YOLO 不可用：{self.reason}"#不可用报错

