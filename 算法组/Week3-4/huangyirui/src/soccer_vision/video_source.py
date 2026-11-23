from __future__ import annotations

import os
import time
from typing import List, Optional
import cv2
import numpy as np
VIDEO_EXT = (".mp4", ".avi", ".mkv", ".mov", ".webm")


class FrameSource:#把读取视频的方式统一包成一个类
    def __init__(self,source:str,width:int = 1280,height:int = 720,fps:int = 30):
        self.source = str(source)#记录来源
        self.width = int(width)
        self.height = int(height)
        self.fps = int(fps)#帧率
        self.kind = "unknown"#类型暂时未知
        self.label = self.source#显示名字
        self.cap: Optional[cv2.VideoCapture] = None#OpenCV读取器对象，留空
        self._files: List[str] = []#目录模式下，所有视频文件的路径
        self._file_idx = 0#当前播到第几个文件
        self.fps_measured = 0.0#实测帧率
        self.nominal_fps = 0.0#文件/相机标称帧率
        self.resolution = (0,0)#实际分辨率
        self.frame_index = 0#记录已读帧数


    def open(self) -> bool:#打开来源，返回值表示是否成功

        if self.source.isdigit():#isdigit判断是否全为数字，若全为数字则返回ture
            return self._open_camera(int(self.source))#全为数字则当作摄像头编号打开

        if os.path.isdir(self.source):#非纯数字序列即可判断为文件夹
            self._files = sorted(#将文件夹内所有视频文件拼接成完整路径
                os.path.join(self.source, f)
                for f in os.listdir(self.source)
                if f.lower().endswith(VIDEO_EXT)
            )

            if not self._files:#没有读取到视频文件
                print(f"[source] 目录里没有视频文件：{self.source}")
                return False
            self.kind = "dir"#标记为目录模式
            return self._open_file(self._files[0])#先打开第一个

        return self._open_file(self.source)#剩下的当单个文件处理


    def _open_camera(self,index:int) -> bool:#按编号打开摄像头，将分辨率、帧率设置为期望值
        self.kind = "camera"#标记类型
        self.label = f"camera:{index}"#显示名
        cap = cv2.VideoCapture(index)#打开第index个摄像头

        if not cap.isOpened():#打不开摄像头
            print(f"[source] 打不开相机 {index}（检查设备节点 / 是否被占用）")
            return False

        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)#请求宽度
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)#请求高度
        cap.set(cv2.CAP_PROP_FPS, self.fps)#请求帧率

        self.cap = cap#保存读取器
        self._measure()#测量实际分辨率与帧率
        return True

    def _open_file(self, path: str) -> bool:#利用给定的视频文件路径将self.cap填成可读帧的读取器，并记下文件名、类型和标称分辨率帧率

        if not os.path.isfile(path):            # 文件不存在
            print(f"[source] 找不到视频文件：{path}")
            return False

        self.kind = "file" if self.kind!="dir" else "dir"
        self.label = os.path.basename(path)#只显示文件名
        cap = cv2.VideoCapture(path)#根据path打开视频文件

        if not cap.isOpened():#打不开报错
            print(f"[source] 打不开视频：{path}")
            return False

        self.cap = cap
        self._measure(measure_fps=False)#文件读取很快，不需要实测帧率，可直接用标称值

        return True

    def _measure(self,measure_fps:bool = True) -> None:#量分辨率和帧率
        cap = self.cap
        assert cap is not None#assert断言此处cap非空不为None，否则后边会报错
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)#取不到宽度就返回0
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)

        self.resolution = (w,h)#记录进self
        self.nominal_fps = float(cap.get(cv2.CAP_PROP_FPS) or 0.0)#读取文件自己标注的帧率，读不到就标零

        if measure_fps:#实测帧率
            n = 20#读20帧
            t0 = time.perf_counter()#开始计时
            ok_count = 0#计数，记录读到了几帧
            for _ in range(n):#循环n遍
                ok,_frame = cap.read()#读一帧
                if ok:#读到了
                    ok_count += 1
            dt = time.perf_counter()-t0#花了多少秒
            self.fps_measured = ok_count/dt if dt>0 else 0.0#帧率=成功帧数/耗时，同时防止除0
        else:
            self.fps_measured = self.nominal_fps#false-文件采用标称帧率

        print(
            f"[source] {self.label}  分辨率 {w}x{h}  "
            f"标称 {self.nominal_fps:.1f} fps  实测 {self.fps_measured:.1f} fps"
        )

        if (w,h)!=(self.width,self.height):#实际分辨率与配置不符
            print(
                f"[source] 警告：实际 {w}x{h} 与配置 {self.width}x{self.height} 不一致，"
                f"若与标定分辨率不同必须重新标定或按比例缩放内参"
            )

    def read(self) -> tuple:#自动切下一个

        while True:#死循环
            if self.cap is None:#没开读取器
                return False,None
            ok,frame = self.cap.read()#读一帧

            if ok:#ok为真即读到了
                self.frame_index += 1#帧号++
                return True,frame

            if self.kind=="dir" and self._file_idx+1<len(self._files):#没读到-如果是目录模式，且后面还有文件，就切下一个继续
                self._file_idx += 1#下一个文件的下标
                self.cap.release()#关掉当前视频
                self._open_file(self._files[self._file_idx])#打开新视频
                continue#continue再读一次

            return False,None#没有True也没有continue回到开头，说明真读不到了，直接返回失败

    def release(self) -> None:#释放资源
        if self.cap is not None:
            self.cap.release()#关闭摄像头
            self.cap = None#置空防止重复释放


    #上下文管理器协议,让文件能写 with FrameSource(...) as src: ...
    def __enter__(self):#进入with代码块时调用
        self.open()#自动打开
        return self#把对象交给as后面的变量
    def __exit__(self,*exc):#用指针离开with代码块时调用
        self.release()#自动释放
