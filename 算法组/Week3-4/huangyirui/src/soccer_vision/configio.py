from __future__ import annotations
from typing import Any

try:#尝试可选依赖包
    import yaml as _pyyaml
except Exception:
    _pyyaml = None

import os


#YAML解析函数
def _strip_comment(line: str) -> str:#去掉行尾注释，但保留引号中的#
    out,quote = [],None#out收集字符,quote判断当前是否在引号里
    for ch in line:
        if quote:#若在引号内
            out.append(ch)#全部保留
            if ch==quote:#引号结束
                quote=None
        elif ch in "\"'":#不在引号内但遇到引号
            quote=ch
            out.append(ch)
        elif ch=='#':#引号外遇到#，后面是注释
            break#结束循环，丢掉本行后面的所有内容
        else:
            out.append(ch)
    return "".join(out).rstrip()#拼接成字符串，并用rstrip去掉右边的所有空白


def _parse_scalar(text: str) -> Any:#将一段文本转换成对应的pythonZhi
    text = text.strip()#用strip去掉首尾空格
    if not text:#空字符==False
        return None
    if text.startswith("[") and text.endswith("]"):#像[1,2,3...]这种的
            inner = text[1:-1].strip()#[1:-1]去掉首尾的方括号
            if not inner:#里面没东西
                return []#返回空列表
            return [_parse_scalar(part) for part in inner.split(",")]#逗号为分割，逐个递归解析

    #形如"abc"或'abc'去掉首尾的引号就是内容
    if (text.startswith('"') and text.endswith('"')) or (
        text.startswith("'") and text.endswith("'")
    ):return text[1:-1]

    low = text.lower()#转小写再比较，TRUE/True/true都能用
    if low in ("true","yes"):
        return True
    if low in ("false","no"):
        return False
    if low in ("null","none","~"):
        return None
    try:#试着当整数
        return int(text)
    except ValueError:#转不了整数就继续往下试,pass就是什么也不做
        pass
    try:#试着当小数
        return float(text)
    except ValueError:
        pass
    return text#都不行，当普通字符串


def _mini_load(text:str) -> Any:#缩进式迷你解析器，遇到key:就直接递归，遇到- 建列表
    lines:list[tuple[int,str]] = []#每一行都存成(缩进的空格数,去掉注释后的内容)这样的形式
    for raw in text.splitlines():#splitlines()按换行切分成若干行
        no_comment = _strip_comment(raw)#去行尾注释
        if not no_comment.strip():#空行直接跳过
            continue

        indent = len(no_comment)-len(no_comment.lstrip(" "))#缩进=总长度-去掉左边空格后的长度
        lines.append((indent,no_comment.strip()))

    def parse_block(idx:int,indent: int) -> tuple[Any,int]:#从idx行开始，解析缩进为indent的整块

        if idx>=len(lines):#越界
            return None, idx
        is_list = lines[idx][1].startswith("- ")#判断某行是不是以- 开头
        container:Any = [] if is_list else {}#是列表建列表，否则建字典
        while idx<len(lines):#解析到缩进变浅
            cur_indent,content = lines[idx]

            if cur_indent<indent:#缩进变浅，结束一个板块
                break
            if cur_indent>indent: #意外缩进，跳过
                idx += 1
                continue

            if is_list:#处理列表元素
                if not content.startswith("- "):
                    break
                item = content[2:].strip()#[2:]去掉开头”- “
                if item.endswith(":") or (": " in item and not item.startswith(("[", '"', "'"))):#判断这个元素是不是【键: 值】形式
                    container.append(item.rstrip(":"))#化成字符串
                    idx += 1
                else:
                    container.append(_parse_scalar(item))#正常解析
                    idx += 1

            else:#处理字典键值

                if content.startswith("- "):
                    break

                if ":" not in content:#没有冒号的行忽略
                    idx += 1
                    continue

                key,_,value = content.partition(":")#partition(":")按第一个冒号切成三段——》(左边,冒号,右边)
                key = key.strip()
                value = value.strip()
                if value:#冒号后面有值，说明是简单键值对
                    container[key] = _parse_scalar(value)
                    idx += 1
                else:# 冒号后面没值，说明下面还有一层
                    child,idx = parse_block(idx+1,indent+2)#利用递归解析子块
                    container[key] = child if child is not None else {}

        return container,idx#返回这块的结果和停下的位置


    return parse_block(0,lines[0][0] if lines else 0)[0]#从第0行开始解析，起始缩进取第一行的缩进，[0]表示只要结果，不要位置


def load_yaml(path:str) -> dict:#读取YAML文件，返回 dict，文件不存在时输出FileNotFoundError

    if not os.path.isfile(path):#检查文件
        raise FileNotFoundError(path)#不在-报告异常

    with open(path,"r",encoding="utf-8") as fh:#with会在退出时自动关闭文件
        text = fh.read()#一次读完整个文件

    if _pyyaml is not None:#有PyYAML就用，直接用pyyaml要规范些
        data = _pyyaml.safe_load(text) or {}#safe_load失败可能返回None，所以or{}
    else:
        data = _mini_load(text) or {}#用自己写的解析器

    return data if isinstance(data,dict) else {"_":data}#如果解析出来不是字典（比如整个文件就是个列表），保证返回类型固定。


#------接下来的是将上面通过yaml读出来的内容处理好然后全部搞进几个类里面----
class CameraIntrinsics:#相机内参-焦距fxyx，主点cxcy，畸变系数
    def __init__(self,K,dist,width:int,height:int,status:str = ""):
        import numpy as np #延迟导入，加快运行速度

        self.K = np.asarray(K, dtype=float).reshape(3, 3)#内参矩阵3×3
        self.dist = np.asarray(dist, dtype=float).reshape(-1)#畸变系数拉成一维
        self.width = int(width)
        self.height = int(height)
        self.status = status#状态说明

    @property
    def is_calibrated(self) -> bool:
            return bool(self.K[0,0]>1.0 and self.K[1,1]>1.0 and self.K[0,2]>0.0)#fx/fy>0且主点非0才算标定过,K[0,0]是fx，K[1,1]是fy，K[0,2]是主点cx。


class CameraExtrinsics:#相机外参-装哪，角度。高之类的量
    def __init__(self,R,t,height_m:float= 0.0,status:str = ""):#记录内参，读进self里
        import numpy as np
        self.R = np.asarray(R, dtype=float).reshape(3, 3)#旋转矩阵
        self.t = np.asarray(t, dtype=float).reshape(3)#平移向量
        self.height_m = float(height_m)#安装高度
        self.status = status

    @property
    def is_calibrated(self) -> bool:
        return bool(self.t[2]>0.02)#相机高度必须为正，否则地面投影无意义

def load_intrinsics(path: str) -> CameraIntrinsics:#读内参yaml

    data = load_yaml(path)#解析成字典
    K = data.get("camera_matrix") or [[0,0,0],[0,0,0],[0,0,1]]#内参矩阵，缺项零矩阵
    dist = data.get("dist_coeffs") or [0,0,0,0,0]#畸变系数，缺项全零
    width = data.get("image_width", 1280)
    height = data.get("image_height", 720)
    status = str(data.get("status",""))#状态字段

    try:#正常数值构造
        return CameraIntrinsics(K=K,dist=dist,width=width,#数值齐全就走这里
                                height=height,status=status)#接上高和状态
    except (ValueError, TypeError):#有TBD报错
        return CameraIntrinsics(K=[[0,0,0],[0,0,0],[0,0,1]],#退回零矩阵
                                dist=[0,0,0,0,0],#畸变清零
                                width=width,height=height,status="todo")#标未标定

def load_extrinsics(path: str) -> CameraExtrinsics:#读外参yaml

    data = load_yaml(path)#解析成字典
    R = data.get("R")#取3×3矩阵

    if R is None:#没有矩阵,扁平写法
        flat = data.get("rotation") or [1,0,0,0,0,-1,0,1,0]#9个数缺项给默认
        R = [flat[0:3],flat[3:6],flat[6:9]]#切成三行三列

    t = data.get("t") or [0.0,0.0,0.0]#平移向量
    height = data.get("camera_height_m", t[2] if len(t) > 2 else 0.0)#高度取不到就用t[2]
    status = str(data.get("status",""))#状态字段

    try:#正常数值构造
        return CameraExtrinsics(R=R,t=t,height_m=height,status=status)
    except (ValueError,TypeError):
        return CameraExtrinsics(R=[[1,0,0],[0,-1,0],[0,0,-1]], t=[0.0,0.0,0.0],#退回单位旋转
                                height_m=0.0, status="todo")#高度清零并标未标定

def load_runtime_config(path:str) -> dict:#将六段参数一次性弄成嵌套后的dict
    data = load_yaml(path)


    def pick(block:str,key:str,default:Any) -> Any:#直接使用外层已读好的data，形成闭包
        node = data.get(block) or {}#先取一段
        if isinstance(node,dict):#确认是字典
            return node.get(key,default)#再取具体项
        return default

    #配置表
    return {
        "camera": {
            "device": pick("camera", "device", 0),                 # 相机编号
            "width": pick("camera", "width", 1280),                # 分辨率宽
            "height": pick("camera", "height", 720),               # 分辨率高
            "fps": pick("camera", "fps", 30),                      # 帧率
        },
        "detect": {
            "ball_weights": pick("detect", "ball_weights", ""),    # 足球模型路径
            "marker_weights": pick("detect", "marker_weights", ""),  # 标志模型路径
            "ball_conf": pick("detect", "ball_conf", 0.35),        # 球的置信度阈值
            "marker_conf": pick("detect", "marker_conf", 0.30),    # 标志的置信度阈值
            "imgsz": pick("detect", "imgsz", 640),                 # 推理输入尺寸
            "device": str(pick("detect", "device", "cpu")),        # cpu 或 0（显卡）
        },
        "geometry": {
            "min_range_m": pick("geometry", "min_range_m", 0.3),   # 最近有效距离
            "max_range_m": pick("geometry", "max_range_m", 15.0),  # 最远有效距离
        },
        "match": {
            "negative_half_only": pick("match", "negative_half_only", False),
            "min_markers": pick("match", "min_markers", 2),        # 最少几个标志才解位姿
            "max_markers": pick("match", "max_markers", 8),        # 最多用几个
            "max_residual_m": pick("match", "max_residual_m", 0.45),  # 残差上限（米）
            "ambiguity_ratio": pick("match", "ambiguity_ratio", 1.2),  # 歧义判定比例
        },
        "filter": {
            "enabled": pick("filter", "enabled", True),            # 是否开启平滑
            "alpha": pick("filter", "alpha", 0.45),                # 平滑系数
            "max_hold_frames": pick("filter", "max_hold_frames", 0),  # 丢失后保持几帧
        },
        "output": {
            "jsonl": pick("output", "jsonl", ""),                  # 结果日志路径
            "video": pick("output", "video", ""),                  # 录像输出路径
        },
    }
