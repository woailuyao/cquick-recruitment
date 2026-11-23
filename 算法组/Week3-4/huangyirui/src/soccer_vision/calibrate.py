
from __future__ import annotations

import argparse#解析命令行参数
import glob as globlib#按通配符（*.png）找文件，改名避免和变量 glob 撞名
import os#判断文件/目录、拼路径、取文件名
import shutil#复制文件做备份
import sys#改模块查找路径（这里其实没用到，留着无妨）
import time#取当前时间写进文件
from pathlib import Path#路径对象

import cv2#OpenCV：找棋盘角点、做标定
import numpy as np#numpy：数组运算

#往上两级 = 项目根目录
PROJECT_ROOT = Path(__file__).resolve().parents[2]
#允许的图片后缀；元组结尾的逗号不是必须的，但写成元组更清楚
IMG_EXT = (".png",".jpg",".jpeg",".bmp",".tif",".tiff")


def collect_images(arg: str) -> list:#把用户给的路径整理成图片文件列表
    #传目录就收集里面的图片，也可以传单张图片路径或*.png这样的通配符
    if os.path.isdir(arg):#情况一：给的是文件夹
        #列表推导式：[对 f 做某操作，for f in 列表，if 条件成立]。这里筛出图片文件并拼成完整路径
        files = [os.path.join(arg,f) for f in sorted(os.listdir(arg)) if f.lower().endswith(IMG_EXT)]
    elif any(ch in arg for ch in "*?["):#情况二：字符串里有通配符 * ? [
        files = sorted(globlib.glob(arg))#glob 按通配符展开，sorted 让顺序稳定
    else:#情况三：给的是单个文件
        files = [arg] if os.path.isfile(arg) else []#存在就放进去，不存在就是空列表
    #再过滤一次，确保只留图片；lower() 让 .PNG 这样的大写后缀也能认
    return [f for f in files if f.lower().endswith(IMG_EXT)]


#角点是棋盘内部四个方格相接的位置
#例如横向有 8 个方格，内部只有 7 列交点
#这里寻找的就是这些交点，后面会把照片中的位置与棋盘上已知的位置配对
def find_corners(gray,pattern: tuple):#在一张灰度图里找棋盘格的内部角点
    #先用SB找角点，找不到就换普通方法，再把角点细化到亚像素
    #findChessboardCornersSB 是新版接口，对模糊和大角度更宽容；flags 用穷举 + 高精度
    ok,corners = cv2.findChessboardCornersSB(gray,pattern,flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if ok:#找到了
        return True,corners.astype(np.float32),"SB"#astype 转成 32 位浮点，后续接口要求这个类型
    #退路：经典版接口。ADAPTIVE_THRESH 用自适应阈值，FAST_CHECK 先快速判断有没有棋盘
    ok,corners = cv2.findChessboardCorners(gray,pattern,flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_FAST_CHECK)
    if not ok:#还是找不到
        return False,None,"none"#返回失败
    #亚像素精化：在初值附近再做一次精细搜索，把角点位置精确到 0.1 像素级
    #(11,11) 是搜索窗大小，( -1,-1 ) 表示自动算，criteria 是迭代停止条件（40 次或变化 < 1e-4）
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,40,1e-4)
    corners = cv2.cornerSubPix(gray,corners,(11,11),(-1,-1),criteria)
    return True,corners,"classic"#返回经典版结果


def write_yaml(path: Path,K,dist,width: int,height: int,rms: float,cols: int,rows: int,square: float,n_ok: int) -> None:
    K = np.asarray(K,dtype=float)#保证 K 是浮点矩阵
    dist = np.asarray(dist,dtype=float).reshape(-1)#畸变系数拉平成一维数组
    if dist.size < 5:#少于 5 个就用 0 补齐（OpenCV 习惯写 5 个）
        dist = np.pad(dist,(0,5 - dist.size))#np.pad 在末尾补 0
    dist5 = dist[:5]#只取前 5 个：k1, k2, p1, p2, k3

    text = f"""# =============================================================================
# camera_intrinsics.yaml —— 相机内参与畸变系数
# 由 src/soccer_vision/calibrate.py 自动生成于 {time.strftime("%Y-%m-%d %H:%M:%S")}
# 标定板 {cols}x{rows} 内角点，方格 {square} m，有效照片 {n_ok} 张，RMS {rms:.3f} px
# -----------------------------------------------------------------------------

status: "done"
calib_date: "{time.strftime("%Y-%m-%d")}"
reprojection_error_px: {rms:.4f}

image_width: {width}
image_height: {height}

board:
  type: "chessboard"
  cols: {cols}
  rows: {rows}
  square_size_m: {square}

# K 矩阵，行优先
camera_matrix:
  - [ {K[0, 0]:.6f}, {K[0, 1]:.6f}, {K[0, 2]:.6f} ]
  - [ {K[1, 0]:.6f}, {K[1, 1]:.6f}, {K[1, 2]:.6f} ]
  - [ {K[2, 0]:.6f}, {K[2, 1]:.6f}, {K[2, 2]:.6f} ]

# OpenCV 顺序：[k1, k2, p1, p2, k3]
dist_coeffs: [ {", ".join(f"{v:.6f}" for v in dist5)} ]
"""#join 把每个数字格式化成 6 位小数，再用逗号连起来
    path.parent.mkdir(parents=True,exist_ok=True)#确保上级目录存在
    if path.exists():#已有文件先备份
        shutil.copyfile(path,str(path) + ".bak")
    path.write_text(text,encoding="utf-8")#写入文件


def main(argv=None) -> int:#主函数
    ap = argparse.ArgumentParser(description="棋盘格内参标定")
    ap.add_argument("--images",required=True,help="标定照片目录 / 通配符 / 单文件")
    ap.add_argument("--cols",type=int,default=9,help="内角点列数（横向格子数 - 1）")
    ap.add_argument("--rows",type=int,default=6,help="内角点行数（纵向格子数 - 1）")
    ap.add_argument("--square",type=float,default=0.025,help="方格边长（米）")
    ap.add_argument("--out",default=str(PROJECT_ROOT / "calibration/camera_intrinsics.yaml"))
    #action="store_true"：命令行写了 --preview 就是 True，不写就是 False
    ap.add_argument("--preview",action="store_true",help="逐张显示找到的角点（需要桌面）")
    args = ap.parse_args(argv)#读参数

    files = collect_images(args.images)#收集图片
    if not files:#一张都没有
        print(f"[error] 没找到图片：{args.images}")
        return 2#出错退出
    print(f"[calib] 找到 {len(files)} 张图，标定板内角点 {args.cols}x{args.rows}，方格 {args.square} m")

    pattern = (args.cols,args.rows)#内部角点数量，元组形式
    #准备棋盘格在真实世界里的坐标：9x6 的板子，每个格子 2.5 cm，Z 都为 0
    objp = np.zeros((args.cols * args.rows,3),np.float32)#先造全 0 的数组
    #mgrid 生成网格坐标；.T 转置、reshape(-1,2) 拉成 N 行 2 列；再乘方格边长换成米
    objp[:,:2] = np.mgrid[0 : args.cols,0 : args.rows].T.reshape(-1,2) * args.square

    obj_points,img_points,used,size,failed = [],[],[],None,[]#多个列表一次定义，分别装：真实点/像素点/用到的文件/分辨率/失败文件
    for i,path in enumerate(files,1):#enumerate 同时拿到序号和元素，序号从 1 开始
        img = cv2.imread(path)#读图；路径不对或格式不支持会返回 None
        if img is None:#读不到
            print(f"  [{i:02d}/{len(files)}] 读不到 {os.path.basename(path)}")
            failed.append(os.path.basename(path))#记进失败列表
            continue#跳过这一张，继续下一张
        gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)#转灰度：找角点不需要颜色
        ok,corners,how = find_corners(gray,pattern)#找角点
        if not ok:#这张找不到
            print(f"  [{i:02d}/{len(files)}] 找不到角点 {os.path.basename(path)}")
            failed.append(os.path.basename(path))
            continue
        if size is None:#第一张成功图：记下分辨率
            size = (gray.shape[1],gray.shape[0])#shape 是 (高, 宽)，所以 [1] 是宽、[0] 是高
        elif size != (gray.shape[1],gray.shape[0]):#后面的图和第一张分辨率不同
            print(f"  [{i:02d}/{len(files)}] 跳过（分辨率 {gray.shape[1]}x{gray.shape[0]} 与首张 {size[0]}x{size[1]} 不一致）")
            failed.append(os.path.basename(path))
            continue
        obj_points.append(objp)#这张图对应的真实世界点（都一样）
        img_points.append(corners)#这张图里检测到的像素点
        used.append(path)#记下用到了哪张
        print(f"  [{i:02d}/{len(files)}] OK ({how}) {os.path.basename(path)}")
        if args.preview:#逐张预览（需要桌面环境）
            try:
                cv2.drawChessboardCorners(img,pattern,corners,ok)#把角点画在图上
                cv2.imshow("corners",cv2.resize(img,None,fx=0.6,fy=0.6))#缩小到 60% 显示
                cv2.waitKey(300)#停 300 毫秒，让人看一眼
            except Exception:  # noqa: BLE001             # 没桌面会抛异常
                args.preview = False#关掉预览，别每张都报错

    if args.preview:#循环结束后关窗口
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001
            pass

    if len(used) < 6:#少于 6 张信息量不够，标出来不可信
        print(f"[error] 只有 {len(used)} 张可用，至少 6 张、建议 15 张以上")
        return 3

    print(f"[calib] 开始标定（{len(used)} 张，分辨率 {size[0]}x{size[1]}）…")
    #核心：OpenCV 一次性算出内参 K、畸变 dist、每张图的外参 rvecs/tvecs，并返回平均重投影误差 rms
    rms,K,dist,rvecs,tvecs = cv2.calibrateCamera(obj_points,img_points,size,None,None)
    print(f"[calib] 重投影误差 RMS = {rms:.4f} px")
    print(f"        fx={K[0, 0]:.2f}  fy={K[1, 1]:.2f}  cx={K[0, 2]:.2f}  cy={K[1, 2]:.2f}")
    print(f"        dist = {np.round(np.asarray(dist).reshape(-1)[:5], 4).tolist()}")

    #每张图的重投影误差，方便找出拍得最差的那几张
    worst = []#准备记录（误差, 文件名）
    for idx,path in enumerate(used):#遍历每张成功标定的图
        #projectPoints 用标出来的参数把真实点重新投到图像上，得到一个"预测像素位置"
        proj,_ = cv2.projectPoints(obj_points[idx],rvecs[idx],tvecs[idx],K,dist)
        #预测位置和实际检测位置的差，取每个点的欧氏距离（norm），再求平均
        err = float(np.mean(np.linalg.norm(proj.reshape(-1,2) - img_points[idx].reshape(-1,2),axis=1)))
        worst.append((err,os.path.basename(path)))
    worst.sort(reverse=True)#按误差从大到小排
    print("[calib] 误差最大的 3 张（可以考虑删掉重标）：")
    for err,name in worst[:3]:#切片 [:3] 取前三
        print(f"        {err:.3f} px  {name}")

    out = Path(args.out)#输出路径
    #把结果写成 YAML；size[0]/size[1] 是宽/高
    write_yaml(out,K,dist,size[0],size[1],float(rms),args.cols,args.rows,args.square,len(used))
    print(f"[calib] 已写入 {out}")
    if failed:#有失败的图就列出来
        print(f"[calib] 跳过了 {len(failed)} 张：{failed}")
    if rms > 2.0:#误差太大，给出改进建议
        print("[warn] RMS 偏大，建议重拍：板子拍满画面、四角各来几张、别一张比一张糊")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())#直接运行才执行 main
