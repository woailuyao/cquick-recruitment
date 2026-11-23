from __future__ import annotations
from pathlib import Path

import argparse
import math
import shutil
import sys
import time
import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[2]
if str(PROJECT_ROOT/"src") not in sys.path:#若找不到
    sys.path.insert(0,str(PROJECT_ROOT/"src"))
from soccer_vision.configio import load_intrinsics
from soccer_vision.geometry import build_extrinsics

def visible_ground(height_m:float,pitch_deg:float,fy:float,image_h:int,fx:float,image_w:int) -> dict:#算视线与地面的交线，返回几个关键距离值

    #atan求正切反函数，image_h/2/fy得到 半个画面高度/焦距=半视场的正切值
    half_v = math.degrees(math.atan(image_h/2.0/fy))#垂直半视场
    half_h = math.degrees(math.atan(image_w/2.0/fx))#水平半视场
    top_ray = pitch_deg-half_v#画面更接近水平
    bottom_ray = pitch_deg+half_v#画面更接近脚下

    def dist(ray_deg: float):#通过俯角算这条射线打到地面距离，即斜边
        if ray_deg<=0.2:#近乎水平
            return float("inf")#return inf无穷
        return height_m/math.tan(math.radians(ray_deg))#水平距离=高/tan(角度)

    width_at_3m = 2.0 * 3.0 * math.tan(math.radians(half_h))#3米画面能覆盖宽度
    
    return {#返回一个字典，打包所有算出来的数
        "half_vfov_deg": half_v,#垂直半视场
        "half_hfov_deg": half_h,#水平半视场
        "far_m": dist(top_ray),#画面最上方能看到的地面，离相机距离
        "near_m": dist(bottom_ray),#画面最下方能看到的地面，离相机距离
        "top_ray_deg": top_ray,#顶边射线的俯角
        "bottom_ray_deg": bottom_ray,#底边射线的俯角
        "width_3m": width_at_3m,#3米可见宽
    }



def main(argv=None) -> int:#main函数
    ap = argparse.ArgumentParser(description="生成 camera_extrinsics.yaml（高度 + 俯仰 + 滚转 + 偏航）")#required=True防止argparse自动报错并退出
    ap.add_argument("--height", type=float, required=True, help="相机光心离地高度（米），卷尺量")
    ap.add_argument("--pitch", type=float, required=True, help="俯角（度），向下为正；水平时 0")
    ap.add_argument("--roll", type=float, default=0.0, help="滚转角（度），默认 0")
    ap.add_argument("--yaw", type=float, default=0.0, help="相机相对载体的水平偏角（度），默认 0")
    ap.add_argument("--intrinsics", default=str(PROJECT_ROOT / "calibration/camera_intrinsics.yaml"))
    ap.add_argument("--out", default=str(PROJECT_ROOT / "calibration/camera_extrinsics.yaml"))
    ap.add_argument("--method", default="tape+level", help="记录到 yaml 的测量方式，写给人看的")
    args = ap.parse_args(argv)#读取命令行参数

    if args.height<=0.02:#高度太小说明量得有问题
        print("[error] --height 必须大于 0（相机离地高度，单位米）")
        return 2 #非零标识报错

    R,t = build_extrinsics(args.height,args.pitch,args.roll,args.yaw)#生成旋转矩阵R和平移向量t
    intr = load_intrinsics(args.intrinsics)#读内参文件
    calibrated = intr.is_calibrated#是否已经标定过
    fy = float(intr.K[1,1]) if calibrated else 900.0#K[1,1]焦距fy，未标定用900
    fx = float(intr.K[0,0]) if calibrated else 900.0#K[0,0]焦距fx
    w,h = intr.width,intr.height#分辨率

    print(f"[extr] 高度 {args.height:.3f} m  俯角 {args.pitch:.1f}°  滚转 {args.roll:.1f}°  偏航 {args.yaw:.1f}°")
    print(f"[extr] R =\n{np.round(R, 4)}")#np.round保留4位小数打印矩阵
    print(f"[extr] t = {np.round(t, 4).tolist()}")#利用tolist()把numpy数组转成普通列表
    print("[extr] 解读：R 的第 1/2/3 列分别是相机 +X（画面右）、+Y（画面下）、+Z（光轴）"
          "在载体坐标系里的方向，第 3 列就是光轴指向")
    print(f"        光轴 -> {np.round(R[:, 2], 4).tolist()}"
          f"（前 {R[0, 2]:+.3f}，左 {R[1, 2]:+.3f}，上 {R[2, 2]:+.3f}）")#:+.3f表示正数

    g = visible_ground(args.height, args.pitch, fy, h, fx, w)#算视野覆盖
    far_txt = "无穷远（已到地平线，容易误检远处标志）" if math.isinf(g["far_m"]) else f"{g['far_m']:.2f} m"#无穷大标明
    print(f"[extr] 垂直半视场 {g['half_vfov_deg']:.1f}°  水平半视场 {g['half_hfov_deg']:.1f}°"
          f"（fy={'标定值' if calibrated else '估计值 900'}）")
    print(f"[extr] 画面底部约 {g['near_m']:.2f} m 处，画面顶部约 {far_txt} 处")
    print(f"[extr] 3 m 处左右可见宽度约 {g['width_3m']:.2f} m")

    if math.isinf(g["far_m"]) or g["far_m"]>12.0:#看太远，远处标志小误差大
        print("[warn] 视野太远：远处标志像素太小、地面投影误差大。建议俯角再大 2~4°。")

    if g["near_m"] > 2.5:#最近处都超过2.5米，脚边看不见
        print("[warn] 近处看不到脚下地面，建议俯角再小一点，或者相机装低一些。")



#----------------------------------------------------------------------------
    text = f"""# =============================================================================
# camera_extrinsics.yaml —— 相机 -> 载体的外参 p_base = R @ p_cam + t
# 由 src/soccer_vision/extrinsics.py 生成于 {time.strftime("%Y-%m-%d %H:%M:%S")}
# 测量方式：{args.method}
# -----------------------------------------------------------------------------

status: "done"
method: "{args.method}"
reference_frame: "base_link"

# 相机光心离地高度（米）与俯角（度，向下为正），留着方便复现
camera_height_m: {args.height:.4f}
pitch_deg: {args.pitch:.2f}
roll_deg: {args.roll:.2f}
yaw_deg: {args.yaw:.2f}

# R：先按「相机朝前水平」建立初始姿态，再绕载体 X/Y/Z 轴依次滚转、俯仰、偏航
# 读法：R 的第 1/2/3 列 = 相机 +X（画面右）、+Y（画面下）、+Z（光轴）在载体的方向
#       第 3 列即光轴指向，俯角 18° 时约为 [0.951, 0, -0.309]
R:
  - [ {R[0, 0]:.6f}, {R[0, 1]:.6f}, {R[0, 2]:.6f} ]
  - [ {R[1, 0]:.6f}, {R[1, 1]:.6f}, {R[1, 2]:.6f} ]
  - [ {R[2, 0]:.6f}, {R[2, 1]:.6f}, {R[2, 2]:.6f} ]

# t：相机光心在载体坐标系中的位置（米）。只关心离地高度，水平偏移按需改。
t: [ {t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f} ]
"""
#----------------------------------------------------------------------------


    out = Path(args.out)#输出路径
    out.parent.mkdir(parents=True, exist_ok=True)#确保上级目录存在

    if out.exists():#备份
        shutil.copyfile(out, str(out) + ".bak")#复制成xxx.yaml.bak

    out.write_text(text, encoding="utf-8")#把文本用UTF-8编码写进文件

    print(f"[extr] 已写入 {out}")
    print("[extr] 下次跑 run_demo.py 就会用这份外参；改完记得再跑一次 --check 确认没写坏")
    return 0#成功return0



if __name__ == "__main__":#只有直接运行本文件时才执行main()，import不执行
    raise SystemExit(main())#用main的返回值当退出码
