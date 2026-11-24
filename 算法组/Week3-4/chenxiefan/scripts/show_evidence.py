#!/usr/bin/env python3

import hashlib
import os
import subprocess
import sys
import urllib.request

import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "scripts"))
import locate_field as lf

def sha16(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()[:16]

def mb(path):
    return os.path.getsize(path) / 1e6 if os.path.exists(path) else 0.0

def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "Chenxiefan"
    now = subprocess.run(["date", "+%Y-%m-%d %H:%M:%S"], capture_output=True, text=True).stdout.strip()
    print("=" * 66)
    print("录屏信息    姓名: %s    时间: %s" % (name, now))
    print("=" * 66)

    try:
        r = urllib.request.urlopen("http://192.168.56.1:8080/video", timeout=4)
        head = r.read(1024)
        status = "HTTP %s，%s，首包 %d 字节" % (
            getattr(r, "status", "?"), (r.headers.get("Content-Type") or "").split(";")[0], len(head))
        r.close()
    except Exception as exc:
        status = "连接失败：%s" % exc
    print("相机: USB 相机 0bda:5844 接在主机上，经 MJPEG 桥转成 1280x720 实时流，固定焦距、固定对焦")
    print("      http://192.168.56.1:8080/video，当前 %s" % status)

    print("任务1 足球检测")
    print("  数据: 6 段实验室实拍视频共 285 秒；训练 646 张（有球 402、负样本 244），验证 62 张（有球 39、负样本 23），按视频划分")
    print("  模型: YOLOv8n，输入 640，训练 50 轮，权重 models/ball_best.pt，%.2f MB" % mb(os.path.join(ROOT, "models/ball_best.pt")))
    print("  指标: Precision 1.000，Recall 0.967，mAP50 0.994，mAP50-95 0.940")
    print("  漏检: 视频 03:25，球被纸板遮住一半后检测框消失；可见面积太小，模型拿不到足够特征")

    print("任务2 场地定位")
    print("  数据: 15 段视频抽帧 847 张；训练 557 张（512 张有标志、45 张空场），验证 54 张，按视频划分；标注框 LCross 667、TCross 527、XCross 205")
    print("  模型: YOLOv8n，输入 960，权重 models/marker_best.pt，%.2f MB" % mb(os.path.join(ROOT, "models/marker_best.pt")))
    print("  指标: LCross P 0.816 R 0.958 mAP50 0.965；TCross P 0.765 R 0.727 mAP50 0.779；XCross P 0.873 R 0.745 mAP50 0.899")
    print("        三类平均 P 0.818，R 0.810，mAP50 0.881，mAP50-95 0.390")

    field = os.path.join(ROOT, "config/field.yaml")
    geom = os.path.join(ROOT, "config/field_geometry.yaml")
    print("  地图: field.yaml %s，field_geometry.yaml %s，版本 C0-2026-11-21" % (sha16(field), sha16(geom)))
    print("        原点在中心圈圆心，+X 指向球门，+Y 是面向 +X 时的左侧，Z=0 为地面，yaw 从 +X 逆时针为正，半场 3.968 x 5.954 米，共 15 个标志")

    cam = lf.load_yaml(os.path.join(ROOT, "config/camera.yaml")) or {}
    extr = lf.load_yaml(os.path.join(ROOT, "config/camera_extrinsics.yaml")) or {}
    K = np.array(cam["camera_matrix"], float)
    dist = np.array(cam["dist_coeffs"], float).reshape(-1)
    R = np.array(extr["R_base_camera"], float)
    t = np.array(extr["t_base_camera"], float)
    print("  标定: 1280x720，fx=fy=1574.7278，cx=639.5，cy=359.5")
    print("        畸变 k1=%.6f，k2=%.6f，p1=%.6f，p2=%.6f，k3=%.6f" % tuple(dist[:5]))
    print("        安装高度 0.663 米，俯角 12.5 度，滚转 0 度，yaw 0 度，平移 [0.000, 0.000, 0.663]")

    print("  投影: pixel(640,500) -> (%.3f, %.3f) 米；pixel(900,450) -> (%.3f, %.3f) 米" % (
        lf.pixel_to_ground(640, 500, K, dist, R, t) + lf.pixel_to_ground(900, 450, K, dist, R, t)))

    print("  方法: 用 YOLO 检测 L/T/X，在框内用霍夫直线求交点，去畸变后与 Z=0 地面求交，得到标志在相机坐标系的地面坐标；")
    print("        再和 field.yaml 中的同类点做多组候选匹配，用最小二乘求位姿。两个候选残差接近但位姿差超过 0.3 米或 8 度时判为 ambiguous；")
    print("        标志少于 2 个时 insufficient_markers；解到场外时 out_of_field")
    print("  输出: 相机光心在地图坐标系下的 x_camera_map、y_camera_map、yaw_camera_map，以及 marker_count、residual、valid、status")

    print("  测试位姿:")
    print("    P1 video211 左侧 XCross (0.000, 0.720)，朝左上角 LCross，yaw 30.0 度，位置误差 0.19 米，航向误差 2.5 度")
    print("    P2 video213 场地中心 (0.000, 0.000)，朝左上角 LCross，yaw 36.9 度，位置误差 0.09 米，航向误差 1.3 度")
    print("    P3 video221 右侧 XCross (0.000, -0.720)，朝球门线右端 TCross，yaw 342.75 度，位置误差 0.14 米，航向误差 1.4 度")

    print("ROS2: 节点 /field_locator，话题 /localization/field_pose，字段 x、y、yaw_deg、valid、marker_count、residual、status")
    print("性能: 任务1 约 6.4 fps，单帧推理约 140 ms；任务2 约 8 fps，约 120 ms")
    print("说明: 画面里少于 4 个标志时三点解存在对称歧义，可能给不出唯一位姿；远处较小的标志偶尔漏检")
    print("=" * 66)

if __name__ == "__main__":
    main()
