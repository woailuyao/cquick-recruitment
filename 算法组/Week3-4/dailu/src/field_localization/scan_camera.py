# -*- coding: utf-8 -*-
"""扫描 0~4 号摄像头，打印能打开的设备编号和分辨率。"""
import cv2

print("正在扫描摄像头（0-4）...\n")
found = []
for i in range(5):
    cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap.release()
        continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    ok, frame = cap.read()
    if ok and frame is not None:
        h, w = frame.shape[:2]
        print(f"  [OK]   --source {i}   实际画面 {w}x{h}")
        found.append(i)
    else:
        print(f"  [打开但读不到帧] --source {i}")
    cap.release()

print()
if found:
    print("可用编号:", found)
    print(f"把 bat 里的 --source 数值改成 {found[0]} 即可")
else:
    print("没有找到可用摄像头：")
    print("  1) 检查 USB 是否插好、设备管理器里有没有图像设备")
    print("  2) 关掉可能占用相机的程序（QQ / 微信 / OBS / 其他调试窗口）")
    print("  3) 换一个 USB 口再试")
