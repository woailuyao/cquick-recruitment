#!/usr/bin/env python3
"""摄像头格式测试：强制 MJPG 编码，逐帧测试读取。"""

import cv2

cap = cv2.VideoCapture("/dev/video0")
if not cap.isOpened():
    print("无法打开 /dev/video0")
    exit(1)

# 强制 MJPG 格式
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
print("fourcc 已设置为 MJPG")

for i in range(10):
    ok, frame = cap.read()
    print(f"第 {i} 帧: {'读取成功' if ok else '读取失败'}")
    if ok:
        print(f"  帧尺寸: {frame.shape}")
        break

cap.release()
print("测试结束")
