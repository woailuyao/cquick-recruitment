import os
import sys
import time

import cv2 as cv
from ultralytics import YOLO

# 1. 加载训练好的模型（与脚本同目录，跨平台都用绝对路径）
HERE = os.path.dirname(os.path.abspath(__file__))
model = YOLO(os.path.join(HERE, 'best.pt'))

# 2. 打开 USB 摄像头
#    - Windows 默认 2 号设备（DirectShow）
#    - Linux / WSL 默认 0 号设备（V4L2），可用环境变量 CAM 覆盖
DEFAULT_IDX = 2 if sys.platform.startswith('win') else 0
CAM_IDX = int(os.environ.get('CAM', DEFAULT_IDX))

if sys.platform.startswith('win'):
    cap = cv.VideoCapture(CAM_IDX, cv.CAP_DSHOW)
else:
    cap = cv.VideoCapture(CAM_IDX, cv.CAP_V4L2)
    if cap.isOpened():
        # UVC 相机默认常是 YUYV，OpenCV 解码错会整屏发绿；强制 MJPG 最稳
        cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*'MJPG'))
        # WSL 走 usbipd 传输，带宽有限：可用环境变量降分辨率/帧率避免撕裂
        #   例：CAM_W=640 CAM_H=480 CAM_FPS=15 python3 live_detect.py
        _w = int(os.environ.get('CAM_W', 1280))
        _h = int(os.environ.get('CAM_H', 720))
        _fps = float(os.environ.get('CAM_FPS', 0))
        cap.set(cv.CAP_PROP_FRAME_WIDTH, _w)
        cap.set(cv.CAP_PROP_FRAME_HEIGHT, _h)
        if _fps > 0:
            cap.set(cv.CAP_PROP_FPS, _fps)
        cap.set(cv.CAP_PROP_BUFFERSIZE, 1)
        print(f"[INFO] 请求分辨率 {_w}x{_h}" + (f" @ {_fps:g}fps" if _fps else ""))

# 记录上一帧时间，用来算 FPS
prev_time = time.time()

if not cap.isOpened():
    print(f" 无法打开摄像头（device index = {CAM_IDX}）")
    print(" Windows: 确认相机已插好；Linux/WSL: sudo chmod 666 /dev/video0 后再试")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # 3. 让模型实时预测当前画面
        results = model(frame, conf=0.85, verbose=False)

        # 4. 获取画好框、带置信度的画面
        annotated_frame = results[0].plot()

        # 5. 计算 FPS
        curr_time = time.time()
        fps = 1 / max(1e-6, curr_time - prev_time)
        prev_time = curr_time

        # 6. 打上 FPS 和 像素中心点
        cv.putText(annotated_frame, f"FPS: {int(fps)}", (20, 40),
                   cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # 如果检测到了球，显示它的像素中心
        if len(results[0].boxes) > 0:
            box = results[0].boxes[0].xyxy[0].cpu().numpy()
            u = int((box[0] + box[2]) / 2)
            v = int((box[1] + box[3]) / 2)
            cv.putText(annotated_frame, f"Center: u={u}, v={v}", (20, 80),
                       cv.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # 7. 弹出窗口显示画面
        cv.imshow("Live Detect - Press Q to quit", annotated_frame)

        # 按 q 键退出
        if cv.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv.destroyAllWindows()
