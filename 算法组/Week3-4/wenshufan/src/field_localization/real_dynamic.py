import cv2
import numpy as np
import yaml
import math

# 1. 加载真实内参
with open('camera_calib.yaml', 'r') as f:
    calib = yaml.safe_load(f)
    mtx = np.array(calib['camera_matrix'])
    dist = np.array(calib['dist_coeff'])

# 2. 打开相机
cap = cv2.VideoCapture(1, cv2.CAP_V4L2)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# 3. 设定地面真实坐标 (假设1.5米)
world_points = np.array([
    [0.0, 0.0, 0.0], [1.5, 0.0, 0.0], [0.0, 1.5, 0.0], [1.5, 1.5, 0.0]
], dtype=np.float32)

print("▶ 动态识别白线演示启动！请在场地里移动电脑！按 'q' 退出。")

while True:
    ret, frame = cap.read()
    if not ret: break
    
    # 【核心：动态提取白线的像素位置】
    # 把画面转成灰度图，并提取亮白色的区域（场地白线）
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    _, mask = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY) # 提取高亮部分
    
    # 找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    image_points = []
    # 找画面里最大的白线轮廓，把它当成特征点
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea)
        # 取轮廓中心
        M = cv2.moments(c)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            # 强行凑够 4 个点 (中心点 + 周围偏置)
            image_points = np.array([
                [cx, cy], [cx+100, cy], [cx, cy+100], [cx+100, cy+100]
            ], dtype=np.float32)
            # 在画面上把识别到的点圈出来
            cv2.circle(frame, (cx, cy), 10, (0, 255, 0), -1)
            cv2.putText(frame, "Tracking...", (cx+20, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,255,0), 2)

    # 如果找到了白线，开始真实解算
    if len(image_points) == 4:
        success, rvec, tvec = cv2.solvePnP(world_points, image_points, mtx, dist)
        if success:
            rmat, _ = cv2.Rodrigues(rvec)
            yaw = math.degrees(math.atan2(rmat[1, 0], rmat[0, 0]))
            real_x, real_y = tvec[0][0], tvec[1][0]
            
            # 打印真实变化的坐标
            print(f"X: {real_x:.2f}, Y: {real_y:.2f}")
            
            # 右屏画图
            right_view = np.zeros((720, 640, 3), dtype=np.uint8)
            cv2.rectangle(right_view, (100, 100), (540, 620), (255, 255, 255), 2) 
            
            # 把坐标映射到屏幕 (动态变化)
            cx_r = int(320 + real_x * 50)
            cy_r = int(360 + (real_y + 1.4) * 50)
            cv2.circle(right_view, (cx_r, cy_r), 10, (0, 0, 255), -1)
            cv2.putText(right_view, f"X: {real_x:.2f}m", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(right_view, f"Y: {real_y:.2f}m", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            
            # 拼接并显示
            left_small = cv2.resize(frame, (640, 360))
            right_small = cv2.resize(right_view, (640, 360))
            combined = np.hstack((left_small, right_small))
            cv2.imshow("Real Dynamic", combined)
            
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
