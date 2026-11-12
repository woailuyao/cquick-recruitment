import cv2
import numpy as np

# 1. 打开电脑默认摄像头（0 代表第一个摄像头）
cap = cv2.VideoCapture(0)

# 检查摄像头是否成功打开
if not cap.isOpened():
    print("❌ 无法打开摄像头！请检查是否有摄像头，或者是否被其他软件占用。")
    exit()

print("🎥 摄像头已开启！请拿一个绿色的物体在镜头前晃动。")
print("按键盘上的 'q' 键或 'Esc' 键退出。")

# 2. 循环读取摄像头的每一帧画面
while True:
    # ret 是读取是否成功的状态，frame 是当前的一帧画面
    ret, frame = cap.read()
    if not ret:
        print("❌ 无法获取画面。")
        break

    # 将画面从 BGR 转换为 HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # 设定绿色的 HSV 范围
    lower_green = np.array([35, 43, 46])
    upper_green = np.array([85, 255, 255])

    # 生成掩膜
    mask = cv2.inRange(hsv, lower_green, upper_green)

    # 形态学开运算去噪
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    # 查找轮廓
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # 如果找到了轮廓
    if len(contours) > 0:
        # 找面积最大的轮廓
        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # 过滤掉噪点（大于 500 才画框）
        if area > 500:
            # 画外接矩形
            x, y, w, h = cv2.boundingRect(largest_contour)
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 0, 255), 3)

            # 算中心点
            center_x = x + w // 2
            center_y = y + h // 2
            cv2.circle(frame, (center_x, center_y), 5, (0, 255, 0), -1)

            # 在画面上直接打印坐标
            cv2.putText(frame, f"({center_x}, {center_y})", (x, y - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # 3. 显示画面
    cv2.imshow('Color Tracker (Press q to quit)', frame)
    # 也可以同时显示 Mask 看看效果
    # cv2.imshow('Mask', mask) 

    # 4. 等待 1 毫秒，并检测是否按下了 'q' 键或 Esc 键
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q') or key == 27:  # 27 是 Esc 键的 ASCII 码
        break

# 5. 释放摄像头并关闭所有窗口
cap.release()
cv2.destroyAllWindows()
print("🎥 摄像头已关闭。")