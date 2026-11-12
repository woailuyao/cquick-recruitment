import cv2
import numpy as np
import sys

cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("摄像头未打开")
    sys.exit()

# 先学习前面40帧背景，后面再检测
bs = cv2.createBackgroundSubtractorMOG2(history=50, varThreshold=16, detectShadows=True)
kx = np.ones((5, 5), np.uint8)
calib_frames = 40
lower, upper = None, None
n = 0

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        n += 1
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if n <= calib_frames:
            bs.apply(frame)
            cv2.putText(frame, f"learning background... {n}/{calib_frames}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow('frame', frame)
            # 按q退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            continue

        if lower is None:
            fg = bs.apply(frame)
            # 去噪
            fg = cv2.medianBlur(fg, 5)
            fg = cv2.morphologyEx(fg, cv2.MORPH_OPEN, kx)
            # 检测有没有新的物体进来（前景掩膜只有 0 和 255 两种值）
            if (fg == 255).sum() < 500:
                cv2.putText(frame, "waiting for target...", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.imshow("frame", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
                continue

            # 自动标定
            sel = hsv[fg == 255].astype(np.int32)
            lo, up = sel.min(0), sel.max(0)
            # 低饱和 -> H 无意义，放开
            low_sat = lo[1] < 70
            lower = np.array([0 if low_sat else max(0, lo[0] - 8), max(0, lo[1] - 40), max(0, lo[2] - 50)], np.uint8)
            upper = np.array([179 if low_sat else min(179, up[0] + 8), 255, 255], np.uint8)
            print(f"[自动标定] 实测 H {lo[0]}-{up[0]}  S {lo[1]}-{up[1]}  V {lo[2]}-{up[2]}")
            print(f"          生成 lower={list(lower)} upper={list(upper)}")
            continue

        mask = cv2.inRange(hsv, lower, upper)
        mask = cv2.morphologyEx(cv2.morphologyEx(mask, cv2.MORPH_OPEN, kx),
                                cv2.MORPH_CLOSE, kx)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 没有轮廓时给个提示，避免以为程序卡死
        if not contours:
            cv2.putText(frame, "no target", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # inRange 生成掩膜, 绘制外接矩形和质点
        for c in contours:
            if cv2.contourArea(c) < 0.005 * frame.shape[0] * frame.shape[1]:
                continue
            x, y, w, h = cv2.boundingRect(c)
            area = cv2.contourArea(c)
            M = cv2.moments(c)
            if M["m00"] == 0:
                continue
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 6, (0, 0, 255), -1)
            cv2.circle(frame, (cx, cy), 2, (255, 255, 255), -1)
            cv2.putText(frame, f"({cx}, {cy}), Area={int(area)}", (x + 5, y + 22 if y < 40 else y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3)
            cv2.putText(frame, f"({cx}, {cy}), Area={int(area)}", (x + 5, y + 22 if y < 40 else y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)

        cv2.imshow("frame", frame)
        cv2.imshow("mask", mask)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('r'):
            lower, upper = None, None

finally:
    cap.release()
    cv2.destroyAllWindows()


