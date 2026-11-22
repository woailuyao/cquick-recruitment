"""
棋盘格标定拍照工具
按 s 保存一张（只有检测到棋盘格才保存），按 q 退出
"""
import os
import cv2

CAM_INDEX = 1                          # 外接 USB 相机（内置摄像头是 0）
OUT_DIR = r"D:\W34_dataset\calib"      # 纯英文路径，避免 OpenCV 写中文路径静默失败
PATTERN = (7, 5)                       # 内角点数量：8×6 格棋盘格 → 7×5 个内角点

# 画面方向校正，取值含义与 detect_realtime.py 保持一致（同一台相机、同一方向）
FRAME_ROTATE = 0
FRAME_FLIP = 0


def correct_frame(frame):
    """按开关对原始帧做旋转/翻转校正，标定图与实时画面必须用同一方向。"""
    if FRAME_ROTATE == 180:
        frame = cv2.rotate(frame, cv2.ROTATE_180)
    elif FRAME_ROTATE == 90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
    elif FRAME_ROTATE == 270 or FRAME_ROTATE == -90:
        frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
    if FRAME_FLIP == 1:
        frame = cv2.flip(frame, 1)
    elif FRAME_FLIP == -1:
        frame = cv2.flip(frame, 0)
    return frame

os.makedirs(OUT_DIR, exist_ok=True)

cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_MSMF)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))

count = 0
found_now = False

while True:
    ok, frame = cap.read()
    if not ok:
        print("读帧失败")
        break
    frame = correct_frame(frame)   # 与实时脚本同一方向校正后再检测角点

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found_now, corners = cv2.findChessboardCorners(gray, PATTERN, None)

    disp = frame.copy()
    if found_now:
        cv2.drawChessboardCorners(disp, PATTERN, corners, found_now)
        cv2.putText(disp, "DETECTED  press s", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
    else:
        cv2.putText(disp, "NOT FOUND  adjust", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
    cv2.putText(disp, "saved: %d" % count, (10, 68), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)

    cv2.imshow("Calib Snapshot", disp)
    key = cv2.waitKey(1) & 0xFF

    if key == ord("s"):
        if not found_now:
            print("没检测到棋盘格，不保存")
            continue
        count += 1
        path = os.path.join(OUT_DIR, "calib_%02d.jpg" % count)
        cv2.imwrite(path, frame)
        print("saved:", path)
    elif key == ord("q") or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
print("共保存 %d 张到 %s" % (count, OUT_DIR))
