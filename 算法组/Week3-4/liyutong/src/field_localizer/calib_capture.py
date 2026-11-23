# -*- coding: utf-8 -*-
"""
棋盘格标定图采集脚本（任务 2 硬要求：内参标定 >= 15 张）
用法：conda activate calib && python calib_capture.py
准备：打印一张 9x6 内角点的棋盘格（OpenCV 官方棋盘），贴在硬板上
操作：对准棋盘格按 空格 拍一张，拍够 15 张以上按 q 退出
      角点检测成功的图存进 calib_imgs/，失败的会提示（说明没对准/太模糊）
"""
import cv2
import os
import datetime

# 棋盘格内角点数（每行、每列的"角点"数，不是格子数）。9x6 是最常见规格，
# 如果你打印的棋盘不一样，改这里
BOARD = (9, 6)

OUT_DIR = "calib_imgs"
os.makedirs(OUT_DIR, exist_ok=True)

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def imwrite_safe(path, img):
    """cv2.imwrite 遇到中文路径会静默失败，用 imencode 绕过"""
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok

def find_corners(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    ok, corners = cv2.findChessboardCorners(gray, BOARD, None)
    if ok:
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return ok, corners

def main():
    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("相机打不开，先跑 camera_check.py 确认设备编号，把这里的 0 改成对应编号")
        return
    # 显式固定分辨率：标定和后续采集必须用同一分辨率，否则内参不能通用
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    for _ in range(10):
        cap.read()
    real_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    real_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    count = 0
    print(f"实际采集分辨率：{real_w}x{real_h}（标定以此为准，后续采集不要改）")
    print(f"目标棋盘格：{BOARD[0]}x{BOARD[1]} 内角点 | 保存目录 {OUT_DIR}/")
    print("对准棋盘按【空格】拍，按 q 退出。凑够 15 张以上（不同角度/距离/位置）")
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败")
            break
        display = frame.copy()
        found, corners = find_corners(frame)
        if found:
            cv2.drawChessboardCorners(display, BOARD, corners, found)
            cv2.putText(display, "OK  press SPACE to capture", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(display, "no chessboard / blurry", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.putText(display, f"saved: {count}", (10, 65),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.imshow("calibration capture", display)
        key = cv2.waitKey(1) & 0xFF
        if key == ord(' '):
            if found:
                name = os.path.join(OUT_DIR, f"calib_{datetime.datetime.now():%Y%m%d_%H%M%S}.jpg")
                imwrite_safe(name, frame)
                count += 1
                print(f"已保存 {name}  （当前 {count} 张）")
            else:
                print("这张没检测到角点，换个角度或离近一点再拍")
        elif key == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    print(f"结束，共 {count} 张存在 {OUT_DIR}/。凑够 15 张后，下一步跑 calib_compute.py 算内参")

if __name__ == "__main__":
    main()
