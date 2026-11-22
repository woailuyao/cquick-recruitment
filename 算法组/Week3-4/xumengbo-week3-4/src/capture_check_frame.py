"""
外参自检 · 拍一张纯相机帧存盘（capture_check_frame）
=====================================================
离线自检第一步：在场地摆好已知标记后，用本工具拍一张纯相机帧，
存到 D:\\loc_demo_snapshots\\，供 pick_pixel_from_image.py 点取像素。

运行：python capture_check_frame.py
按键：s 保存当前帧；q/Esc 退出。
"""
import os
import cv2

SOURCE = 1                       # 外接 USB 相机；内置摄像头试 0
CAM_BACKEND = cv2.CAP_MSMF
IMG_W, IMG_H = 1280, 720
FRAME_ROTATE = 0                # 0=不旋转；180=上下颠倒转 180°
FRAME_FLIP = 0
OUT_DIR = r"D:\loc_demo_snapshots"


def correct_frame(frame):
    """与 field_locator.py 同一套方向校正。"""
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


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    cap = cv2.VideoCapture(SOURCE, CAM_BACKEND)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, IMG_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, IMG_H)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if not cap.isOpened():
        raise RuntimeError("cannot open camera; try SOURCE=0 (built-in).")

    count = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        frame = correct_frame(frame)
        disp = frame.copy()
        cv2.putText(disp, "s: save  q: quit  saved: %d" % count,
                    (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.imshow("Check Frame Capture", disp)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q") or key == 27:
            break
        if key == ord("s"):
            count += 1
            path = os.path.join(OUT_DIR, f"check_{count:03d}.jpg")
            cv2.imwrite(path, frame)
            print("saved:", path)

    cap.release()
    cv2.destroyAllWindows()
    print("共保存 %d 张到 %s" % (count, OUT_DIR))


if __name__ == "__main__":
    main()
