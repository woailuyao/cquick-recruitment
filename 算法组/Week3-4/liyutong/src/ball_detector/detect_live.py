# -*- coding: utf-8 -*-
"""
任务 1 实时检测：相机画面 + 足球检测框 + 置信度 + 像素中心 + FPS
用法：conda activate calib && python detect_live.py
录屏时这个画面就是验收画面（左右分屏是任务 2 的界面，先不用管）
"""
import cv2
import time
from ultralytics import YOLO

CAM_INDEX = 0        # 跑过 camera_check.py 后改成实际编号
CAM_W, CAM_H = 640, 480   # 必须与标定时的分辨率一致
CONF = 0.4           # 置信度阈值，误检多就调高

def main():
    model = YOLO("runs_ball/baseline/weights/best.pt")
    cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print(f"相机 {CAM_INDEX} 打不开，先跑 camera_check.py")
        return
    print("按 q 退出。这一路画面保持连续运行，别中断——录屏要的就是它")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

    t0, frames, fps = time.time(), 0, 0.0
    while True:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败")
            break
        frames += 1
        if time.time() - t0 > 0.5:
            fps = frames / (time.time() - t0)
            t0, frames = time.time(), 0

        results = model.predict(frame, conf=CONF, verbose=False)[0]
        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            conf = float(box.conf[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
            cv2.putText(frame, f"ball {conf:.2f} ({cx},{cy})", (x1, y1 - 8),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(frame, f"FPS {fps:.1f}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.imshow("task1 live detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
