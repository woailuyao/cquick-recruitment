
import collections
import os
import time

import cv2
import numpy as np
from ultralytics import YOLO

# ================= 配置区 =================
MODEL_PATH = r"C:\Users\wen\Desktop\best.pt"
CAMERA_INDEX = 0
IMGSZ = 640
CONF = 0.25
IOU = 0.45
WIN_W, WIN_H = 1280, 720
FPS_WINDOW = 30          # 实时帧率的统计窗口（帧）
# =========================================


def ensure_weights(path):
    """权重文件存在性检查：缺失时给出明确的获取指引后退出。"""
    if not os.path.isfile(path):
        raise SystemExit(
            "[error] 权重未就位：" + path + "\n"
            "请从 README 中的百度网盘链接（提取码 27vp）下载 best.pt，"
            "放到对应位置后再运行。")


class RealtimeFPS:
    """用最近 N 帧的到达时刻算实时帧率：(N-1)/(t_last - t_first)。
    相比逐帧 1/dt 更稳定，相比全程平均更能反映当前状态。"""

    def __init__(self, n=30):
        self.t = collections.deque(maxlen=n)

    def tick(self):
        self.t.append(time.perf_counter())

    def value(self):
        if len(self.t) < 2:
            return 0.0
        span = self.t[-1] - self.t[0]
        return (len(self.t) - 1) / span if span > 1e-9 else 0.0


ensure_weights(MODEL_PATH)
model = YOLO(MODEL_PATH)
print("类别:", model.names)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIN_W)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, WIN_H)
if not cap.isOpened():
    raise SystemExit(f"打不开摄像头 {CAMERA_INDEX}")
print(f"实际分辨率 {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
      f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

# 预热：相机 3 帧等曝光稳定，模型一次空推理排除首次初始化耗时
for _ in range(3):
    cap.read()
model.predict(np.zeros((WIN_H, WIN_W, 3), dtype=np.uint8),
              imgsz=IMGSZ, conf=CONF, iou=IOU, verbose=False)

cv2.namedWindow("ball realtime", cv2.WINDOW_NORMAL)
cv2.resizeWindow("ball realtime", WIN_W, WIN_H)

fps = RealtimeFPS(FPS_WINDOW)
frame_id = 0

while True:
    ok, frame = cap.read()
    if not ok:
        print("取帧失败")
        break
    frame_id += 1

    t0 = time.perf_counter()
    res = model.predict(frame, imgsz=IMGSZ, conf=CONF, iou=IOU, classes=[0], verbose=False)[0]
    infer_ms = (time.perf_counter() - t0) * 1000.0

    dets = []
    if res.boxes is not None and len(res.boxes) > 0:
        for box, cf in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.conf.cpu().numpy()):
            x1, y1, x2, y2 = (int(v) for v in box)
            dets.append((x1, y1, x2, y2, float(cf), (x1 + x2) / 2, (y1 + y2) / 2))

    for x1, y1, x2, y2, cf, cu, cv_ in dets:
        col = (80, 220, 80) if cf >= 0.7 else (60, 200, 250) if cf >= 0.4 else (70, 70, 230)
        cv2.rectangle(frame, (x1, y1), (x2, y2), col, 2)
        cv2.drawMarker(frame, (int(cu), int(cv_)), (0, 0, 255), cv2.MARKER_CROSS, 12, 2)
        cv2.putText(frame, f"ball {cf:.2f}", (x1, max(16, y1 - 28)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        cv2.putText(frame, f"{x1},{y1},{x2},{y2}", (x1, max(14, y1 - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.putText(frame, f"c=({cu:.0f},{cv_:.0f})", (x1, min(frame.shape[0] - 6, y2 + 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)

    hud = [f"frame #{frame_id}   {time.strftime('%H:%M:%S')}",
           f"FPS {fps.value():5.1f}   infer {infer_ms:5.2f} ms   dets {len(dets)}"]
    cv2.rectangle(frame, (0, 0), (430, 58), (20, 20, 20), -1)
    for i, s in enumerate(hud):
        cv2.putText(frame, s, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                    0.5, (120, 255, 120), 1, cv2.LINE_AA)

    cv2.imshow("ball realtime", frame)
    fps.tick()                        # 每帧结束打点，间隔含推理+绘制+显示
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

cap.release()
cv2.destroyAllWindows()
print(f"总帧数 {frame_id}")