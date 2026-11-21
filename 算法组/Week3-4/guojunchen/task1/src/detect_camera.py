import os
import time
from pathlib import Path

import cv2
from ultralytics import YOLO

# ===== 参数（改这里就行） =====
HERE = Path(__file__).resolve().parent
TASK1 = HERE.parent

weights = str(TASK1 / 'weights' / 'best.pt')
source = 1              # 1 = USB 单目相机；也可以填视频文件路径
imgsz = 640
conf = 0.3
min_area = 300          # 小于这个面积的框当作噪点丢掉
show_window = True
save_video = False
out_path = str(TASK1 / '01_detect_output.mp4')
max_frames = 0          # 0 = 一直跑

print(f'模型: {weights}')
if not Path(weights).exists():
    print('找不到权重文件:', weights)
    print('本仓库不附带 .pt 文件。请从网盘下载后放到该路径：')
    print('  网盘: https://pan.baidu.com/s/1UhbZ_PyChPVQwycBPv7Qaw?pwd=5408  提取码 5408')
    print('  task1_best.pt -> task1/weights/best.pt')
    raise SystemExit(1)
model = YOLO(weights)

print(f'相机: {source}')
cap, frame = None, None

if isinstance(source, str) and not source.isdigit():
    cap = cv2.VideoCapture(source)
    ok, img = cap.read()
    frame = img if ok else None
else:
    idx = int(source)
    # USB 相机：MSMF 和默认后端都试一下，选能出正常画面的那个
    # （有的机器 MSMF 会给出几乎全黑的帧，但照样返回 True）
    for backend in (cv2.CAP_MSMF, cv2.CAP_ANY):
        c = cv2.VideoCapture(idx, backend)
        if not c.isOpened():
            c.release()
            continue
        c.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        c.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        c.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        for _ in range(60):
            ok, img = c.read()
            if ok and img is not None and float(img.mean()) > 20:
                cap, frame = c, img
                break
        if cap is not None:
            break
        c.release()

if frame is None:
    print(f'相机 {source} 打开了但没有画面：')
    print('  1) 被别的程序占用（录屏软件 / 相机 App / 之前的 python 没退）')
    print('  2) 镜头被挡住，或者环境太暗')
    print('  3) USB 没插稳，换个口试试')
    if os.name != 'nt':
        import glob
        devs = sorted(glob.glob('/dev/video*'))
        if not devs:
            print('  4) 【这台机器上根本没有摄像头设备】/dev/video* 一个都没有。')
            print('     WSL 默认看不到 Windows 的 USB 相机。两种办法：')
            print('       a) 把上面 source 改成视频路径，例如 source = "/mnt/d/xxx.mp4"')
            print('       b) 在 Windows 用 usbipd 把相机接入 WSL')
        else:
            print(f'  4) 找到了这些设备 {devs}，但编号 {source} 打不开，'
                  f'用 v4l2-ctl --list-devices 看实际编号')
    raise SystemExit(1)

# 第一次推理慢（要初始化 CUDA），先跑一次不算进 FPS
model.predict(frame, imgsz=imgsz, conf=conf, verbose=False)

writer = None
if save_video:
    h, w = frame.shape[:2]
    writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*'mp4v'), 30.0, (w, h))
    if not writer.isOpened():
        print(f'写不了视频（路径带中文或没有写权限）: {out_path}')
        writer = None

fps = 0.0
count = 0
last = time.time()

while True:
    ok, img = cap.read()
    if not ok:
        print('读不到画面，退出')
        break

    t0 = time.time()
    results = model.predict(img, imgsz=imgsz, conf=conf, verbose=False)[0]

    # 面积筛选
    valid = []
    for box in results.boxes:
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        area = (x2 - x1) * (y2 - y1)
        if area > min_area:
            valid.append((area, x1, y1, x2, y2, float(box.conf[0])))

    # 面积从大到小排序
    valid = sorted(valid, key=lambda x: x[0], reverse=True)

    for i, (area, x1, y1, x2, y2, c) in enumerate(valid):
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 0, 255), 2)
        cv2.circle(img, (u, v), 4, (0, 255, 0), -1)
        cv2.putText(img, f'ball {c:.2f} ({u},{v})', (int(x1), max(16, int(y1) - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        print(f'  目标{i + 1}: area={int(area)} conf={c:.2f} center=({u},{v})')

    now = time.time()
    fps = 0.9 * fps + 0.1 * (1.0 / max(now - last, 1e-6))
    last = now

    cv2.putText(img, f'FPS {fps:.1f}  balls {len(valid)}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)

    if writer is not None:
        writer.write(img)

    if show_window:
        cv2.imshow('ball detection', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s'):
            shot = Path(out_path).with_name(f'capture_{int(time.time())}.jpg')
            cv2.imencode('.jpg', img)[1].tofile(str(shot))
            print(f'已保存 {shot}')

    count += 1
    if max_frames and count >= max_frames:
        break

cap.release()
if writer is not None:
    writer.release()
cv2.destroyAllWindows()
print(f'结束，共处理 {count} 帧')
