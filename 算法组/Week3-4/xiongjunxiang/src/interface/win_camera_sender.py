#!/usr/bin/env python3
"""【在 Windows 上运行】相机采集 + JPEG 压缩 + TCP 推送到 WSL。

用法（Windows PowerShell）
    pip install opencv-python
    python win_camera_sender.py                     # 默认相机 0，1280x720@30
    python win_camera_sender.py --camera 1          # 指定相机序号
    python win_camera_sender.py --width 1280 --height 720 --fps 30 --quality 80
    python win_camera_sender.py --list              # 列出可用相机序号

WSL 端接收：
    python tools/camera_preview.py --camera tcp:5599
    python tools/demo_realtime.py --camera tcp:5599 --anchor-name hou_378

协议：每条消息 = 4 字节大端长度 + JPEG 数据。
"""

import argparse
import socket
import struct
import sys
import time

import cv2
import numpy as np


def list_cameras(max_idx=6):
    print("探测相机序号（每个试 0.3 秒）...")
    ok_list = []
    for i in range(max_idx):
        for backend, name in ((cv2.CAP_DSHOW, "DSHOW"), (cv2.CAP_MSMF, "MSMF"),
                              (0, "AUTO")):
            cap = cv2.VideoCapture(i, backend) if backend else cv2.VideoCapture(i)
            if cap.isOpened():
                ok, fr = cap.read()
                if ok and fr is not None:
                    print("  [%d] %-5s  %dx%d" % (i, name, fr.shape[1], fr.shape[0]))
                    ok_list.append(i)
                cap.release()
                break
            cap.release()
    if not ok_list:
        print("  没找到可用相机")



def identify(max_idx=6, seconds=2.5):
    """逐个打开相机并在窗口里标出序号，方便认出哪个是哪个。"""
    print("逐个预览，每个 %g 秒；按 q 跳过当前，按 Esc 退出" % seconds)
    for i in range(max_idx):
        cap = None
        for backend in (cv2.CAP_DSHOW, cv2.CAP_MSMF, 0):
            c = cv2.VideoCapture(i, backend) if backend else cv2.VideoCapture(i)
            if c.isOpened():
                ok, fr = c.read()
                if ok and fr is not None:
                    cap = c
                    break
            c.release()
        if cap is None:
            continue
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < seconds:
            ok, fr = cap.read()
            if not ok:
                break
            vis = fr.copy()
            cv2.putText(vis, "camera index = %d" % i, (12, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 255, 0), 3)
            cv2.putText(vis, "%dx%d  -- press q to skip" % (fr.shape[1], fr.shape[0]),
                        (12, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("identify cameras", vis)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27):
                break
        cap.release()
    cv2.destroyAllWindows()
    print("再用 --camera N 指定要用的那台")
    return 0
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--quality", type=int, default=80, help="JPEG 质量 1-100")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5599)
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--identify", action="store_true", help="逐个预览并标出序号")
    ap.add_argument("--preview", action="store_true", help="同时在 Windows 显示画面")
    ap.add_argument("--max-frames", type=int, default=0)
    a = ap.parse_args()

    if a.list:
        list_cameras()
        return 0
    if a.identify:
        return identify()

    # Windows 下用 DSHOW 最稳，失败再退回默认
    cap = cv2.VideoCapture(a.camera, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(a.camera)
    if not cap.isOpened():
        print("打不开相机 %d，用 --list 看序号" % a.camera)
        return 1
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, a.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, a.height)
    cap.set(cv2.CAP_PROP_FPS, a.fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    ok, fr = cap.read()
    if not ok or fr is None:
        print("相机打不开或读不到帧")
        return 1
    print("相机 %d 实际输出 %dx%d @ 请求 %d fps"
          % (a.camera, fr.shape[1], fr.shape[0], a.fps))

    print("等待 WSL 端监听 %s:%d ..." % (a.host, a.port))
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    for attempt in range(60):
        try:
            s.connect((a.host, a.port))
            break
        except OSError:
            time.sleep(1)
    else:
        print("连不上 %s:%d —— 先在 WSL 里启动接收端" % (a.host, a.port))
        return 1
    print("已连接，开始推流（Ctrl+C 停止）")

    params = [int(cv2.IMWRITE_JPEG_QUALITY), int(a.quality)]
    t0 = time.perf_counter()
    n = 0
    sent_bytes = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("取帧失败，结束")
                break
            okj, buf = cv2.imencode(".jpg", frame, params)
            if not okj:
                continue
            data = buf.tobytes()
            s.sendall(struct.pack(">I", len(data)) + data)
            sent_bytes += len(data)
            n += 1
            if a.preview:
                cv2.imshow("sender (Windows)", frame)
                if (cv2.waitKey(1) & 0xFF) in (ord("q"), 27):
                    break
            if n % 90 == 0:
                dt = time.perf_counter() - t0
                print("已发 %d 帧  %.1f fps  %.2f MB/s  帧尺寸 %dx%d"
                      % (n, n / dt, sent_bytes / dt / 1048576,
                         frame.shape[1], frame.shape[0]))
            if a.max_frames and n >= a.max_frames:
                break
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        dt = max(1e-6, time.perf_counter() - t0)
        print("共发 %d 帧，平均 %.1f fps，%.2f MB/s" % (n, n / dt, sent_bytes / dt / 1048576))
        cap.release()
        s.close()
        if a.preview:
            cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    sys.exit(main())
