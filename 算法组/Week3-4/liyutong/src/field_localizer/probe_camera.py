# -*- coding: utf-8 -*-
"""
相机探测脚本：把 Windows 上三种取流后端都试一遍，定位到底是哪一步打不开
用法：conda activate calib && python probe_camera.py
"""
import cv2
import numpy as np

BACKENDS = [
    ("MSMF (Windows 默认)", cv2.CAP_MSMF),
    ("DSHOW (DirectShow)", cv2.CAP_DSHOW),
    ("ANY (自动选择)", cv2.CAP_ANY),
]

def imwrite_safe(path, img):
    ok, buf = cv2.imencode(".jpg", img)
    if ok:
        buf.tofile(path)
    return ok

def try_open(index, backend_name, backend_flag):
    cap = cv2.VideoCapture(index, backend_flag)
    if not cap.isOpened():
        cap.release()
        return False
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    ok, frame = cap.read()
    if not ok or frame is None:
        print(f"    打开成功但读不到画面（分辨率报 {w}x{h}）")
        cap.release()
        return False
    real = frame.shape[1], frame.shape[0]
    print(f"    ✅ 可用  报称 {w}x{h} / 实际帧 {real[0]}x{real[1]}")
    name = f"probe_{backend_name.split()[0]}_{index}.jpg"
    imwrite_safe(name, frame)
    print(f"    已存一张到 {name}，打开看看是不是你要的那台相机")
    cap.release()
    return True

def main():
    print("=" * 60)
    print("相机后端探测")
    print("=" * 60)
    any_ok = False
    for backend_name, flag in BACKENDS:
        print(f"\n[{backend_name}]")
        for index in range(4):
            print(f"  相机 {index}:")
            if try_open(index, backend_name, flag):
                any_ok = True

    print("\n" + "=" * 60)
    if any_ok:
        print("结论：至少有一种后端能用。用能用的那个后端名改 camera_check.py")
        print("      把 cv2.CAP_DSHOW 换成对应的 cv2.CAP_MSMF 即可")
    else:
        print("结论：三种后端全部打不开，按顺序排查：")
        print("  1. 设置 → 隐私和安全性 → 相机 → 打开「让桌面应用访问你的相机」")
        print("  2. 关掉可能占用相机的程序：Windows 相机 App、微信、腾讯会议、OBS")
        print("  3. 设备管理器里确认「USB Camera」没有黄色感叹号、没被禁用")
        print("  4. 换一个 USB 口（优先主机后面板 USB 3.0），换根数据线")
    print("=" * 60)

if __name__ == "__main__":
    main()
