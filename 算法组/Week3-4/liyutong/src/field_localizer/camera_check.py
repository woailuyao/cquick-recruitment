# -*- coding: utf-8 -*-
"""
相机自检脚本（第 3-4 周考核 · 第一步）
用途：确认相机能用、记录考核要求的采集参数（接口、分辨率、实际帧率、拍摄日期）
用法：conda activate cv_practice && python camera_check.py
操作：按 s 保存当前帧，按 q 退出
"""
import cv2
import time
import datetime
import os

def imwrite_safe(path, img):
    """cv2.imwrite 遇到中文路径会静默失败，用 imencode 绕过"""
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok

def probe_camera(index):
    """探测第 index 号相机是否可用，返回 (cap, 真实分辨率, 实测帧率) 或 None"""
    cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)  # Windows 用 DSHOW 打开更快
    if not cap.isOpened():
        cap.release()
        return None
    # 固定到工作分辨率 640x480（与标定、采集保持一致）
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # 预热丢掉前几帧
    for _ in range(10):
        cap.read()
    # 实测帧率：数 60 帧用了多久
    n, t0 = 0, time.time()
    while n < 60:
        ok, _ = cap.read()
        if not ok:
            break
        n += 1
    fps = n / (time.time() - t0) if time.time() > t0 else 0
    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    return cap, (w, h), round(fps, 1)

def main():
    print("=" * 50)
    print("相机自检 |", datetime.datetime.now().strftime("%Y-%m-%d %H:%M"))
    print("=" * 50)
    found = False
    for i in range(4):
        result = probe_camera(i)
        if result is None:
            print(f"[相机 {i}] 不可用")
            continue
        found = True
        cap, (w, h), fps = result
        print(f"[相机 {i}] 可用  分辨率 {w}x{h}  实测帧率 {fps} FPS")
        print("-" * 50)
        print("  >>> 把这行抄进考核记录：接口=USB / 设备编号=%d / 分辨率=%dx%d / 帧率=%.1f FPS / 日期=%s"
              % (i, w, h, fps, datetime.date.today()))
        print("-" * 50)
        print("  窗口里按 s 保存一张测试帧，按 q 关闭这个相机")
        while True:
            ok, frame = cap.read()
            if not ok:
                print("  读帧失败，跳到下一个相机")
                break
            cv2.imshow(f"camera {i}", frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                name = f"camera{i}_test_{datetime.datetime.now():%H%M%S}.jpg"
                ok = imwrite_safe(name, frame)
                print(f"  已保存 {name}" if ok else f"  保存失败 {name}")
            elif key == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()
    if not found:
        print("没找到任何可用相机：检查 USB 线、换 USB 口，或被其他软件（微信/相机 App）占用")

if __name__ == "__main__":
    main()
