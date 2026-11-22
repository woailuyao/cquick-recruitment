# -*- coding: utf-8 -*-
"""
相机能力与"固定对焦"验证

细则 2.1 / 2.2 要求：
  * 固定焦距、固定对焦，关闭自动对焦和自动变焦；
  * 分辨率 1280x720，目标 30fps，并记录**实际**采集帧率（CAP_PROP_FPS 不可信）。

本脚本做三件事：
  1. 按连续取帧的实际耗时测真实帧率；
  2. 尝试关闭自动对焦、固定焦点，并读回确认驱动是否真的接受了；
  3. 检查自动曝光/自动白平衡是否可锁（场景里光照变化会让检测阈值漂移）。
"""

import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
OUT = os.path.join(ROOT, "snapshots")
os.makedirs(OUT, exist_ok=True)

W, H = 1280, 720


def get(cap, prop):
    v = cap.get(prop)
    return None if v is None else v


def main(index=0):
    # 必须用 MSMF：实测 DSHOW 后端只能 10 fps，MSMF 才是 30 fps
    cap = cv2.VideoCapture(index, cv2.CAP_MSMF)
    if not cap.isOpened():
        print("MSMF 打不开，退回默认后端 ...")
        cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        print("打开相机失败")
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)
    cap.set(cv2.CAP_PROP_FPS, 30)
    # 曝光拨回自动：上一次手动曝光会被驱动记住，导致只有 3 fps
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)

    for _ in range(10):
        cap.read()

    aw = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    ah = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    print("=" * 70)
    print(f"协商分辨率: {aw}x{ah}   驱动报告帧率: {cap.get(cv2.CAP_PROP_FPS):.1f}")
    print("=" * 70)

    # ---------------- 真实帧率 ----------------
    N = 90
    t0 = time.perf_counter()
    got = 0
    for _ in range(N):
        ok, f = cap.read()
        if not ok:
            break
        got += 1
    dt = time.perf_counter() - t0
    print(f"\n[实测帧率] 连续取 {got} 帧耗时 {dt:.2f}s -> {got / dt:.2f} fps "
          f"(驱动报 {cap.get(cv2.CAP_PROP_FPS):.1f})")

    # ---------------- 自动对焦 ----------------
    print("\n[对焦] 尝试锁定手动对焦 ...")
    before_af = get(cap, cv2.CAP_PROP_AUTOFOCUS)
    focus_before = get(cap, cv2.CAP_PROP_FOCUS)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    time.sleep(0.3)
    af_after = get(cap, cv2.CAP_PROP_AUTOFOCUS)
    print(f"  AUTOFOCUS 属性: 之前={before_af}  设为0后读回={af_after} "
          f"({'驱动已接受' if af_after == 0 else '驱动未响应，可能不支持'})")

    if focus_before is not None:
        cap.set(cv2.CAP_PROP_FOCUS, focus_before)
        time.sleep(0.3)
        print(f"  FOCUS 属性: 原值={focus_before} 读回={get(cap, cv2.CAP_PROP_FOCUS)}")

    # 对焦锁定的实际效果：连拍看清晰度是否稳定
    sharp = []
    for _ in range(20):
        ok, f = cap.read()
        if ok:
            g = cv2.cvtColor(f, cv2.COLOR_BGR2GRAY)
            sharp.append(cv2.Laplacian(g, cv2.CV_64F).var())
    if sharp:
        print(f"  清晰度(Laplacian 方差) 均值={np.mean(sharp):.1f} "
              f"波动={np.std(sharp):.1f}  "
              f"({'稳定' if np.std(sharp) < np.mean(sharp) * 0.15 else '波动较大，可能仍在自动对焦'})")

    # ---------------- 曝光 / 白平衡 ----------------
    print("\n[曝光/白平衡]")
    for name, prop in (("AUTO_EXPOSURE", cv2.CAP_PROP_AUTO_EXPOSURE),
                       ("EXPOSURE", cv2.CAP_PROP_EXPOSURE),
                       ("AUTO_WB", cv2.CAP_PROP_AUTO_WB),
                       ("WB_TEMPERATURE", cv2.CAP_PROP_WB_TEMPERATURE)):
        v = get(cap, prop)
        print(f"  {name:16s} = {v}")

    ok, frame = cap.read()
    if ok:
        p = os.path.join(OUT, "camera_verify.jpg")
        cv2.imwrite(p, frame)
        print(f"\n快照 -> {p}")
        print(f"画面统计: mean={np.mean(frame):.1f} std={np.std(frame):.1f}")

    cap.release()
    print("\n提示：内置摄像头的焦点/曝光能否真正锁定取决于驱动。"
          "\n      若上面 AUTOFOCUS 显示『驱动未响应』，采集和考核期间就不要让镜头前出现"
          "\n      远处/近处物体来回切换，否则对焦会漂、内参标定会失效。")
    return 0


if __name__ == "__main__":
    idx = int(sys.argv[1]) if len(sys.argv) > 1 else 0
    sys.exit(main(idx))
