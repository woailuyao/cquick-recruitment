# -*- coding: utf-8 -*-
"""
录制脚本：按标定分辨率 640x480 录视频到 videos/
用法：
    python record_video.py                        # 足球视频 -> video1.mp4 …
    python record_video.py --name L               # 标志素材 -> L_1.mp4 …
    python record_video.py --name L --guides      # 带"地面距离辅助线"取景
    python record_video.py --name gt1 --cls 真值  # 顶部显示当前拍的是什么

窗口里：空格 = 开始 / 再按空格停止并保存，q = 退出

--guides 会在画面上画几条水平线，标出"这条线对应的地面距离"。
辅助线只画在预览窗口，**不会写进视频文件**，所以不影响训练数据。
"""
import os
import sys
import time

import cv2
import numpy as np

OUT_DIR = "videos"
WIDTH, HEIGHT = 640, 480      # 必须与标定分辨率一致
FPS = 20.0                    # 这台相机 640x480 实测约 20~30，用 20 较稳

# 辅助线显示的地面距离（米），近 -> 远
GUIDE_DISTANCES = (0.6, 0.8, 1.0, 1.5, 2.0, 3.0, 4.0, 6.0)


def build_guides():
    """算出"正前方 d 米的地面点"落在画面的哪一行。返回 [(d, v), ...]"""
    try:
        from localize import load_intrinsics, load_extrinsics
        K, dist = load_intrinsics()
        R_gc, h = load_extrinsics()
    except Exception as e:
        print("辅助线不可用（读不到内参/外参）：", e)
        return []
    out = []
    for d in GUIDE_DISTANCES:
        cam = (R_gc @ np.array([[0.0, d, -h]]).T).T
        uv, _ = cv2.projectPoints(cam, np.zeros(3), np.zeros(3), K, dist)
        v = float(uv.reshape(-1, 2)[0, 1])
        if 8 < v < HEIGHT - 30:
            out.append((d, v))
    return out


def draw_guides(show, guides, w, h):
    """画辅助线：1~3 m 是绿区（成像最好），其余黄区"""
    cv2.line(show, (w // 2, 0), (w // 2, h), (90, 90, 90), 1)
    for d, v in guides:
        good = 0.9 <= d <= 3.2
        color = (0, 200, 0) if good else (0, 200, 220)
        y = int(round(v))
        cv2.line(show, (0, y), (w - 1, y), color, 1)
        txt = f"{d:g} m"
        cv2.putText(show, txt, (w - 62, y - 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)


def main():
    argv = sys.argv[1:]

    def arg(name, default=None):
        if name in argv:
            i = argv.index(name)
            if i + 1 < len(argv):
                return argv[i + 1]
        return default

    prefix = arg("--name", "") or ""
    label = arg("--cls", "") or prefix
    guides_on = "--guides" in argv

    os.makedirs(OUT_DIR, exist_ok=True)

    def make_path(n):
        if prefix:
            return os.path.join(OUT_DIR, f"{prefix}_{n}.mp4")
        return os.path.join(OUT_DIR, f"video{n}.mp4")

    # 编号按同类前缀续号：video2.mp4 已存在 -> 下一个 video3.mp4；
    # 带前缀时 L_1.mp4、L_2.mp4 …，避免出现 L_8 这种从足球视频数下去的怪编号
    idx = 1
    while os.path.exists(make_path(idx)):
        idx += 1

    out_path = make_path(idx)

    cap = cv2.VideoCapture(0, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("相机打不开")
        return
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    for _ in range(10):
        cap.read()

    guides = build_guides() if guides_on else []
    if guides_on:
        print("辅助线对应距离：", ", ".join(f"{d:g}m->v{int(v)}" for d, v in guides))
        print("绿色线之间（0.9~3.2 m）成像最清楚，优先把标志点放这一带")

    writer = None
    recording = False
    frames = 0
    t_start = 0
    print(f"将保存到 {out_path}")
    print("空格 = 开始/停止录制 ，q = 退出（停止后自动保存）")

    while True:
        ok, frame = cap.read()
        if not ok:
            print("读帧失败")
            break
        h, w = frame.shape[:2]
        show = frame.copy()

        if guides:
            draw_guides(show, guides, w, h)

        if recording:
            frames += 1
            writer.write(frame)                      # 写的是干净帧，不含任何叠加
            sec = time.time() - t_start
            cv2.circle(show, (30, 30), 12, (0, 0, 255), -1)
            cv2.putText(show, f"REC {sec:5.1f}s  {frames} frames", (50, 38),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            if sec > 40:
                cv2.putText(show, ">40s! stop soon", (w - 250, 70),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        else:
            cv2.putText(show, "SPACE to record", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        if label:
            cv2.putText(show, f"CLASS: {label}", (10, h - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(show, f"{w}x{h}  -> {os.path.basename(out_path)}", (10, h - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)

        cv2.imshow("record  (SPACE=start/stop, Q=quit)", show)
        k = cv2.waitKey(1) & 0xFF
        if k == ord(' '):
            if not recording:
                writer = cv2.VideoWriter(out_path, cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
                if not writer.isOpened():
                    print("创建视频文件失败，检查 mp4v 编码支持")
                    break
                recording = True
                frames = 0
                t_start = time.time()
                print(f"开始录制 -> {out_path}")
            else:
                recording = False
                sec = time.time() - t_start
                writer.release()
                writer = None
                if sec < 8:
                    print(f"⚠️ 只录了 {sec:.1f} 秒，太短了（建议 20~30 s），这段作废")
                    try:
                        os.remove(out_path)
                    except OSError:
                        pass
                else:
                    print(f"已保存 {out_path}（{frames} 帧，约 {frames / FPS:.1f} 秒）")
                    idx += 1
                    out_path = make_path(idx)
                    print(f"下一段将保存到 {out_path}")
        elif k == ord('q'):
            break

    if writer is not None:
        writer.release()
        print(f"退出前已保存 {out_path}")
    cap.release()
    cv2.destroyAllWindows()
    print(f"videos/ 下现有：{sorted(os.listdir(OUT_DIR))}")


if __name__ == "__main__":
    main()
