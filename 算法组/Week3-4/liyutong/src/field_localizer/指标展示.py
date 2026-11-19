# -*- coding: utf-8 -*-
"""
录屏用的「指标与失败案例」展示页

细则第四节·任务 1·视频证据要求第 4 条要求录屏里出现：
    训练完成后的模型信息 / 独立测试集 Precision·Recall·mAP / 至少一个误检或漏检案例分析

这个脚本把这三样一次性打在一个窗口里（上：指标卡；下：两个漏检实例），
配 Win+G 录 20 秒即可。

用法
    python 指标展示.py            # 开窗口（录屏用）
    python 指标展示.py --save      # 只存图不弹窗（自检用）
"""
import argparse
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
os.chdir(HERE)

W, H = 1280, 720
STRIP_H = 232


def put(img, text, org, scale=0.5, color=(255, 255, 255), th=1):
    """cv2.putText 只认 ASCII，先描黑边保证在任意背景上都看得清。"""
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, th, cv2.LINE_AA)


def head(img, y, text):
    put(img, text, (18, y), 0.6, (0, 255, 255), 2)
    cv2.line(img, (18, y + 7), (W - 18, y + 7), (0, 255, 255), 1, cv2.LINE_AA)


def size_mb(p):
    try:
        return os.path.getsize(p) / 1048576.0
    except OSError:
        return float("nan")


def build_card():
    img = np.full((H, W, 3), 30, np.uint8)

    put(img, "CQuick Vision - Week3/4 - Model Card & Failure Cases", (18, 30), 0.72,
        (255, 255, 255), 2)
    put(img, "liyutong   2026-11-17   (Win+G recording: whole screen, one take)", (18, 54),
        0.48, (170, 170, 170))

    head(img, 84, "TASK 1  Ball detection  (YOLOv8n, single class 'ball')")
    bp = "runs_ball/baseline/weights/best.pt"
    rows = [
        "weights : %s   %.2f MB   trained 50 epochs @640 on CPU" % (bp, size_mb(bp)),
        "dataset : 6 self-recorded clips -> 428 frames  (train 288 / val 98 / test 42, split by clip)",
        "TEST set (42 frames, never seen in training):",
        "   Precision 0.970    Recall 0.776    mAP50 0.847    mAP50-95 0.754",
        "VAL  set (98 frames):  P 0.979  R 0.980  mAP50 0.967  mAP50-95 0.890",
    ]
    y = 110
    for i, t in enumerate(rows):
        put(img, t, (28, y), 0.47, (200, 255, 200) if i < 2 else (255, 255, 255))
        y += 23
    put(img, "test < val because 62% of test frames come from one long-shot clip (ball 9-11 px).",
        (28, y), 0.44, (150, 190, 255))

    head(img, 254, "TASK 2  Field marker detection  (YOLOv8n, L / T / X)")
    mp = "runs_marker/baseline/weights/best.pt"
    rows2 = [
        "weights : %s   %.2f MB   trained 50 epochs @640 on CPU" % (mp, size_mb(mp)),
        "dataset : 23 clips -> 298 frames (250 with markers, 48 neg) / 292 boxes",
        "TEST set (50 frames):  P 0.709  R 0.786  mAP50 0.709  mAP50-95 0.341",
        "   boxes: L 96 / T 84 / X 112      per-class AP50:  L 0.307  T 0.995  X 0.824",
        "L low = only ONE corner per frame was labelled (2-3 more are real but unlabelled),",
        "so correct detections count as false positives. T / X fully labelled -> 0.995.",
    ]
    y = 280
    for i, t in enumerate(rows2):
        put(img, t, (28, y), 0.47,
            (200, 255, 200) if i < 2 else ((255, 210, 150) if i >= 4 else (255, 255, 255)))
        y += 23

    head(img, 428, "FAILURE CASES  (missed detections, same test set)")
    return img


def annotate_case(path, tag, note):
    f = cv2.imread(path)
    if f is None:
        return None
    h, w = f.shape[:2]
    nw = int(w * STRIP_H / h)
    f = cv2.resize(f, (nw, STRIP_H), interpolation=cv2.INTER_LINEAR)
    lab = os.path.splitext(path)[0].replace("images", "labels") + ".txt"
    if os.path.exists(lab):
        for ln in open(lab):
            c, x, y, bw, bh = map(float, ln.split())
            x1 = int((x - bw / 2) * nw)
            y1 = int((y - bh / 2) * STRIP_H)
            x2 = int((x + bw / 2) * nw)
            y2 = int((y + bh / 2) * STRIP_H)
            # 小框放大画出来，否则 16 px 的球在缩略图上根本看不见
            cv2.rectangle(f, (x1 - 8, y1 - 8), (x2 + 8, y2 + 8), (0, 255, 255), 1)
            cv2.arrowedLine(f, (x1 - 40, y1 - 34), (x1 - 10, y1 - 10), (0, 255, 255), 1,
                            cv2.LINE_AA, tipLength=0.3)
    put(f, tag, (8, 20), 0.5, (0, 255, 255), 1)
    put(f, note, (8, STRIP_H - 10), 0.42, (255, 255, 255), 1)
    return f


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--save", action="store_true", help="只存图，不弹窗")
    ap.add_argument("--out", default="_指标展示.jpg")
    args = ap.parse_args()

    img = build_card()
    c1 = annotate_case("soccer_dataset/images/test/video6_video6_003.jpg",
                       "MISS 1", "ball only 16 px on the goal line")
    c2 = annotate_case("soccer_dataset/images/test/video6_video6_045.jpg",
                       "MISS 2", "ball motion-blurred when kicked")
    tiles = [c for c in (c1, c2) if c is not None]
    if tiles:
        strip = np.hstack(tiles)
        maxw = W - 200
        if strip.shape[1] > maxw:
            strip = cv2.resize(strip, (maxw, STRIP_H))
        y0 = H - STRIP_H - 10
        img[y0:y0 + STRIP_H, 18:18 + strip.shape[1]] = strip
        put(img, "both missed at conf 0.25", (18 + strip.shape[1] + 14, y0 + 26), 0.46,
            (150, 190, 255))
        put(img, "-> recall 0.776", (18 + strip.shape[1] + 14, y0 + 52), 0.46, (150, 190, 255))
        put(img, "same model, same test split", (18 + strip.shape[1] + 14, y0 + 78), 0.4,
            (140, 140, 140))

    ok, buf = cv2.imencode(".jpg", img)
    if ok:
        with open(args.out, "wb") as fh:      # cv2.imwrite 写中文路径会静默失败
            fh.write(buf.tobytes())
        print("已保存", args.out)

    if not args.save:
        cv2.imshow("model card & failure cases (q=quit s=save)", img)
        print("按键：q 退出 / s 存图")
        while True:
            k = cv2.waitKeyEx(0) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                n = args.out.replace(".jpg", "_snap.jpg")
                ok2, b2 = cv2.imencode(".jpg", img)
                if ok2:
                    with open(n, "wb") as fh:
                        fh.write(b2.tobytes())
                    print("已存", n)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
