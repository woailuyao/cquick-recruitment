# -*- coding: utf-8 -*-
"""extract_facing_x_frames.py —— 抽取「朝向 +X 轴（yaw≈0）」的标注素材

依据 D:/captures/定点拍摄记录.md 的真值：
  M5  (-2.00, 0.00)  yaw=0.00° 直面球门，手持，从未进过标志数据集
  M9  (0.00, +2.60)  「朝向球门并慢转」——取后半段（已转向球门）的画面
（M13/M15 虽也是 yaw=0，但固定机位帧已在 fixed_work 数据集里各占 13 张，
 再抽全是近重复帧；M8 过曝作废，不作训练素材。）

输出：D:/captures/_anno/facing_x_work/
  - M5_xxxxx.jpg / M9_xxxxx.jpg（1280x720 原帧）
  - classes.txt（L / T）
  - manifest.csv（帧号 / 时间 / 来源 / 建议划分）
  - 标注说明.txt
标注完把本目录追加到 prepare_landmark_dataset.py 的 --src 列表即可。
"""
import argparse
import csv
import os

import cv2

OUT = r"D:\captures\_anno\facing_x_work"

# (视频, 起始秒, 结束秒, 张数, 建议划分)
PLAN = [
    ("M5", 2.0, 14.0, 12, "train"),   # 手持，画面有自然抖动变化
    ("M9", 8.0, 14.0, 10, "train"),   # 后半段：已转向球门方向
]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()
    os.makedirs(args.out, exist_ok=True)

    rows = []
    total = 0
    for vid, t0, t1, n, split in PLAN:
        path = rf"D:\captures\{vid}.mp4"
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
            print("打不开视频:", path)
            return 1
        fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
        nframes = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        dur = nframes / fps
        t1 = min(t1, dur - 0.3)
        # 跳过每段首尾，均匀取 n 张
        times = [t0 + (t1 - t0) * i / (n - 1) for i in range(n)]
        for t in times:
            cap.set(cv2.CAP_PROP_POS_MSEC, t * 1000.0)
            ok, frame = cap.read()
            if not ok:
                print("%s @%.1fs 读取失败" % (vid, t))
                continue
            fidx = int(round(t * fps))
            name = "%s_%05d.jpg" % (vid, fidx)
            cv2.imwrite(os.path.join(args.out, name), frame)
            rows.append({"video": vid, "file": name,
                         "frame": fidx, "time_s": round(t, 2),
                         "suggested_split": split})
            total += 1
        cap.release()
        print("%s: 抽 %d 张（%.1f~%.1fs，fps=%.2f）" % (vid, n, t0, t1, fps))

    # classes.txt（labelImg 类别顺序，必须与之前一致）
    with open(os.path.join(args.out, "classes.txt"), "w",
              encoding="utf-8") as f:
        f.write("L\nT\n")

    # manifest
    with open(os.path.join(args.out, "manifest.csv"), "w", newline="",
              encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=["video", "file", "frame",
                                          "time_s", "suggested_split"])
        w.writeheader()
        w.writerows(rows)

    # 标注说明
    with open(os.path.join(args.out, "标注说明.txt"), "w",
              encoding="utf-8") as f:
        f.write("朝向 +X 轴补充素材标注说明\n")
        f.write("==========================\n")
        f.write("来源：M5（-2,0 yaw=0 直面球门，手持，未进过数据集）\n")
        f.write("      M9（0,2.60 朝球门慢转，取后半段已朝球门的画面）\n")
        f.write("类别：0=L，1=T（classes.txt 只有两行，不要新增类别）\n")
        f.write("要求：只画场地白胶带拼出的 L/T 标志本体；白线拐角、球门网、\n")
        f.write("      门框一律不画框。每张图都要有 txt（labelImg 会自动生成）。\n")
        f.write("划分：全部建议 train（manifest.csv 有 suggested_split 列）。\n")
        f.write("标完后合并命令见 README（--src 追加本目录）。\n")
    print("共 %d 张 -> %s" % (total, args.out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
