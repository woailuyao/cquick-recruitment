# -*- coding: utf-8 -*-
"""
抽帧脚本：从原始视频抽帧做数据集
用法：
    python extract_frames.py                              # 默认 videos/ -> raw/，间隔 0.4 s
    python extract_frames.py --src videos --dst raw_mark --interval 0.2 --prefix L T X P
前提：把原始视频放进 --src 目录，一段视频一个文件
输出：<dst>/<视频名>/ 下的 jpg，供 split_dataset.py 按视频段划分

策略：
  1. 先按 INTERVAL_SEC 列出全片所有候选帧
  2. 若候选数 > MAX_PER_VIDEO，则在整个时长上「均匀取样」，而不是只取开头
     （避免长的视频后段素材全丢掉）
  3. 逐帧读、按需保存，比反向 seek 更可靠
  4. 拉普拉斯方差去模糊；另外用「缩略图差异」去掉几乎重复的帧
"""
import cv2
import os
import numpy as np

VIDEO_DIR = "videos"
OUT_DIR = "raw"
INTERVAL_SEC = 0.4      # 候选帧间隔
BLUR_THRESHOLD = 60.0   # 拉普拉斯方差低于这个值判为模糊帧，丢掉
MAX_PER_VIDEO = 120     # 每段最多留多少张，避免同一段占比过大
DUP_THRESHOLD = 1.5     # 缩略图平均绝对差 < 该值视为重复帧（0~255 灰度）


def imwrite_safe(path, img):
    ok, buf = cv2.imencode(os.path.splitext(path)[1], img)
    if ok:
        buf.tofile(path)
    return ok


def thumb_sig(frame):
    """32x32 灰度缩略图，用于判断两帧是否几乎一样"""
    g = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return cv2.resize(g, (32, 32), interpolation=cv2.INTER_AREA).astype(np.int16)


def main():
    global VIDEO_DIR, OUT_DIR, INTERVAL_SEC, MAX_PER_VIDEO
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", default=VIDEO_DIR)
    ap.add_argument("--dst", default=OUT_DIR)
    ap.add_argument("--interval", type=float, default=INTERVAL_SEC, help="候选帧间隔（秒）")
    ap.add_argument("--max-per-video", type=int, default=MAX_PER_VIDEO)
    ap.add_argument("--prefix", nargs="*", default=None,
                    help="只处理这些前缀的视频，例如 --prefix L T X P")
    a = ap.parse_args()
    VIDEO_DIR, OUT_DIR = a.src, a.dst
    INTERVAL_SEC, MAX_PER_VIDEO = a.interval, a.max_per_video

    if not os.path.isdir(VIDEO_DIR):
        print(f"请先建 {VIDEO_DIR}/ 目录，把原始视频放进去，然后重跑本脚本")
        return
    videos = [f for f in os.listdir(VIDEO_DIR)
              if f.lower().endswith((".mp4", ".avi", ".mov", ".mkv"))]
    if a.prefix:
        videos = [f for f in videos
                  if any(os.path.splitext(f)[0].split("_")[0] == p for p in a.prefix)]
    if not videos:
        print(f"{VIDEO_DIR}/ 里没有符合条件的视频文件")
        return
    print(f"找到 {len(videos)} 段视频")
    print(f"参数：候选间隔 {INTERVAL_SEC}s / 模糊阈值 {BLUR_THRESHOLD} / "
          f"每段上限 {MAX_PER_VIDEO} / 重复阈值 {DUP_THRESHOLD}\n")

    grand = 0
    for v in sorted(videos):
        seg = os.path.splitext(v)[0]
        out = os.path.join(OUT_DIR, seg)
        os.makedirs(out, exist_ok=True)
        # 不删除任何已有文件：若本段目录里已有图，跳过本段（避免新旧混在一起）
        existing = [f for f in os.listdir(out) if f.lower().endswith(".jpg")]
        if existing:
            print(f"  {v}: 目录 {out}/ 已有 {len(existing)} 张，跳过"
                  f"（如需重抽，先把该目录改名或移走）")
            grand += len(existing)
            continue

        cap = cv2.VideoCapture(os.path.join(VIDEO_DIR, v))
        if not cap.isOpened():
            print(f"  打不开：{v}")
            continue
        fps = cap.get(cv2.CAP_PROP_FPS) or 30
        n_total = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        step = max(1, int(round(fps * INTERVAL_SEC)))
        dur = n_total / fps if fps else 0

        # 全片候选帧索引
        cand = list(range(0, n_total, step))
        if len(cand) > MAX_PER_VIDEO:
            pick = np.linspace(0, len(cand) - 1, MAX_PER_VIDEO).round().astype(int)
            cand = [cand[i] for i in sorted(set(pick))]
        wanted = set(cand)

        saved = blur_drop = dup_drop = 0
        last_sig = None
        idx = 0
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            if idx in wanted:
                gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                var = cv2.Laplacian(gray, cv2.CV_64F).var()
                if var < BLUR_THRESHOLD:
                    blur_drop += 1
                else:
                    sig = thumb_sig(frame)
                    if last_sig is not None and np.abs(sig - last_sig).mean() < DUP_THRESHOLD:
                        dup_drop += 1
                    else:
                        name = os.path.join(out, f"{seg}_{saved:03d}.jpg")
                        imwrite_safe(name, frame)
                        last_sig = sig
                        saved += 1
            idx += 1
        cap.release()
        grand += saved
        print(f"  {v}: {dur:5.1f}s / {n_total} 帧 -> 留 {saved:3d} 张"
              f"（候选 {len(cand)}，模糊丢 {blur_drop}，重复丢 {dup_drop}）")

    print(f"\n总计 {grand} 张，存在 {OUT_DIR}/ 下按视频分段。")
    if grand < 200:
        print(f"⚠️ 不足 200 张，建议再补录视频（当前差 {200 - grand} 张）")
    else:
        print(f"✅ 满足 ≥200 张的硬要求（多出 {grand - 200} 张）")
    print("下一步：labelImg 标注 raw/<段名>/（只标 ball），再跑 python split_dataset.py")


if __name__ == "__main__":
    main()
