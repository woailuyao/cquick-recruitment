"""从现有视频/照片自动生成标志数据集（伪标签蒸馏用）。

背景：考核要求用 YOLO，但没有时间再拍、也没时间手标 300 张。
      做法是用几何检测器（vision2/markers_thick.py）先给视频自动打标签，
      再用这些标签训练 yolov8n。模型学到的是这个几何检测器的行为，但：
        * 快得多（YOLO 推理 ~10 ms/帧，几何检测 ~300 ms/帧）
        * 输出更稳定（不会因为线宽/光照毛刺而抖动）
      这不是"真实标注"，README 里必须如实写清楚。

用法：
    python tools/make_marker_dataset.py --root . --every 8
    # 生成到 dataset/marker/{images,labels}/{train,val,test}
"""

import argparse
import json
import os
import sys

import cv2
import numpy as np

def _repo_root():
    """向上找到包含 config/field.yaml 的目录，作为仓库根。"""
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(4):
        if os.path.exists(os.path.join(d, "config", "field.yaml")):
            return d
        d = os.path.dirname(d)
    return os.getcwd()

ROOT = _repo_root()
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from vision2 import markers_thick as mt  # noqa: E402
from vision2.console import setup_console  # noqa: E402
from vision2.io_utils import imread, imwrite, list_images  # noqa: E402

setup_console()

CLASS_ID = {"L": 0, "T": 1, "X": 2, "P": 3}


def video_frames(path, every):
    cap = cv2.VideoCapture(path)
    idx = 0
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        if idx % every == 0:
            yield idx, frame
        idx += 1
    cap.release()


def to_label(det, W, H, box_half):
    """把检测结果变成 YOLO 标签：class cx cy w h（归一化）。"""
    cx, cy = det["u"], det["v"]
    half = box_half
    x1, x2 = max(0.0, cx - half), min(W - 1.0, cx + half)
    y1, y2 = max(0.0, cy - half), min(H - 1.0, cy + half)
    if x2 - x1 < 24 or y2 - y1 < 24:
        return None
    return "%d %.6f %.6f %.6f %.6f" % (
        CLASS_ID[det["type"]],
        ((x1 + x2) / 2) / W, ((y1 + y2) / 2) / H,
        (x2 - x1) / W, (y2 - y1) / H,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=".", help="输出根目录（会建 dataset/marker/...）")
    ap.add_argument("--videos", default="", help="视频文件夹，默认 data/calib/intrinsics")
    ap.add_argument("--photos", default="", help="照片文件夹（可选，棋盘格照片里也有场地线）")
    ap.add_argument("--every", type=int, default=8, help="每多少帧取一张")
    ap.add_argument("--max-frames", type=int, default=160, help="最多生成多少张")
    ap.add_argument("--min-det", type=int, default=1, help="每张至少几个目标才保留")
    args = ap.parse_args()

    vdir = args.videos or os.path.join(ROOT, "data", "calib", "intrinsics")
    import glob
    vids = []
    for ext in ("mp4", "avi", "mov", "mkv", "MP4", "AVI", "MOV"):
        vids += glob.glob(os.path.join(vdir, "*." + ext))
    vids = sorted(set(os.path.abspath(v) for v in vids))
    if not vids:
        print("[错误] 没找到视频：%s" % vdir)
        return 1

    out_root = os.path.join(args.root, "dataset", "marker")
    for split in ("train", "val", "test"):
        os.makedirs(os.path.join(out_root, "images", split), exist_ok=True)
        os.makedirs(os.path.join(out_root, "labels", split), exist_ok=True)

    records = []
    print("视频：%s" % ", ".join(os.path.basename(v) for v in vids))
    for v in vids:
        for idx, frame in video_frames(v, args.every):
            if len(records) >= args.max_frames:
                break
            H, W = frame.shape[:2]
            dets = mt.detect(frame)
            if len(dets) < args.min_det:
                continue
            # 框大小按线宽定：太小的框学不到上下文，太大又混进别的标志
            w = max([d.get("w", 30.0) for d in dets])
            box_half = float(np.clip(w * 2.2, 110.0, 150.0))
            labels = []
            for d in dets:
                line = to_label(d, W, H, box_half)
                if line:
                    labels.append(line)
            if not labels:
                continue
            records.append({"video": os.path.basename(v), "frame": idx,
                            "n": len(labels), "labels": labels,
                            "types": [d["type"] for d in dets]})

    if len(records) < 30:
        print("[错误] 只生成 %d 张，太少了。把 --every 调小（比如 4）再试。" % len(records))
        return 1

    # 按时间顺序划分，避免相邻帧泄漏
    n = len(records)
    n_train = int(n * 0.8)
    n_val = int(n * 0.9)
    splits = [("train", records[:n_train]), ("val", records[n_train:n_val]), ("test", records[n_val:])]

    stat = {k: {"images": 0, "labels": 0, "classes": {"L": 0, "T": 0, "X": 0}} for k, _ in splits}
    done = 0
    for split, recs in splits:
        for r in recs:
            # 重新读帧（简单可靠，代价是慢一点）
            cap = cv2.VideoCapture(os.path.join(vdir, r["video"]))
            cap.set(cv2.CAP_PROP_POS_FRAMES, r["frame"])
            ok, frame = cap.read()
            cap.release()
            if not ok:
                continue
            name = "%s_%05d" % (os.path.splitext(r["video"])[0][:20], r["frame"])
            if not imwrite(os.path.join(out_root, "images", split, name + ".jpg"), frame):
                continue
            with open(os.path.join(out_root, "labels", split, name + ".txt"), "w", encoding="utf-8") as f:
                f.write("\n".join(r["labels"]) + "\n")
            stat[split]["images"] += 1
            stat[split]["labels"] += r["n"]
            for t in r["types"]:
                stat[split]["classes"][t] = stat[split]["classes"].get(t, 0) + 1
            done += 1

    print("")
    print("生成完成：%d 张" % done)
    for split, _ in splits:
        s = stat[split]
        print("  %-5s 图片 %3d  标签 %3d  类别 %s" % (split, s["images"], s["labels"], s["classes"]))
    with open(os.path.join(out_root, "auto_label_stat.json"), "w", encoding="utf-8") as f:
        json.dump({"splits": stat, "every": args.every, "note": "伪标签由几何检测器生成"}, f,
                  ensure_ascii=False, indent=2)
    print("")
    print("数据集在：%s" % out_root)
    print("下一步：yolo detect train model=yolov8n.pt data=dataset/marker_data.yaml epochs=40 imgsz=640")
    return 0


if __name__ == "__main__":
    sys.exit(main())
