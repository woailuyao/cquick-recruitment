# -*- coding: utf-8 -*-
"""从本人实拍视频抽帧，并【按视频段整体】划分 train/val/test。

这是细则 1.3 / 1.4 最容易扣分的地方：不能把同一段视频的相邻帧随机打散到
三个集合里，否则评估指标虚高。本脚本以"视频文件"为单位分配集合，并输出
split_manifest.csv 作为证据，评审抽查时可以直接拿出来。

目录约定（拍完素材后）:
    raw_videos/  V1_20261115_left.mp4  V2_...mp4  ...   (>=5 段独立视频)
    soccer_dataset/images/{train,val,test}/
    soccer_dataset/labels/{train,val,test}/    <- 标注工具生成，本脚本不伪造标签
    soccer_dataset/data.yaml
    soccer_dataset/split_manifest.csv
    soccer_dataset/dataset_summary.md

运行:
    python prepare_dataset.py --videos-dir raw_videos --out soccer_dataset --every 15

注意: 抽帧后仍需人工标注 >=200 张有效图；本脚本不会生成任何假标签。
"""
import argparse
import csv
from pathlib import Path

import cv2

VIDEO_EXT = {".mp4", ".avi", ".mov", ".mkv", ".mjpeg"}


def split_by_video(names):
    """按视频段分配，保证同一段视频的所有帧只属于一个集合。"""
    n = len(names)
    if n < 3:
        raise ValueError("至少需要 3 段视频才能划分 train/val/test（细则要求 >= 5 段）")
    n_test = max(1, round(n * 0.1))
    n_val = max(1, round(n * 0.2))
    n_train = n - n_val - n_test
    # 不洗牌：按文件名顺序分配，便于复现与人工核对
    assign = {}
    for i, name in enumerate(names):
        if i < n_train:
            assign[name] = "train"
        elif i < n_train + n_val:
            assign[name] = "val"
        else:
            assign[name] = "test"
    return assign


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--videos-dir", default="raw_videos")
    ap.add_argument("--out", default="soccer_dataset")
    ap.add_argument("--every", type=int, default=15,
                    help="每隔 N 帧抽一张，避免相邻帧高度相似")
    ap.add_argument("--max-per-video", type=int, default=60)
    a = ap.parse_args()

    vids = sorted(p for p in Path(a.videos_dir).glob("*")
                  if p.suffix.lower() in VIDEO_EXT)
    if not vids:
        raise SystemExit("[FAIL] {} 里没有视频，等能拍摄后再运行".format(a.videos_dir))
    print("发现 {} 段视频；细则要求 >= 5 段独立采集".format(len(vids)))
    if len(vids) < 5:
        print("[WARN] 段数不足 5，最终必须补齐")

    assign = split_by_video([v.name for v in vids])
    out = Path(a.out)
    for part in ("train", "val", "test"):
        (out / "images" / part).mkdir(parents=True, exist_ok=True)
        (out / "labels" / part).mkdir(parents=True, exist_ok=True)

    manifest, to_annotate = [], []
    per_split = {"train": 0, "val": 0, "test": 0}
    for v in vids:
        part = assign[v.name]
        cap = cv2.VideoCapture(str(v))
        if not cap.isOpened():
            print("[WARN] 打不开 {}，跳过".format(v.name))
            continue
        src_fps = cap.get(cv2.CAP_PROP_FPS)
        idx, saved = 0, 0
        while saved < a.max_per_video:
            ok, frame = cap.read()
            if not ok:
                break
            if idx % a.every == 0:
                img_name = "{}_f{:06d}.jpg".format(v.stem, idx)
                cv2.imwrite(str(out / "images" / part / img_name), frame)
                manifest.append([img_name, part, v.name, idx])
                to_annotate.append(str(out / "images" / part / img_name))
                per_split[part] += 1
                saved += 1
            idx += 1
        cap.release()
        print("  {:<32s} {:5s}  源帧率 {:5.1f} -> 抽 {:3d} 张".format(
            v.name, part, src_fps, saved))

    with (out / "split_manifest.csv").open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["image", "split", "source_video", "frame_index"])
        w.writerows(manifest)
    (out / "to_annotate.txt").write_text("\n".join(to_annotate) + "\n",
                                          encoding="utf-8")

    root = out.resolve().as_posix()
    (out / "data.yaml").write_text(
        "# 由 prepare_dataset.py 生成，可直接传给 yolo 命令\n"
        "# 注意: 若在 Windows 侧跑本脚本而 yolo 在 WSL 里训练，请把 path 改成\n"
        "#       该目录在 WSL 中的绝对路径，或直接在 WSL 里重新运行本脚本。\n"
        "path: {}\n"
        "train: images/train\n"
        "val: images/val\n"
        "test: images/test\n"
        "nc: 1\n"
        "names: ['ball']\n".format(root),
        encoding="utf-8")

    (out / "dataset_summary.md").write_text(
        "# 数据集摘要（草稿，标注完成后补全）\n\n"
        "- 数据来源：本人于实验室使用同一台单目 RGB 摄像头实拍，非网络/他人数据\n"
        "- 原始视频段数：{}\n"
        "- 抽帧间隔：每 {} 帧一张；每段最多 {} 张\n"
        "- 抽帧总数：{}（train {} / val {} / test {}）\n"
        "- 划分方式：**按视频段整体划分**，同一段视频的帧不会跨集合；"
        "明细见 split_manifest.csv\n"
        "- 类别：单类别 `ball`，class_id = 0\n"
        "- 有效标注图片数 / 标注实例数：待标注完成后填写（要求 >= 200 张、建议 >= 200 实例）\n"
        "- 采集日期 / 设备节点 / 分辨率 / 实测 FPS：见 capture_log.csv\n".format(
            len(vids), a.every, a.max_per_video, sum(per_split.values()),
            per_split["train"], per_split["val"], per_split["test"]),
        encoding="utf-8")

    print("\n抽帧总数 {} (train {} / val {} / test {})".format(
        sum(per_split.values()), per_split["train"], per_split["val"],
        per_split["test"]))
    print("已生成: data.yaml / split_manifest.csv / to_annotate.txt / dataset_summary.md")
    print("下一步: 用 LabelImg 或 CVAT 按 to_annotate.txt 标注，"
          "把 .txt 放到对应的 labels/<split>/ 下，再跑 verify_labels.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
