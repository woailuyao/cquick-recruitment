"""诊断：统计 train/val/test 三个 split 的球框尺度分布。

用途：考核报告里说明「测试集为什么难」、判断是否需要提高 imgsz、
以及解释 mAP50-95 偏低的原因。不修改任何数据，只读。

用法：
  python tools/analyze_boxes.py
  python tools/analyze_boxes.py --data config/data.yaml --out logs/box_stats.txt
"""
import argparse
import statistics as st
from collections import Counter, defaultdict
from pathlib import Path

import cv2
import yaml

ROOT = Path(__file__).resolve().parents[1]
IMG_EXTS = {".jpg", ".jpeg", ".png", ".bmp"}
# 按 COCO 惯例的绝对像素尺度分档（以原图为准，不是 640 缩放后）
BINS = [(0, 16, "tiny <16px"), (16, 32, "small 16-32"),
        (32, 96, "medium 32-96"), (96, 1e9, "large >96px")]


def group_key(stem: str) -> str:
    head, sep, tail = stem.rpartition("_f")
    return head if (sep and tail.isdigit()) else stem


def bin_of(v):
    for lo, hi, name in BINS:
        if lo <= v < hi:
            return name
    return BINS[-1][2]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(ROOT / "config" / "data.yaml"))
    ap.add_argument("--out", default=None, help="同时把报告写到文件")
    args = ap.parse_args()

    cfg = yaml.safe_load(Path(args.data).read_text(encoding="utf-8"))
    base = Path(cfg["path"])
    if not base.is_absolute():
        base = (Path(args.data).parent / base).resolve()

    lines = []

    def emit(s=""):
        print(s)
        lines.append(s)

    emit(f"数据集：{base}   类别：{cfg.get('names')}")
    emit("口径：球框像素尺寸按【原图】统计；训练时 imgsz=640 会再缩放，"
         "实际送入网络的目标更小。\n")

    for split in ("train", "val", "test"):
        img_dir = base / cfg[split]
        lbl_dir = base / ("labels/" + split)
        widths, heights, areas, ratios, centers = [], [], [], [], []
        n_img = n_empty = 0
        by_group = defaultdict(lambda: [0, 0])  # group -> [图片数, 球框数]
        raw_lines = Counter()
        resolutions = Counter()

        for img_path in sorted(img_dir.iterdir()):
            if img_path.suffix.lower() not in IMG_EXTS:
                continue
            n_img += 1
            img = cv2.imread(str(img_path))
            if img is None:
                continue
            H, W = img.shape[:2]
            resolutions[f"{W}x{H}"] += 1
            lp = lbl_dir / (img_path.stem + ".txt")
            txt = lp.read_text(encoding="utf-8").strip() if lp.exists() else ""
            if not txt:
                n_empty += 1
            g = group_key(img_path.stem)
            by_group[g][0] += 1
            for line in txt.splitlines():
                p = line.split()
                if len(p) < 5:
                    continue
                _, cx, cy, w, h = (float(x) for x in p[:5])
                pw, ph = w * W, h * H
                widths.append(pw)
                heights.append(ph)
                areas.append(pw * ph)
                # 注意：YOLO 的 w/h 是分别除以图像宽/高归一化的，
                # 直接算 w/h 会混入 720/1280 的宽高比因子，必须用像素尺寸还原。
                ratios.append(pw / ph if ph > 0 else 0)
                centers.append((cx, cy))
                by_group[g][1] += 1
                raw_lines[bin_of(max(pw, ph))] += 1

        emit(f"===== {split} =====")
        emit(f"图片 {n_img} 张 | 含球 {n_img-n_empty} 张 | 空标签 {n_empty} 张 "
             f"| 球框 {len(widths)} 个 | 分辨率 {dict(resolutions)}")
        if widths:
            def q(v, p):
                v = sorted(v)
                return v[min(len(v)-1, int(len(v)*p))]
            emit(f"框宽 px : min {min(widths):.1f} / p25 {q(widths,.25):.1f} / "
                 f"中位 {st.median(widths):.1f} / p75 {q(widths,.75):.1f} / max {max(widths):.1f}")
            emit(f"框高 px : min {min(heights):.1f} / 中位 {st.median(heights):.1f} "
                 f"/ max {max(heights):.1f}")
            emit(f"面积 px²: 中位 {st.median(areas):.0f}  占全图比例中位 "
                 f"{st.median([a/(1280*720)*100 for a in areas]):.3f}% (按1280x720估)")
            emit(f"尺度分档(按 max(w,h)): " +
                 " | ".join(f"{name} {raw_lines[name]}" for _, _, name in BINS))
            emit(f"宽高比中位 {st.median(ratios):.2f}  "
                 f"归一化中心 x 中位 {st.median([c[0] for c in centers]):.3f} "
                 f"y 中位 {st.median([c[1] for c in centers]):.3f}")
        emit("视频组：" + ", ".join(f"{g}({v[0]}张/{v[1]}框)"
                                  for g, v in sorted(by_group.items())))
        emit("")

    # 跨 split 对比
    emit("===== 结论提示 =====")
    emit("若 test 的中位框宽明显小于 train，说明测试集是【尺度外 OOD】，"
         "mAP 偏低主要是分布差异而非模型坏掉；")
    emit("提高 imgsz（960/1280）或补充同尺度训练帧才有明显收益，"
         "单纯加 epoch 收益有限。")

    if args.out:
        out = Path(args.out)
        if not out.is_absolute():
            out = ROOT / out
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text("\n".join(lines) + "\n", encoding="utf-8")
        print(f"\n报告已写入 {out}")


if __name__ == "__main__":
    main()
