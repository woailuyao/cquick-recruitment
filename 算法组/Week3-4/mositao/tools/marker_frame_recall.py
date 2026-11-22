"""帧级命中率评估。

任务二真正关心的不是 mAP，而是「单帧里能稳稳抓到几个标志」——外参加 PnP
至少需要 3 个已知点才能解出无歧义的位姿。mAP 把每帧摊平成一个框指标，
看不出「一帧里漏了两个」和「每帧各漏一个」的区别，这里把口径换到帧上。

用法（项目根 week3-4 下）：

    python tools\\marker_frame_recall.py --weights weights\\best_marker_v9.pt

口径：
    正确命中     = 类别一致 且 IoU >= 0.5（一对一贪心匹配）
    类别无视命中 = 任意类别 且 IoU >= 0.5
        两者之差说明漏的原因：差值大 = 「看见了但认错类」，
        差值小 = 「压根没看见」，后者才需要更多数据或更强增强。

只跑一次推理（conf 0.05），再按多个阈值重算统计，避免重复前向。
"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path

import numpy as np

IMG_EXT = {".jpg", ".jpeg", ".png", ".bmp"}


def iou_matrix(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """a, b: (N,4) / (M,4)，xyxy 归一化。返回 (N,M) 的 IoU。"""
    if len(a) == 0 or len(b) == 0:
        return np.zeros((len(a), len(b)), dtype=np.float32)
    x1 = np.maximum(a[:, None, 0], b[None, :, 0])
    y1 = np.maximum(a[:, None, 1], b[None, :, 1])
    x2 = np.minimum(a[:, None, 2], b[None, :, 2])
    y2 = np.minimum(a[:, None, 3], b[None, :, 3])
    inter = np.clip(x2 - x1, 0, None) * np.clip(y2 - y1, 0, None)
    area_a = (a[:, 2] - a[:, 0]) * (a[:, 3] - a[:, 1])
    area_b = (b[:, 2] - b[:, 0]) * (b[:, 3] - b[:, 1])
    union = area_a[:, None] + area_b[None, :] - inter
    return np.where(union > 0, inter / np.maximum(union, 1e-9), 0.0).astype(np.float32)


def read_label(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """返回 (classes (N,), xyxy 归一化 (N,4))。"""
    cls, box = [], []
    if path.exists():
        for line in path.read_text(encoding="utf-8").splitlines():
            parts = line.split()
            if len(parts) < 5:
                continue
            c = int(float(parts[0]))
            cx, cy, w, h = (float(v) for v in parts[1:5])
            cls.append(c)
            box.append([cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2])
    return np.array(cls, dtype=np.int32), np.array(box, dtype=np.float32).reshape(-1, 4)


def match_count(
    gt_cls: np.ndarray,
    gt_box: np.ndarray,
    pd_cls: np.ndarray,
    pd_box: np.ndarray,
    iou_thr: float,
    ignore_class: bool,
) -> int:
    """一对一贪心匹配，返回命中个数。"""
    if len(gt_box) == 0 or len(pd_box) == 0:
        return 0
    ious = iou_matrix(gt_box, pd_box)
    if not ignore_class:
        ious = np.where(gt_cls[:, None] == pd_cls[None, :], ious, 0.0)
    used_gt: set[int] = set()
    used_pd: set[int] = set()
    hits = 0
    # 按 IoU 从大到小取
    for flat in np.argsort(ious, axis=None)[::-1]:
        i, j = divmod(int(flat), ious.shape[1])
        if ious[i, j] < iou_thr:
            break
        if i in used_gt or j in used_pd:
            continue
        used_gt.add(i)
        used_pd.add(j)
        hits += 1
    return hits


def main() -> None:
    ap = argparse.ArgumentParser(description="标志检测帧级命中率")
    ap.add_argument("--weights", default="weights/best_marker_v9.pt")
    ap.add_argument("--images", default="dataset_marker/images/test")
    ap.add_argument("--labels", default="dataset_marker/labels/test")
    ap.add_argument("--confs", default="0.50,0.35,0.25,0.15,0.10")
    ap.add_argument("--iou", type=float, default=0.5)
    ap.add_argument("--imgsz", type=int, default=640)
    ap.add_argument("--device", default="0")
    ap.add_argument("--out", default="outputs/marker_frame_recall.csv")
    args = ap.parse_args()

    root = Path.cwd()
    img_dir = root / args.images
    lbl_dir = root / args.labels
    weights = root / args.weights
    if not weights.exists():
        raise SystemExit(f"权重不存在: {args.weights}")

    stem_of = {}
    for p in sorted(img_dir.iterdir()):
        if p.suffix.lower() in IMG_EXT:
            stem_of[p.stem] = p
    if not stem_of:
        raise SystemExit(f"测试图目录为空: {args.images}")

    from ultralytics import YOLO

    model = YOLO(str(weights))
    paths = [str(p) for p in stem_of.values()]
    names = list(stem_of.keys())
    results = model.predict(
        paths, imgsz=args.imgsz, conf=0.05, iou=0.7, device=args.device, verbose=False
    )
    names_map = model.names

    dets: dict[str, tuple[np.ndarray, np.ndarray, np.ndarray]] = {}
    for name, res in zip(names, results):
        b = res.boxes
        if b is None or len(b) == 0:
            dets[name] = (np.zeros(0, np.int32), np.zeros(0, np.float32), np.zeros((0, 4), np.float32))
            continue
        dets[name] = (
            b.cls.cpu().numpy().astype(np.int32),
            b.conf.cpu().numpy().astype(np.float32),
            b.xyxy.cpu().numpy().astype(np.float32),
        )

    # 归一化到 [0,1]，与 GT 同尺度
    for name, res in zip(names, results):
        h, w = res.orig_shape[:2]
        c, s, xyxy = dets[name]
        if len(xyxy):
            xyxy = xyxy / np.array([w, h, w, h], dtype=np.float32)
        dets[name] = (c, s, xyxy)

    gts = {n: read_label(lbl_dir / f"{n}.txt") for n in names}

    confs = [float(x) for x in args.confs.split(",") if x.strip()]
    rows = []
    print(f"\n权重 {args.weights}    测试帧 {len(names)}    IoU>={args.iou}\n")
    header = ("conf", "总GT框", "正确命中", "框召回", "帧均命中", "GT>=3帧", "其中>=3", "类别无视召回")
    print("{:>6} {:>7} {:>8} {:>8} {:>8} {:>8} {:>8} {:>10}".format(*header))
    for conf in confs:
        tot_gt = tot_ok = tot_ca = 0
        per_frame = []
        frames_gt3 = frames_ok3 = 0
        for n in names:
            gt_c, gt_b = gts[n]
            pd_c, pd_s, pd_b = dets[n]
            keep = pd_s >= conf
            pd_c, pd_b = pd_c[keep], pd_b[keep]
            ok = match_count(gt_c, gt_b, pd_c, pd_b, args.iou, ignore_class=False)
            ca = match_count(gt_c, gt_b, pd_c, pd_b, args.iou, ignore_class=True)
            tot_gt += len(gt_b)
            tot_ok += ok
            tot_ca += ca
            per_frame.append((n, len(gt_b), ok, ca, len(pd_b)))
            if len(gt_b) >= 3:
                frames_gt3 += 1
                if ok >= 3:
                    frames_ok3 += 1
        recall = tot_ok / tot_gt if tot_gt else 0.0
        recall_ca = tot_ca / tot_gt if tot_gt else 0.0
        mean_hit = np.mean([f[2] for f in per_frame]) if per_frame else 0.0
        print(
            "{:>6.2f} {:>7d} {:>8d} {:>8.1%} {:>8.2f} {:>8d} {:>8d} {:>10.1%}".format(
                conf, tot_gt, tot_ok, recall, mean_hit, frames_gt3, frames_ok3, recall_ca
            )
        )
        rows.append(
            dict(
                conf=conf,
                gt_boxes=tot_gt,
                correct=tot_ok,
                recall=round(recall, 4),
                recall_ignore_class=round(recall_ca, 4),
                mean_hits=round(float(mean_hit), 3),
                frames_gt3=frames_gt3,
                frames_hit3=frames_ok3,
                pose_ok_rate=round(frames_ok3 / frames_gt3, 4) if frames_gt3 else 0.0,
            )
        )

    # 主阈值（conf 0.25）的逐帧明细
    main_conf = 0.25 if 0.25 in confs else confs[0]
    detail = []
    for n in names:
        gt_c, gt_b = gts[n]
        pd_c, pd_s, pd_b = dets[n]
        keep = pd_s >= main_conf
        pd_c, pd_b = pd_c[keep], pd_b[keep]
        ok = match_count(gt_c, gt_b, pd_c, pd_b, args.iou, ignore_class=False)
        ca = match_count(gt_c, gt_b, pd_c, pd_b, args.iou, ignore_class=True)
        detail.append(
            dict(image=n, gt=len(gt_b), hit=ok, hit_any_class=ca, det=len(pd_b),
                 gt_names=",".join(names_map[int(c)] for c in gt_c),
                 det_names=",".join(f"{names_map[int(c)]}:{s:.2f}"
                                    for c, s in zip(pd_c, pd_s[keep])))
        )

    out = root / args.out
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    out_detail = out.with_name(out.stem + "_detail" + out.suffix)
    with out_detail.open("w", newline="", encoding="utf-8-sig") as f:
        w = csv.DictWriter(f, fieldnames=list(detail[0].keys()))
        w.writeheader()
        w.writerows(detail)
    print(f"\n汇总 -> {args.out}    逐帧明细(conf {main_conf}) -> {out_detail.name}")

    # 命中不足 3 的帧，单独列出来便于查因
    bad = [d for d in detail if d["gt"] >= 3 and d["hit"] < 3]
    if bad:
        print(f"\n有 GT>=3 但命中<3 的帧 {len(bad)} 张：")
        for d in bad:
            print(f"  {d['image']}  GT[{d['gt_names']}]  命中{d['hit']}/{d['gt']}  检出[{d['det_names']}]")


if __name__ == "__main__":
    main()
