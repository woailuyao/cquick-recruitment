# -*- coding: utf-8 -*-
"""低机位素材的预标注：用「低置信度 + 去重」的模型框当种子，人工只改类别。

为什么不用 tools/prelabel_marker.py：
    那个工具是几何法（白线 Hough + 交点 + 分支数定类），在高机位很准；但 0.73 m 的
    掠射视角下白线几乎贴着画面横向铺开，分支被判错，实测 4 个框全落在凳子腿和球门网
    上。低机位改用模型本身当种子更靠谱：v9 在 conf 0.02~0.05 时能把交点位置找对
    （实测置信度 0.10~0.22），只是分不清 L/T/X —— 那正好交给人工按 1/2/3 一键改。

产出：
    <图片目录>/<同名>.txt      YOLO 格式种子标签（类别先按模型判断，可信度低）
    outputs/qc_marker/preview_low/*.jpg   带框预览，标出每个框的置信度
    outputs/qc_marker/prelabel_low.csv    每张图给了几个种子

用法（项目根目录）：
    run.bat tools\\prelabel_marker_lowconf.py --images dataset\\raw_marker
    run.bat tools\\prelabel_marker_lowconf.py --images dataset\\raw_marker --conf 0.03 --max-per-image 6

接下来人工复核：
    run.bat tools\\annotate.py --dir dataset\\raw_marker --mode click --classes LCross,TCross,XCross
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import OUTPUT_DIR, WEIGHTS_DIR                        # noqa: E402
from tools.realtime_field import dedup_dets, refine_junction                # noqa: E402

CLASSES = ["LCross", "TCross", "XCross"]        # 与 dataset_marker/marker_data.yaml 一致
EXTS = (".png", ".jpg", ".jpeg", ".bmp")
# 预览用色（BGR）
COLORS = {0: (0, 220, 0), 1: (230, 160, 0), 2: (0, 0, 235)}


def main() -> int:
    ap = argparse.ArgumentParser(description="低机位素材预标注（低置信 + 去重）")
    ap.add_argument("--images", default="dataset/raw_marker")
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_marker_v9.pt"))
    ap.add_argument("--conf", type=float, default=0.04, help="种子阈值（低机位 0.02~0.06）")
    ap.add_argument("--max-per-image", type=int, default=8)
    ap.add_argument("--dedup", type=float, default=35.0, help="同点合并半径（像素）")
    ap.add_argument("--preview", default=str(OUTPUT_DIR / "qc_marker" / "preview_low"))
    ap.add_argument("--device", default="0")
    ap.add_argument("--limit", type=int, default=0)
    args = ap.parse_args()

    img_dir = Path(args.images) if Path(args.images).is_absolute() else ROOT / args.images
    imgs = sorted(p for p in img_dir.rglob("*") if p.suffix.lower() in EXTS)
    if args.limit:
        imgs = imgs[:args.limit]
    if not imgs:
        print(f"没找到图片：{img_dir}")
        return 2

    prev_dir = Path(args.preview) if Path(args.preview).is_absolute() else ROOT / args.preview
    prev_dir.mkdir(parents=True, exist_ok=True)

    from ultralytics import YOLO
    model = YOLO(args.weights)
    print("=" * 78)
    print(f"低机位预标注｜{len(imgs)} 张｜conf>={args.conf}｜去重 {args.dedup:.0f}px｜上限 {args.max_per_image}")
    print("=" * 78)

    rows, total = [], 0
    for k, p in enumerate(imgs, 1):
        img = cv2.imread(str(p))
        if img is None:
            print(f"  跳过（读不出）{p.name}")
            continue
        h, w = img.shape[:2]
        res = model.predict(img, conf=args.conf, imgsz=640, device=args.device, verbose=False)[0]
        dets = []
        for b in res.boxes:
            typ = res.names[int(b.cls[0])]
            if typ not in CLASSES:
                continue
            box = [float(v) for v in b.xyxy[0]]
            u, v, mode, _nl = refine_junction(img, box)
            dets.append({"type": typ, "conf": float(b.conf[0]), "box": box,
                         "u": float(u), "v": float(v), "w": float(max(box[2] - box[0],
                                                                        box[3] - box[1]))})
        dets = dedup_dets(dets, radius=args.dedup)
        dets.sort(key=lambda d: -d["conf"])
        dets = dets[:args.max_per_image]

        lines = []
        for d in dets:
            cls = CLASSES.index(d["type"])
            cu, cv_ = d["u"] / w, d["v"] / h
            bw, bh = 0.05, 0.09        # 掠射视角下交点尺寸接近，先给统一框，人工可改
            lines.append(f"{cls} {cu:.6f} {cv_:.6f} {bw:.6f} {bh:.6f}")
        p.with_suffix(".txt").write_text("\n".join(lines) + ("\n" if lines else ""),
                                         encoding="utf-8")
        total += len(dets)
        rows.append({"image": p.name, "seeds": len(dets),
                     "confs": ";".join(f"{d['conf']:.2f}" for d in dets)})

        vis = img.copy()
        for d in dets:
            cls = CLASSES.index(d["type"])
            u, v = int(d["u"]), int(d["v"])
            r = 26
            cv2.rectangle(vis, (u - r, v - r), (u + r, v + r), COLORS[cls], 2)
            cv2.putText(vis, f"{CLASSES[cls][0]}{d['conf']:.2f}", (u - r, v - r - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, COLORS[cls], 2)
        cv2.putText(vis, f"{p.name}  seeds={len(dets)}  (1=L 2=T 3=X, D=next)", (8, h - 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.imwrite(str(prev_dir / f"{p.stem}.jpg"), vis, [cv2.IMWRITE_JPEG_QUALITY, 82])
        if k % 20 == 0 or k == len(imgs):
            print(f"  {k}/{len(imgs)}  累计种子 {total}")

    csv_path = OUTPUT_DIR / "qc_marker" / "prelabel_low.csv"
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8-sig") as f:
        wr = csv.DictWriter(f, fieldnames=["image", "seeds", "confs"])
        wr.writeheader()
        wr.writerows(rows)

    empty = sum(1 for r in rows if r["seeds"] == 0)
    print("-" * 78)
    print(f"完成：{len(rows)} 张，种子框 {total} 个（平均 {total / max(len(rows), 1):.1f}/张），"
          f"零种子 {empty} 张")
    print(f"预览：{prev_dir.relative_to(ROOT)}")
    print(f"清单：{csv_path.relative_to(ROOT)}")
    print("\n下一步：人工复核（1=L 2=T 3=X，D/空格 下一张）")
    print(f"  run.bat tools\\annotate.py --dir {args.images} --mode click "
          f"--classes LCross,TCross,XCross")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
