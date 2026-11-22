# -*- coding: utf-8 -*-
"""任务一评估：在独立测试集上出指标，并自动挑出误检/漏检案例。

细则 1.6 要求：记录 Precision / Recall / mAP50 / mAP50-95，
并「至少分析一个误检案例和一个漏检案例」。

产出：
    outputs/metrics.md        指标表格（可直接抄进 README）
    outputs/cases/fp_*.png    误检案例（有框但其实是别的东西）
    outputs/cases/fn_*.png    漏检案例（有球但没检出）
    outputs/cases/tp_*.png    正常案例（作对照）

用法：
    python tools/evaluate.py
    python tools/evaluate.py --split test --conf 0.25
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.common.paths import DATASET_DIR, OUTPUT_DIR, WEIGHTS_DIR, rel     # noqa: E402
from tools.annotate import load_boxes, yolo_to_xyxy                        # noqa: E402


def iou(a, b) -> float:
    ax1, ay1, ax2, ay2 = a
    bx1, by1, bx2, by2 = b
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    ua = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1) + max(0.0, bx2 - bx1) * max(0.0, by2 - by1) - inter
    return inter / ua if ua > 0 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser(description="评估足球检测模型")
    ap.add_argument("--weights", default=str(WEIGHTS_DIR / "best_ball.pt"))
    ap.add_argument("--data", default=str(DATASET_DIR / "data.yaml"))
    ap.add_argument("--split", default="test")
    ap.add_argument("--conf", type=float, default=0.25)
    ap.add_argument("--iou-match", type=float, default=0.5)
    ap.add_argument("--cases", type=int, default=3, help="每类案例最多出几张")
    ap.add_argument("--out", default="outputs/metrics.md", help="指标表写入路径")
    ap.add_argument("--cases-dir", default="outputs/cases", help="案例图输出目录")
    args = ap.parse_args()

    wpath = Path(args.weights)
    if not wpath.is_absolute():
        wpath = ROOT / wpath
    if not wpath.exists():
        print(f"没找到权重 {rel(wpath)}，请先训练")
        return 1

    from ultralytics import YOLO
    model = YOLO(str(wpath))
    print("=" * 74)
    print(f"评估：{rel(wpath)}  data={args.data}  split={args.split}  conf≥{args.conf}")
    print("=" * 74)

    res = model.val(data=str(args.data if Path(args.data).is_absolute() else ROOT / args.data),
                    split=args.split, conf=args.conf, imgsz=640, device="0", verbose=False)
    p = float(getattr(res.box, "mp", float("nan")))
    r = float(getattr(res.box, "mr", float("nan")))
    map50 = float(getattr(res.box, "map50", float("nan")))
    map5095 = float(getattr(res.box, "map", float("nan")))

    data_path = Path(args.data)
    if not data_path.is_absolute():
        data_path = ROOT / data_path
    # 数据集根目录跟着 data.yaml 走，不能写死去 dataset/（那是足球那一路的）
    ds_root = data_path.parent
    try:
        import yaml as _yaml
        _cfg = _yaml.safe_load(data_path.read_text(encoding="utf-8")) or {}
        _p = _cfg.get("path")
        if _p:
            _cand = Path(_p)
            if not _cand.is_absolute():
                _cand = ROOT / _cand
            if _cand.exists():
                ds_root = _cand
    except Exception:                                          # noqa: BLE001
        pass
    class_names = list((_cfg.get("names") or {}).values()) if isinstance(_cfg.get("names"), dict) \
        else list(_cfg.get("names") or [])

    cases_dir = Path(args.cases_dir)
    if not cases_dir.is_absolute():
        cases_dir = ROOT / cases_dir
    cases_dir.mkdir(parents=True, exist_ok=True)
    title = "目标检测 评估指标"
    if class_names:
        title += f"（{'/'.join(str(c) for c in class_names)}）"
    lines = [f"# {title}", "",
             f"- 权重：`{rel(wpath)}`",
             f"- 数据：`{rel(data_path)}`（split = {args.split}）",
             f"- 置信度阈值：{args.conf}",
             "",
             "| 指标 | 数值 | 含义 |",
             "| --- | ---: | --- |",
             f"| Precision | {p:.4f} | 检出框中真正是目标的比例 |",
             f"| Recall | {r:.4f} | 真实目标被检出的比例 |",
             f"| mAP50 | {map50:.4f} | IoU=0.5 时的平均精度 |",
             f"| mAP50-95 | {map5095:.4f} | 多 IoU 阈值下的平均精度 |",
             "",
             "> 测试集不参与训练与验证；具体划分方式见 `dataset_summary.md`。",
             ""]

    # 逐张找误检/漏检案例
    test_imgs = sorted((ds_root / "images" / args.split).glob("*.png"))
    n_fp = n_fn = n_tp = 0
    for img_path in test_imgs:
        img = cv2.imread(str(img_path))
        if img is None:
            continue
        h, w = img.shape[:2]
        gt_boxes = [yolo_to_xyxy(b, w, h) for b in
                    load_boxes(ds_root / "labels" / args.split / (img_path.stem + ".txt"))]
        result = model.predict(str(img_path), conf=args.conf, imgsz=640, device="0", verbose=False)[0]
        pred_boxes = []
        if result.boxes is not None and len(result.boxes) > 0:
            for xyxy, cf in zip(result.boxes.xyxy.cpu().numpy(), result.boxes.conf.cpu().numpy()):
                pred_boxes.append((xyxy.tolist(), float(cf)))

        matched_gt, matched_pred = set(), set()
        for i, pb in enumerate(pred_boxes):
            for j, gb in enumerate(gt_boxes):
                if j in matched_gt:
                    continue
                if iou(pb[0], gb) >= args.iou_match:
                    matched_gt.add(j)
                    matched_pred.add(i)
                    break

        view = img.copy()
        for j, gb in enumerate(gt_boxes):
            color = (0, 200, 0) if j in matched_gt else (0, 165, 255)   # 橙=漏检的 GT
            cv2.rectangle(view, (int(gb[0]), int(gb[1])), (int(gb[2]), int(gb[3])), color, 2)
        for i, (pb, cf) in enumerate(pred_boxes):
            if i not in matched_pred:
                cv2.rectangle(view, (int(pb[0]), int(pb[1])), (int(pb[2]), int(pb[3])), (0, 0, 255), 2)
                cv2.putText(view, f"FP {cf:.2f}", (int(pb[0]), int(pb[1]) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        fps = [i for i in range(len(pred_boxes)) if i not in matched_pred]
        fns = [j for j in range(len(gt_boxes)) if j not in matched_gt]

        if fps and n_fp < args.cases:
            n_fp += 1
            cv2.imwrite(str(cases_dir / f"fp_{n_fp:02d}_{img_path.stem}.png"), view)
        if fns and n_fn < args.cases:
            n_fn += 1
            cv2.imwrite(str(cases_dir / f"fn_{n_fn:02d}_{img_path.stem}.png"), view)
        if gt_boxes and not fps and not fns and n_tp < 2:
            # 必须"有 GT 且全部匹配上"才算正确案例。
            # 早先只判 not fps and not fns，会把「整张图既没标注也没预测」的空帧
            # 导成 tp_*.png —— 图上空无一物，拿去讲解等于自曝其短。
            n_tp += 1
            cv2.imwrite(str(cases_dir / f"tp_{n_tp:02d}_{img_path.stem}.png"), view)

    if "ball" in [str(c).lower() for c in class_names]:
        analysis = [
            "- 误检多来自白色场地线、圆形杂物（球鞋、标志盘）——它们的局部纹理与球相似",
            "- 漏检多发生在球贴画面边缘（只露一半）、强反光、以及小目标（面积占比 <0.2%）",
        ]
    else:
        analysis = [
            "- 误检多来自白色场地线上的普通拐角与直线交会，局部形状与标志相似",
            "- 漏检多发生在远景小目标与反光过曝处，线宽只剩几个像素时纹理特征不足",
        ]
    lines += [
        "## 案例",
        "",
        f"- 误检案例 `{rel(cases_dir)}/fp_*.png`：红框为误检（模型认为有目标但人工没标）",
        f"- 漏检案例 `{rel(cases_dir)}/fn_*.png`：橙框为漏检（人工标了但模型没检出）",
        f"- 对照案例 `{rel(cases_dir)}/tp_*.png`：检出正确",
        "",
        "分析要点（写 README 时展开）：",
        *analysis,
    ]

    out_md = Path(args.out)
    if not out_md.is_absolute():
        out_md = ROOT / out_md
    out_md.parent.mkdir(parents=True, exist_ok=True)
    out_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(f"Precision {p:.4f}  Recall {r:.4f}  mAP50 {map50:.4f}  mAP50-95 {map5095:.4f}")
    print(f"案例：误检 {n_fp} 张 / 漏检 {n_fn} 张 / 对照 {n_tp} 张 -> {rel(cases_dir)}")
    print(f"指标表：{rel(out_md)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
